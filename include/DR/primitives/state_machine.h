
#pragma once

#include <functional>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/leaf_system.h>

#include <DR/common/exception.h>

namespace DR {

/// Helper macro to throw an exception within methods that should not be called post-finalize.
#define DR_STATE_MACHINE_THROW_IF_FINALIZED() ThrowIfFinalized(__func__)

/// Helper macro to throw an exception within methods that should not be called pre-finalize.
#define DR_STATE_MACHINE_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)

/// Type used to identify a node in a StateMachine system.
using StateMachineNodeIndex = drake::TypeSafeIndex<class StateMachineNodeTag>;

std::ostream& operator<<(std::ostream& os, const std::vector<StateMachineNodeIndex>& indices) {
  for (const auto& index : indices) {
    os << static_cast<int64_t>(index) << ", ";
  }
  return os;
}

/**

 A state machine class that will continually poll assigned transition functions (graph edges) and update
 its internal state (graph nodes).

 The state machine can be polled to check its current internal state by calling `current_state()`.
 The state machine autonomously updates at the frequency `update_freq` set in the constructor.

 At each update, the state machine:
 -- outbound edges (`EdgeFunction`s) from the current state node (at node index `current_state()`) are evaluated to
    determine whether any outbound edges are valid (the `EdgeFunction` returns true).
 -- If an outbound edge is valid, the state machine will follow the outbound edge and update the current state.

 This edge polling and traversal process is repeated until no outbound edges are valid. Once the traversal process
 finishes, the current state is recorded into the system context and the update is complete.

 An implication of this updating method is that more than one edge can be traversed (and more than one node visited) in
 a single update.  This can happen when multiple `EdgeFunction`s evaluate true at one moment along a contiguous path in
 the state machine graph.

 Advantages of following valid edges until settling on a state include:
 1) The state machine can detect circumstances where the state might cycle indefinitely between a few nodes by detecting
when it has visited a node more than once in a single update.  The action taken in this case is to throw a
`std::runtime_error`. 2) The state machine can skip-over nodes that would be visited for just one iteration.  Such
circumstances typically result in jerky or delayed behavior.  Already stale states are hopped over until the state
machine settles at a node that has yet unsatisfied exit conditions (i.e., the node has no valid outbound edges).

  For a simple example of using this code see 'primitives/test/state_machine_test.cc' test
 'StateMachineTest.StateTransitions'.  For use in a robotics context, see 'primitives/test/state_machine_test.cc' test
 'StateMachineTest.Pendulum'.

 To configure the state machine's graph, add nodes with `AddNode(node_name)` and then add edges to existing nodes with
 `AddEdge(from_node,  to_node, exit_condition_fn)`.  Where `from_node` and `to_node` are type `StateMachineNodeIndex`
 and `exit_condition_fn` is a function of type `EdgeFunction`.

 Requirements for a feasible state machine graph include:
 -- There must be at least 2 nodes.
 -- A start state node must be set.
 -- Each node must have at least 1 outbound edge.  An outbound edge is a state transition from one node to itself or any
    other node in the state machine graph.

 These conditions are checked when `Finalize()` is called.

 The state machine will not allow further modification to its graph (e.g., adding nodes and edges) after it has been
 finalized.

 The state machine will not permit itself to be updated or polled for its current state until it has been finalized.

 # Deriving a subclass from state machine.
 Edge functions receive a `const drake::systems::Context` from the state machine to permit them to poll input ports for
relevent state information. To make use of this functionality, derive a subclass of `StateMachine` and add input ports
that receive state information relevent to any edge functions in the state machine graph.  When an edge function is
called, evaluate the relevent input port in `EdgeFunction`, using the state machine context parameter provided to the
method.

 @tparam T a Drake default scalar type
 */
template <typename T>
class StateMachine : public drake::systems::LeafSystem<T> {
 public:
  /**
   An edge function is called for each outbound edge from the current state at each update cycle.
   Functions fitting the `EdgeFunction` signature must be non-static class members of a `StateMachine` subclass in
   order to make use of the context parameter.
   A state machine context is passed to this function when StateMachine polls the edge function.  The context should be
   used to evaluate state machine input ports that might be added to a `StateMachine` subclass.
   */
  typedef std::function<bool(const drake::systems::Context<T>&)> EdgeFunction;

  /// Input type used for enabling/disabling the state machine updates.
  enum OperationSignalType {
    /// The state machine will update its state at the update rate.
    kActive,
    /// The state machine will not attempt update its state or make calls its edge functions.
    kInactive,
  };

  // No copy-construct or copy-assign.
  StateMachine(const StateMachine&) = delete;
  void operator=(const StateMachine&) = delete;
  // No move-construct or move-assign.
  StateMachine(StateMachine&&) = delete;
  void operator=(StateMachine&&) = delete;
  // Default destructor.
  virtual ~StateMachine() = default;
  // No default constructor.
  StateMachine() = delete;

  /**
  The only constructor for StateMachine, sets the update frequency that determines how often state transitions occur.
  @param update_freq determines the frequency (in Hz) at which `Update(.)` will be called.  A higher update frequency
         will poll inputs necessary to determine edge validity more often, leading to more computation per unit time
         (i.e., possibly slower simulation or harder-to-satisfy realtime constraints). Lower state machine update
         frequencies will save on computation but increase the risk of missing a state transition.
   */
  explicit StateMachine(double update_freq) : update_freq_(update_freq) {
    DR_DEMAND(update_freq > 0.0);
    // Declare the abstract Node for the plan.
    current_state_index_ =
        this->DeclareAbstractState(drake::AbstractValue::Make<StateMachineNodeIndex>(StateMachineNodeIndex()));

    // Declare the periodic unrestricted update used for updating the state from any queued plans.
    const double period = 1.0 / update_freq_;
    const double offset = 0.0;

    this->DeclarePeriodicUnrestrictedUpdateEvent(period, offset, &StateMachine<T>::Update);

    operation_signal_input_port_index_ =
        this->DeclareAbstractInputPort("enable", drake::Value<OperationSignalType>()).get_index();
  }

  /// This input port signal determins whether the state machine is permitted to update.
  /// See OperationSignalType documentation for the effects of different values.
  const drake::systems::InputPort<T>& operation_signal_input_port() const {
    DR_STATE_MACHINE_THROW_IF_NOT_FINALIZED();
    drake::log()->debug("operation_signal_input_port");
    return drake::systems::System<T>::get_input_port(operation_signal_input_port_index_);
  }

  /// Gets the current state of the state machine.
  /// @returns the StateMachineNodeIndex of the currently active node in the state machine graph.
  StateMachineNodeIndex current_state(const drake::systems::Context<T>& context) const {
    DR_STATE_MACHINE_THROW_IF_NOT_FINALIZED();
    const StateMachineNodeIndex& node =
        context.template get_abstract_state<StateMachineNodeIndex>(current_state_index_);
    return node;
  }

  /// @returns the vector of the indices of states visited in the most recent (complete) update cycle.
  ///          @note If this method is called during an update (e.g. in an `EdgeFunction`) it
  ///          will return the states visited in the previous update. The mutable state in mutable context has not yet
  ///          been written to the const state in the context.
  const std::vector<StateMachineNodeIndex>& states_visited_last_update(
      const drake::systems::Context<T>& context) const {
    DR_STATE_MACHINE_THROW_IF_NOT_FINALIZED();
    return context.template get_abstract_state<std::vector<StateMachineNodeIndex>>(states_visited_last_update_index_);
  }

  double update_freq() const { return update_freq_; }

  bool HasNode(const std::string& name) const { return node_name_to_index_.find(name) != node_name_to_index_.end(); }

  bool HasNode(StateMachineNodeIndex node_index) const { return (node_index < node_index_to_name_.size()); }

  StateMachineNodeIndex GetNodeIndexByName(const std::string& name) const {
    DR_DEMAND(HasNode(name));
    return node_name_to_index_.find(name)->second;
  }
  const std::string& GetNodeNameByIndex(StateMachineNodeIndex index) const {
    DR_DEMAND(HasNode(index));
    return node_index_to_name_[index];
  }

  int num_nodes() const { return node_index_to_name_.size(); }

  bool is_finalized() const { return is_finalized_; }

  void Finalize() {
    DR_STATE_MACHINE_THROW_IF_FINALIZED();

    // Check that the state machine has nodes.
    DR_DEMAND(num_nodes() >= 2);

    // Because a node can't be visited twice, the max number of visited nodes is equal to the total nodes.
    // Allocate memory for visited nodes tracking.
    states_visited_last_update_index_ =
        this->DeclareAbstractState(drake::AbstractValue::Make<std::vector<StateMachineNodeIndex>>(
            std::vector<StateMachineNodeIndex>(num_nodes())));

    // Check that the state machine has a node designated as the start state.
    DR_DEMAND(start_state_.is_valid());

    // Check that all nodes have outbound edges.
    for (const auto& name_index : node_name_to_index_) {
      const std::string& outbound_node_name = name_index.first;
      const StateMachineNodeIndex outbound_node = name_index.second;
      const OutboundEdges& outbound_edges = edges_.at(outbound_node);
      if (outbound_edges.empty()) {
        drake::log()->critical(
            "There are no outbound edges from node {}.  Each node must have at least one outbound edge.",
            outbound_node_name);
        throw std::logic_error("no outbound edges from node!");
      }
    }
    is_finalized_ = true;
  }

  StateMachineNodeIndex start_state() const { return start_state_; }
  void set_start_state(StateMachineNodeIndex node) {
    DR_DEMAND(HasNode(node));
    start_state_ = node;
  }

  void set_start_state(const std::string& nodename) { set_start_state(GetNodeIndexByName(nodename)); }

  /// Sets the state machine's current state to the start state.
  void ResetToStartState(drake::systems::State<T>* state) const { set_current_state(start_state(), state); }

  void SetDefaultState(const drake::systems::Context<T>&, drake::systems::State<T>* state) const final {
    this->ResetToStartState(state);
  }

  /**
   Adds a node to the state machine graph.
   @param name the name of the new node in the state machine graph.
   @return StateMachineNodeIndex the index of the node added to the state machine graph.
   @pre The state machine is not finalized.
   @pre The node name must be unique.
   */
  StateMachineNodeIndex AddNode(const std::string& name) {
    DR_STATE_MACHINE_THROW_IF_FINALIZED();
    // Check whether this state machine already contains a node with this name.
    DR_DEMAND(!HasNode(name));
    const StateMachineNodeIndex index(node_index_to_name_.size());
    node_index_to_name_.push_back(name);
    node_name_to_index_[name] = index;
    return index;
  }

  /**
   Adds an edge to the state machine graph.
   @param outbound_node_name the name of an existing node in the state machine graph.  When the state machine's current
          state is at this node, the `edge_function` will be checked for validity.
   @param inbound_node_name the name of an existing node in the state machine graph.  When the `edge_function` is
          checked and is valid, the state machine's state will transition to this node.
   @param edge_function a function of type `EdgeFunction`, this function is checked and is valid, the state machine's
          state will transition to node `inbound_node_name`.
   @pre The state machine is not finalized.
   */
  void AddEdge(const std::string& outbound_node_name, const std::string& inbound_node_name,
               EdgeFunction edge_function) {
    // `edge_function` is being passed by value for performance reasons (so that it can be moved).
    AddEdge(GetNodeIndexByName(outbound_node_name), GetNodeIndexByName(inbound_node_name), std::move(edge_function));
  }

  /**
   Adds an edge to the state machine graph.
   @param outbound_node the index of an existing node in the state machine graph.  When the state machine's current
          state is at this node, the `edge_function` will be checked for validity.
   @param inbound_node the index of an existing node in the state machine graph.  When the `edge_function` is
          checked and is valid, the state machine's state will transition to this node.
   @param edge_function a function of type `EdgeFunction`, this function is
          checked and is valid, the state machine's state will transition to node `inbound_node_name`.
   @pre The state machine is not finalized.
   @note `edge_function` will overwrite any existing edge directed from `outbound_node` to `inbound_node`.
   */
  void AddEdge(StateMachineNodeIndex outbound_node, StateMachineNodeIndex inbound_node, EdgeFunction edge_function) {
    DR_STATE_MACHINE_THROW_IF_FINALIZED();
    DR_DEMAND(HasNode(outbound_node));
    DR_DEMAND(HasNode(inbound_node));

    // Add outbound_edge transition function to outbound_node
    // `edge_function` is being passed by value for performance reasons (so that it can be moved).
    edges_[outbound_node][inbound_node] = std::move(edge_function);
  }

  /**
   Gets all outbound edges whose EdgeFunctions return true--- indicating that the edge should be followed to the
   inbound node.
   @param outbound_node the node whose outbound edges will be checked for validity.
   @param context a const reference to the state machine context.
   @return the `StateMachineNodeIndex` of the nodes connected by a valid outbound edge.
   */
  StateMachineNodeIndex CalcNextState(StateMachineNodeIndex outbound_node,
                                      const drake::systems::Context<T>& context) const {
    DR_STATE_MACHINE_THROW_IF_NOT_FINALIZED();
    StateMachineNodeIndex inbound_node;

    drake::log()->trace(" >> [CalcNextState] from {}", GetNodeNameByIndex(outbound_node));
    // Get destination node indices and edge transition functions.
    const OutboundEdges& edges = edges_.at(outbound_node);

    // Populate output from valid edge (i.e., EdgeFunction returns true).
    for (const auto& node_function_pair : edges) {
      const EdgeFunction& outbound_edge = node_function_pair.second;
      if (outbound_edge(context)) {
        if (!inbound_node.is_valid()) {
          inbound_node = node_function_pair.first;
        } else {
          throw std::runtime_error("StateMachine::CalcNextState detected more than one valid destination node.");
        }
      }
    }
    // If no valid outbound edges, throw an error.
    if (!inbound_node.is_valid()) {
      throw std::runtime_error("StateMachine::CalcNextState found no valid outbound edges from the current state.");
    }
    drake::log()->trace(" -- CalcNextState: Follow edge {} --> {}", GetNodeNameByIndex(outbound_node),
                        GetNodeNameByIndex(inbound_node));
    return inbound_node;
  }

 private:
  void set_current_state(StateMachineNodeIndex new_node, drake::systems::State<T>* state) const {
    state->template get_mutable_abstract_state<StateMachineNodeIndex>(current_state_index_) = new_node;
  }

  void ThrowIfFinalized(const std::string& source_method) const {
    if (is_finalized()) {
      throw std::logic_error("Post-finalize calls to '" + std::string(source_method) +
                             "()' are "
                             "not allowed; calls to this method must happen before Finalize().");
    }
  }

  void ThrowIfNotFinalized(const std::string& source_method) const {
    if (!is_finalized()) {
      throw std::logic_error("Pre-finalize calls to '" + std::string(source_method) +
                             "()' are "
                             "not allowed; you must call Finalize() first.");
    }
  }

  // This method is called at the update frequency on every unrestricted update.
  void Update(const drake::systems::Context<T>& context, drake::systems::State<T>* state) const {
    DR_STATE_MACHINE_THROW_IF_NOT_FINALIZED();
    drake::log()->debug(" >> StateMachine::Update()");

    // Follow valid edges to next state.
    StateMachineNodeIndex current_node = this->current_state(context);

    // Record all visited state nodes indices.
    std::vector<StateMachineNodeIndex>& states_visited =
        state->template get_mutable_abstract_state<std::vector<StateMachineNodeIndex>>(
            states_visited_last_update_index_);

#ifndef NDEBUG
    // Bookkeeping for update function, records whether a node has been visited in the current update.
    // Initialize whether any nodes have been visited to false.
    std::vector<bool> node_was_visited(num_nodes(), false);

#endif

    // Clear the vector of nodes visited this cycle (does not deallocate reserved size).
    states_visited.clear();

    drake::log()->debug(" -- current_node: {}", current_node);

    if (operation_signal_input_port().template Eval<OperationSignalType>(context) == OperationSignalType::kActive) {
      StateMachineNodeIndex inbound_node = current_node;
      do {
        current_node = inbound_node;

        // Add the current node to the vector of nodes visited this cycle.
        states_visited.push_back(current_node);

#ifndef NDEBUG
        // Prevent infinite loops, only ever visit a node once.
        if (!node_was_visited[current_node]) {
          node_was_visited[current_node] = true;
        } else {
          throw std::runtime_error("StateMachine's Node got caught in an infinite loop!");
        }
#else
        // Prevent infinite loops, can only traverse total count of nodes before we must have repeated once.
        // NOTE: This is a fast way to check a necessary condition, but is not as strict of a check as the debug-only
        // version above.
        if (static_cast<int>(states_visited.size()) > num_nodes()) {
          throw std::runtime_error("StateMachine's Node got caught in an infinite loop!");
        }
#endif

        // Collect available transitions from this node to others.
        drake::log()->debug(" -- visit: {}", GetNodeNameByIndex(current_node));

        inbound_node = this->CalcNextState(current_node, context);
      } while (inbound_node != current_node);
      drake::log()->trace("State machine visited the following nodes this update cycle: {}", states_visited);
    }

    // Update active node.
    this->set_current_state(current_node, state);

    drake::log()->debug(" << StateMachine::Update()");
  }

  drake::systems::AbstractStateIndex current_state_index_;

  // Bookkeeping for update function, records nodes visited in order during each update.
  drake::systems::AbstractStateIndex states_visited_last_update_index_;

  drake::systems::InputPortIndex operation_signal_input_port_index_{};

  // When a node is added to the tree its index and name are stored here.
  std::map<std::string, StateMachineNodeIndex> node_name_to_index_;
  std::vector<std::string> node_index_to_name_;

  // A container pairing incoming node indices with the edge functions that lead to those nodes.
  typedef std::unordered_map<StateMachineNodeIndex, EdgeFunction> OutboundEdges;

  // A container pairing each node index to all outbound edges from that node stored in container `OutboundEdges`.
  // The nested `OutboundEdges` container pairs all outbound edges with their destination incoming node index.
  std::unordered_map<StateMachineNodeIndex, OutboundEdges> edges_;

  // Determines the frequency (in Hz) at which `Update(.)` will be called.
  const double update_freq_;

  // Indicates whether the state machine has been fully configured.
  bool is_finalized_{false};

  // Start state: the node that the state machine will enter from its uninitialized state (first step).
  StateMachineNodeIndex start_state_{};
};

}  // namespace DR
