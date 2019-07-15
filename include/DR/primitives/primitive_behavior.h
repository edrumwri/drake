#pragma once

#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>

#include <drake/common/copyable_unique_ptr.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>

#include <DR/primitives/plan.h>

namespace DR {

/**
 A base class for primitive behaviors that permits derived classes to easily implement control with long-term planning
 using the block diagram paradigm. %PrimitiveBehavior provides many of the building blocks for developing robot
 controllers, a pointer to the robot plant, estimated robot state, and generalized "effort" outputs. 

 @section planning-system-operation Planning system operation

 %PrimitiveBehavior triggers its planning system whenever 1) a plan is not _active_, meaning that no plan has been
 computed or the last plan that was computed is no longer valid (due to, e.g., its time of validity expiring); 2) the
 output port is calculated (i.e., its `Calc()` or `Eval()` methods are called); and 3) the operation signal input is not
 set to `OperationalSignalType::kInactive`. 

 The planning mechanism runs in a separate thread, and only one plan is computed at any instance in time. While a plan
 is being computed, an _active_ plan, which lives in the %PrimitiveBehavior's State, might or might not already exist
 for the controller to use. The controller must be able to generate control outputs in either case.
 
 After a plan is computed, another step must be completed before the plan becomes active: the %PrimitiveBehavior must be
 advanced in time to the inverse of the next `copy_freq()` time, at which point the plan will be copied to the
 %PrimitiveBehavior's State. Note that we only update the %PrimitiveBehavior's state infrequently (i.e., at copy_freq())
 in order to minimize the number of mutex locks and heap allocations. Though not the expected case, it is conceivable
 that multiple plans may be computed in between updates to the State. In this case, only the last plan is copied to the
 state; the others are treated as out-of-date and are discarded. 

 @subsection warnings Warnings 
 
 @subsubsection concurrency-warnings Warning related to concurrency

 Drake systems are nominally "const", which allows separating data and code for effective parallelism and better design.
 %PrimitiveBehavior is not, though it pretends like it is so that it can act as a Drake system: %PrimitiveBehavior uses
 several `mutable` variables to allow the planning process to run in the background. Other kinds of solutions, like
 message passing, would allow the system to be constant in appearance, though the IPC mechanisms living in the operating
 system would still act as hidden state. By keeping the IPC mechanism living in this System, we not only give that
 hidden state more visibility, we also can inspect it.

 **These mutable variables mean that you should not use the same %PrimitiveBehavior-derived system to operate upon
 multiple Context objects simultaneously.**

 @subsubsection real-time-performance-warnings Warnings related to real-time performance

 Note that if a simulation (via Simulator) of the Diagram runs slower than real-time, performance disparities might be
 observed: this will appear to allow the planner to compute plans much faster than nominal.

 Also note that the plan-to-state copy operation, which happens at `copy_freq()` is relatively expensive: it performs
 both mutex locking and heap allocations. Thus `copy_freq()` should be set to a sufficiently low frequency to keep from
 overrunning the real-time requirement.

 State:
 - plan (abstract): The plan computed by a derived class.

 Input ports:
 - operation signal (abstract): a variable of type OperationSignal that indicates whether the primitive sends zero
   outputs.
 - robot_q_estimated (vector): the estimated robot generalized positions, ordered the same way that the MultibodyPlant
   orders the robot's generalized positions
 - robot_v_estimated (vector): the estimated robot generalized velocities, ordered the same way that the MultibodyPlant
   orders the robot's generalized velocities

 Output ports:
 - control output (vector): the vector of generalized effort commands
*/
template <typename T>
class PrimitiveBehavior : public drake::systems::LeafSystem<T> {
 public:
  PrimitiveBehavior(const drake::multibody::MultibodyPlant<T>* robot_plant, double copy_freq);
  virtual ~PrimitiveBehavior() { BlockOnPlanningThreadTermination(); }

  /// Gets the frequency (in Context time) with which any completed plans are copied to this behavior's State.
  double copy_freq() const { return copy_freq_; }

  /// Input type used for enabling/disabling the control output.
  enum OperationSignalType {
    /// The primitive will generate commands: calling the output function
    /// *might* not zero the generalized effort commands.
    kActive,

    /// The primitive will not generate commands: calling the output function
    /// will zero the generalized effort commands.
    kInactive,
  };

  /// Gets a pointer to the currently active plan (if any). Returns nullptr if no plan is active.
  const Plan<T>* active_plan(const drake::systems::Context<T>& context) const {
    const Plan<T>* plan = context.template get_abstract_state<drake::copyable_unique_ptr<Plan<T>>>(plan_index_).get();
    const T& t = context.get_time();
    if (plan && t >= plan->start_time() && t <= plan->end_time()) {
      return plan;
    } else {
      return nullptr;
    }
  }

  /// Halts computation until the currently active planning thread (if any) terminates. Returns immediately if there is
  /// no currently active planning thread.
  void BlockOnPlanningThreadTermination() const {
    if (thread_.joinable())
      thread_.join();
  }

  /// Gets the input port for the operation signal. See DoCalcControlOutput().
  const drake::systems::InputPort<T>& operation_signal_input_port() const {
    return drake::systems::System<T>::get_input_port(operation_signal_input_port_index_);
  }

  /// Gets the input port for the estimated robot generalized positions.
  const drake::systems::InputPort<T>& robot_q_estimated_input_port() const {
    return drake::systems::System<T>::get_input_port(robot_q_estimated_input_port_index_);
  }

  /// Gets the input port for the estimated robot generalized velocities.
  const drake::systems::InputPort<T>& robot_v_estimated_input_port() const {
    return drake::systems::System<T>::get_input_port(robot_v_estimated_input_port_index_);
  }

  /// Gets the output port for the generalized effort output.
  const drake::systems::OutputPort<T>& generalized_effort_output_port() const {
    return drake::systems::System<T>::get_output_port(generalized_effort_output_port_index_);
  }

  /// Gets a reference to the MultibodyPlant that `this` was constructed with.
  const drake::multibody::MultibodyPlant<T>& robot_plant() const { return *robot_plant_; }

  /// Determines whether the behavior's planning thread is active.
  // Note: this method takes a Context argument, even though it doesn't use it, so that we can possibly re-design this
  // method in the future to be truly "const".
  bool is_planning(const drake::systems::Context<T>&) const { return thread_.joinable(); }

 protected:
  /// Derived classes must implement this method to compute a plan. This method can return an empty `unique_ptr` to
  /// indicate that the method failed to compute a plan (e.g., because a valid plan does not exist).
  virtual std::unique_ptr<Plan<T>> DoComputePlan(const drake::systems::Context<T>& context) const = 0;

  /// Derived classes must implement this method to compute a control output. This method is only called when the
  /// `generalized_effort_output_port()` is calculated (i.e., using its `Calc()` method) *and* the operation signal
  /// input port is set to `kActive`.
  virtual void DoCalcControlOutput(
      const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const = 0;

 private:
  void UpdatePlanStateFromPlanQueue(const drake::systems::Context<T>& context, drake::systems::State<T>* state) const;
  void CalcControlOutput(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const;
  void SetDefaultState(const drake::systems::Context<T>& context, drake::systems::State<T>* state) const final;
  void StartPlanning(const drake::systems::Context<T>& context) const;

  std::unique_ptr<Plan<T>> ComputePlan(const drake::systems::Context<T>& context) const {
    return DoComputePlan(context);
  }

  // The pointer to the robot being controlled.
  const drake::multibody::MultibodyPlant<T>* robot_plant_{nullptr};

  // The frequency (in Hz) with which the plan is copied from the plan queue to the PrimitiveBehavior's abstract state.
  double copy_freq_{0.0};

  // Mutable variables for supporting multi-threaded operation.
  mutable std::thread thread_{};
  mutable std::queue<drake::copyable_unique_ptr<Plan<T>>> plan_queue_{};
  mutable std::mutex plan_queue_mutex_{};

  // Port indices.
  drake::systems::InputPortIndex operation_signal_input_port_index_{};
  drake::systems::InputPortIndex robot_q_estimated_input_port_index_{};
  drake::systems::InputPortIndex robot_v_estimated_input_port_index_{};
  drake::systems::OutputPortIndex generalized_effort_output_port_index_{};
  drake::systems::AbstractStateIndex plan_index_;
};

/// Constructs the primitive behavior, based on the given robot model, that copies any plans into this behavior's State
/// at the specified frequency (in Context time, not real time). The primitive will retain a pointer to the
/// plant while this object remains alive.
/// @param robot_plant a pointer to a plant containing only the robot to be controlled
/// @param copy_freq the frequency in Context time with which plans are copied to this behavior's State
/// @pre `robot_plant` is not `nullptr`
/// @pre `copy_freq` >= 0.0
template <typename T>
PrimitiveBehavior<T>::PrimitiveBehavior(const drake::multibody::MultibodyPlant<T>* robot_plant, double copy_freq)
    : robot_plant_(robot_plant), copy_freq_(copy_freq) {
  DRAKE_DEMAND(copy_freq > 0.0);

  // Declare the periodic unrestricted update used for updating the state from any queued plans.
  const double period = 1.0 / copy_freq;
  const double offset = 0.0;
  this->DeclarePeriodicUnrestrictedUpdateEvent(period, offset, &PrimitiveBehavior<T>::UpdatePlanStateFromPlanQueue);

  // Declare the operation signal input port.
  operation_signal_input_port_index_ = this->DeclareAbstractInputPort(
          "operation_signal", drake::Value<OperationSignalType>()).get_index();

  // Declare estimated state inputs.
  robot_q_estimated_input_port_index_ = this->DeclareVectorInputPort("robot_q_estimated",
      drake::systems::BasicVector<T>(robot_plant->num_actuators())).get_index();
  robot_v_estimated_input_port_index_ = this->DeclareVectorInputPort("robot_v_estimated",
      drake::systems::BasicVector<T>(robot_plant->num_actuators())).get_index();

  // Declare the generalized effort output port.
  generalized_effort_output_port_index_ = this->DeclareVectorOutputPort("generalized_effort",
      drake::systems::BasicVector<T>(robot_plant->num_actuators()),
      &PrimitiveBehavior::CalcControlOutput).get_index();

  // Declare the abstract state for the plan.
  plan_index_ = this->DeclareAbstractState(drake::AbstractValue::Make(drake::copyable_unique_ptr<Plan<T>>()));
}

template <typename T>
void PrimitiveBehavior<T>::SetDefaultState(
    const drake::systems::Context<T>&, drake::systems::State<T>* state) const {
  // Reset the pointer to the plan.
  state->template get_mutable_abstract_state<drake::copyable_unique_ptr<Plan<T>>>(plan_index_).reset();
}

template <typename T>
void PrimitiveBehavior<T>::UpdatePlanStateFromPlanQueue(
    const drake::systems::Context<T>&, drake::systems::State<T>* state) const {
  // TODO(edrumwri): Re-consider whether this is a good idea.
  // This loop exists because it is conceivable that multiple plans are placed into the queue, while normally a plan is
  // transferred into the state at a greater rate than plans are computed. In this case, we treat the older plans as
  // stale and simply discard them.
  plan_queue_mutex_.lock();
  while (!plan_queue_.empty()) {
    drake::copyable_unique_ptr<Plan<T>> plan = plan_queue_.front();
    plan_queue_.pop();
    if (plan_queue_.empty())
      state->template get_mutable_abstract_state<drake::copyable_unique_ptr<Plan<T>>>(plan_index_) = std::move(plan);
  }
  plan_queue_mutex_.unlock();
}

template <typename T>
void PrimitiveBehavior<T>::StartPlanning(const drake::systems::Context<T>& context) const {
  // Wait for the thread to finish executing.
  BlockOnPlanningThreadTermination();

  // Start the planning thread.
  auto f = [this](const drake::systems::Context<T>& c) -> void {
    // Compute the plan. This is of course doing a heap allocation, which we would normally avoid in the context of a
    // control loop, but given that this is happening during a long-term planning process, it seems less problematic.
    drake::copyable_unique_ptr<Plan<T>> plan(ComputePlan(c));

    // Store the plan.
    plan_queue_mutex_.lock();
    plan_queue_.push(std::move(plan));
    plan_queue_mutex_.unlock();
  };
  thread_ = std::thread(f, std::ref(context));
}

template <typename T>
void PrimitiveBehavior<T>::CalcControlOutput(
    const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const {
  // Zero the output if the operation signal is inactive.
  if (operation_signal_input_port().template Eval<PrimitiveBehavior<T>::OperationSignalType>(context) ==
      OperationSignalType::kInactive) {
    output->SetZero();
    return;
  }

  // Start planning, if necessary.
  if (!thread_.joinable())
    StartPlanning(context);

  DoCalcControlOutput(context, output);
}

}  // namespace DR

// Instantiate templates.
extern template class DR::PrimitiveBehavior<double>;
