
/*
  This class demonstrates and tests the state machine when used in a robotics context.

  This file implements a simulation of a single pendulum with a torque limit swinging-up to balance at its upright
  position (i.e., an inverted pendulum).  A state machine monitors the state of the pendulum (joint angle and velocity)
  to determine when to switch to the needed controller.

  See `SwingUpBehaviorScheduler` documentation for a description of the swing-up task and how the state machine enables
  programming tasks that require multiple controllers triggered by entering certain regions of state space.
*/
#include <DR/primitives/state_machine.h>

#include <gtest/gtest.h>

#include <DR/common/exception.h>
#include <DR/primitives/primitive_behavior.h>
#include <DR/tools/differential_inverse_kinematics.h>

#include <drake/geometry/geometry_visualization.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/benchmarks/acrobot/make_acrobot_plant.h>
#include <drake/multibody/benchmarks/pendulum/make_pendulum_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/controllers/pid_controller.h>
#include <drake/systems/primitives/demultiplexer.h>
#include <drake/systems/primitives/gain.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/saturation.h>
#include <drake/systems/primitives/wrap_to_system.h>

using drake::multibody::benchmarks::pendulum::MakePendulumPlant;
using drake::multibody::benchmarks::pendulum::PendulumParameters;

namespace DR {
namespace {

// Tolerance for checking the equality of two states.
// 1 degree = 0.0174 rad.
const double kPosTolerance = 0.01;
const double kVelTolerance = 0.01;
// Update frequency of state machine.
const double kUpdateFreq = 10.0;
// Add energy kd gain.  Negative value acts to increase error from desired state.
const double kDrivingGain = -2.0;
// Remove energy kd gain.
const double kDampingGain = 1.0;
// Stabilization gains.
const double kPIDKpGain = 50.0;
const double kPIDKdGain = 1.0;
// Distance from upright pendulum state to start stabilization controller.
const double kStabilizationTolerance = M_PI_4;
// Torque limit.
const double kMaxTorque = 15.0;
// Maximum time in seconds allowed to complete swing up then swing down.
// NOTE: This duration was measured from the test.
const double kMaxSimulationTime = 9.5;

/*
  This class is derived from a state machine and programmed to switch between the behaviors needed to swing-up then
  balance a single pendulum at its upright position.

  States & behaviors:

  1) swing-up: energy is added to the pendulum by applying torque to the pendulum's revolute joint in the same direction
     as the current velocity.  The pendulum will swing higher until it gains enough energy to reach its upright
     position.  Once the pendulum enters the region around its upright position, the state machine switches to a
     stabilization behavior.

  2) stabilization: The revolute joint's actuator implements a PD controller that damps the
     velocity of the pendulum while reducing the positional error from the upright (unstable equilibrium) position.
     Once the pendulum reaches its upright position, the state machine switches to a remove energy behavior.

  3) remove energy: energy is removed from the pendulum system as it falls back down to its lowest energy position (the
     stable equilibrium position) by applying torque opposing the direction of the pendulum's velocity until it settles
     at the bottom of the swing.  Once the pendulum reaches the bottom position, the state machine switches back to the
     swing-up behavior and restarts the procedure above.
*/
template <typename T>
class SwingUpBehaviorScheduler : public StateMachine<T> {
 public:
  SwingUpBehaviorScheduler() = delete;
  explicit SwingUpBehaviorScheduler(double update_freq, const drake::multibody::MultibodyPlant<T>& robot_plant)
      : StateMachine<T>(update_freq), num_outputs_(robot_plant.num_actuators()) {
    // Declare the generalized effort output port.
    generalized_effort_output_port_index_ =
        this->DeclareVectorOutputPort("generalized_effort", drake::systems::BasicVector<T>(num_outputs_),
                                      &SwingUpBehaviorScheduler<T>::CalcControlOutput)
            .get_index();

    // Declare robot state inputs.
    current_q_input_port_index_ =
        this->DeclareVectorInputPort("current_q", drake::systems::BasicVector<double>(robot_plant.num_positions()))
            .get_index();
    current_v_input_port_index_ =
        this->DeclareVectorInputPort("current_v", drake::systems::BasicVector<double>(robot_plant.num_velocities()))
            .get_index();
  }
  ~SwingUpBehaviorScheduler() {}

  void AddControllerEffortInput(const std::string& behavior_name) {
    controller_effort_input_port_index_[behavior_name] =
        this->DeclareVectorInputPort(behavior_name + "_effort", drake::systems::BasicVector<double>(num_outputs_))
            .get_index();
  }

  /// Gets the output port for the generalized effort output.
  const drake::systems::OutputPort<T>& generalized_effort_output_port() const {
    return drake::systems::System<T>::get_output_port(generalized_effort_output_port_index_);
  }

  const drake::systems::InputPort<T>& controller_effort_input_port(const std::string& behavior_name) const {
    return drake::systems::System<T>::get_input_port(controller_effort_input_port_index_.at(behavior_name));
  }

  const drake::systems::InputPort<T>& current_q_input_port() const {
    return drake::systems::System<T>::get_input_port(current_q_input_port_index_);
  }

  const drake::systems::InputPort<T>& current_v_input_port() const {
    return drake::systems::System<T>::get_input_port(current_v_input_port_index_);
  }

  /// Use the state machine to determine which behavior to poll for effort command.
  void CalcControlOutput(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const {
    const std::string& behavior_name = this->GetNodeNameByIndex(this->current_state(context));
    drake::VectorX<double> effort = this->controller_effort_input_port(behavior_name).Eval(context);
    output->SetFromVector(effort);
    // Get q and v inputs.
    drake::VectorX<double> q_current = this->current_q_input_port().Eval(context);
    drake::VectorX<double> v_current = this->current_v_input_port().Eval(context);
    drake::log()->trace("q=[{}], v=[{}], u=[{}]", q_current.transpose(), v_current.transpose(), effort.transpose());
  }

  /**
   A function whose signature can be bound to a `StateMachine::EdgeFunction`. Returns a Boolean indicating whether a
   state transition is valid.
   @param q_target the target joint positions of this check.
   @param v_target the target joint velocities of this check.
   @param q_tolerance max norm distance error between the vector from evaluating `current_q_input_port()` and
          `q_target`.
   @param v_tolerance max norm distance error between the vector from evaluating `current_v_input_port()` and
          `v_target`.
   @param context a const reference to the state machine context, used to evaluate input ports of this class.
   @return true the evaluated state from the input ports is within the specified tolerances from the
           target state (`q_target` and `v_target`).
   @return false the evaluated state from the input ports is NOT within the specified tolerances from the
           target state (`q_target` and `v_target`).
   */
  bool IsNearState(const drake::VectorX<double>& q_target, const drake::VectorX<double>& v_target, double q_tolerance,
                   double v_tolerance, const drake::systems::Context<T>& context) const {
    // Get q and v inputs.
    drake::VectorX<double> q_current = this->current_q_input_port().Eval(context);
    drake::VectorX<double> v_current = this->current_v_input_port().Eval(context);
    // Calculate errors.
    drake::VectorX<double> v_error = v_current - v_target;
    drake::VectorX<double> q_error = q_current - q_target;

    // Check constraints.
    bool in_v_constraint = v_error.norm() < v_tolerance;
    bool in_q_constraint = q_error.norm() < q_tolerance;

    bool result = in_v_constraint && in_q_constraint;

    return result;
  }

  /// See documentation for `IsNearState(.)`.
  /// @returns The negation of `IsNearState(.)`.
  bool IsNotNearState(const drake::VectorX<double>& q_target, const drake::VectorX<double>& v_target,
                      double q_tolerance, double v_tolerance, const drake::systems::Context<T>& context) {
    return !IsNearState(q_target, v_target, q_tolerance, v_tolerance, context);
  }

 private:
  int num_outputs_{0};
  drake::systems::InputPortIndex current_q_input_port_index_{};
  drake::systems::InputPortIndex current_v_input_port_index_{};

  std::map<std::string, drake::systems::InputPortIndex> controller_effort_input_port_index_;

  drake::systems::OutputPortIndex generalized_effort_output_port_index_{};
};

// An empty plan that is always within its valid time interval.
// The plan returns an empty `drake::VectorX<double>` when evaluated.
// TODO(drum): Remove the empty plan when PrimitiveBehavior has been updated to accommodate fully reactive behaviors.
class EmptyPlan : public Plan<double> {
 public:
  EmptyPlan() = default;
  const double& start_time() const final { return start_time_; }
  const double& end_time() const final { return end_time_; }
  drake::VectorX<double> Evaluate(const double&) const final { return drake::VectorX<double>{}; }

 private:
  std::unique_ptr<Plan<double>> DoClone() const final { return std::make_unique<EmptyPlan>(); }

  double start_time_{0.0};
  double end_time_{std::numeric_limits<double>::max()};
};

// A derived `PrimitiveBehavior` that implements a PID controller and ignores its `Plan`.
class PIDBehavior : public PrimitiveBehavior<double> {
 public:
  PIDBehavior(const drake::multibody::MultibodyPlant<double>* plant, double freq, const drake::VectorX<double>& kp,
              const drake::VectorX<double>& ki, const drake::VectorX<double>& kd)
      : PrimitiveBehavior(plant, freq) {
    DR_DEMAND(this->robot_plant().num_positions() == kp.size());
    DR_DEMAND(this->robot_plant().num_actuators() == kp.size());

    pid_controller_ = std::make_unique<drake::systems::controllers::PidController<double>>(kp, ki, kd);
    pid_controller_context_ = pid_controller_->CreateDefaultContext();

    // Declare some needed inputs.
    current_state_input_port_index_ =
        this->DeclareVectorInputPort(
                "current_state", drake::systems::BasicVector<double>(plant->num_positions() + plant->num_velocities()))
            .get_index();
    desired_state_input_port_index_ =
        this->DeclareVectorInputPort(
                "desired_state", drake::systems::BasicVector<double>(plant->num_positions() + plant->num_velocities()))
            .get_index();
  }

  const drake::systems::InputPort<double>& current_state_input_port() const {
    return drake::systems::System<double>::get_input_port(current_state_input_port_index_);
  }
  const drake::systems::InputPort<double>& desired_state_input_port() const {
    return drake::systems::System<double>::get_input_port(desired_state_input_port_index_);
  }

 private:
  std::unique_ptr<Plan<double>> DoComputePlan(const drake::systems::Context<double>&) const final {
    // Return the empty plan.
    return std::make_unique<EmptyPlan>();
  }

  void DoCalcControlOutput(const drake::systems::Context<double>& context,
                           drake::systems::BasicVector<double>* output) const final {
    DR_DEMAND(this->robot_plant().num_actuators() == output->size());
    // Pass through inputs and outputs from PID controller.
    pid_controller_context_->FixInputPort(pid_controller_->get_input_port_desired_state().get_index(),
                                          this->desired_state_input_port().Eval(context));
    pid_controller_context_->FixInputPort(pid_controller_->get_input_port_estimated_state().get_index(),
                                          this->current_state_input_port().Eval(context));
    output->SetFromVector(pid_controller_->get_output_port_control().Eval(*pid_controller_context_));
  }

  drake::systems::InputPortIndex current_state_input_port_index_{};
  drake::systems::InputPortIndex desired_state_input_port_index_{};

  std::unique_ptr<drake::systems::controllers::PidController<double>> pid_controller_;
  std::unique_ptr<drake::systems::Context<double>> pid_controller_context_;
};

class StateMachineControllerTest : public ::testing::Test {
 public:
  void SetPendulumVelocity(double qdot) const {
    drake::VectorX<double> qdot_vec(1);
    qdot_vec << qdot;
    pendulum_->SetVelocities(pendulum_context_, qdot_vec);
  }

  void SetPendulumPosition(double q) const {
    drake::VectorX<double> q_vec(1);
    q_vec << q;
    pendulum_->SetPositions(pendulum_context_, q_vec);
  }

  drake::multibody::MultibodyPlant<double>& pendulum_plant() { return *pendulum_; }
  drake::systems::Simulator<double>& simulator() { return *simulator_; }
  SwingUpBehaviorScheduler<double>& state_machine() { return *state_machine_; }
  drake::systems::Context<double>& state_machine_context() { return *state_machine_context_; }
  drake::systems::Context<double>& pendulum_context() { return *pendulum_context_; }

 private:
  void SetUp() {
    drake::systems::DiagramBuilder<double> builder;

    scene_graph_ = builder.template AddSystem<drake::geometry::SceneGraph<double>>();

    // Construct the pendulum.
    PendulumParameters pendulum_parameters;
    pendulum_ = builder.AddSystem(MakePendulumPlant(pendulum_parameters, scene_graph_));

    state_machine_ = builder.template AddSystem<SwingUpBehaviorScheduler<double>>(kUpdateFreq, *pendulum_);

    state_machine_->AddNode("add-energy");
    state_machine_->AddNode("stabilize");
    state_machine_->AddNode("remove-energy");

    const drake::Vector1<double> top{M_PI};
    const drake::Vector1<double> bottom{0.0};
    const drake::Vector1<double> stopped{0.0};

    using std::placeholders::_1;

    // Try to stabilize pendulum at the top once swing-up adds enough energy.
    state_machine_->AddEdge("add-energy", "stabilize",
                            std::bind(&SwingUpBehaviorScheduler<double>::IsNearState, state_machine_, top, stopped,
                                      kStabilizationTolerance, std::numeric_limits<double>::max(), _1));

    // Otherwise stay in this state.
    state_machine_->AddEdge("add-energy", "add-energy",
                            std::bind(&SwingUpBehaviorScheduler<double>::IsNotNearState, state_machine_, top, stopped,
                                      kStabilizationTolerance, std::numeric_limits<double>::max(), _1));

    // Once stabilized, drop pendulum.  damp pendulum to remove energy from system.
    state_machine_->AddEdge("stabilize", "remove-energy",
                            std::bind(&SwingUpBehaviorScheduler<double>::IsNearState, state_machine_, top, stopped,
                                      kPosTolerance, kVelTolerance, _1));

    // Otherwise stay in this state.
    state_machine_->AddEdge("stabilize", "stabilize",
                            std::bind(&SwingUpBehaviorScheduler<double>::IsNotNearState, state_machine_, top, stopped,
                                      kPosTolerance, kVelTolerance, _1));

    // Once settled at bottom, start swing-up.
    state_machine_->AddEdge("remove-energy", "add-energy",
                            std::bind(&SwingUpBehaviorScheduler<double>::IsNearState, state_machine_, bottom, stopped,
                                      kPosTolerance, kVelTolerance, _1));

    // Otherwise stay in this state.
    state_machine_->AddEdge("remove-energy", "remove-energy",
                            std::bind(&SwingUpBehaviorScheduler<double>::IsNotNearState, state_machine_, bottom,
                                      stopped, kPosTolerance, kVelTolerance, _1));

    // Start at swing-up.
    state_machine_->set_start_state("add-energy");

    // Now the state machine graph is complete.
    state_machine_->Finalize();

    state_machine_->AddControllerEffortInput("add-energy");
    state_machine_->AddControllerEffortInput("stabilize");
    state_machine_->AddControllerEffortInput("remove-energy");

    // Create necessary controllers.
    DR_DEMAND(kDrivingGain < 0.0);
    DR_DEMAND(kDampingGain > 0.0);
    DR_DEMAND(kPIDKpGain > 0.0);
    DR_DEMAND(kPIDKdGain > 0.0);

    const auto* add_energy_controller = builder.AddSystem<PIDBehavior>(
        pendulum_, kUpdateFreq, drake::Vector1<double>{0.0} /* Kp */, drake::Vector1<double>{0.0} /* Ki */,
        drake::Vector1<double>{kDrivingGain} /* Kd */);

    const auto* remove_energy_controller = builder.AddSystem<PIDBehavior>(
        pendulum_, kUpdateFreq, drake::Vector1<double>{0.0} /* Kp */, drake::Vector1<double>{0.0} /* Ki */,
        drake::Vector1<double>{kDampingGain} /* Kd */);

    const auto* stabilize_controller = builder.AddSystem<PIDBehavior>(
        pendulum_, kUpdateFreq, drake::Vector1<double>{kPIDKpGain} /* Kp */, drake::Vector1<double>{0.0} /* Ki */,
        drake::Vector1<double>{kPIDKdGain} /* Kd */);

    // Connect the behavior effort outputs to the state_machine effort inputs.
    builder.Connect(add_energy_controller->generalized_effort_output_port(),
                    state_machine_->controller_effort_input_port("add-energy"));
    builder.Connect(stabilize_controller->generalized_effort_output_port(),
                    state_machine_->controller_effort_input_port("stabilize"));
    builder.Connect(remove_energy_controller->generalized_effort_output_port(),
                    state_machine_->controller_effort_input_port("remove-energy"));

    // Connect state inputs.
    const drake::systems::Demultiplexer<double>* demuxer =
        builder.template AddSystem<drake::systems::Demultiplexer<double>>(2 /* size */, 1 /* output ports size */);
    builder.Connect(pendulum_->get_state_output_port(), demuxer->get_input_port(0));

    // Wrap inputs to stabilizer controller.
    auto* wrap_to_ = builder.template AddSystem<drake::systems::WrapToSystem<double>>(pendulum_->num_positions());
    for (int i = 0; i < pendulum_->num_positions(); ++i) {
      wrap_to_->set_interval(i, 0, 2.0 * M_PI);
    }
    builder.Connect(demuxer->get_output_port(0), wrap_to_->get_input_port(0));

    // Re-mux the state vector with the wrapped position.
    const auto* muxer = builder.template AddSystem<drake::systems::Multiplexer<double>>(
        std::vector<int>{pendulum_->num_positions(), pendulum_->num_velocities()});
    builder.Connect(wrap_to_->get_output_port(0), muxer->get_input_port(0));
    builder.Connect(demuxer->get_output_port(1), muxer->get_input_port(1));

    // Connect state vector to controllers.
    builder.Connect(muxer->get_output_port(0), add_energy_controller->current_state_input_port());
    builder.Connect(muxer->get_output_port(0), stabilize_controller->current_state_input_port());
    builder.Connect(muxer->get_output_port(0), remove_energy_controller->current_state_input_port());

    // Connect state vector to state machine (q and v on separate ports).
    builder.Connect(wrap_to_->get_output_port(0), state_machine_->current_q_input_port());
    builder.Connect(demuxer->get_output_port(1), state_machine_->current_v_input_port());

    // Connect state machine effort output to torque limiter.
    const drake::systems::Saturation<double>* torque_limiter_ =
        builder.template AddSystem<drake::systems::Saturation<double>>(drake::Vector1<double>{-kMaxTorque},
                                                                       drake::Vector1<double>{kMaxTorque});
    builder.Connect(state_machine_->generalized_effort_output_port(), torque_limiter_->get_input_port());

    // Connect the torque limiter output to the plant input.
    builder.Connect(torque_limiter_->get_output_port(), pendulum_->get_actuation_input_port());

    // Construct the diagram.
    diagram_ = builder.Build();

    // Create the context.
    std::unique_ptr<drake::systems::Context<double>> context = diagram_->CreateDefaultContext();

    // Get relevant contexts.
    pendulum_context_ = &diagram_->GetMutableSubsystemContext(*pendulum_, context.get());
    state_machine_context_ = &diagram_->GetMutableSubsystemContext(*state_machine_, context.get());

    // Setup stabilization controller.
    drake::Vector2<double> upright_state{M_PI, 0.0};

    // Fix desired upright position to input desired state port of stabilization controller.
    diagram_->GetMutableSubsystemContext(*stabilize_controller, context.get())
        .FixInputPort(stabilize_controller->desired_state_input_port().get_index(), upright_state);

    // Fix damping and driving controllers at desired zero state.
    diagram_->GetMutableSubsystemContext(*add_energy_controller, context.get())
        .FixInputPort(add_energy_controller->desired_state_input_port().get_index(), drake::Vector2<double>::Zero());

    diagram_->GetMutableSubsystemContext(*remove_energy_controller, context.get())
        .FixInputPort(remove_energy_controller->desired_state_input_port().get_index(), drake::Vector2<double>::Zero());

    // Activate all behaviors.
    diagram_->GetMutableSubsystemContext(*stabilize_controller, context.get())
        .FixInputPort(stabilize_controller->operation_signal_input_port().get_index(),
                      drake::AbstractValue::Make(PrimitiveBehavior<double>::OperationSignalType::kActive));

    diagram_->GetMutableSubsystemContext(*add_energy_controller, context.get())
        .FixInputPort(add_energy_controller->operation_signal_input_port().get_index(),
                      drake::AbstractValue::Make(PrimitiveBehavior<double>::OperationSignalType::kActive));

    diagram_->GetMutableSubsystemContext(*remove_energy_controller, context.get())
        .FixInputPort(remove_energy_controller->operation_signal_input_port().get_index(),
                      drake::AbstractValue::Make(PrimitiveBehavior<double>::OperationSignalType::kActive));

    // make simulator.
    simulator_ = std::make_unique<drake::systems::Simulator<double>>(*diagram_, std::move(context));
  }

  drake::multibody::MultibodyPlant<double>* pendulum_{nullptr};
  SwingUpBehaviorScheduler<double>* state_machine_{nullptr};
  drake::systems::Context<double>* pendulum_context_{nullptr};
  drake::systems::Context<double>* state_machine_context_{nullptr};
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::systems::Simulator<double>> simulator_;
  drake::geometry::SceneGraph<double>* scene_graph_;
};

// Verifies that the state machine works on a robotics example.
TEST_F(StateMachineControllerTest, Pendulum) {
  // Enable state machine.
  this->state_machine_context().FixInputPort(this->state_machine().operation_signal_input_port().get_index(),
                                             drake::AbstractValue::Make(SwingUpBehaviorScheduler<double>::kActive));

  ASSERT_NO_THROW(this->simulator().Initialize());

  // Test whether swing-up visits all nodes.
  // Set non-zero initial velocity.
  this->SetPendulumVelocity(0.1);

  // Set initial position at bottom.
  this->SetPendulumPosition(0.0);

  // Calc a step that will include at least one state machine update.
  const double kDTEpsilon = 1e-7;
  const double kAtLeastOneUpdateDT = kDTEpsilon + 1.0 / kUpdateFreq;

  // Record all nodes visited during simulation of swing-up, starting at current node.
  std::vector<StateMachineNodeIndex> visited_swing_up{
      this->state_machine().current_state(this->state_machine_context())};

  // The expected order of nodes visited.
  const std::vector<std::string> visited_node_names_swing_up{"add-energy", "stabilize", "remove-energy", "add-energy"};

  // Simulate for duration required to complete swing-up then swing-down.
  do {
    drake::log()->trace(
        "time={}, state={}", this->simulator().get_context().get_time(),
        this->state_machine().GetNodeNameByIndex(this->state_machine().current_state(this->state_machine_context())));

    // Advance simulation to fit in at least one step.
    ASSERT_NO_THROW(this->simulator().AdvanceTo(this->simulator().get_context().get_time() + kAtLeastOneUpdateDT));

    // Record nodes visited this update.
    const std::vector<StateMachineNodeIndex>& nodes_visited =
        this->state_machine().states_visited_last_update(this->state_machine_context());

    // There should be no multiple-transition updates for this particular test.
    ASSERT_LE(nodes_visited.size(), 2);

    // Record node_index with other visited node indices if it is different from the last recorded node index.
    StateMachineNodeIndex current_state = this->state_machine().current_state(this->state_machine_context());
    if (current_state != visited_swing_up.back()) visited_swing_up.push_back(current_state);

    // Maximum time required to simulate this task.
    ASSERT_LT(this->simulator().get_context().get_time(), kMaxSimulationTime);

    // Terminate loop only after recording the results from the last update arriving back at state "add-energy".
  } while (visited_swing_up.size() < visited_node_names_swing_up.size() ||
           this->state_machine().current_state(this->state_machine_context()) !=
               this->state_machine().GetNodeIndexByName("add-energy"));

  // Check whether all nodes visited match expectation.
  ASSERT_EQ(visited_swing_up.size(), visited_node_names_swing_up.size());
  for (int i = 0; i < static_cast<int>(visited_swing_up.size()); ++i) {
    ASSERT_EQ(this->state_machine().GetNodeIndexByName(visited_node_names_swing_up[i]), visited_swing_up[i]);
  }

  // Current node after reaching top is now "remove-energy".
  ASSERT_EQ(this->state_machine().current_state(this->state_machine_context()),
            this->state_machine().GetNodeIndexByName("add-energy"));

  // Disable state machine.
  this->state_machine_context().FixInputPort(this->state_machine().operation_signal_input_port().get_index(),
                                             drake::AbstractValue::Make(SwingUpBehaviorScheduler<double>::kInactive));

  // Test whether the state machine traverses multiple nodes quickly when already at goal for starting node.
  // Reset state to start state "add-energy".
  this->state_machine().ResetToStartState(&this->state_machine_context().get_mutable_state());

  // Set zero initial velocity.
  this->SetPendulumVelocity(0.0);

  // Set initial position at top.
  this->SetPendulumPosition(M_PI);

  // Advance simulation to fit in at least one step.
  ASSERT_NO_THROW(this->simulator().AdvanceTo(this->simulator().get_context().get_time() + kAtLeastOneUpdateDT));

  // The state machine is inactive so no nodes have been visited.
  ASSERT_TRUE(this->state_machine().states_visited_last_update(this->state_machine_context()).empty());

  // The state machine is inactive so the curent state is still at "add-energy"
  ASSERT_EQ(this->state_machine().current_state(this->state_machine_context()),
            this->state_machine().GetNodeIndexByName("add-energy"));

  // Enable state machine.
  this->state_machine_context().FixInputPort(this->state_machine().operation_signal_input_port().get_index(),
                                             drake::AbstractValue::Make(SwingUpBehaviorScheduler<double>::kActive));

  // Advance simulation to fit in at least one step.
  ASSERT_NO_THROW(this->simulator().AdvanceTo(this->simulator().get_context().get_time() + kAtLeastOneUpdateDT));

  // Check whether all nodes visited match expectation.
  const std::vector<std::string> visited_node_names_at_top{"add-energy", "stabilize", "remove-energy"};

  const std::vector<StateMachineNodeIndex>& visited_at_top =
      this->state_machine().states_visited_last_update(this->state_machine_context());
  ASSERT_EQ(visited_at_top.size(), visited_node_names_at_top.size());
  for (int i = 0; i < static_cast<int>(visited_node_names_at_top.size()); ++i) {
    ASSERT_EQ(this->state_machine().GetNodeIndexByName(visited_node_names_at_top[i]), visited_at_top[i]);
  }

  // Current node after reaching top is now "remove-energy".
  ASSERT_EQ(this->state_machine().current_state(this->state_machine_context()),
            this->state_machine().GetNodeIndexByName("remove-energy"));
}

}  // namespace
}  // namespace DR
