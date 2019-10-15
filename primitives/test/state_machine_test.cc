#include <DR/primitives/state_machine.h>

#include <gtest/gtest.h>

#include <drake/math/rigid_transform.h>
#include <drake/systems/analysis/simulator.h>

namespace DR {
namespace {

template <typename T>
class PoseTrackingStateMachine : public StateMachine<T> {
 public:
  PoseTrackingStateMachine() = delete;
  explicit PoseTrackingStateMachine(double update_rate) : StateMachine<T>(update_rate) {
    pose_input_port_index_ =
        this->template DeclareAbstractInputPort("current_pose", drake::Value<drake::math::RigidTransform<T>>())
            .get_index();
  }
  ~PoseTrackingStateMachine() {}

  /**
   A function whose signature can be bound to a `StateMachine::EdgeFunction`.  Returns a boolean indicating whether a
   state transition is valid.
   @param target_pose the target pose of the check, distance from this pose is compared to the linear and angular
          tolerances.
   @param linear_tolerance the Euclidian distance tolerance for translation error.
   @param angular_tolerance angular tolerance in radians for rotation error.
   @param context a const reference to the state machine context, used to evaluate input ports of this class.
   @return true the pose from evaluating `pose_input_port()` is within the specified tolerances from the
          `target_pose`.
   @return false the pose from evaluating `pose_input_port()` is NOT within the specified tolerances from the
          `target_pose`.
   */
  bool IsNearPose(const drake::math::RigidTransform<T>& target_pose, double linear_tolerance, double angular_tolerance,
                  const drake::systems::Context<T>& context) const {
    const drake::math::RigidTransform<T>& current_pose =
        pose_input_port().template Eval<drake::math::RigidTransform<T>>(context);

    bool in_angular = current_pose.rotation().IsNearlyEqualTo(target_pose.rotation(), angular_tolerance);
    bool in_linear = (current_pose.translation() - target_pose.translation()).norm() < linear_tolerance;

    bool result = in_angular && in_linear;

    return result;
  }

  /// See documentation for `IsNearPose(.)`.
  /// @returns The negation of `IsNearPose(.)`.
  bool IsNotNearPose(const drake::math::RigidTransform<T>& target_pose, double linear_tolerance,
                     double angular_tolerance, const drake::systems::Context<T>& context) {
    return !IsNearPose(target_pose, linear_tolerance, angular_tolerance, context);
  }

  const drake::systems::InputPort<T>& pose_input_port() const {
    return drake::systems::System<T>::get_input_port(pose_input_port_index_);
  }

 private:
  drake::systems::InputPortIndex pose_input_port_index_;
};

/*
  This class provides an example system whose output is provided as input to the state machine class defined above.  The
  provided input is evaluated by the state machine's EdgeFunctions to determine whether certain state transitions should
  be made.

  The internal state of this system (the `pose_`) would normally be derived from simulation.  For example this class
  could be the interface of an object tracking algorithm that processes camera data to determine the pose of a
  manipuland.  In this test case we are setting `pose_` explicitly.
*/
template <typename T>
class PoseSource : public drake::systems::LeafSystem<T> {
 public:
  PoseSource() {
    // Declare the generalized effort output port.
    pose_output_port_index_ =
        this->DeclareAbstractOutputPort("pose", drake::math::RigidTransform<T>(), &PoseSource::CalcPoseOutput)
            .get_index();
  }
  ~PoseSource() {}

  const drake::systems::OutputPort<T>& pose_output_port() const {
    return drake::systems::System<T>::get_output_port(pose_output_port_index_);
  }

  void CalcPoseOutput(const drake::systems::Context<T>& context, drake::math::RigidTransform<T>* output) const {
    (void)context;
    *output = pose();
  }

  const drake::math::RigidTransform<T>& pose() const { return pose_; }

  void set_pose(const drake::math::RigidTransform<T>& pose) { pose_ = pose; }

 private:
  drake::math::RigidTransform<T> pose_;

  drake::systems::OutputPortIndex pose_output_port_index_;
};

// Update rate.
const double kUpdateFreq = 1000.0;

// Run the state machine for a bit.
const double kMaxTime = 10.0;

class StateMachineTest : public ::testing::Test {
 public:
  drake::systems::DiagramBuilder<double>& builder() { return builder_; }

 private:
  void SetUp() override {}

  // Example Pong state machine modeled after a game of ping-pong.
  drake::systems::DiagramBuilder<double> builder_;
};
}  // namespace

// Verifies that the state machine works on a simple example.
TEST_F(StateMachineTest, StateTransitions) {
  PoseTrackingStateMachine<double>* state_machine =
      builder().template AddSystem<PoseTrackingStateMachine<double>>(kUpdateFreq);
  state_machine->set_name("PongGame");

  PoseSource<double>* ball = builder().template AddSystem<PoseSource<double>>();
  builder().Connect(ball->pose_output_port(), state_machine->pose_input_port());

  // Left's ball (ball moving from left to right).
  StateMachineNodeIndex left_node = state_machine->AddNode("left-ball");
  // Right's ball (ball moving from right to left).
  StateMachineNodeIndex right_node = state_machine->AddNode("right-ball");
  // Game starts in state "left-ball".
  state_machine->set_start_state("left-ball");

  // Set transitions.
  // Poses that trigger exit transition.
  drake::math::RigidTransform<double> left_side(drake::Quaternion<double>::Identity(),
                                                drake::Vector3<double>{0.0, 1.0, 0.0});
  drake::math::RigidTransform<double> right_side(drake::Quaternion<double>::Identity(),
                                                 drake::Vector3<double>{0.0, -1.0, 0.0});
  // No angular constraint, position only.
  const double kAngularTolerance = std::numeric_limits<double>::max();
  // Position tolerance's only constraint will be that it does not make the goal regions intersect-- overlapping exit
  // condtions could lead to an infinite loop in the state machine (only caught at runtime).
  const double kLinearTolerance = 0.5;
  // Checking intersecting exit states is only trivial in this case.
  DR_DEMAND(kLinearTolerance < (left_side.translation() - right_side.translation()).norm());

  using std::placeholders::_1;
  // Left hits ball.
  state_machine->AddEdge("left-ball", "right-ball",
                         std::bind(&PoseTrackingStateMachine<double>::IsNearPose, state_machine, right_side,
                                   kLinearTolerance, kAngularTolerance, _1));
  // Right hits ball.
  state_machine->AddEdge("right-ball", "left-ball",
                         std::bind(&PoseTrackingStateMachine<double>::IsNearPose, state_machine, left_side,
                                   kLinearTolerance, kAngularTolerance, _1));
  // Add self-directed edges.
  // Left does not hit ball.
  state_machine->AddEdge("left-ball", "left-ball",
                         std::bind(&PoseTrackingStateMachine<double>::IsNotNearPose, state_machine, right_side,
                                   kLinearTolerance, kAngularTolerance, _1));
  // Right does not hit ball.
  state_machine->AddEdge("right-ball", "right-ball",
                         std::bind(&PoseTrackingStateMachine<double>::IsNotNearPose, state_machine, left_side,
                                   kLinearTolerance, kAngularTolerance, _1));

  // Finish setting-up state machine.
  state_machine->Finalize();

  // Make simulation.
  std::unique_ptr<drake::systems::Diagram<double>> diagram = builder().Build();
  std::unique_ptr<drake::systems::Context<double>> context = diagram->CreateDefaultContext();
  diagram->SetDefaultContext(context.get());
  std::unique_ptr<drake::systems::Simulator<double>> simulator =
      std::make_unique<drake::systems::Simulator<double>>(*diagram.get(), std::move(context));
  simulator->Initialize();

  // Left starts, ball initial pose is at left pose.
  ball->set_pose(left_side);

  drake::systems::Context<double>& fsm_context =
      diagram->GetMutableSubsystemContext(*state_machine, &simulator->get_mutable_context());

  // Enable state machine.
  fsm_context.FixInputPort(state_machine->operation_signal_input_port().get_index(),
                           drake::AbstractValue::Make(PoseTrackingStateMachine<double>::kActive));

  // State machine should be at node left_node.
  ASSERT_NO_THROW(simulator->AdvanceTo(simulator->get_context().get_time() + 1.0));
  ASSERT_EQ(state_machine->current_state(fsm_context), left_node);

  // Disable state machine.
  fsm_context.FixInputPort(state_machine->operation_signal_input_port().get_index(),
                           drake::AbstractValue::Make(PoseTrackingStateMachine<double>::kInactive));

  // Change ball position to Exit State of node left_node.
  ball->set_pose(right_side);

  // State machine should NOT exit left_node because it is inactive.
  ASSERT_NO_THROW(simulator->AdvanceTo(simulator->get_context().get_time() + 1.0));
  ASSERT_EQ(state_machine->current_state(fsm_context), left_node);

  // Enable state machine.
  fsm_context.FixInputPort(state_machine->operation_signal_input_port().get_index(),
                           drake::AbstractValue::Make(PoseTrackingStateMachine<double>::kActive));

  // State machine should exit left_node and enter state right_node.
  ASSERT_NO_THROW(simulator->AdvanceTo(simulator->get_context().get_time() + 1.0));
  ASSERT_EQ(state_machine->current_state(fsm_context), right_node);

  // Set ball pose to some distant pose.
  ball->set_pose(drake::math::RigidTransform<double>(drake::Quaternion<double>::Identity(),
                                                     drake::Vector3<double>{100.0, 100.0, 100.0}));
  // State machine should stay in last active state: right_node.
  ASSERT_NO_THROW(simulator->AdvanceTo(simulator->get_context().get_time() + 1.0));
  ASSERT_EQ(state_machine->current_state(fsm_context), right_node);

  // Change ball position to Exit State of node right_node.
  ball->set_pose(left_side);
  // State machine should exit right_node and enter state left_node.
  ASSERT_NO_THROW(simulator->AdvanceTo(simulator->get_context().get_time() + 1.0));
  ASSERT_EQ(state_machine->current_state(fsm_context), left_node);

  // Calc a step that will include at least one state machine update.
  const double kDTEpsilon = 1e-7;
  const double kAtLeastOneUpdateDT = kDTEpsilon + 1.0 / state_machine->update_freq();

  StateMachineNodeIndex previous_node_index;
  while (simulator->get_context().get_time() < kMaxTime) {
    ASSERT_NO_THROW(simulator->AdvanceTo(simulator->get_context().get_time() + kAtLeastOneUpdateDT));
    StateMachineNodeIndex index = state_machine->current_state(fsm_context);
    // The state machine should never stay in the same state.
    if (previous_node_index.is_valid()) {
      ASSERT_NE(previous_node_index, index);
    }
    if (index == left_node) {
      ball->set_pose(right_side);
    } else if (index == right_node) {
      ball->set_pose(left_side);
    } else {
      FAIL() << "state machine entered an invalid state.";
    }
    previous_node_index = index;
  }
}

// Tests whether the state machine will throw and error in circumstances where it cycles back to the same
// state in one update. This behavior signifies that the state machine's internal state is ambiguous because multiple
// states are valid at the same time.
TEST_F(StateMachineTest, InfiniteLoop) {
  PoseTrackingStateMachine<double>* state_machine =
      builder().template AddSystem<PoseTrackingStateMachine<double>>(kUpdateFreq);
  state_machine->set_name("PongGame");

  PoseSource<double>* ball = builder().template AddSystem<PoseSource<double>>();
  builder().Connect(ball->pose_output_port(), state_machine->pose_input_port());

  // Left's ball (ball moving from left to right)
  state_machine->AddNode("left-ball");
  // Right's ball (ball moving from right to left)
  state_machine->AddNode("right-ball");
  // Game starts in state left_node.
  state_machine->set_start_state("left-ball");

  // Set transitions.
  // Poses that trigger exit transition.
  const drake::math::RigidTransform<double> left_side(drake::Quaternion<double>::Identity(),
                                                      drake::Vector3<double>{0.0, 1.0, 0.0});
  const drake::math::RigidTransform<double> right_side(drake::Quaternion<double>::Identity(),
                                                       drake::Vector3<double>{0.0, -1.0, 0.0});

  // No angular constraint, position only.
  const double kAngularTolerance = std::numeric_limits<double>::max();

  // Position tolerance's only constraint will be that it does not make the goal regions intersect-- overlapping exit
  // condtions could lead to an infinite loop in the state machine (only caught at runtime).
  // Make the exit states intersect to start an infinite loop.
  const double kLinearTolerance = std::numeric_limits<double>::max();

  using std::placeholders::_1;
  // Left hits ball.
  state_machine->AddEdge("left-ball", "right-ball",
                         std::bind(&PoseTrackingStateMachine<double>::IsNearPose, state_machine, right_side,
                                   kLinearTolerance, kAngularTolerance, _1));
  // Right hits ball.
  state_machine->AddEdge("right-ball", "left-ball",
                         std::bind(&PoseTrackingStateMachine<double>::IsNearPose, state_machine, left_side,
                                   kLinearTolerance, kAngularTolerance, _1));

  // Add self-directed edges.
  // Left does not hit ball.
  state_machine->AddEdge("left-ball", "left-ball",
                         std::bind(&PoseTrackingStateMachine<double>::IsNotNearPose, state_machine, right_side,
                                   kLinearTolerance, kAngularTolerance, _1));
  // Right does not hit ball.
  state_machine->AddEdge("right-ball", "right-ball",
                         std::bind(&PoseTrackingStateMachine<double>::IsNotNearPose, state_machine, left_side,
                                   kLinearTolerance, kAngularTolerance, _1));

  // Finish setting-up state_machine.
  state_machine->Finalize();

  // Make simulation.
  std::unique_ptr<drake::systems::Diagram<double>> diagram = builder().Build();
  std::unique_ptr<drake::systems::Context<double>> context = diagram->CreateDefaultContext();
  diagram->SetDefaultContext(context.get());
  std::unique_ptr<drake::systems::Simulator<double>> simulator =
      std::make_unique<drake::systems::Simulator<double>>(*diagram.get(), std::move(context));
  simulator->Initialize();

  // Left starts, ball initial pose is at left pose.
  ball->set_pose(left_side);

  drake::systems::Context<double>& fsm_context =
      diagram->GetMutableSubsystemContext(*state_machine, &simulator->get_mutable_context());

  // Enable state machine.
  fsm_context.FixInputPort(state_machine->operation_signal_input_port().get_index(),
                           drake::AbstractValue::Make(PoseTrackingStateMachine<double>::kActive));
  // State machine will enter the same state twice in one update and thrown an error.
  ASSERT_THROW(simulator->AdvanceTo(simulator->get_context().get_time() + 1.0), std::runtime_error);
}

// Triggers and catches several errors resulting from misuse of the state machine class.
TEST_F(StateMachineTest, ErrorStates) {
  PoseTrackingStateMachine<double>* state_machine =
      builder().template AddSystem<PoseTrackingStateMachine<double>>(kUpdateFreq);
  state_machine->set_name("PongGame");

  PoseSource<double>* ball = builder().template AddSystem<PoseSource<double>>();
  builder().Connect(ball->pose_output_port(), state_machine->pose_input_port());

  // Abort: no nodes (requires >= 2).
  EXPECT_DEATH({ state_machine->Finalize(); }, ".*condition 'num_nodes\\(\\) >= 2' failed.*");

  // Left's ball (ball moving from left to right)
  StateMachineNodeIndex left_node = state_machine->AddNode("left-ball");

  // Abort: 1 node (requires >= 2).
  EXPECT_DEATH({ state_machine->Finalize(); }, ".*condition 'num_nodes\\(\\) >= 2' failed.*");

  // Right's ball (ball moving from right to left)
  StateMachineNodeIndex right_node = state_machine->AddNode("right-ball");

  // Abort: add a node of an existing name.
  EXPECT_DEATH({ state_machine->AddNode("right-ball"); }, ".*condition '!HasNode\\(name\\)' failed.*");

  // Abort: no start state.
  EXPECT_DEATH({ state_machine->Finalize(); }, ".*condition 'start_state_\\.is_valid\\(\\)' failed.*");

  // Game starts in state "left-ball".
  state_machine->set_start_state("left-ball");

  // Set transitions.
  // Poses that trigger exit transition.
  drake::math::RigidTransform<double> left_side(drake::Quaternion<double>::Identity(),
                                                drake::Vector3<double>{0.0, 1.0, 0.0});
  drake::math::RigidTransform<double> right_side(drake::Quaternion<double>::Identity(),
                                                 drake::Vector3<double>{0.0, -1.0, 0.0});
  // No angular constraint, position only.
  const double kAngularTolerance = std::numeric_limits<double>::max();
  // Position tolerance's only constraint will be that it does not make the goal regions intersect-- overlapping exit
  // condtions could lead to an infinite loop in the state machine (only caught at runtime).
  const double kLinearTolerance = 0.5;
  // Checking intersecting exit states is only trivial in this case.
  DR_DEMAND(kLinearTolerance < (left_side.translation() - right_side.translation()).norm());

  using std::placeholders::_1;
  // Left hits ball.
  state_machine->AddEdge("left-ball", "right-ball",
                         std::bind(&PoseTrackingStateMachine<double>::IsNearPose, state_machine, right_side,
                                   kLinearTolerance, kAngularTolerance, _1));
  // Right does not hit ball.

  // Abort: Unknown State
  EXPECT_DEATH(
      {
        state_machine->AddEdge("right-ball", "unknown-ball",
                               std::bind(&PoseTrackingStateMachine<double>::IsNearPose, state_machine, left_side,
                                         kLinearTolerance, kAngularTolerance, _1));
      },
      ".*condition 'HasNode\\(name\\)' failed.*");

  EXPECT_DEATH(
      {
        state_machine->AddEdge(StateMachineNodeIndex(10), left_node,
                               std::bind(&PoseTrackingStateMachine<double>::IsNearPose, state_machine, left_side,
                                         kLinearTolerance, kAngularTolerance, _1));
      },
      ".*condition 'HasNode\\(outbound_node\\)' failed.*");

  // Finish setting-up state machine.
  // Should fail because one node has no exit condition.
  EXPECT_THROW(state_machine->Finalize(), std::logic_error);

  state_machine->AddEdge(right_node, left_node,
                         std::bind(&PoseTrackingStateMachine<double>::IsNearPose, state_machine, left_side,
                                   kLinearTolerance, kAngularTolerance, _1));

  // Add self-directed edges.
  // Left does not hit ball.
  state_machine->AddEdge("left-ball", "left-ball",
                         std::bind(&PoseTrackingStateMachine<double>::IsNotNearPose, state_machine, right_side,
                                   kLinearTolerance, kAngularTolerance, _1));
  // Right does not hit ball.
  state_machine->AddEdge("right-ball", "right-ball",
                         std::bind(&PoseTrackingStateMachine<double>::IsNotNearPose, state_machine, left_side,
                                   kLinearTolerance, kAngularTolerance, _1));

  EXPECT_NO_THROW(state_machine->Finalize());

  // Throw an error: finalize twice.
  EXPECT_THROW(state_machine->Finalize(), std::logic_error);

  // Abort: Edit state machine after finalize.
  EXPECT_THROW(state_machine->AddEdge("right-ball", "left-ball",
                                      std::bind(&PoseTrackingStateMachine<double>::IsNearPose, state_machine,
                                                right_side, kLinearTolerance, kAngularTolerance, _1)),
               std::logic_error);

  EXPECT_THROW(state_machine->AddNode("right-ball"), std::logic_error);
}

}  // namespace DR
