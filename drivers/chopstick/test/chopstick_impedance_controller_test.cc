#include <cstdlib>

#include <Eigen/SVD>

#include <drake/geometry/geometry_visualization.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/demultiplexer.h>

#include <DR/common/actuator_demultiplexer.h>
#include <DR/common/environment.h>
#include <DR/drivers/chopstick/chopstick_config.h>
#include <DR/drivers/chopstick/chopstick_kinematics.h>
#include <DR/interfaces/task_space_impedance_controller.h>
#include <DR/primitives/primitive_behavior.h>
#include <DR/simulation/model_generator.h>

#include <gtest/gtest.h>

using drake::Vector3;
using drake::VectorX;
using drake::math::RigidTransform;
using drake::math::RotationMatrix;
using Vector3d = Vector3<double>;
using VectorXd = VectorX<double>;

namespace DR {

using drake::multibody::ModelInstanceIndex;
using Bodyd = drake::multibody::Body<double>;
// TODO(drum) Replace the TaskSpaceImpedanceController below with ChopstickImpedanceController.
using ChopstickImpedanceControllerd = TaskSpaceImpedanceController<double>;
using ChopstickKinematicsd = ChopstickKinematics<double>;
using Contextd = drake::systems::Context<double>;
using ContinuousStated = drake::systems::ContinuousState<double>;
using Framed = drake::multibody::Frame<double>;
using MultibodyPlantd = drake::multibody::MultibodyPlant<double>;
using RigidTransformd = drake::math::RigidTransform<double>;
using TaskSpaceGoald = TaskSpaceGoal<double>;

namespace {

// TODO(drum) The vector below needs to be set from some robot-specific configuration.
// Set the vector from the origin of the left chopstick frame A to the origin of Frame B, which is rigidly attached to
// (and aligned with) Frame A and located at the tip of the left chopstick.
const Vector3d p_AB(1.0, 0.0, 0.0);

// TODO(drum) The vector below needs to be set from some robot-specific configuration.
// Set the vector from the origin of the left chopstick frame I to the origin of Frame J, which is rigidly attached to
// (and aligned with) Frame I and located at the tip of the left chopstick.
const Vector3d p_IJ(1.0, 0.0, 0.0);

// Fixture for testing the task space impedance controller.
class ChopstickImpedanceControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create an inner diagram for the MultibodyPlant and SceneGraph.
    drake::systems::DiagramBuilder<double> inner_builder;
    auto scene_graph_plant = drake::multibody::AddMultibodyPlantSceneGraph(&inner_builder);
    robot_plant_ = &scene_graph_plant.plant;

    // Add the robot models to the plants.
    std::tie(robot_left_instance_, robot_right_instance_) = AddChopsticksToMBP(robot_plant_);

    // Finalize the plant and build the diagram, but not before exporting some useful ports.
    robot_plant_->Finalize();
    drake::systems::DiagramBuilder<double> builder;
    const drake::systems::InputPortIndex left_chopstick_actuation_input_port_index =
        inner_builder.ExportInput(robot_plant_->get_actuation_input_port(robot_left_instance_));
    const drake::systems::InputPortIndex right_chopstick_actuation_input_port_index =
        inner_builder.ExportInput(robot_plant_->get_actuation_input_port(robot_right_instance_));
    const drake::systems::OutputPortIndex robot_plant_state_output_port_index =
        inner_builder.ExportOutput(robot_plant_->get_state_output_port());
    const auto& universal_system = *builder.AddSystem(inner_builder.Build());

    // Construct the kinematics system using an arbitrary seed (NOTE: no kinematics operations in this
    // controller use randomness or, by extension, a random seed).
    int seed = 0;
    kinematics_ = std::make_unique<ChopstickKinematicsd>(robot_plant_, seed);

    // Create the goals.
    const Framed& world_frame = robot_plant_->world_frame();
    const Bodyd& left_chopstick_body = robot_plant_->GetBodyByName("end_effector", robot_left_instance_);
    const Bodyd& right_chopstick_body = robot_plant_->GetBodyByName("end_effector", robot_right_instance_);
    const Framed& left_chopstick_frame_L = left_chopstick_body.body_frame();
    const Framed& right_chopstick_frame_R = right_chopstick_body.body_frame();
    auto left_goal = std::make_unique<TaskSpaceGoald>(TaskSpaceType::kCartesianOnly, &left_chopstick_frame_L, p_AB,
                                                      &world_frame, Vector3d::Zero());
    auto right_goal = std::make_unique<TaskSpaceGoald>(TaskSpaceType::kCartesianOnly, &right_chopstick_frame_R, p_IJ,
                                                       &world_frame, Vector3d::Zero());
    left_goal_pointer_ = left_goal.get();
    right_goal_pointer_ = right_goal.get();

    // Construct the impedance controller.
    std::vector<std::unique_ptr<TaskSpaceGoald>> goals;
    goals.emplace_back(std::move(left_goal));
    goals.emplace_back(std::move(right_goal));
    impedance_controller_ = builder.AddSystem<ChopstickImpedanceControllerd>(robot_plant_, &universal_system,
                                                                             robot_plant_, std::move(goals));

    // TODO(drum): Replace the code below with a bespoke continuous state demultiplexer.
    // The demultiplexer converts a x vector into q and v vectors.
    std::vector<int> output_port_sizes = {robot_plant_->num_positions(), robot_plant_->num_velocities()};
    auto mbp_demuxer = builder.AddSystem<drake::systems::Demultiplexer<double>>(output_port_sizes);

    // "Estimated" states are true states for this test.
    builder.Connect(universal_system.get_output_port(robot_plant_state_output_port_index),
                    mbp_demuxer->get_input_port(0));
    builder.Connect(mbp_demuxer->get_output_port(0), impedance_controller_->universal_q_estimated_input_port());
    builder.Connect(mbp_demuxer->get_output_port(1), impedance_controller_->universal_v_estimated_input_port());

    // The impedance controller goes (nearly) straight to the actuator inputs.
    auto actuator_demuxer = builder.AddSystem<ActuatorDemultiplexer<double>>(robot_plant_);
    builder.Connect(impedance_controller_->actuation_output_port(), actuator_demuxer->full_actuation_input_port());
    builder.Connect(actuator_demuxer->actuated_model_output_port(robot_left_instance_),
                    universal_system.get_input_port(left_chopstick_actuation_input_port_index));
    builder.Connect(actuator_demuxer->actuated_model_output_port(robot_right_instance_),
                    universal_system.get_input_port(right_chopstick_actuation_input_port_index));

    // Export outputs from the controller.
    left_x_NoSo_N_input_ = builder.ExportInput(impedance_controller_->x_NS_N_desired_input_port(*left_goal_pointer_));
    left_xd_NoSo_N_input_ = builder.ExportInput(impedance_controller_->xd_NS_N_desired_input_port(*left_goal_pointer_));
    left_xdd_NoSo_N_input_ =
        builder.ExportInput(impedance_controller_->xdd_NS_N_desired_input_port(*left_goal_pointer_));
    right_x_NoSo_N_input_ = builder.ExportInput(impedance_controller_->x_NS_N_desired_input_port(*right_goal_pointer_));
    right_xd_NoSo_N_input_ =
        builder.ExportInput(impedance_controller_->xd_NS_N_desired_input_port(*right_goal_pointer_));
    right_xdd_NoSo_N_input_ =
        builder.ExportInput(impedance_controller_->xdd_NS_N_desired_input_port(*right_goal_pointer_));
    left_kp_gain_input_ =
        builder.ExportInput(impedance_controller_->task_space_kp_gain_input_port(*left_goal_pointer_));
    left_kd_gain_input_ =
        builder.ExportInput(impedance_controller_->task_space_kd_gain_input_port(*left_goal_pointer_));
    right_kp_gain_input_ =
        builder.ExportInput(impedance_controller_->task_space_kp_gain_input_port(*right_goal_pointer_));
    right_kd_gain_input_ =
        builder.ExportInput(impedance_controller_->task_space_kd_gain_input_port(*right_goal_pointer_));

    diagram_ = builder.Build();

    // Create a context.
    context_ = diagram_->CreateDefaultContext();
  }

  // Sets the task space gains to "very compliant" with "low damping".
  void SetConstantGains() {
    context_->FixInputPort(left_kp_gain_input_, Vector3d::Ones() * 1);
    context_->FixInputPort(left_kd_gain_input_, Vector3d::Ones() * 0.1);
    context_->FixInputPort(right_kp_gain_input_, Vector3d::Ones() * 1);
    context_->FixInputPort(right_kd_gain_input_, Vector3d::Ones() * 0.1);
  }

  drake::systems::InputPortIndex left_x_NoSo_N_input_, left_xd_NoSo_N_input_, left_xdd_NoSo_N_input_,
      left_kp_gain_input_, left_kd_gain_input_;
  drake::systems::InputPortIndex right_x_NoSo_N_input_, right_xd_NoSo_N_input_, right_xdd_NoSo_N_input_,
      right_kp_gain_input_, right_kd_gain_input_;
  ModelInstanceIndex robot_left_instance_, robot_right_instance_;
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  MultibodyPlantd* robot_plant_{nullptr};
  ChopstickImpedanceControllerd* impedance_controller_{nullptr};
  std::unique_ptr<ChopstickKinematicsd> kinematics_;
  std::unique_ptr<Contextd> context_;
  const TaskSpaceGoald* left_goal_pointer_;
  const TaskSpaceGoald* right_goal_pointer_;
};

// Verifies that the impedance controller realizes the desired task space acceleration.
TEST_F(ChopstickImpedanceControllerTest, RealizesDesiredAccelerationWithoutContact) {
  // Set arbitrary generalized position, velocity, and desired acceleration for the plant.
  VectorXd q(10);
  q << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0;
  const VectorXd v = q * 10;
  const VectorXd vdot = v * 10;

  // Set the state.
  VectorXd x(q.size() + v.size());
  x.head(q.size()) = q;
  x.tail(v.size()) = v;
  context_->get_mutable_continuous_state().SetFromVector(x);

  // Compute the task space position that results from the arbitrary generalized position.
  const Vector3d x_left =
      kinematics_->CalcForwardKinematics(q, left_goal_pointer_->p_RS(), left_goal_pointer_->robot_frame_R())
          .translation();
  const Vector3d x_right =
      kinematics_->CalcForwardKinematics(q, right_goal_pointer_->p_RS(), right_goal_pointer_->robot_frame_R())
          .translation();

  // Compute the task space velocity that results from the arbitrary generalized velocity.
  const Vector3d xd_left =
      kinematics_->CalcFrameVelocity(q, v, left_goal_pointer_->p_RS(), left_goal_pointer_->robot_frame_R())
          .translational();
  const Vector3d xd_right =
      kinematics_->CalcFrameVelocity(q, v, right_goal_pointer_->p_RS(), right_goal_pointer_->robot_frame_R())
          .translational();

  // Compute the task space acceleration that results from the arbitrary generalized acceleration.
  const Vector3d xdd_left =
      kinematics_->CalcFrameAcceleration(q, v, vdot, left_goal_pointer_->p_RS(), left_goal_pointer_->robot_frame_R())
          .tail<3>();
  const Vector3d xdd_right =
      kinematics_->CalcFrameAcceleration(q, v, vdot, right_goal_pointer_->p_RS(), right_goal_pointer_->robot_frame_R())
          .tail<3>();

  // Wire the task space acceleration into the impedance controller.
  // Fix inputs.
  context_->FixInputPort(left_x_NoSo_N_input_, x_left);
  context_->FixInputPort(left_xd_NoSo_N_input_, xd_left);
  context_->FixInputPort(left_xdd_NoSo_N_input_, xdd_left);
  context_->FixInputPort(right_x_NoSo_N_input_, x_right);
  context_->FixInputPort(right_xd_NoSo_N_input_, xd_right);
  context_->FixInputPort(right_xdd_NoSo_N_input_, xdd_right);
  SetConstantGains();

  // Calculate the generalized acceleration that results from the control inputs.
  std::unique_ptr<ContinuousStated> xdot = diagram_->AllocateTimeDerivatives();
  diagram_->CalcTimeDerivatives(*context_, xdot.get());
  const VectorXd vdot_actual = xdot->CopyToVector().tail(vdot.size());

  // Compute the end-effector acceleration that results from the generalized acceleration. Note that the generalized
  // acceleration that results from the control inputs may be different than the set generalized acceleration because
  // of kinematic redundancy.
  const Vector3d xdd_left_actual =
      kinematics_
          ->CalcFrameAcceleration(q, v, vdot_actual, left_goal_pointer_->p_RS(), left_goal_pointer_->robot_frame_R())
          .tail<3>();
  const Vector3d xdd_right_actual =
      kinematics_
          ->CalcFrameAcceleration(q, v, vdot_actual, right_goal_pointer_->p_RS(), right_goal_pointer_->robot_frame_R())
          .tail<3>();

  // This is approximately the tightest tolerance at which the test below passes (the slop seems due to the least
  // squares solve).
  const double tol = 1e4 * std::numeric_limits<double>::epsilon();
  EXPECT_LT((xdd_left_actual - xdd_left).norm(), tol);
  EXPECT_LT((xdd_right_actual - xdd_right).norm(), tol);
}

}  // namespace
}  // namespace DR
