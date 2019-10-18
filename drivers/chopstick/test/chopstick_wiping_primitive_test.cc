#include "chopstick_wiping_primitive.h"

#include <cstdlib>

#include <Eigen/SVD>

#include <drake/common/drake_optional.h>
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
#include <DR/drivers/chopstick/chopstick_impedance_controller.h>
#include <DR/drivers/chopstick/chopstick_kinematics.h>
#include <DR/primitives/primitive_behavior.h>
#include <DR/simulation/model_generator.h>

#include <gtest/gtest.h>

using drake::Vector3;
using drake::VectorX;
using drake::math::RigidTransform;
using drake::math::RotationMatrix;

namespace DR {
namespace {

// Some constants for this test.
const double kPeriod = M_PI * 2;       // The period of the Lissajous curve (in radians).
const double kPlanCopyFrequency = 25;  // The frequency (cycles per second) at which the plan is copied. Higher
                                       // frequencies yield less lag but cause the tests to run more slowly.
const double kTableTopWidth = 1.0;     // The width of the virtual table (in m).
const double kTableTopLength = 1.0;    // The length of the virtual table (in m).
const double kTableTopHeight = 1.0;    // The height of the virtual table (in m).
const double kp_cart_gain = 1e3;       // The error feedback gain for position error in Cartesian space.
const double kd_cart_gain = 1e2;       // The error feedback gain for velocity error in Cartesian space.
const double a = 1.0, b = 2.0;         // Parameters for Lissajous Figure-8 pattern.
wiping_primitive::LissajousParameters lissajous{
    .A = kTableTopWidth / 2, .a = a, .delta = 0, .B = kTableTopLength / 2, .b = b};

// Fixture for testing the wiping primitive.
class WipingTest : public ::testing::Test {
 public:
  const MultibodyPlantd& robot_plant() const { return *robot_plant_; }
  const MultibodyPlantd& universal_plant() const { return *universal_plant_; }
  const Contextd& universal_plant_context(const Contextd& context) const {
    return diagram_->GetSubsystemContext(*universal_plant_, context);
  }
  Contextd& universal_plant_context(Contextd* context) const {
    return diagram_->GetMutableSubsystemContext(*universal_plant_, context);
  }
  const ChopstickKinematicsd& kinematics() const { return *kinematics_; }
  ChopstickKinematicsd& kinematics() { return *kinematics_; }
  const drake::systems::Diagram<double>& diagram() const { return *diagram_; }
  ModelInstanceIndex robot_left_chopstick_model_instance() const { return robot_left_instance_; }
  ModelInstanceIndex universal_left_chopstick_model_instance() const { return universal_left_instance_; }
  InputPortIndex operation_signal_input() const { return operation_signal_input_; }

  // TODO(drum): Figure out how to effectively pass desired_translational_velocity by reference *and* pass in a nullopt.
  // Sets the initial generalized position for the robot to that predicted by the Lissajous curve.
  void SetInitialRobotPositions(Contextd* universal_plant_context,
                                drake::optional<Vector3d> desired_translational_velocity) {
    const double evaluation_time = 0;
    wiping_primitive::EndEffectorTrajectory traj(kTableTopHeight, lissajous, evaluation_time);
    VectorXd lissajous_output = traj.Evaluate(evaluation_time);
    VectorXd q = VectorXd::Zero(robot_plant().num_positions());
    RigidTransformd X_WG_target(lissajous_output.head<3>());

    // Set kinematics to solve to really high accuracy. Must use numerical IK since analytical IK does not yet support
    // position-only.
    kinematics().set_use_numerical_ik(true);
    kinematics().set_ik_type(ChopstickKinematicsd::InverseKinematicsType::kPositionOnly);
    kinematics().set_ik_tolerance(1e-10);

    // Get the frame.
    const Bodyd& chopstick_body = robot_plant().GetBodyByName("end_effector", robot_left_chopstick_model_instance());
    const auto& F = dynamic_cast<const Framed&>(chopstick_body.body_frame());

    // Solve IK.
    const VectorXd q_left = kinematics().SolveInverseKinematics(X_WG_target, wiping_primitive::p_LM, F);
    universal_plant_->SetPositions(universal_plant_context, robot_left_chopstick_model_instance(), q_left);

    // Solve velocity kinematics.
    if (desired_translational_velocity.has_value()) {
      kinematics().SolveTranslationalVelocityInverseKinematics(F, wiping_primitive::p_LM,
                                                               robot_left_chopstick_model_instance(), q_left,
                                                               desired_translational_velocity.value());
    }
  }

  // Returns the current end-effector location of the left chopstick, given the current state in the Context.
  RigidTransformd CalcLeftChopstickEndEffectorLocation(const Contextd& context) const {
    // Get the frame.
    const Bodyd& chopstick_body = robot_plant().GetBodyByName("end_effector", robot_left_chopstick_model_instance());
    const auto& F = dynamic_cast<const Framed&>(chopstick_body.body_frame());

    const VectorXd q_instance =
        robot_plant().GetPositions(universal_plant_context(context), universal_left_chopstick_model_instance());
    VectorXd q = VectorXd::Zero(robot_plant().num_positions());
    robot_plant().SetPositionsInArray(robot_left_chopstick_model_instance(), q_instance, &q);
    const RigidTransformd X_WE = kinematics().CalcForwardKinematics(q, wiping_primitive::p_LM, F);
    return X_WE;
  }

  std::unique_ptr<Contextd> transfer_context() { return std::move(context_); }

 private:
  void SetUp() override {
    // Construct the robot plant.
    robot_plant_ = std::make_unique<MultibodyPlantd>();

    // Construct the universal plant.
    drake::systems::DiagramBuilder<double> builder;
    auto scene_graph_plant = drake::multibody::AddMultibodyPlantSceneGraph(&builder);
    universal_plant_ = &scene_graph_plant.plant;

    // Add the robot models to the plants.
    std::tie(robot_left_instance_, robot_right_instance_) = AddChopsticksToMBP(robot_plant_.get());
    std::tie(universal_left_instance_, universal_right_instance_) = AddChopsticksToMBP(universal_plant_);

    // Finalize the plants.
    robot_plant_->Finalize();
    universal_plant_->Finalize();

    // Construct the kinematics system using an arbitrary seed to make any
    // randomized operations deterministic.
    int seed = 0;
    kinematics_ = std::make_unique<ChopstickKinematicsd>(robot_plant_.get(), seed);

    // Construct the wiping primitive.
    wiping_primitive_ = builder.AddSystem<wiping_primitive::WipingPrimitive>(
        universal_plant_, robot_plant_.get(), universal_left_instance_, robot_left_instance_, universal_right_instance_,
        robot_right_instance_, Vector3d::Ones() * kp_cart_gain, Vector3d::Ones() * kd_cart_gain, kPlanCopyFrequency,
        kTableTopHeight, lissajous);

#ifdef SIMPLE_CONTROLLER
    static std::unique_ptr<TaskSpaceGoald> left_goal, right_goal;

    // Make No collocated with Mo.
    const Vector3d left_goal_MoNo(0, 0, 0);
    const Vector3d right_goal_MoNo(0, 0, 0);

    const Bodyd& left_chopstick_body = universal_plant_->GetBodyByName("end_effector", universal_left_instance_);
    const Bodyd& right_chopstick_body = universal_plant_->GetBodyByName("end_effector", universal_right_instance_);
    const Framed& left_chopstick_frame_L = left_chopstick_body.body_frame();
    const Framed& right_chopstick_frame_R = right_chopstick_body.body_frame();
    const Framed& world_frame = universal_plant_->world_frame();
    left_goal = std::make_unique<TaskSpaceGoald>(TaskSpaceType::kCartesianOnly, &left_chopstick_frame_L,
                                                 wiping_primitive::p_LM, &world_frame, left_goal_MoNo);
    right_goal = std::make_unique<TaskSpaceGoald>(TaskSpaceType::kCartesianOnly, &right_chopstick_frame_R,
                                                  wiping_primitive::p_RS, &world_frame, right_goal_MoNo);
    std::vector<const TaskSpaceGoald*> goals = {left_goal.get(), right_goal.get()};
    auto controller =
        builder.AddSystem<ChopstickImpedanceController<double>>(universal_plant_, robot_plant_.get(), std::move(goals));
    controller->set_contact_affects_control(false);
    auto left_x_NoSo_N_input = builder.ExportInput(controller->x_NoSo_N_desired_input_port(*left_goal));
    auto left_xd_NoSo_N_input = builder.ExportInput(controller->xd_NoSo_N_desired_input_port(*left_goal));
    auto left_xdd_NoSo_N_input = builder.ExportInput(controller->xdd_NoSo_N_desired_input_port(*left_goal));
    auto right_x_NoSo_N_input = builder.ExportInput(controller->x_NoSo_N_desired_input_port(*right_goal));
    auto right_xd_NoSo_N_input = builder.ExportInput(controller->xd_NoSo_N_desired_input_port(*right_goal));
    auto right_xdd_NoSo_N_input = builder.ExportInput(controller->xdd_NoSo_N_desired_input_port(*right_goal));
    auto left_kp_gain_input = builder.ExportInput(controller->task_space_kp_gain_input_port(*left_goal));
    auto left_kd_gain_input = builder.ExportInput(controller->task_space_kd_gain_input_port(*left_goal));
    auto right_kp_gain_input = builder.ExportInput(controller->task_space_kp_gain_input_port(*right_goal));
    auto right_kd_gain_input = builder.ExportInput(controller->task_space_kd_gain_input_port(*right_goal));
    std::vector<int> output_port_sizes = {universal_plant_->num_positions(), universal_plant_->num_velocities()};
    auto mbp_demuxer = builder.AddSystem<drake::systems::Demultiplexer<double>>(output_port_sizes);
    builder.Connect(universal_plant_->get_state_output_port(), mbp_demuxer->get_input_port(0));
    builder.Connect(mbp_demuxer->get_output_port(0), controller->universal_q_estimated_input_port());
    builder.Connect(mbp_demuxer->get_output_port(1), controller->universal_v_estimated_input_port());
    auto actuator_demuxer = builder.AddSystem<ActuatorDemultiplexer<double>>(robot_plant_.get());
    builder.Connect(controller->actuation_output_port(), actuator_demuxer->full_actuation_input_port());
    builder.Connect(actuator_demuxer->actuated_model_output_port(robot_left_instance_),
                    universal_plant_->get_actuation_input_port(universal_left_instance_));
    builder.Connect(actuator_demuxer->actuated_model_output_port(robot_right_instance_),
                    universal_plant_->get_actuation_input_port(universal_right_instance_));
#else
    // The demultiplexer converts a x vector into q and v vectors.
    std::vector<int> output_port_sizes = {universal_plant_->num_positions(), universal_plant_->num_velocities()};
    auto mbp_demuxer = builder.AddSystem<drake::systems::Demultiplexer<double>>(output_port_sizes);

    // "Estimated" states are true states for this test.
    builder.Connect(universal_plant_->get_state_output_port(), mbp_demuxer->get_input_port(0));
    builder.Connect(mbp_demuxer->get_output_port(0), wiping_primitive_->universal_q_estimated_input_port());
    builder.Connect(mbp_demuxer->get_output_port(1), wiping_primitive_->universal_v_estimated_input_port());

    // The wiping primitive goes (nearly) straight to the actuator inputs.
    auto actuator_demuxer = builder.AddSystem<ActuatorDemultiplexer<double>>(robot_plant_.get());
    builder.Connect(wiping_primitive_->generalized_effort_output_port(), actuator_demuxer->full_actuation_input_port());
    builder.Connect(actuator_demuxer->actuated_model_output_port(robot_left_instance_),
                    universal_plant_->get_actuation_input_port(universal_left_instance_));
    builder.Connect(actuator_demuxer->actuated_model_output_port(robot_right_instance_),
                    universal_plant_->get_actuation_input_port(universal_right_instance_));

    // Export the control signal input port from the primitive.
    operation_signal_input_ = builder.ExportInput(wiping_primitive_->operation_signal_input_port());
#endif  // #ifdef SIMPLE_CONTROLLER

    // Connect the visualizer and construct the diagram.
    drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph_plant.scene_graph);
    diagram_ = builder.Build();

    // Create a context.
    context_ = diagram_->CreateDefaultContext();

    // Set the robot initial configuration.
    SetInitialRobotPositions(&universal_plant_context(context_.get()), drake::optional<Vector3d>{});

#ifdef SIMPLE_CONTROLLER
    // Fix inputs.
    context_->FixInputPort(left_xd_NoSo_N_input, Vector3d::Zero());
    context_->FixInputPort(left_xdd_NoSo_N_input, Vector3d::Zero());
    context_->FixInputPort(right_xd_NoSo_N_input, Vector3d::Zero());
    context_->FixInputPort(right_xdd_NoSo_N_input, Vector3d::Zero());
    context_->FixInputPort(left_kp_gain_input, Vector3d::Zero() * 1);
    context_->FixInputPort(left_kd_gain_input, Vector3d::Zero() * 0.1);
    context_->FixInputPort(right_kp_gain_input, Vector3d::Zero() * 1);
    context_->FixInputPort(right_kd_gain_input, Vector3d::Zero() * 0.1);

    // Get the initial chopstick locations.
    const RigidTransformd X_WR = right_goal->robot_frame_R().CalcPoseInWorld(universal_plant_context(*context_));
    const RigidTransformd X_WRg(X_WR.rotation(), X_WR * wiping_primitive::p_RS);
    const RigidTransformd X_WL = left_goal->robot_frame_R().CalcPoseInWorld(universal_plant_context(*context_));
    const RigidTransformd X_WLg(X_WL.rotation(), X_WL * wiping_primitive::p_RS);

    // Set the desired vectors from So (chopstick tip) to No (world origin).
    context_->FixInputPort(left_x_NoSo_N_input, X_WLg.translation());
    context_->FixInputPort(right_x_NoSo_N_input, X_WRg.translation());

#else
    // Get the initial right chopstick location and set it in the primitive.
    const RigidTransformd X_WR =
        wiping_primitive_->right_goal().robot_frame_R().CalcPoseInWorld(universal_plant_context(*context_));
    const RigidTransformd X_WRg(X_WR.rotation(), X_WR * wiping_primitive::p_RS);
    wiping_primitive_->set_initial_right_chopstick_location(X_WRg.translation());
#endif  // #ifdef SIMPLE_CONTROLLER
  }

  ModelInstanceIndex robot_left_instance_, universal_left_instance_, robot_right_instance_, universal_right_instance_;
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<MultibodyPlantd> robot_plant_;
  MultibodyPlantd* universal_plant_{nullptr};
  std::unique_ptr<ChopstickKinematicsd> kinematics_;
  wiping_primitive::WipingPrimitive* wiping_primitive_{nullptr};
  InputPortIndex operation_signal_input_;
  std::unique_ptr<Contextd> context_;
};

// Simulates the wiping motion in a complete cycle and verifies that the robot
// approximately reaches the endpoint.
TEST_F(WipingTest, EndPointReached) {
  // Fix the primitive control input.
  std::unique_ptr<Contextd> unique_context = transfer_context();
  drake::systems::Simulator<double> simulator(diagram(), std::move(unique_context));
  Contextd& context = simulator.get_mutable_context();

#ifndef SIMPLE_CONTROLLER
  context.FixInputPort(operation_signal_input(),
                       drake::AbstractValue::Make(PrimitiveBehavior<double>::OperationSignalType::kActive));
#endif

  // Set the end-effector trajectory.
  const wiping_primitive::EndEffectorTrajectory traj(kTableTopHeight, lissajous, context.get_time());

  DR::logging::set_log_level("debug");

  // Simulate at various points through the cycle and ensure that the end-effector is near where it should be.
  const double acceptable_error = 0.0001;  // (i.e., 0.1 mm).
  double t_inc = kPeriod / 10;
  for (double t = t_inc; t < kPeriod; t += t_inc) {
    simulator.AdvanceTo(t);

    // Get the current end-effector location.
    const RigidTransformd X_WE = CalcLeftChopstickEndEffectorLocation(context);

    // Get the desired end-effector location.
    const VectorXd lissajous_output = traj.Evaluate(context.get_time());
    RigidTransformd X_WE_target(lissajous_output.head<3>());
    EXPECT_LT((X_WE.translation() - X_WE_target.translation()).norm(), acceptable_error);
  }
}

}  // namespace
}  // namespace DR
