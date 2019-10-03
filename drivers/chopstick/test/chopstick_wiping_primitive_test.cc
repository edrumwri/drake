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

// TODO(edrumwri) The vector below needs to be set from some robot-specific configuration.
// Set the vector from the origin of F to the origin of G, expressed in F's frame (specific to the Chopsticks
// model).
const Vector3<double> p_FG(1.0, 0.0, 0.0);

// A Lissajous curve is described by:
// x = A * sin(a * t + delta)
// y = B * sin(b * t)
struct LissajousParameters {
  double A{0};      // The amplitude for the horizontal component.
  double a{0};      // The frequency gain for the horizontal component.
  double delta{0};  // The phase offset for the horizontal component.
  double B{0};      // The amplitude for the vertical component.
  double b{0};      // The frequency gain for the vertical component.
};

// Some constants for this test.
const double kPeriod = M_PI * 2;       // The period of the Lissajous curve (in radians).
const double kPlanCopyFrequency = 25;  // The frequency (cycles per second) at which the plan is copied. Higher
                                       // frequencies yield less lag but cause the tests to run more slowly.
const double kTableTopWidth = 1.0;     // The width of the virtual table (in m).
const double kTableTopLength = 1.0;    // The length of the virtual table (in m).
const double kTableTopHeight = 1.0;    // The height of the virtual table (in m).
const double kp_cart_gain = 1e3;       // The error feedback gain for position error in Cartesian space.
const double kv_cart_gain = 1e2;       // The error feedback gain for velocity error in Cartesian space.
const double a = 1.0, b = 2.0;         // Parameters for Lissajous Figure-8 pattern.
LissajousParameters lissajous{.A = kTableTopWidth / 2, .a = a, .delta = 0, .B = kTableTopLength / 2, .b = b};

// Gives a Lissajous curve in Cartesian space and its derivatives.
class EndEffectorTrajectory : public Plan<double> {
 public:
  EndEffectorTrajectory(const EndEffectorTrajectory&) = default;
  EndEffectorTrajectory& operator=(const EndEffectorTrajectory&) = default;

  EndEffectorTrajectory(double height, const LissajousParameters& lp, double start_time)
      : lp_(lp), h_(height), start_time_(start_time) {}

  // Outputs a nine-dimensional vector containing Cartesian position, Cartesian velocity,
  // and Cartesian acceleration.
  drake::VectorX<double> Evaluate(const double& t) const override {
    drake::VectorX<double> output(9);
    output[0] = lp_.A * std::sin(lp_.a * t + lp_.delta);
    output[1] = lp_.B * std::sin(lp_.b * t);
    output[2] = h_;
    output[3] = lp_.A * std::cos(lp_.a * t + lp_.delta) * lp_.a;           // time derivative of output[0].
    output[4] = lp_.B * std::cos(lp_.b * t) * lp_.b;                       // time derivative of output[1].
    output[5] = 0.0;                                                       // time derivative of output[2].
    output[6] = lp_.A * -std::sin(lp_.a * t + lp_.delta) * lp_.a * lp_.a;  // time derivative of output[3].
    output[7] = lp_.B * -std::sin(lp_.b * t) * lp_.b * lp_.b;              // time derivative of output[4].
    output[8] = 0;                                                         // time derivative of output[5].
    return output;
  }

  const double& start_time() const override { return start_time_; }
  const double& end_time() const override { return end_time_; }

 private:
  std::unique_ptr<Plan<double>> DoClone() const override { return std::make_unique<EndEffectorTrajectory>(*this); }
  LissajousParameters lp_;
  double h_{0};  // The z-component of the trajectory.
  double start_time_{0};
  double end_time_{std::numeric_limits<double>::infinity()};
};

class WipingPrimitive : public PrimitiveBehavior<double> {
 public:
  WipingPrimitive(const drake::multibody::MultibodyPlant<double>* all_plant,
                  const drake::multibody::MultibodyPlant<double>* robot_plant,
                  drake::multibody::ModelInstanceIndex robot_left_chopstick_model_instance,
                  drake::multibody::ModelInstanceIndex all_left_chopstick_model_instance)
      : PrimitiveBehavior(robot_plant, kPlanCopyFrequency),
        all_plant_(all_plant),
        robot_left_chopstick_model_instance_(robot_left_chopstick_model_instance),
        all_left_chopstick_model_instance_(all_left_chopstick_model_instance) {
    // Declare some needed inputs.
    all_q_estimated_input_port_index_ =
        this->DeclareVectorInputPort("all_q_estimated", drake::systems::BasicVector<double>(all_plant->num_positions()))
            .get_index();
    all_v_estimated_input_port_index_ =
        this->DeclareVectorInputPort("all_v_estimated",
                                     drake::systems::BasicVector<double>(all_plant->num_velocities()))
            .get_index();

    // Construct the impedance controller and its context.
    impedance_controller_ = std::make_unique<ChopstickImpedanceController<double>>(all_plant_);
    impedance_controller_context_ = impedance_controller_->CreateDefaultContext();

    // Construct the robot plant context.
    robot_plant_context_ = robot_plant->CreateDefaultContext();

    // Construct the kinematics.
    int seed = 0;  // Deterministic seed.
    kinematics_ = std::make_unique<ChopstickKinematics<double>>(robot_plant, seed);
  }

  const drake::systems::InputPort<double>& all_q_estimated_input_port() const {
    return drake::systems::System<double>::get_input_port(all_q_estimated_input_port_index_);
  }

  const drake::systems::InputPort<double>& all_v_estimated_input_port() const {
    return drake::systems::System<double>::get_input_port(all_v_estimated_input_port_index_);
  }

 private:
  // The control output uses an impedance controller to realize the desired kinematic trajectory for wiping. At the
  // moment, there are no constraints incorporated into the impedance controller, so the impedance controller acts
  // exactly as an inverse kinematics controller.
  void DoCalcControlOutput(const drake::systems::Context<double>& context,
                           drake::systems::BasicVector<double>* output) const override {
    // Get the active plan.
    const Plan<double>* active_plan = this->active_plan(context);

    // TODO(edrumwri): Consider changing this, as it lets the robot drop for a split second.
    if (!active_plan) {
      output->SetZero();
      return;
    }

    // Get q and v fed into this behavior.
    Eigen::VectorBlock<const VectorX<double>> q = this->get_input_port(all_q_estimated_input_port_index_).Eval(context);
    Eigen::VectorBlock<const VectorX<double>> v = this->get_input_port(all_v_estimated_input_port_index_).Eval(context);

    // TODO(edrumwri) Refactor this code into the impedance controller.
    // Get the q and v for the robot.
    const VectorX<double> q_robot_left = all_plant_->GetPositionsFromArray(all_left_chopstick_model_instance_, q);
    const VectorX<double> v_robot_left = all_plant_->GetVelocitiesFromArray(all_left_chopstick_model_instance_, v);
    VectorX<double> q_robot = VectorX<double>::Zero(robot_plant().num_positions());
    VectorX<double> v_robot = VectorX<double>::Zero(robot_plant().num_velocities());
    robot_plant().SetPositionsInArray(robot_left_chopstick_model_instance_, q_robot_left, &q_robot);
    robot_plant().SetVelocitiesInArray(robot_left_chopstick_model_instance_, v_robot_left, &v_robot);

    // Get the desired end effector position, velocity, and acceleration.
    const drake::VectorX<double> x_xd_xdd_des = active_plan->Evaluate(context.get_time());
    DR_DEMAND(x_xd_xdd_des.size() == 9);
    auto x_des = x_xd_xdd_des.segment<3>(0);
    auto xd_des = x_xd_xdd_des.segment<3>(3);
    auto xdd_des = x_xd_xdd_des.segment<3>(6);

    // Get the frame.
    const drake::multibody::Body<double>& chopstick_body =
        robot_plant().GetBodyByName("end_effector", robot_left_chopstick_model_instance_);
    const auto& F = dynamic_cast<const drake::multibody::Frame<double>&>(chopstick_body.body_frame());

    // Get the position differential.
    const drake::math::RigidTransform<double> x = kinematics_->CalcForwardKinematics(q_robot, p_FG, F);
    const drake::Vector3<double> x_err =
        DifferentialInverseKinematics<double>::CalcPositionDifferential(x.translation(), x_des);

    // Get the end-effector velocity error.
    const drake::Vector3<double> xd_err =
        xd_des - kinematics_->CalcFrameVelocity(q_robot, v_robot, p_FG, F).translational();

    // Construct a desired end-effector acceleration.
    const drake::Vector3<double> xdd_total = xdd_des + x_err * kp_cart_gain + xd_err * kv_cart_gain;

    // Compute acceleration kinematics to determine a vdot that realizes xdd_total. The acceleration kinematics is
    // Jl * vdot + Jl_bias = xdd_total, where Jl and Jl_bias are the linear components of the Jacobian matrix and
    // acceleration bias term, respectively.
    drake::MatrixX<double> J(6, robot_plant().num_velocities());
    const drake::multibody::Frame<double>& world_frame = robot_plant().world_frame();
    robot_plant().SetPositions(robot_plant_context_.get(), q_robot);
    robot_plant().SetVelocities(robot_plant_context_.get(), v_robot);
    robot_plant().CalcJacobianSpatialVelocity(*robot_plant_context_, drake::multibody::JacobianWrtVariable::kV, F, p_FG,
                                              world_frame, world_frame, &J);
    auto Jl = J.bottomRows<3>();
    const drake::Vector6<double> Jl_bias = robot_plant().CalcBiasForJacobianSpatialVelocity(
        *robot_plant_context_, drake::multibody::JacobianWrtVariable::kV, F, p_FG, world_frame, world_frame);
    Eigen::JacobiSVD<drake::MatrixX<double>> svd_Jl(Jl, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const VectorX<double> vdot = svd_Jl.solve(xdd_total - Jl_bias.tail<3>());

    // Compute all_vdot.
    const VectorX<double> vdot_left = robot_plant().GetVelocitiesFromArray(robot_left_chopstick_model_instance_, vdot);
    VectorX<double> vdot_all = VectorX<double>::Zero(all_plant_->num_velocities());
    all_plant_->SetVelocitiesInArray(all_left_chopstick_model_instance_, vdot_left, &vdot_all);

    // Wire up the impedance controller's inputs.
    Eigen::VectorBlock<const drake::VectorX<double>> q_input =
        this->get_input_port(all_q_estimated_input_port_index_).Eval(context);
    Eigen::VectorBlock<const drake::VectorX<double>> v_input =
        this->get_input_port(all_v_estimated_input_port_index_).Eval(context);
    impedance_controller_context_->FixInputPort(impedance_controller_->all_q_estimated_input_port().get_index(),
                                                q_input);
    impedance_controller_context_->FixInputPort(impedance_controller_->all_v_estimated_input_port().get_index(),
                                                v_input);
    impedance_controller_context_->FixInputPort(impedance_controller_->all_vdot_desired_input_port().get_index(),
                                                vdot_all);

    // Evaluate the impedance controller's output.
    Eigen::VectorBlock<const drake::VectorX<double>> u_all =
        impedance_controller_->generalized_effort_output_port().Eval(*impedance_controller_context_);

    // Just get the actuation for the left chopstick.
    const drake::VectorX<double> u_left_chopstick =
        all_plant_->GetActuationFromArray(all_left_chopstick_model_instance_, u_all);
    VectorX<double> u_both = VectorX<double>::Zero(robot_plant().num_actuated_dofs());
    robot_plant().SetActuationInArray(robot_left_chopstick_model_instance_, u_left_chopstick, &u_both);

    output->SetFromVector(u_all);
  }

  // Computes the plan. For wiping, this will consist of a kinematic plan and a desired force profile to push into the
  // table.
  std::unique_ptr<Plan<double>> DoComputePlan(const drake::systems::Context<double>& context) const {
    // Plan the sinusoidal path over the tabletop.
    return std::make_unique<EndEffectorTrajectory>(kTableTopHeight, lissajous, context.get_time());
  }

  std::unique_ptr<ChopstickImpedanceController<double>> impedance_controller_;
  const drake::multibody::MultibodyPlant<double>* all_plant_;
  std::unique_ptr<drake::systems::Context<double>> impedance_controller_context_;
  drake::systems::InputPortIndex all_q_estimated_input_port_index_{};
  drake::systems::InputPortIndex all_v_estimated_input_port_index_{};
  drake::multibody::ModelInstanceIndex robot_left_chopstick_model_instance_;
  drake::multibody::ModelInstanceIndex all_left_chopstick_model_instance_;
  mutable std::unique_ptr<drake::systems::Context<double>> robot_plant_context_;
  std::unique_ptr<ChopstickKinematics<double>> kinematics_;
};

// Fixture for testing the wiping primitive.
class WipingTest : public ::testing::Test {
 public:
  const drake::multibody::MultibodyPlant<double>& robot_plant() const { return *robot_plant_; }
  const drake::multibody::MultibodyPlant<double>& all_plant() const { return *all_plant_; }
  const drake::systems::Context<double>& all_plant_context(const drake::systems::Context<double>& context) const {
    return diagram_->GetSubsystemContext(*all_plant_, context);
  }
  drake::systems::Context<double>& all_plant_context(drake::systems::Context<double>* context) const {
    return diagram_->GetMutableSubsystemContext(*all_plant_, context);
  }
  const ChopstickKinematics<double>& kinematics() const { return *kinematics_; }
  ChopstickKinematics<double>& kinematics() { return *kinematics_; }
  const drake::systems::Diagram<double>& diagram() const { return *diagram_; }
  drake::multibody::ModelInstanceIndex robot_left_chopstick_model_instance() const { return robot_left_instance_; }
  drake::multibody::ModelInstanceIndex all_left_chopstick_model_instance() const { return all_left_instance_; }
  drake::systems::InputPortIndex operation_signal_input() const { return operation_signal_input_; }

  // Sets the initial generalized position for the robot to that predicted by the Lissajous curve.
  void SetInitialRobotPositions(drake::systems::Context<double>* all_plant_context) {
    const double evaluation_time = 0;
    EndEffectorTrajectory traj(kTableTopHeight, lissajous, evaluation_time);
    drake::VectorX<double> lissajous_output = traj.Evaluate(evaluation_time);
    drake::VectorX<double> q = drake::VectorX<double>::Zero(robot_plant().num_positions());
    drake::math::RigidTransform<double> X_WG_target(lissajous_output.head<3>());

    // Set kinematics to solve to really high accuracy. Must use numerical IK since analytical IK does not yet support
    // position-only.
    kinematics().set_use_numerical_ik(true);
    kinematics().set_ik_type(ChopstickKinematics<double>::InverseKinematicsType::kPositionOnly);
    kinematics().set_ik_tolerance(1e-10);

    // Get the frame.
    const drake::multibody::Body<double>& chopstick_body =
        robot_plant().GetBodyByName("end_effector", robot_left_chopstick_model_instance());
    const auto& F = dynamic_cast<const drake::multibody::Frame<double>&>(chopstick_body.body_frame());

    // Solve IK.
    const drake::VectorX<double> q_left = kinematics().SolveInverseKinematics(X_WG_target, p_FG, F);
    all_plant_->SetPositions(all_plant_context, robot_left_chopstick_model_instance(), q_left);
  }

  // Returns the current end-effector location of the left chopstick, given the current state in the Context.
  drake::math::RigidTransform<double> CalcLeftChopstickEndEffectorLocation(
      const drake::systems::Context<double>& context) const {
    // Get the frame.
    const drake::multibody::Body<double>& chopstick_body =
        robot_plant().GetBodyByName("end_effector", robot_left_chopstick_model_instance());
    const auto& F = dynamic_cast<const drake::multibody::Frame<double>&>(chopstick_body.body_frame());

    const drake::VectorX<double> q_instance =
        robot_plant().GetPositions(all_plant_context(context), all_left_chopstick_model_instance());
    drake::VectorX<double> q = drake::VectorX<double>::Zero(robot_plant().num_positions());
    robot_plant().SetPositionsInArray(robot_left_chopstick_model_instance(), q_instance, &q);
    const drake::math::RigidTransform<double> X_WE = kinematics().CalcForwardKinematics(q, p_FG, F);
    return X_WE;
  }

 private:
  void SetUp() override {
    // TODO(edrumwri) Turn the next block of code into a utility. It's going to be impossible to maintain as-is.
    // Get the absolute model path from an environment variable.
    const char* absolute_model_path_env_var = std::getenv("DR_ABSOLUTE_MODEL_PATH");
    ASSERT_NE(absolute_model_path_env_var, nullptr);
    std::string absolute_model_path = std::string(absolute_model_path_env_var);

    // Add a trailing slash if necessary.
    if (absolute_model_path.back() != '/') absolute_model_path += '/';

    // Construct the robot plant.
    robot_plant_ = std::make_unique<drake::multibody::MultibodyPlant<double>>();

    // Construct the "all" plant.
    drake::systems::DiagramBuilder<double> builder;
    auto scene_graph_plant = drake::multibody::AddMultibodyPlantSceneGraph(&builder);
    all_plant_ = &scene_graph_plant.plant;

    // Add the robot models to the plants. We never use the right chopstick so we don't save its model instance.
    ModelGenerator<double> model_generator;
    std::vector<RobotInstanceConfig> robots = CreateChopstickRobotsConfig();
    ASSERT_EQ(robots.front().name(), "chopstick_left");
    ASSERT_EQ(robots.back().name(), "chopstick_right");
    robot_left_instance_ = model_generator.AddRobotToMBP(robots.front(), robot_plant_.get());
    model_generator.AddRobotToMBP(robots.back(), robot_plant_.get());
    all_left_instance_ = model_generator.AddRobotToMBP(robots.front(), all_plant_);
    drake::multibody::ModelInstanceIndex all_right_instance = model_generator.AddRobotToMBP(robots.back(), all_plant_);

    // Finalize the plants.
    robot_plant_->Finalize();
    all_plant_->Finalize();

    // Construct the kinematics system using an arbitrary seed to make any
    // randomized operations deterministic.
    int seed = 0;
    kinematics_ = std::make_unique<ChopstickKinematics<double>>(robot_plant_.get(), seed);

    // Construct the wiping primitive.
    wiping_primitive_ =
        builder.AddSystem<WipingPrimitive>(all_plant_, robot_plant_.get(), robot_left_instance_, all_left_instance_);

    // The demultiplexer converts a x vector into q and v vectors.
    std::vector<int> output_port_sizes = {all_plant_->num_positions(), all_plant_->num_velocities()};
    auto mbp_demuxer = builder.AddSystem<drake::systems::Demultiplexer<double>>(output_port_sizes);

    // "Estimated" states are true states for this test.
    builder.Connect(all_plant_->get_state_output_port(), mbp_demuxer->get_input_port(0));
    builder.Connect(mbp_demuxer->get_output_port(0), wiping_primitive_->all_q_estimated_input_port());
    builder.Connect(mbp_demuxer->get_output_port(1), wiping_primitive_->all_v_estimated_input_port());

    // The wiping primitive goes (nearly) straight to the actuator inputs.
    auto actuator_demuxer = builder.AddSystem<ActuatorDemultiplexer<double>>(robot_plant_.get());
    builder.Connect(wiping_primitive_->generalized_effort_output_port(), actuator_demuxer->full_actuation_input_port());
    builder.Connect(actuator_demuxer->actuated_model_output_port(robot_left_instance_),
                    all_plant_->get_actuation_input_port(all_left_instance_));
    auto zero_right_actuation_input = builder.AddSystem<drake::systems::ConstantVectorSource<double>>(
        drake::VectorX<double>::Zero(all_plant_->num_actuated_dofs(all_right_instance)));
    builder.Connect(zero_right_actuation_input->get_output_port(),
                    all_plant_->get_actuation_input_port(all_right_instance));

    // Export the control signal input port from the primitive.
    operation_signal_input_ = builder.ExportInput(wiping_primitive_->operation_signal_input_port());

    // Connect the visualizer and construct the diagram.
    drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph_plant.scene_graph);
    diagram_ = builder.Build();
  }

  drake::multibody::ModelInstanceIndex robot_left_instance_, all_left_instance_;
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> robot_plant_;
  drake::multibody::MultibodyPlant<double>* all_plant_{nullptr};
  std::unique_ptr<ChopstickKinematics<double>> kinematics_;
  WipingPrimitive* wiping_primitive_{nullptr};
  drake::systems::InputPortIndex operation_signal_input_;
};

// Simulates the wiping motion in a complete cycle and verifies that the robot
// approximately reaches the endpoint.
TEST_F(WipingTest, EndPointReached) {
  // Fix the primitive control input.
  drake::systems::Simulator<double> simulator(diagram());
  drake::systems::Context<double>& context = simulator.get_mutable_context();
  context.FixInputPort(operation_signal_input(),
                       drake::AbstractValue::Make(PrimitiveBehavior<double>::OperationSignalType::kActive));

  // Set the end-effector trajectory.
  const EndEffectorTrajectory traj(kTableTopHeight, lissajous, context.get_time());

  // Set the robot initial configuration.
  SetInitialRobotPositions(&all_plant_context(&context));

  // Simulate at various points through the cycle and ensure that the end-effector is near where it should be.
  const double acceptable_error = 0.0001;  // (i.e., 0.1 mm).
  double t_inc = kPeriod / 10;
  for (double t = t_inc; t < kPeriod; t += t_inc) {
    simulator.AdvanceTo(t);

    // Get the current end-effector location.
    const drake::math::RigidTransform<double> X_WE = CalcLeftChopstickEndEffectorLocation(context);

    // Get the desired end-effector location.
    const drake::VectorX<double> lissajous_output = traj.Evaluate(context.get_time());
    drake::math::RigidTransform<double> X_WE_target(lissajous_output.head<3>());
    EXPECT_LT((X_WE.translation() - X_WE_target.translation()).norm(), acceptable_error);
  }
}

}  // namespace
}  // namespace DR
