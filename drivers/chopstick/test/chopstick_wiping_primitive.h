#pragma once

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <DR/drivers/chopstick/chopstick_impedance_controller.h>
#include <DR/drivers/chopstick/chopstick_kinematics.h>
#include <DR/primitives/primitive_behavior.h>

namespace DR {

using BasicVectord = drake::systems::BasicVector<double>;
using Bodyd = drake::multibody::Body<double>;
using ChopstickImpedanceControllerd = ChopstickImpedanceController<double>;
using ChopstickKinematicsd = ChopstickKinematics<double>;
using Contextd = drake::systems::Context<double>;
using Framed = drake::multibody::Frame<double>;
using InputPortd = drake::systems::InputPort<double>;
using MultibodyPlantd = drake::multibody::MultibodyPlant<double>;
using Pland = Plan<double>;
using RigidTransformd = drake::math::RigidTransform<double>;
using TaskSpaceGoald = TaskSpaceGoal<double>;
using VectorXd = drake::VectorX<double>;
using Vector3d = drake::Vector3<double>;
using drake::multibody::ModelInstanceIndex;
using drake::systems::InputPortIndex;

namespace wiping_primitive {

// TODO(drum) The vector below needs to be set from some robot-specific configuration.
// Set the vector from the origin of the left chopstick frame L to the origin of its tip (which we'll be using for
// manipulation in these tests), expressed in L's frame (specific to the Chopsticks model).
const Vector3d p_LM(1.0, 0.0, 0.0);

// TODO(drum) The vector below needs to be set from some robot-specific configuration.
// Set the vector from the origin of the right chopstick frame R to the origin of its tip (which we'll be using for
// manipulation in these tests), expressed in R's frame (specific to the Chopsticks model).
const Vector3d p_RS(1.0, 0.0, 0.0);

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

// Gives a Lissajous curve in Cartesian space and its derivatives.
class EndEffectorTrajectory : public Pland {
 public:
  EndEffectorTrajectory(const EndEffectorTrajectory&) = default;
  EndEffectorTrajectory& operator=(const EndEffectorTrajectory&) = default;

  EndEffectorTrajectory(double height, const LissajousParameters& lp, double start_time)
      : lp_(lp), h_(height), start_time_(start_time) {}

  // Outputs a nine-dimensional vector containing Cartesian position, Cartesian velocity,
  // and Cartesian acceleration.
  VectorXd Evaluate(const double& t) const override {
    VectorXd output(9);
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
  std::unique_ptr<Pland> DoClone() const override { return std::make_unique<EndEffectorTrajectory>(*this); }
  LissajousParameters lp_;
  double h_{0};  // The z-component of the trajectory.
  double start_time_{0};
  double end_time_{std::numeric_limits<double>::infinity()};
};

class WipingPrimitive : public PrimitiveBehavior<double> {
 public:
  WipingPrimitive(const MultibodyPlantd* universal_plant, const MultibodyPlantd* robot_plant,
                  ModelInstanceIndex universal_left_chopstick_model_instance,
                  ModelInstanceIndex robot_left_chopstick_model_instance,
                  ModelInstanceIndex universal_right_chopstick_model_instance,
                  ModelInstanceIndex robot_right_chopstick_model_instance, const Vector3d& kp_gain,
                  const Vector3d& kd_gain, double plan_copy_frequency, double tabletop_height,
                  const LissajousParameters& lissajous)
      : PrimitiveBehavior(robot_plant, plan_copy_frequency),
        universal_plant_(universal_plant),
        universal_left_chopstick_model_instance_(universal_left_chopstick_model_instance),
        robot_left_chopstick_model_instance_(robot_left_chopstick_model_instance),
        universal_right_chopstick_model_instance_(universal_right_chopstick_model_instance),
        robot_right_chopstick_model_instance_(robot_right_chopstick_model_instance),
        kp_gain_(kp_gain),
        kd_gain_(kd_gain),
        tabletop_height_(tabletop_height),
        lissajous_(lissajous) {
    // Declare some needed inputs.
    universal_q_estimated_input_port_index_ =
        this->DeclareVectorInputPort("universal_q_estimated", BasicVectord(universal_plant->num_positions()))
            .get_index();
    universal_v_estimated_input_port_index_ =
        this->DeclareVectorInputPort("universal_v_estimated", BasicVectord(universal_plant->num_velocities()))
            .get_index();

    const Bodyd& left_chopstick_body =
        universal_plant->GetBodyByName("end_effector", universal_left_chopstick_model_instance_);
    const Bodyd& right_chopstick_body =
        universal_plant->GetBodyByName("end_effector", universal_right_chopstick_model_instance_);
    const Framed& left_chopstick_frame_L = left_chopstick_body.body_frame();
    const Framed& right_chopstick_frame_R = right_chopstick_body.body_frame();

    // Create the goals.
    const Framed& world_frame = universal_plant->world_frame();
    left_goal_ = std::make_unique<TaskSpaceGoald>(TaskSpaceType::kCartesianOnly, &left_chopstick_frame_L, p_LM,
                                                  &world_frame, Vector3d::Zero());
    right_goal_ = std::make_unique<TaskSpaceGoald>(TaskSpaceType::kCartesianOnly, &right_chopstick_frame_R, p_RS,
                                                   &world_frame, Vector3d::Zero());

    // Construct the impedance controller and its context.
    std::vector<const TaskSpaceGoald*> goals = {left_goal_.get(), right_goal_.get()};
    impedance_controller_ =
        std::make_unique<ChopstickImpedanceControllerd>(universal_plant, robot_plant, std::move(goals));
    impedance_controller_context_ = impedance_controller_->CreateDefaultContext();

    // TODO(drum): Remove the next line.
    impedance_controller_->set_contact_affects_control(false);

    // Wire up right chopstick velocity and acceleration goals.
    impedance_controller_context_->FixInputPort(
        impedance_controller_->xd_NoSo_N_desired_input_port(*right_goal_).get_index(), Vector3d::Zero());
    impedance_controller_context_->FixInputPort(
        impedance_controller_->xdd_NoSo_N_desired_input_port(*right_goal_).get_index(), Vector3d::Zero());

    // Wire up gains for the chopsticks.
    impedance_controller_context_->FixInputPort(
        impedance_controller_->task_space_kp_gain_input_port(*left_goal_).get_index(), kp_gain_);
    impedance_controller_context_->FixInputPort(
        impedance_controller_->task_space_kd_gain_input_port(*left_goal_).get_index(), kd_gain_);
    impedance_controller_context_->FixInputPort(
        impedance_controller_->task_space_kp_gain_input_port(*right_goal_).get_index(), kp_gain_);
    impedance_controller_context_->FixInputPort(
        impedance_controller_->task_space_kd_gain_input_port(*right_goal_).get_index(), kd_gain_);
  }

  void set_initial_right_chopstick_location(const Vector3d& initial_right_chopstick_location) {
    // Wire up right chopstick Cartesian goal.
    impedance_controller_context_->FixInputPort(
        impedance_controller_->x_NoSo_N_desired_input_port(*right_goal_).get_index(), initial_right_chopstick_location);
  }

  const TaskSpaceGoald& right_goal() const { return *right_goal_; }
  const InputPortd& universal_q_estimated_input_port() const {
    return drake::systems::System<double>::get_input_port(universal_q_estimated_input_port_index_);
  }
  const InputPortd& universal_v_estimated_input_port() const {
    return drake::systems::System<double>::get_input_port(universal_v_estimated_input_port_index_);
  }

 private:
  // Computes the control output in actuation space. Since there are two chopsticks, we have to wire this to
  // an ActuationDemultiplexer.
  void DoCalcControlOutput(const Contextd& context, BasicVectord* output) const override {
    // Get the active plan.
    const Pland* active_plan = this->active_plan(context);
    if (!active_plan) {
      output->SetZero();
      return;
    }

    // Get the desired end effector position, velocity, and acceleration.
    const VectorXd x_xd_xdd_des = active_plan->Evaluate(context.get_time());
    DR_DEMAND(x_xd_xdd_des.size() == 9);
    auto x_des = x_xd_xdd_des.segment<3>(0);
    auto xd_des = x_xd_xdd_des.segment<3>(3);
    auto xdd_des = x_xd_xdd_des.segment<3>(6);

    // Wire up these inputs.
    impedance_controller_context_->FixInputPort(
        impedance_controller_->x_NoSo_N_desired_input_port(*left_goal_).get_index(), x_des);
    impedance_controller_context_->FixInputPort(
        impedance_controller_->xd_NoSo_N_desired_input_port(*left_goal_).get_index(), xd_des);
    impedance_controller_context_->FixInputPort(
        impedance_controller_->xdd_NoSo_N_desired_input_port(*left_goal_).get_index(), xdd_des);

    // Wire up q and v.
    Eigen::VectorBlock<const VectorXd> q = this->get_input_port(universal_q_estimated_input_port_index_).Eval(context);
    Eigen::VectorBlock<const VectorXd> v = this->get_input_port(universal_v_estimated_input_port_index_).Eval(context);
    impedance_controller_context_->FixInputPort(impedance_controller_->universal_q_estimated_input_port().get_index(),
                                                q);
    impedance_controller_context_->FixInputPort(impedance_controller_->universal_v_estimated_input_port().get_index(),
                                                v);

    // Evaluate the task space impedance controller with these inputs.
    Eigen::VectorBlock<const VectorXd> u =
        impedance_controller_->actuation_output_port().Eval(*impedance_controller_context_);
    output->SetFromVector(u);
  }

  // Computes the plan. For wiping, this will consist of a kinematic plan and a desired force profile to push into the
  // table.
  std::unique_ptr<Pland> DoComputePlan(const Contextd& context) const {
    // Plan the sinusoidal path over the tabletop.
    return std::make_unique<EndEffectorTrajectory>(tabletop_height_, lissajous_, context.get_time());
  }

  std::unique_ptr<ChopstickImpedanceControllerd> impedance_controller_;
  const MultibodyPlantd* universal_plant_;
  std::unique_ptr<Contextd> impedance_controller_context_;
  InputPortIndex universal_q_estimated_input_port_index_{};
  InputPortIndex universal_v_estimated_input_port_index_{};
  ModelInstanceIndex universal_left_chopstick_model_instance_;
  ModelInstanceIndex robot_left_chopstick_model_instance_;
  ModelInstanceIndex universal_right_chopstick_model_instance_;
  ModelInstanceIndex robot_right_chopstick_model_instance_;
  mutable std::unique_ptr<Contextd> robot_plant_context_;
  std::unique_ptr<ChopstickKinematicsd> kinematics_;
  std::unique_ptr<TaskSpaceGoald> left_goal_, right_goal_;
  Vector3d kp_gain_, kd_gain_;
  double tabletop_height_;
  LissajousParameters lissajous_;
};

}  //  namespace wiping_primitive
}  //  namespace DR
