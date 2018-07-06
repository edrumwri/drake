
#include "drake/examples/iiwa_soccer/impedance_controller.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/frame_visualizer.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::BasicVector;
using systems::Context;
using systems::InputPortDescriptor;
using systems::OutputPort;
using systems::PublishEvent;
using systems::ContinuousState;
using math::RollPitchYaw;
using math::RotationMatrix;
using Eigen::Vector3d;


const InputPortDescriptor<double>& ImpedanceController::get_input_port_estimated_state() const {
  return this->get_input_port(0);
}

const InputPortDescriptor<double>& ImpedanceController::get_input_port_cartesian_target() const {
  return this->get_input_port(1);
}

const OutputPort<double>& ImpedanceController::get_output_port_control() const {
  return this->get_output_port(0);
}

const OutputPort<double>& ImpedanceController::get_output_port_hand_pos() const {
  return this->get_output_port(1);
}

void ImpedanceController::DoCalcTimeDerivatives(const Context<double>& context, ContinuousState<double>* derivatives) const {
  //context.get_continuous_state_vector();
  //const systems::BasicVector<double>* target_vector = this->EvalVectorInput(context, 1);
  // TODO(mws): Check the derivative calculation below; this function should be
  // computing the derivative of the controller state stored in the context,
  // which it does not appear to be doing.
  derivatives->get_mutable_vector()[0] = (x_target[0] - hand_pos[0]);
  derivatives->get_mutable_vector()[1] = (x_target[1] - hand_pos[1]);
  derivatives->get_mutable_vector()[2] = (x_target[2] - hand_pos[2]);

  //.SetFromVector(10000.0 * (x_target.colwise() - hand_pos));
}


/**
 *  Calculate what torques to apply to the joints.
 * @param context
 * @param output
 */
void ImpedanceController::DoControlCalc(const Context<double>& context, BasicVector<double>* const output) const {
  const systems::BasicVector<double>* st_vector = this->EvalVectorInput(context, 0); // state est
  const systems::BasicVector<double>* target_vector = this->EvalVectorInput(context, 1);


   // Is there a better way to do this?
  x_target[0] = target_vector->GetAtIndex(0);
  x_target[1] = target_vector->GetAtIndex(1); // Y can't change.
  x_target[2] = target_vector->GetAtIndex(2);


  VectorX<double> st_vecx = st_vector->CopyToVector();
  auto st_just_pos = st_vecx.topRows(state_size_/2);
  auto st_just_vel = st_vecx.bottomRows(state_size_/2);
  auto kinematics_cache = tree_.doKinematics(st_just_pos);

//  Eigen::Isometry3d ee_frame = tree_.CalcFramePoseInWorldFrame(kinematics_cache, controlled_frame_);
//
//  Vector3d ee_vec = ee_frame.rotation()*Vector3d(0,1,0);
//  Vector3d joint_vec = ee_frame.rotation()*Vector3d(0,0,1);
//
//  Vector3d goal_vec(0,0,1);
//
//  Vector3d normal_vec = joint_vec.cross(goal_vec);
//  double ctrl_angle = asin(normal_vec.dot(ee_vec));
//
//  drake::log()->log(ctrl_angle);




  // Rotation rate.
  //auto st_rot_rate = st_vecx.segment(8,3);

  Matrix6X<double> J = tree_.CalcFrameSpatialVelocityJacobianInWorldFrame(kinematics_cache, controlled_frame_, true);

  // angular velocity and then linear velocity.
  auto hand_world_pose = tree_.CalcFramePoseInWorldFrame(kinematics_cache, controlled_frame_);
  hand_pos = hand_world_pose.translation();
  auto J_linear = J.block(3, 0, 3, command_output_size_);

  // Stuff for angle management. Not yet ready
  // auto hand_rot = hand_world_pose.rotation();
  // auto rpy = RollPitchYaw<double>(RotationMatrix<double>(hand_rot));
  // auto J_rot = J.block(0, 0, 3, command_output_size_);

//  // + 0.0*context.get_continuous_state_vector().CopyToVector().cwiseAbs()
   auto torque = J_linear.transpose()*((k_p_).cwiseProduct(x_target.colwise() - hand_pos) + k_d_.cwiseProduct(J_linear * -st_just_vel));

//RollPitchYaw<double> rpy_goal(0, 0, 0);
//  Vector3<double> z(100,0,0);
//auto rot_t = J_rot.transpose()*((z).cwiseProduct(rpy_goal.vector() - rpy.vector()));// + 0.0001* k_d_.cwiseProduct(J_rot * -st_rot_rate));
//drake::log()->info(rot_t[5]);


  BasicVector<double> torque_out(torque); // Convert to BasicVector for some reason.
  torque_out.SetAtIndex(2, -st_vector->GetAtIndex(2) - st_vector->GetAtIndex(9) + torque_out.GetAtIndex(2)); // TMP TMP TMP kills arm tilting too much.
  //torque_out.SetAtIndex(5, -ctrl_angle * 10 + torque_out.GetAtIndex(5));
  // Don't flip out if NaN somehow comes out.
  for (int i = 0; i < torque_out.size(); i++) {
    if (std::isnan(torque_out.GetAtIndex(i))) {
      drake::log()->warn("Impedance controller thinks it should be putting out NaN torques. Check set points and things upstream.");
      torque_out.SetAtIndex(i, previous_torques_.GetAtIndex(i));
    }
  }

  previous_torques_.SetFrom(torque_out);
  output->SetFrom(torque_out);
}

void ImpedanceController::DoPublish(const Context<double>& context, const std::vector<const PublishEvent<double>*>& events) const {
  if (draw_status_ && (0.05 - std::fmod(context.get_time(), 0.05)) < 0.001) {
    // Draws the location of the controller's target.
    drake::lcmt_viewer_draw frame_msg{};
    frame_msg.timestamp = 0;
    frame_msg.num_links = 1;
    frame_msg.link_name.resize(1);
    frame_msg.robot_num.resize(1, 0);

    Eigen::Isometry3f pose;
    pose.setIdentity();

    Eigen::Quaternion<float> goal_quat =
        Eigen::Quaternion<float>(pose.linear());
    frame_msg.link_name[0] = "imped_controller_target";
    frame_msg.position.push_back({static_cast<float>(x_target(0)), static_cast<float>(x_target(1)),
                                  static_cast<float>(x_target(2))});
    frame_msg.quaternion.push_back(
        {goal_quat.w(), goal_quat.x(), goal_quat.y(), goal_quat.z()});

    const int num_bytes = frame_msg.getEncodedSize();
    const size_t size_bytes = static_cast<size_t>(num_bytes);
    std::vector<uint8_t> bytes(size_bytes);
    frame_msg.encode(bytes.data(), 0, num_bytes);
    lcm_->Publish("DRAKE_DRAW_FRAMES_T", bytes.data(), num_bytes, {});
  }
}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
