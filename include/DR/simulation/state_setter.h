/*
 This file defines functions that set the state values of bodies in a diagram context according to configuration
 parameters provided by SingleBodyInstanceConfig & RobotInstanceConfig from 'DR/simulation/config.h'.
 */

#pragma once

#include <memory>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/prismatic_joint.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <vector>

#include <DR/common/exception.h>
#include <DR/simulation/config.h>

namespace DR {

/*
 Sets the position and velocity of the joints of the model.
 @param model_instance the instance index of the model in `mbp`.
 @param config the configuration of the model, describes the desired base pose and joint positions and velocities
        that will be applied the robot model.
 @param mbp a const reference to the universal plant whose positions and velocities are set by this function.
 @param mbp_context a raw pointer to the mutable context of `mbp`, this object stores the state
        information of `mbp`.
  NOTE: joints not referred to by any of the joint_instance_configs will have their values zeroed.
 */
template <typename T>
void SetJointPositionsAndVelocities(drake::multibody::ModelInstanceIndex model_instance,
                                    const std::vector<JointInstanceConfig>& joint_instance_configs,
                                    const drake::multibody::MultibodyPlant<T>& mbp,
                                    drake::systems::Context<T>* mbp_context) {
  // Set position and velocity of each joint.
  for (const auto& joint : joint_instance_configs) {
    if (joint.type() == JointInstanceConfig::JointType::kRevoluteJointType) {
      const auto& robot_joint =
          mbp.template GetJointByName<drake::multibody::RevoluteJoint>(joint.name(), model_instance);
      robot_joint.set_angle(mbp_context, joint.position());
      robot_joint.set_angular_rate(mbp_context, joint.velocity());
      continue;
    } else if (joint.type() == JointInstanceConfig::JointType::kPrismaticJointType) {
      const auto& robot_joint =
          mbp.template GetJointByName<drake::multibody::PrismaticJoint>(joint.name(), model_instance);
      robot_joint.set_translation(mbp_context, joint.position());
      robot_joint.set_translation_rate(mbp_context, joint.velocity());
      continue;
    } else {
      throw std::runtime_error("Unsupported joint type");
    }
  }
}

/*
Sets the free body pose and spatial velocity of `body`.
@param body the floating body whoise pose and velocity are set by this function.
@param pose the desired 6D pose of the `body`.
@param spatial_velocity the desired 6D spatial_velocity of the `body`.
@param mbp a const reference to the universal plant whose positions and velocities are set by this function.
@param mbp_context a raw pointer to the mutable context of `mbp`, this object stores the state
       information of `mbp`.
*/
template <typename T>
void SetBodyPoseAndVelocity(const drake::multibody::Body<T>& body, const drake::math::RigidTransform<double>& pose,
                            const drake::multibody::SpatialVelocity<double>& spatial_velocity,
                            const drake::multibody::MultibodyPlant<T>& mbp, drake::systems::Context<T>* mbp_context) {
  if (!body.is_floating()) {
    return;
  }
  mbp.SetFreeBodyPose(mbp_context, body, pose);
  mbp.SetFreeBodySpatialVelocity(mbp_context, body, spatial_velocity);
}
}  // namespace DR
