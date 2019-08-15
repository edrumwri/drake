#pragma once

#include <memory>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/prismatic_joint.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>

#include <DR/common/exception.h>
#include <DR/simulation/config.h>

namespace DR {
/**
 Defines static functions that set the state values of bodies in a diagram
 context according to the values specified in a UnloadingTaskConfig struct.
 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
         Options for T are the following: (☑ = supported)
         ☑ double
         ☐ drake::AutoDiffXd
         ☐ drake::symbolic::Expression
 */
template <typename T>
class StateSetter {
 public:
  /**
   Sets the state of bodies in the world.
   NOTE: this function sets all states, not just kinematic ones.
   @param config the UnloadingTaskConfig describing all parameters of the task.
   @param mbp a const reference to the universal plant.
   @param diagram a const reference to the systems connected to `mbp`.
   @param diagram_context a raw pointer to the context of `diagram`.
   */
  void SetState(const UnloadingTaskConfig& config, const drake::multibody::MultibodyPlant<T>& mbp,
                const drake::systems::Diagram<T>& diagram, drake::systems::Context<T>* diagram_context) {
    config.ValidateConfig();

    drake::systems::Context<T>& mbp_context = diagram.GetMutableSubsystemContext(mbp, diagram_context);

    // StateSetter should address all states.  Abstract states are not currently addressed because there are none. This
    // check will trigger an error if abstract states are added but not addressed here.
    // TODO(samzapo): Set initial abstract states.
    DR_DEMAND(mbp_context.num_abstract_states() == 0);

    // NOTE: StateSetter does not address check num_discrete_state_groups.  Discrete states specifically left out
    // because MBP can run in two modes: continuous mode, where the equations of motions are integrated using an ODE
    // solver, and discrete mode, where the equations of motion are integrated using a time-stepping type scheme. In the
    // latter case, the kinematic state will be discrete variables.

    SetPositionAndVelocity(config, mbp, &mbp_context);
  }

 private:
  /*
    Sets the position and velocity of the joints of the model.
    @param model_instance the instance index of the model in `mbp`.
    @param config the configuration of the model, describes the desired base pose and joint positions and velocities
           that will be applied the robot model.
    @param mbp a const reference to the universal plant whose positions and velocities are set by this function.
    @param mbp_context a raw pointer to the mutable context of `mbp`, this object stores the state
           information of `mbp`.
    */
  void SetModelPositionAndVelocity(drake::multibody::ModelInstanceIndex model_instance,
                                   const RobotInstanceConfig& config, const drake::multibody::MultibodyPlant<T>& mbp,
                                   drake::systems::Context<T>* mbp_context) {
    // Zero-out PositionAndVelocity vector.
    mbp.SetPositionsAndVelocities(
        mbp_context, model_instance,
        drake::VectorX<T>::Zero(mbp.num_positions(model_instance) + mbp.num_velocities(model_instance)));

    // Set position and velocity of each joint.
    for (const auto& joint : config.joint_instance_configs()) {
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
  void SetBodyPoseAndVelocity(const drake::multibody::Body<T>& body, const drake::math::RigidTransform<double>& pose,
                              const drake::multibody::SpatialVelocity<double>& spatial_velocity,
                              const drake::multibody::MultibodyPlant<T>& mbp, drake::systems::Context<T>* mbp_context) {
    DR_DEMAND(body.is_floating());
    mbp.SetFreeBodyPose(mbp_context, body, pose);
    mbp.SetFreeBodySpatialVelocity(mbp_context, body, spatial_velocity);
  }

  /*
   Sets the position and velocity of the robots and other free bodies in
   the world.
   @param config the UnloadingTaskConfig describing parameters of the robot model's controllers.
   @param mbp a const reference to the universal plant whose positions and velocities are set by this function.
   @param mbp_context a raw pointer to the context of `mbp`.
   */
  void SetPositionAndVelocity(const UnloadingTaskConfig& config, const drake::multibody::MultibodyPlant<T>& mbp,
                              drake::systems::Context<T>* mbp_context) {
    config.ValidateConfig();

    for (const auto& robot : config.robot_instance_configs()) {
      drake::multibody::ModelInstanceIndex model_instance = mbp.GetModelInstanceByName(robot.name());

      SetModelPositionAndVelocity(model_instance, robot, mbp, mbp_context);

      // Set position and velocity of the robot's base link (if floating).
      // NOTE: if not floating, the pose of the base link of the robot was previously set by
      // `ModelGenerator::AddRobotToMBP`.
      if (robot.is_floating()) {
        SetBodyPoseAndVelocity(mbp.GetBodyByName("base", model_instance), robot.pose(), robot.spatial_velocity(), mbp,
                               mbp_context);
      }
    }

    for (const auto& manipuland : config.manipuland_instance_configs()) {
      // Set the pose and spatial velocity of this floating body in world frame.
      SetBodyPoseAndVelocity(mbp.GetBodyByName(manipuland.name()), manipuland.pose(), manipuland.spatial_velocity(),
                             mbp, mbp_context);
    }
  }
};
}  // namespace DR
