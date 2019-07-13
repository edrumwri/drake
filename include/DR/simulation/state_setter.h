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
   @param config the UnloadingTaskConfig describing parameters of the robot
          model's controllers
   @param mbp a const reference to the MultibodyPlant that represents the
          robot model
   @param diagram a const reference to the built world simulation and control
          diagram, containing the world model and controllers
   @param diagram_context a raw pointer to the context of the diagram param
   */
  void SetState(const UnloadingTaskConfig& config,
                const drake::multibody::MultibodyPlant<T>& mbp,
                const drake::systems::Diagram<T>& diagram,
                drake::systems::Context<T>* diagram_context) {
    config.ValidateConfig();

    drake::systems::Context<T>& mbp_context =
        diagram.GetMutableSubsystemContext(mbp, diagram_context);

    SetPositionAndVelocity(config, mbp, &mbp_context);
  }

 private:
  /**
   Sets the position and velocity of the robots and other free bodies in
   the world
   @param config the UnloadingTaskConfig describing parameters of the robot
          model's controllers
   @param mbp a const reference to the MultibodyPlant that represents the
          robot model
   @param mbp_context a raw pointer to the context of @p mbp
   */
  void SetPositionAndVelocity(const UnloadingTaskConfig& config,
                              const drake::multibody::MultibodyPlant<T>& mbp,
                              drake::systems::Context<T>* mbp_context) {
    config.ValidateConfig();

    for (const auto& manipuland : config.manipuland_instance_configs()) {
      DR_DEMAND(manipuland.is_floating());
      // Set the pose of this floating body in world frame
      mbp.SetFreeBodyPose(mbp_context, mbp.GetBodyByName(manipuland.name()),
                          manipuland.pose());
    }
  }
};
}  // namespace DR
