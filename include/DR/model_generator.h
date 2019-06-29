#pragma once

#include <drake/common/drake_assert.h>
#include <drake/common/drake_optional.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>

#include <DR/config.h>

namespace DR {

/**
 * The ModelGenerator creates a Builder, MultibodyPlant, and SceneGraph
 * according to configuration parameters provided in a UnloadingTaskConfig.
 */
class ModelGenerator {
 public:
  /**
   * Creates and finalizes the MultiBodyPlant representing the world.
   * Work done in this function includes:
   * 1) adding robot model, manipuland, environment bodies to the world.
   * 2) setting all body poses that will remain fixed
   *    (e.g., fixed frames, welds).
   * @param config the UnloadingTaskConfig describing parameters of the
   * world model.
   * @return a raw pointer to the MultibodyPlant (world model) that was built
   * and finalized in this scope.
   */
  static drake::multibody::MultibodyPlant<double>* CreateSceneAndRobot(
      const UnloadingTaskConfig& config,
      drake::systems::DiagramBuilder<double>* builder);

  /**
   * Moves the model to the given pose in the world and then generates a weld
   * joint between the world frame and the model for states that will remain
   * fixed.
   * @param body_name the base link of the robot or object (should be either
   * "base" for the root link of a robot, or the object name for fixed objects).
   * @param model_instance the instance index of the target model.
   * @param X_WM the transform from the world frame to the intended pose of
   * the model.
   * @param mbp raw pointer to the MultiBodyPlant representing the world.
   */
  static void PlaceAtPose(
      std::string body_name,
      const drake::multibody::ModelInstanceIndex& model_instance,
      const drake::math::RigidTransformd& X_WM,
      drake::multibody::MultibodyPlant<double>* mbp);

  /**
   * Generates a static plane (modeled as a half-space) into the multibody
   * plant.
   * @param mbp a raw pointer to the MultibodyPlant to populate with the object.
   * @param name Sets the name of the model instance (must be unique).
   * @param normal Sets the normal of the plane; determines the direction that
   * the plane faces.
   * @param center the center of the plane.
   * @param static_friction sets the static (V == 0.0) Coulomb friction of the
   * object.
   * @param dynamic_friction sets the dynamic (V != 0.0) Coulomb friction of the
   * object.
   * @param visual_box_dimensions (optional) the dimensions of the box to
   * display instead of an infinite plane.
   */
  static void AddStaticPlaneToMBP(
      drake::multibody::MultibodyPlant<double>* mbp, std::string name,
      const drake::Vector3<double>& normal,
      const drake::Vector3<double>& center,
      const drake::multibody::CoulombFriction<double>& surface_friction,
      const drake::optional<drake::Vector3<double>>& visual_box_dimensions =
          {});

  /**
   * Generates a static model of a truck trailer (composed of five half-spaces)
   * into the multibody plant.  The trailer-sized corridor is infinitely long in
   * the -x direction and constrained by half the trailer length in the +x
   * direction.
   * @param config the EnvironmentInstanceConfig describing features of the
   * environment.
   * @param mbp a raw pointer to the MultibodyPlant to populate with the model.
   */
  static void AddEnvironmentToMBP(
      const EnvironmentInstanceConfig& config,
      drake::multibody::MultibodyPlant<double>* mbp);

  /**
   * Generates a body and places it in the MultibodyPlant according to the
   * config (e.g., a box that can be picked-up, a static box on the floor).
   * @param config the BodyInstanceConfig describing the attributes of the
   * dynamic object (e.g., friction, mass, geometry).
   * @param mbp a raw pointer to the MultibodyPlant to populate with the model.
   */
  static void AddBodyToMBP(const BodyInstanceConfig& config,
                           drake::multibody::MultibodyPlant<double>* mbp);
};
}  // namespace DR
