#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <utility>

#include <drake/common/drake_optional.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>

#include <DR/common/exception.h>
#include <DR/simulation/config.h>
#include <DR/simulation/shape_to_unit_inertia.h>

namespace DR {

/**
 The ModelGenerator creates a MultibodyPlant and SceneGraph
 according to configuration parameters provided in a UnloadingTaskConfig.
 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
         Options for T are the following: (☑ = supported)
         ☑ double
         ☐ drake::AutoDiffXd
         ☐ drake::symbolic::Expression
 */
template <typename T>
class ModelGenerator {
 public:
  /**
   Creates and finalizes the universal plant.
   Work done in this function includes:
   1) adding robot model, manipuland, environment bodies to the universal plant.
   2) setting all body poses that will remain fixed (e.g., fixed frames, welds).
   @param config the UnloadingTaskConfig stores parameters used to generate the scene and robot in the universal plant.
   @param builder a raw pointer to the DiagramBuilder that is responsible for connecting systems to the universal plant.
   @return a raw pointer to the universal plant that was built and finalized in this scope.
   */
  std::tuple<drake::multibody::MultibodyPlant<T>*, drake::geometry::SceneGraph<T>*> CreateSceneAndRobot(
      const UnloadingTaskConfig& config, drake::systems::DiagramBuilder<T>* builder) {
    config.ValidateConfig();

    drake::multibody::AddMultibodyPlantSceneGraphResult<T> result = drake::multibody::AddMultibodyPlantSceneGraph(
        builder, std::make_unique<drake::multibody::MultibodyPlant<T>>(config.simulator_instance_config().step_size()));
    drake::multibody::MultibodyPlant<T>* mbp = &(result.plant);
    drake::geometry::SceneGraph<T>* scene_graph = &(result.scene_graph);

    // Add Robots to world.
    for (const auto& robot : config.robot_instance_configs()) {
      AddRobotToMBP(robot, mbp);
    }

    // Add Manipulands to the world.
    for (const auto& manipuland : config.manipuland_instance_configs()) {
      AddBodyToMBP(manipuland, mbp);
    }

    // Add environment to the world.
    AddEnvironmentToMBP(config.environment_instance_config(), mbp);

    // Now the model is complete.
    mbp->Finalize();

    return std::tie(mbp, scene_graph);
  }

  /**
   Generates a new MultibodyPlant to represent the robot described by the RobotInstanceConfig. The returned
   MultibodyPlant can be used for, e.g., getting generalized inertia of just the robot, not every body in a simulation.
   @param config a description of the robot model.
   @return a unique pointer to the new robot plant.
   */
  std::unique_ptr<typename drake::multibody::MultibodyPlant<T>> CreateStandaloneRobotPlant(
      const RobotInstanceConfig& config) {
    auto mbp = std::make_unique<typename drake::multibody::MultibodyPlant<T>>();

    // Add robot model to the new multibody plant instance.
    AddRobotToMBP(config, mbp.get());

    // Add gravity to the MultibodyPlant.
    mbp->mutable_gravity_field().set_gravity_vector(config.gravity());

    mbp->Finalize();
    return mbp;
  }

  /**
   Generates a robot model described by RobotInstanceConfig and adds it to the MultibodyPlant
   @param RobotInstanceConfig configuration class with file path to the robot description file and other paramerers for
          the robot instance.
   @param MultibodyPlant raw pointer to the universal plant, the robot model will be added to this system.
   @return the index of the newly created model instance.
   */
  drake::multibody::ModelInstanceIndex AddRobotToMBP(
      const RobotInstanceConfig& config, drake::multibody::MultibodyPlant<T>* mbp) {
    // TODO(samzapo): Implement for T = AutoDiff
    // The class `drake::multibody::Parser(mbp)` is only defined
    // for `drake::multibody::MultibodyPlant<T>*` for `T = double` not `T = AutoDiff`
    static_assert(std::is_same<T, double>::value,
                  "ModelGenerator<T>::AddRobotToMBP is only implemented for T = double");

    std::string abs_model_path = config.model_directory() + "/" + config.model_file();

    DR_DEMAND(mbp);
    drake::multibody::ModelInstanceIndex model_instance =
        drake::multibody::Parser(mbp).AddModelFromFile(abs_model_path, config.name());

    DR_DEMAND(mbp->HasBodyNamed("base", model_instance), "Robot model must have link named 'base'");

    if (!config.is_floating()) {
      PlaceAtPose("base", model_instance, config.pose(), mbp);
    }

    return model_instance;
  }

  /**
   Moves the model to the given pose in the world and then generates a weld
   joint between the world frame and the model for states that will remain
   fixed.
   @param body_name the base link of the robot or object (should be either
          "base" for the root link of a robot, or the object name for fixed
          objects).
   @param model_instance the instance index of the target model.
   @param X_WM the transform from the world frame to the intended pose of
          the model.
   @param mbp raw pointer to the MultiBodyPlant representing the world.
   */
  void PlaceAtPose(std::string body_name, const drake::multibody::ModelInstanceIndex& model_instance,
                   const drake::math::RigidTransform<double>& X_WM, drake::multibody::MultibodyPlant<T>* mbp) {
    // This adds an offset to the named body in the model instance and then
    // welds the body to world.
    const drake::multibody::Body<T>& world_body = mbp->world_body();
    const drake::multibody::Body<T>& model_body = mbp->GetBodyByName(body_name, model_instance);
    mbp->template AddJoint<drake::multibody::WeldJoint>(body_name + "_world_weld", world_body, X_WM, model_body,
                                                        drake::math::RigidTransform<double>::Identity() /* X_CJ */,
                                                        drake::math::RigidTransform<double>::Identity() /* X_JpJc */);
  }

  /**
   Generates a static plane (modeled as a half-space) into the multibody
   plant.
   @param mbp a raw pointer to the MultibodyPlant to populate with the object.
   @param name Sets the name of the model instance (must be unique).
   @param normal Sets the normal of the plane; determines the direction that
          the plane faces.
   @param center the center of the plane.
   @param surface_friction sets the static (V == 0.0) and dynamic (V != 0.0)
          Coulomb friction of the object.
   @param visual_box_dimensions (optional) the dimensions of the box to
          display instead of an infinite plane.
   */
  void AddStaticPlaneToMBP(drake::multibody::MultibodyPlant<T>* mbp, std::string name,
                           const drake::Vector3<double>& normal, const drake::Vector3<double>& center,
                           const drake::multibody::CoulombFriction<double>& surface_friction,
                           const drake::optional<drake::Vector3<double>>& visual_box_dimensions = {}) {
    DR_DEMAND(mbp);

    drake::math::RigidTransform<double> X_WG(drake::geometry::HalfSpace::MakePose(normal, center));
    X_WG.set_translation(center);

    // A half-space for the ground geometry.
    mbp->RegisterCollisionGeometry(mbp->world_body(), X_WG, drake::geometry::HalfSpace(), name + "_collision",
                                   surface_friction);

    const drake::Vector4<double> grey(0.55, 0.55, 0.55, 0.25);

    // Add visual geometry for the plane.
    if (visual_box_dimensions) {
      // Display body as a box centered at the body origin.
      const drake::Vector3<double>& box_dims = visual_box_dimensions.value();
      mbp->RegisterVisualGeometry(mbp->world_body(), X_WG, drake::geometry::Box(box_dims[0], box_dims[1], box_dims[2]),
                                  name + "visual", grey);
    } else {
      // Display body as an infinite plane.
      mbp->RegisterVisualGeometry(mbp->world_body(), X_WG, drake::geometry::HalfSpace(), name + "visual", grey);
    }
  }

  /**
   Generates a static model of a truck trailer (composed of five half-spaces)
   into the multibody plant.  The trailer-sized corridor is infinitely long in
   the -x direction and constrained by half the trailer length in the +x
   direction.
   @param config the EnvironmentInstanceConfig describing features of the
          environment.
   @param mbp a raw pointer to the MultibodyPlant to populate with the model.
   */
  void AddEnvironmentToMBP(const EnvironmentInstanceConfig& config, drake::multibody::MultibodyPlant<T>* mbp) {
    // Add gravity to the MultibodyPlant.
    mbp->mutable_gravity_field().set_gravity_vector(config.gravity());

    // Add environment bodies (fixed to world frame) to the MultibodyPlant.
    for (const auto& instance : config.body_instance_configs()) {
      AddBodyToMBP(instance, mbp);
    }

    const drake::Vector3<double> origin(0.0, 0.0, 0.0);
    if (config.is_floor_environment()) {
      AddStaticPlaneToMBP(mbp, std::move("floor"), drake::Vector3<double>(0.0, 0.0, 1.0), origin,
                          config.coulomb_friction());
    } else if (config.is_trailer_environment()) {
      const drake::Vector3<double>& trailer_size = config.trailer_size();
      const double height = trailer_size[2];
      const double length = trailer_size[0];
      const double width = trailer_size[1];
      const double kVisualTrailerWallThickness = 0.001;
      const std::string name("trailer");
      const drake::Vector3<double> side_wall_dimensions(height, length, kVisualTrailerWallThickness);
      const drake::Vector3<double> end_wall_dimensions(height, width, kVisualTrailerWallThickness);
      const drake::Vector3<double> ceiling_floor_dimensions(width, length, kVisualTrailerWallThickness);

      AddStaticPlaneToMBP(mbp, name + std::string("_floor"), drake::Vector3<double>(0.0, 0.0, 1.0),
                          origin + drake::Vector3<double>(0.0, 0.0, 0.0), config.coulomb_friction(),
                          ceiling_floor_dimensions);

      AddStaticPlaneToMBP(mbp, name + std::string("_ceiling"), drake::Vector3<double>(0.0, 0.0, -1.0),
                          origin + drake::Vector3<double>(0.0, 0.0, height), config.coulomb_friction(),
                          ceiling_floor_dimensions);

      AddStaticPlaneToMBP(mbp, name + std::string("_wall_left"), drake::Vector3<double>(0.0, -1.0, 0.0),
                          origin + drake::Vector3<double>(0.0, width / 2.0, height / 2.0), config.coulomb_friction(),
                          side_wall_dimensions);

      AddStaticPlaneToMBP(mbp, name + std::string("_wall_right"), drake::Vector3<double>(0.0, 1.0, 0.0),
                          origin + drake::Vector3<double>(0.0, -width / 2.0, height / 2.0), config.coulomb_friction(),
                          side_wall_dimensions);

      AddStaticPlaneToMBP(mbp, name + std::string("_wall_front"), drake::Vector3<double>(-1.0, 0.0, 0.0),
                          origin + drake::Vector3<double>(length / 2.0, 0.0, height / 2.0), config.coulomb_friction(),
                          end_wall_dimensions);
    } else {
      // TODO(samzapo): add SDF-based static environment.
      throw std::runtime_error("other environments not yet supported!");
    }
  }

  /**
   Generates a body and places it in the MultibodyPlant according to the
   config (e.g., a box that can be picked-up, a static box on the floor).
   @param config the BodyInstanceConfig describing the attributes of the
          dynamic object (e.g., friction, mass, geometry).
   @param mbp a raw pointer to the MultibodyPlant to populate with the model.
   */
  void AddBodyToMBP(const BodyInstanceConfig& config, drake::multibody::MultibodyPlant<T>* mbp) {
    // Set pose of the model in the world.
    const drake::math::RigidTransform<double>& X_WM = config.pose();

    drake::multibody::ModelInstanceIndex model_instance;

    // TODO(samzapo): Support "Center of mass" offset from body origin.
    ShapeToUnitInertia<double> reifier;
    config.geometry().Reify(&reifier);
    drake::multibody::SpatialInertia<double> M_Bcm(config.mass(), drake::Vector3<double>::Zero(),
                                                   reifier.unit_inertia());
    model_instance = mbp->AddModelInstance(config.name());

    const drake::multibody::RigidBody<T>& object = mbp->AddRigidBody(config.name(), model_instance, M_Bcm);

    const drake::math::RigidTransform<double> X_MP;
    // Add geometry for the object.
    // Pose of geometry S in body frame B.
    mbp->RegisterCollisionGeometry(object, X_MP, config.geometry(), config.name() + "_collision",
                                   config.coulomb_friction());

    // Add visual for the object.
    const drake::Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
    mbp->RegisterVisualGeometry(object, X_MP, config.geometry(), config.name() + "_visual", orange);

    if (!config.is_floating()) {
      PlaceAtPose(config.name(), model_instance, X_WM, mbp);
    } else {
      // initial state will be set in context after model is finalized and
      // diagram is built.
    }
  }
};

}  // namespace DR
