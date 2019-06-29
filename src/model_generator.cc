#include <PCESystems/model_generator.h>
#include <PCESystems/shape_to_unit_inertia.h>

namespace DR {

using drake::geometry::Box;
using drake::geometry::HalfSpace;
using drake::math::RigidTransform;
using drake::multibody::Body;
using drake::multibody::CoulombFriction;
using drake::multibody::FixedOffsetFrame;
using drake::multibody::Frame;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::Parser;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::multibody::WeldJoint;

drake::multibody::MultibodyPlant<double>* ModelGenerator::CreateSceneAndRobot(
    const UnloadingTaskConfig& config,
    drake::systems::DiagramBuilder<double>* builder) {
  config.ValidateConfig();

  auto items = drake::multibody::AddMultibodyPlantSceneGraph(builder);
  drake::multibody::MultibodyPlant<double>* mbp = &(items.plant);

  // Add Manipulands to the scene.
  for (const auto& instance : config.manipuland_instance_config()) {
    AddBodyToMBP(instance, mbp);
  }

  // Add environment to the scene.
  AddEnvironmentToMBP(config.environment_instance_config(), mbp);

  // Now the model is complete.
  mbp->Finalize();

  return mbp;
}

void ModelGenerator::PlaceAtPose(
    std::string body_name, const ModelInstanceIndex& model_instance,
    const drake::math::RigidTransform<double>& X_WM,
    drake::multibody::MultibodyPlant<double>* mbp) {
  // Behavior same as:
  //    MultibodyPlant::WeldFrames(
  //      mbp->GetBodyByName(body_name,model_instance).body_frame(),
  //      mbp->world_frame(),
  //      X_WM
  //    );
  // This adds an offset to the named body in the model instance and then welds
  // the body to world.
  const drake::multibody::Body<double>& world_body = mbp->world_body();
  const drake::multibody::Body<double>& model_body =
      mbp->GetBodyByName(body_name, model_instance);
  mbp->AddJoint<drake::multibody::WeldJoint>(
      body_name + "_world_weld", world_body, X_WM, model_body,
      drake::math::RigidTransform<double>::Identity() /* X_CJ */,
      drake::math::RigidTransform<double>::Identity() /* X_JpJc */);
}

void ModelGenerator::AddStaticPlaneToMBP(
    drake::multibody::MultibodyPlant<double>* mbp, std::string name,
    const drake::Vector3<double>& normal, const drake::Vector3<double>& center,
    const CoulombFriction<double>& surface_friction,
    const drake::optional<drake::Vector3<double>>& visual_box_dimensions) {
  DRAKE_DEMAND(mbp);

  RigidTransform<double> X_WG(HalfSpace::MakePose(normal, center));
  X_WG.set_translation(center);

  // A half-space for the ground geometry.
  mbp->RegisterCollisionGeometry(mbp->world_body(), X_WG, HalfSpace(),
                                 name + "_collision", surface_friction);

  const drake::Vector4<double> grey(0.55, 0.55, 0.55, 0.25);

  // Add visual geometry for the plane.
  if (visual_box_dimensions) {
    // Display body as a box centered at the body origin.
    const drake::Vector3<double>& box_dims = visual_box_dimensions.value();
    mbp->RegisterVisualGeometry(mbp->world_body(), X_WG,
                                Box(box_dims[0], box_dims[1], box_dims[2]),
                                name + "visual", grey);
  } else {
    // Display body as an infinite plane.
    mbp->RegisterVisualGeometry(mbp->world_body(), X_WG, HalfSpace(),
                                name + "visual", grey);
  }
}

void ModelGenerator::AddEnvironmentToMBP(
    const EnvironmentInstanceConfig& config,
    drake::multibody::MultibodyPlant<double>* mbp) {
  // Add gravity to the MultibodyPlant.
  mbp->mutable_gravity_field().set_gravity_vector(config.gravity());

  // Add environment bodies (fixed to world frame) to the MultibodyPlant.
  for (const auto& instance : config.GetBodyInstanceConfigs()) {
    AddBodyToMBP(instance, mbp);
  }

  const drake::Vector3<double> origin(0.0, 0.0, 0.0);
  if (config.IsFloorEnvironment()) {
    AddStaticPlaneToMBP(mbp, std::move("floor"),
                        drake::Vector3<double>(0.0, 0.0, 1.0), origin,
                        config.coulomb_friction());
  } else if (config.IsTrailerEnvironment()) {
    const drake::Vector3<double>& trailer_size = config.trailer_size();
    const double height = trailer_size[2];
    const double length = trailer_size[0];
    const double width = trailer_size[1];
    const double kVisualTrailerWallThickness = 0.001;
    const std::string name("trailer");
    const drake::Vector3<double> side_wall_dimensions(
        height, length, kVisualTrailerWallThickness);
    const drake::Vector3<double> end_wall_dimensions(
        height, width, kVisualTrailerWallThickness);
    const drake::Vector3<double> ceiling_floor_dimensions(
        width, length, kVisualTrailerWallThickness);

    AddStaticPlaneToMBP(mbp, name + std::string("_floor"),
                        drake::Vector3<double>(0.0, 0.0, 1.0),
                        origin + drake::Vector3<double>(0.0, 0.0, 0.0),
                        config.coulomb_friction(), ceiling_floor_dimensions);

    AddStaticPlaneToMBP(mbp, name + std::string("_ceiling"),
                        drake::Vector3<double>(0.0, 0.0, -1.0),
                        origin + drake::Vector3<double>(0.0, 0.0, height),
                        config.coulomb_friction(), ceiling_floor_dimensions);

    AddStaticPlaneToMBP(
        mbp, name + std::string("_wall_left"),
        drake::Vector3<double>(0.0, -1.0, 0.0),
        origin + drake::Vector3<double>(0.0, width / 2.0, height / 2.0),
        config.coulomb_friction(), side_wall_dimensions);

    AddStaticPlaneToMBP(
        mbp, name + std::string("_wall_right"),
        drake::Vector3<double>(0.0, 1.0, 0.0),
        origin + drake::Vector3<double>(0.0, -width / 2.0, height / 2.0),
        config.coulomb_friction(), side_wall_dimensions);

    AddStaticPlaneToMBP(
        mbp, name + std::string("_wall_front"),
        drake::Vector3<double>(-1.0, 0.0, 0.0),
        origin + drake::Vector3<double>(length / 2.0, 0.0, height / 2.0),
        config.coulomb_friction(), end_wall_dimensions);
  } else {
    // TODO(samzapo): add SDF-based static environment.
    throw std::runtime_error("other environments not yet supported!");
  }
}
void ModelGenerator::AddBodyToMBP(
    const BodyInstanceConfig& config,
    drake::multibody::MultibodyPlant<double>* mbp) {
  // Set pose of the model in the world.
  const RigidTransform<double>& X_WM = config.pose();

  ModelInstanceIndex model_instance;

  // TODO(samzapo): Support "Center of mass" offset from body origin.
  ShapeToUnitInertia<double> reifier;
  config.geometry().Reify(&reifier);
  SpatialInertia<double> M_Bcm(config.mass(), drake::Vector3<double>::Zero(),
                               reifier.unit_inertia());
  model_instance = mbp->AddModelInstance(config.name());

  const RigidBody<double>& object =
      mbp->AddRigidBody(config.name(), model_instance, M_Bcm);

  const RigidTransform<double> X_MP;
  // Add geometry for the object.
  // Pose of geometry S in body frame B.
  mbp->RegisterCollisionGeometry(object, X_MP, config.geometry(),
                                 config.name() + "_collision",
                                 config.coulomb_friction());

  // Add visual for the object.
  const drake::Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
  mbp->RegisterVisualGeometry(object, X_MP, config.geometry(),
                              config.name() + "_visual", orange);

  if (!config.is_floating()) {
    PlaceAtPose(config.name(), model_instance, X_WM, mbp);
  } else {
    // initial state will be set in context after model is finalized and
    // diagram is built.
  }
}

}  // namespace DR
