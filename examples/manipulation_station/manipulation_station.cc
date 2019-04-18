#include "drake/examples/manipulation_station/manipulation_station.h"

#include <list>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/examples/manipulation_station/combined_manipulator_and_gripper_model.h"
#include "drake/geometry/dev/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/sensors/dev/rgbd_camera.h"

namespace drake {
namespace examples {
namespace manipulation_station {

using Eigen::Isometry3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::SceneGraph;
using math::RigidTransform;
using math::RollPitchYaw;
using math::RotationMatrix;
using multibody::Joint;
using multibody::MultibodyPlant;
using multibody::PrismaticJoint;
using multibody::RevoluteJoint;
using multibody::SpatialInertia;

namespace internal {

// TODO(edrumwri): Move this to its own file.
// Load a SDF model and weld it to the MultibodyPlant.
// @param model_path Full path to the sdf model file. i.e. with
// FindResourceOrThrow
// @param model_name Name of the added model instance.
// @param parent Frame P from the MultibodyPlant to which the new model is
// welded to.
// @param child_frame_name Defines frame C (the child frame), assumed to be
// present in the model being added.
// @param X_PC Transformation of frame C relative to frame P.
template <typename T>
multibody::ModelInstanceIndex AddAndWeldModelFrom(
    const std::string& model_path, const std::string& model_name,
    const multibody::Frame<T>& parent, const std::string& child_frame_name,
    const Isometry3<double>& X_PC, MultibodyPlant<T>* plant) {
  DRAKE_THROW_UNLESS(!plant->HasModelInstanceNamed(model_name));

  multibody::Parser parser(plant);
  const multibody::ModelInstanceIndex new_model =
      parser.AddModelFromFile(model_path, model_name);
  const auto& child_frame = plant->GetFrameByName(child_frame_name, new_model);
  plant->WeldFrames(parent, child_frame, X_PC);
  return new_model;
}

// TODO(russt): Get these from SDF instead of having them hard-coded (#10022).
void get_camera_poses(std::map<std::string, RigidTransform<double>>* pose_map) {
  pose_map->emplace("0", RigidTransform<double>(
                             RollPitchYaw<double>(1.69101, 0.176488, 0.432721),
                             Vector3d(-0.233066, -0.451461, 0.466761)));

  pose_map->emplace("1", RigidTransform<double>(
                             RollPitchYaw<double>(-1.68974, 0.20245, -0.706783),
                             Vector3d(-0.197236, 0.468471, 0.436499)));

  pose_map->emplace("2", RigidTransform<double>(
                             RollPitchYaw<double>(0.0438918, 1.03776, -3.13612),
                             Vector3d(0.786905, -0.0284378, 1.04287)));
}

}  // namespace internal

template <typename T>
ManipulationStation<T>::ManipulationStation(
    std::unique_ptr<MultibodyPlant<T>> plant,
    std::unique_ptr<CombinedManipulatorAndGripperModel<T>> robot_model)
    : owned_plant_(std::move(plant)),
      owned_scene_graph_(std::make_unique<SceneGraph<T>>()),
      robot_model_(std::move(robot_model)) {
  // This class holds the unique_ptrs explicitly for plant and scene_graph
  // until Finalize() is called (when they are moved into the Diagram). Grab
  // the raw pointers, which should stay valid for the lifetime of the Diagram.
  plant_ = owned_plant_.get();
  scene_graph_ = owned_scene_graph_.get();
  plant_->RegisterAsSourceForSceneGraph(scene_graph_);
  scene_graph_->set_name("scene_graph");

  plant_->template AddForceElement<multibody::UniformGravityFieldElement>();
  plant_->set_name("plant");

  this->set_name("manipulation_station");
}

template <typename T>
void ManipulationStation<T>::AddManipulandFromFile(
    const std::string& model_file, const RigidTransform<double>& X_WObject) {
  multibody::Parser parser(plant_);
  const auto model_index =
      parser.AddModelFromFile(FindResourceOrThrow(model_file));
  const auto indices = plant_->GetBodyIndices(model_index);
  // Only support single-body objects for now.
  // Note: this could be generalized fairly easily... would just want to
  // set default/random positions for the non-floating-base elements below.
  DRAKE_DEMAND(indices.size() == 1);
  object_ids_.push_back(indices[0]);

  object_poses_.push_back(X_WObject);
}

template <typename T>
void ManipulationStation<T>::SetupClutterClearingStation(
    const optional<const math::RigidTransformd>& X_WCameraBody) {
  DRAKE_DEMAND(setup_ == ManipulationStationSetup::kNone);
  setup_ = ManipulationStationSetup::kClutterClearing;

  // Add the bins.
  {
    const std::string sdf_path = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/bin.sdf");

    Isometry3<double> X_WC =
        RigidTransform<double>(RotationMatrix<double>::MakeZRotation(M_PI_2),
                               Vector3d(-0.145, -0.63, 0.235))
            .GetAsIsometry3();
    internal::AddAndWeldModelFrom(sdf_path, "bin1", plant_->world_frame(),
                                  "bin_base", X_WC, plant_);

    X_WC = RigidTransform<double>(RotationMatrix<double>::MakeZRotation(M_PI),
                                  Vector3d(0.5, -0.1, 0.235))
               .GetAsIsometry3();
    internal::AddAndWeldModelFrom(sdf_path, "bin2", plant_->world_frame(),
                                  "bin_base", X_WC, plant_);
  }

  // Add the camera.
  {
    // Typical D415 intrinsics for 848 x 480 resolution, note that rgb and
    // depth are slightly different. And we are not able to model that at the
    // moment.
    // RGB:
    // - w: 848, h: 480, fx: 616.285, fy: 615.778, ppx: 405.418, ppy: 232.864
    // DEPTH:
    // - w: 848, h: 480, fx: 645.138, fy: 645.138, ppx: 420.789, ppy: 239.13
    // For this camera, we are going to assume that fx = fy, and we can compute
    // fov_y by: fy = height / 2 / tan(fov_y / 2)
    const double kFocalY = 645.;
    const int kHeight = 480;
    const int kWidth = 848;
    const double fov_y = std::atan(kHeight / 2. / kFocalY) * 2;
    geometry::dev::render::DepthCameraProperties camera_properties(
        kWidth, kHeight, fov_y, geometry::dev::render::Fidelity::kLow, 0.1,
        2.0);

    RegisterRgbdCamera("0", plant_->world_frame(),
                       X_WCameraBody.value_or(math::RigidTransformd(
                           math::RollPitchYaw<double>(-0.3, 0.8, 1.5),
                           Eigen::Vector3d(0, -1.5, 1.5))),
                       camera_properties);
  }

  robot_model_->AddRobotModelToMultibodyPlant();
}

template <typename T>
void ManipulationStation<T>::SetupDefaultStation() {
  DRAKE_DEMAND(setup_ == ManipulationStationSetup::kNone);
  setup_ = ManipulationStationSetup::kDefault;

  // Add the table and 80/20 workcell frame.
  {
    const double dx_table_center_to_robot_base = 0.3257;
    const double dz_table_top_robot_base = 0.0127;
    const std::string sdf_path = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/"
        "amazon_table_simplified.sdf");

    const Isometry3<double> X_WT =
        RigidTransform<double>(Vector3d(dx_table_center_to_robot_base, 0,
                                        -dz_table_top_robot_base))
            .GetAsIsometry3();
    internal::AddAndWeldModelFrom(sdf_path, "table", plant_->world_frame(),
                                  "amazon_table", X_WT, plant_);
  }

  // Add the cupboard.
  {
    const double dx_table_center_to_robot_base = 0.3257;
    const double dz_table_top_robot_base = 0.0127;
    const double dx_cupboard_to_table_center = 0.43 + 0.15;
    const double dz_cupboard_to_table_center = 0.02;
    const double cupboard_height = 0.815;

    const std::string sdf_path = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/cupboard.sdf");

    const Isometry3<double> X_WC =
        RigidTransform<double>(
            RotationMatrix<double>::MakeZRotation(M_PI),
            Vector3d(
                dx_table_center_to_robot_base + dx_cupboard_to_table_center, 0,
                dz_cupboard_to_table_center + cupboard_height / 2.0 -
                    dz_table_top_robot_base))
            .GetAsIsometry3();
    internal::AddAndWeldModelFrom(sdf_path, "cupboard", plant_->world_frame(),
                                  "cupboard_body", X_WC, plant_);
  }

  // Add the object.
  {
    multibody::Parser parser(plant_);
    const auto model_index = parser.AddModelFromFile(FindResourceOrThrow(
        "drake/examples/manipulation_station/models/061_foam_brick.sdf"));
    const auto indices = plant_->GetBodyIndices(model_index);
    DRAKE_DEMAND(indices.size() == 1);
    object_ids_.push_back(indices[0]);

    RigidTransform<T> X_WObject;
    X_WObject.set_translation(Eigen::Vector3d(0.6, 0, 0));
    X_WObject.set_rotation(RotationMatrix<T>::Identity());
    object_poses_.push_back(X_WObject);
  }

  robot_model_->AddRobotModelToMultibodyPlant();

  // Add default cameras.
  {
    std::map<std::string, RigidTransform<double>> camera_poses;
    internal::get_camera_poses(&camera_poses);
    // Typical D415 intrinsics for 848 x 480 resolution, note that rgb and
    // depth are slightly different. And we are not able to model that at the
    // moment.
    // RGB:
    // - w: 848, h: 480, fx: 616.285, fy: 615.778, ppx: 405.418, ppy: 232.864
    // DEPTH:
    // - w: 848, h: 480, fx: 645.138, fy: 645.138, ppx: 420.789, ppy: 239.13
    // For this camera, we are going to assume that fx = fy, and we can compute
    // fov_y by: fy = height / 2 / tan(fov_y / 2)
    const double kFocalY = 645.;
    const int kHeight = 480;
    const int kWidth = 848;
    const double fov_y = std::atan(kHeight / 2. / kFocalY) * 2;
    geometry::dev::render::DepthCameraProperties camera_properties(
        kWidth, kHeight, fov_y, geometry::dev::render::Fidelity::kLow, 0.1,
        2.0);
    for (const auto& camera_pair : camera_poses) {
      RegisterRgbdCamera(camera_pair.first, plant_->world_frame(),
                         camera_pair.second, camera_properties);
    }
  }
}

template <typename T>
void ManipulationStation<T>::SetDefaultState(
    const systems::Context<T>& station_context,
    systems::State<T>* state) const {
  // Call the base class method, to initialize all systems in this diagram.
  systems::Diagram<T>::SetDefaultState(station_context, state);

  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);

  DRAKE_DEMAND(object_ids_.size() == object_poses_.size());

  for (uint64_t i = 0; i < object_ids_.size(); i++) {
    plant_->SetFreeBodyPose(plant_context, &plant_state,
                            plant_->get_body(object_ids_[i]),
                            object_poses_[i].GetAsIsometry3());
  }

  // TODO: Make sure the controller state is initialized to the robot state.
  robot_model_->SetManipulatorPositions(station_context,
      robot_model_->GetManipulatorPositions(station_context, *this),
      *this, state);
  robot_model_->SetManipulatorVelocities(station_context,
      VectorX<T>::Zero(robot_model_->num_manipulator_joints()), *this, state);
  robot_model_->SetGripperPositionsToDefaultOpen(station_context, *this, state);
  robot_model_->SetGripperVelocities(station_context,
      VectorX<T>::Zero(robot_model_->num_gripper_joints()), *this, state);
}

template <typename T>
void ManipulationStation<T>::SetRandomState(
    const systems::Context<T>& station_context, systems::State<T>* state,
    RandomGenerator* generator) const {
  // Call the base class method, to initialize all systems in this diagram.
  systems::Diagram<T>::SetRandomState(station_context, state, generator);

  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);

  // Separate the objects by lifting them up in z (in a random order).
  // TODO(russt): Replace this with an explicit projection into a statically
  // stable configuration.
  std::vector<multibody::BodyIndex> shuffled_object_ids(object_ids_);
  std::shuffle(shuffled_object_ids.begin(), shuffled_object_ids.end(),
               *generator);
  double z_offset = 0.1;
  for (const auto body_index : shuffled_object_ids) {
    math::RigidTransform<T> pose =
        plant_->GetFreeBodyPose(plant_context, plant_->get_body(body_index));
    pose.set_translation(pose.translation() + Vector3d{0, 0, z_offset});
    z_offset += 0.1;
    plant_->SetFreeBodyPose(plant_context, &plant_state,
                            plant_->get_body(body_index),
                            pose.GetAsIsometry3());
  }

  // TODO: Make sure the controller state is initialized to the robot state.
  // Use SetIiwaPosition to make sure the controller state is initialized to
  // the IIWA state.
/*
  robot_model_->SetManipulatorPositions(station_context,
      robot_model_->GetManipulatorPositions(station_context), state);
  robot_model_->SetManipulatorVelocities(station_context,
      VectorX<T>::Zero(robot_model_->num_manipulator_joints()), state);
  robot_model_->SetGripperPositions(station_context,
      robot_model_->GetGripperPositions(station_context), state);
  robot_model_->SetGripperVelocities(station_context,
      VectorX<T>::Zero(robot_model_->num_gripper_joints()), state);
*/
}

template <typename T>
const multibody::MultibodyPlant<T>&
ManipulationStation<T>::get_controller_plant() const {
  DRAKE_DEMAND(robot_model_.get());
  return robot_model_->get_controller_plant();
}

/*
template <typename T>
T ManipulationStation<T>::GetWsgPosition(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);

  Vector2<T> positions =
      plant_->GetPositions(plant_context, wsg_model_.model_instance);
  return positions(1) - positions(0);
}

template <typename T>
T ManipulationStation<T>::GetWsgVelocity(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);

  Vector2<T> velocities =
      plant_->GetVelocities(plant_context, wsg_model_.model_instance);
  return velocities(1) - velocities(0);
}

template <typename T>
void ManipulationStation<T>::SetGripperPosition(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const T& q) const {
  DRAKE_DEMAND(state != nullptr);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);
  robot_model_->SetPositions(plant_context, &plant_state,
      wsg_model_.model_instance, positions);

  // Set the position history in the state interpolator to match.
  const auto& wsg_controller = dynamic_cast<
      const manipulation::schunk_wsg::SchunkWsgPositionController&>(
      this->GetSubsystemByName("wsg_controller"));
  wsg_controller.set_initial_position(
      &this->GetMutableSubsystemState(wsg_controller, state), q);
}

template <typename T>
void ManipulationStation<T>::SetGripperVelocity(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const T& v) const {
  DRAKE_DEMAND(state != nullptr);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);
  robot_model_->SetGripperVelocity(plant_context, &plant_state, v);
}
*/

template <typename T>
void ManipulationStation<T>::Finalize() {
  // Note: This deferred diagram construction method/workflow exists because we
  //   - cannot finalize plant until all of my objects are added, and
  //   - cannot wire up my diagram until we have finalized the plant.
  plant_->Finalize();

  systems::DiagramBuilder<T> builder;

  builder.AddSystem(std::move(owned_plant_));
  builder.AddSystem(std::move(owned_scene_graph_));

  builder.Connect(
      plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(plant_->get_source_id().value()));
  builder.Connect(scene_graph_->get_query_output_port(),
                  plant_->get_geometry_query_input_port());

  // Build the control diagram for the manipulator+gripper model.
  robot_model_->BuildControlDiagram(&builder);

  // Add the RBG-D cameras.
  {
    render_scene_graph_ =
        builder.template AddSystem<geometry::dev::SceneGraph>(*scene_graph_);
    render_scene_graph_->set_name("dev_scene_graph_for_rendering");

    builder.Connect(plant_->get_geometry_poses_output_port(),
                    render_scene_graph_->get_source_pose_port(
                        plant_->get_source_id().value()));

    for (const auto& info_pair : camera_information_) {
      std::string camera_name = "camera_" + info_pair.first;
      const CameraInformation& info = info_pair.second;

      const optional<geometry::FrameId> parent_body_id =
          plant_->GetBodyFrameIdIfExists(info.parent_frame->body().index());
      DRAKE_THROW_UNLESS(parent_body_id.has_value());
      const Isometry3<double> X_PC =
          info.parent_frame->GetFixedPoseInBodyFrame() *
          info.X_PC.GetAsIsometry3();

      auto camera =
          builder.template AddSystem<systems::sensors::dev::RgbdCamera>(
              camera_name, parent_body_id.value(), X_PC, info.properties,
              false);
      builder.Connect(render_scene_graph_->get_query_output_port(),
                      camera->query_object_input_port());

      builder.ExportOutput(camera->color_image_output_port(),
                           camera_name + "_rgb_image");
      builder.ExportOutput(camera->GetOutputPort("depth_image_16u"),
                           camera_name + "_depth_image");
      builder.ExportOutput(camera->label_image_output_port(),
                           camera_name + "_label_image");
    }
  }

  builder.ExportOutput(scene_graph_->get_pose_bundle_output_port(),
                       "pose_bundle");

  builder.ExportOutput(plant_->get_contact_results_output_port(),
                       "contact_results");
  builder.ExportOutput(plant_->get_continuous_state_output_port(),
                       "plant_continuous_state");
  builder.ExportOutput(plant_->get_geometry_poses_output_port(),
                       "geometry_poses");

  builder.BuildInto(this);
}

template <typename T>
std::vector<std::string> ManipulationStation<T>::get_camera_names() const {
  std::vector<std::string> names;
  names.reserve(camera_information_.size());
  for (const auto& info : camera_information_) {
    names.emplace_back(info.first);
  }
  return names;
}

template <typename T>
void ManipulationStation<T>::RegisterRgbdCamera(
    const std::string& name, const multibody::Frame<T>& parent_frame,
    const RigidTransform<double>& X_PC,
    const geometry::dev::render::DepthCameraProperties& properties) {
  CameraInformation info;
  info.parent_frame = &parent_frame;
  info.X_PC = X_PC;
  info.properties = properties;

  camera_information_[name] = info;
}

template <typename T>
std::map<std::string, RigidTransform<double>>
ManipulationStation<T>::GetStaticCameraPosesInWorld() const {
  std::map<std::string, RigidTransform<double>> static_camera_poses;

  for (const auto& info : camera_information_) {
    const auto& frame_P = *info.second.parent_frame;

    // TODO(siyuan.feng@tri.global): We really only just need to make sure
    // the parent frame is a AnchoredFrame(i.e. there is a rigid kinematic path
    // from it to the world). However, the computation to query X_WP given a
    // partially constructed plant is not feasible at the moment, so we are
    // looking for cameras that are directly attached to the world instead.
    const bool is_anchored =
        frame_P.body().index() == plant_->world_frame().body().index();
    if (is_anchored) {
      static_camera_poses.emplace(
          info.first,
          RigidTransform<double>(frame_P.GetFixedPoseInBodyFrame()) *
              info.second.X_PC);
    }
  }

  return static_camera_poses;
}

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

// TODO(russt): Support at least NONSYMBOLIC_SCALARS.  See #9573.
//   (and don't forget to include default_scalars.h)
template class ::drake::examples::manipulation_station::ManipulationStation<
    double>;
