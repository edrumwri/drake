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
ManipulationStation<T>::ManipulationStation(double time_step)
    : owned_plant_(std::make_unique<MultibodyPlant<T>>(time_step)),
      owned_scene_graph_(std::make_unique<SceneGraph<T>>()),
      robot_model_->get_controller_plant()(std::make_unique<MultibodyPlant<T>>()) {
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
    const optional<const math::RigidTransformd>& X_WCameraBody,
    IiwaCollisionModel collision_model) {
  DRAKE_DEMAND(setup_ == Setup::kNone);
  setup_ = Setup::kClutterClearing;

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

  AddDefaultIiwa(collision_model);
  AddDefaultWsg();
}

template <typename T>
void ManipulationStation<T>::SetupDefaultStation(
    const IiwaCollisionModel collision_model) {
  DRAKE_DEMAND(setup_ == Setup::kNone);
  setup_ = Setup::kDefault;

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

  // Add the default iiwa/wsg models.
  AddDefaultIiwa(collision_model);
  AddDefaultWsg();

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
      robot_model_->GetManipulatorPositions(station_context), state);
  robot_model_->SetManipulatorVelocities(station_context,
      VectorX<T>::Zero(robot_model_->num_manipulator_joints()), state);
  robot_model_->SetGripperPositionsToDefaultOpen(station_context, state);
  robot_model_->SetGripperVelocities(station_context,
      VectorX<T>::Zero(robot_model_->num_gripper_joints()), state);
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
  robot_model_->SetManipulatorPositions(station_context,
      robot_model_->GetManipulatorPositions(station_context), state);
  robot_model_->SetManipulatorVelocities(station_context,
      VectorX<T>::Zero(robot_model_->num_manipulator_joints()), state);
  robot_model_->SetGripperPositions(station_context,
      robot_model_->GetGripperPositions(station_context), state);
  robot_model_->SetGripperVelocities(station_context,
      VectorX<T>::Zero(robot_model_->num_gripper_joints()), state);
}

/*
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

  // Finalize the robot model.
  robot_model_->Finalize(&builder);

  const int num_manipulator_positions =
      plant_->num_positions(manipulator_model_.model_instance);
  DRAKE_THROW_UNLESS(num_manipulator_positions ==
                     plant_->num_velocities(manipulator_model_.model_instance));

  // Export the commanded positions via a PassThrough.
  auto manipulator_position =
      builder.template AddSystem<systems::PassThrough>(
          num_manipulator_positions);
  builder.ExportInput(manipulator_position->get_input_port(),
                      "manipulator_position");
  builder.ExportOutput(manipulator_position->get_output_port(),
                       "manipulator_position_commanded");

  // Export manipulator "state" outputs.
  {
    auto demux = builder.template AddSystem<systems::Demultiplexer>(
        2 * num_manipulator_positions, num_manipulator_positions);
    builder.Connect(
        plant_->get_continuous_state_output_port(
            manipulator_model_.model_instance),
        demux->get_input_port(0));
    builder.ExportOutput(demux->get_output_port(0),
        "manipulator_position_measured");
    builder.ExportOutput(demux->get_output_port(1),
        "manipulator_velocity_estimated");

    builder.ExportOutput(
        plant_->get_continuous_state_output_port(
        manipulator_model_.model_instance),
        "manipulator_state_estimated");
  }

    // Add the inverse dynamics controller.
    auto manipulator_controller = builder.template AddSystem<
        systems::controllers::InverseDynamicsController>(
        *robot_model_->controller_plant(),
        robot_model_->manipulator_kp(),
        robot_model_->manipulator_ki(),
        robot_model_->manipulator_kd(), false);
    manipulator_controller->set_name("manipulator_controller");
    builder.Connect(
        plant_->get_continuous_state_output_port(manipulator_model_.model_instance),
        manipulator_controller->get_input_port_estimated_state());

    // Add in feedforward torque.
    auto adder =
        builder.template AddSystem<systems::Adder>(2, num_manipulator_positions);
    builder.Connect(manipulator_controller->get_output_port_control(),
                    adder->get_input_port(0));
    builder.ExportInput(adder->get_input_port(1),
        "manipulator_feedforward_torque");
    builder.Connect(adder->get_output_port(),
        plant_->get_actuation_input_port(manipulator_model_.model_instance));

    // Approximate desired state command from a discrete derivative of the
    // position command input port.
    auto desired_state_from_position = builder.template AddSystem<
        systems::StateInterpolatorWithDiscreteDerivative>(num_manipulator_positions,
                                                          plant_->time_step());
    desired_state_from_position->set_name("desired_state_from_position");
    builder.Connect(desired_state_from_position->get_output_port(),
                    manipulator_controller->get_input_port_desired_state());
    builder.Connect(manipulator_position->get_output_port(),
                    desired_state_from_position->get_input_port());

    // Export commanded torques:
    builder.ExportOutput(adder->get_output_port(),
        "manipulator_torque_commanded");
    builder.ExportOutput(adder->get_output_port(),
        "manipulator_torque_measured");
  }

  {
    auto wsg_controller = builder.template AddSystem<
        manipulation::schunk_wsg::SchunkWsgPositionController>(
        manipulation::schunk_wsg::kSchunkWsgLcmStatusPeriod, wsg_kp_, wsg_kd_);
    wsg_controller->set_name("wsg_controller");

    builder.Connect(
        wsg_controller->get_generalized_force_output_port(),
        plant_->get_actuation_input_port(wsg_model_.model_instance));
    builder.Connect(
        plant_->get_continuous_state_output_port(wsg_model_.model_instance),
        wsg_controller->get_state_input_port());

    builder.ExportInput(wsg_controller->get_desired_position_input_port(),
                        "wsg_position");
    builder.ExportInput(wsg_controller->get_force_limit_input_port(),
                        "wsg_force_limit");

    auto wsg_mbp_state_to_wsg_state = builder.template AddSystem(
        manipulation::schunk_wsg::MakeMultibodyStateToWsgStateSystem<double>());
    builder.Connect(
        plant_->get_continuous_state_output_port(wsg_model_.model_instance),
        wsg_mbp_state_to_wsg_state->get_input_port());

    builder.ExportOutput(wsg_mbp_state_to_wsg_state->get_output_port(),
                         "wsg_state_measured");

    builder.ExportOutput(wsg_controller->get_grip_force_output_port(),
                         "wsg_force_measured");
  }

  builder.ExportOutput(plant_->get_generalized_contact_forces_output_port(
                           manipulator_model_.model_instance),
                       "manipulator_torque_external");

  {  // RGB-D Cameras
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
*/

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
