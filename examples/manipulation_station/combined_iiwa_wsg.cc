#include "drake/examples/manipulation_station/combined_iiwa_wsg.h"

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/dev/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/sensors/dev/rgbd_camera.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using drake::VectorX;
using drake::examples::manipulation_station::CombinedIiwaWsg;
using drake::math::RollPitchYaw;
using drake::math::RigidTransform;
using drake::multibody::Body;
using drake::multibody::Frame;
using drake::multibody::Joint;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::systems::Adder;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::Demultiplexer;
using drake::systems::PassThrough;
using drake::systems::State;
using drake::systems::StateInterpolatorWithDiscreteDerivative;

namespace internal {

// TODO(amcastro-tri): Refactor this into schunk_wsg directory, and cover it
// with a unit test.  Potentially tighten the tolerance in
// station_simulation_test.
// @param gripper_body_frame_name Name of a frame that's attached to the
// gripper's main body.
SpatialInertia<double> MakeCompositeGripperInertia(
    const std::string& wsg_sdf_path,
    const std::string& gripper_body_frame_name) {
  MultibodyPlant<double> plant;
  Parser parser(&plant);
  parser.AddModelFromFile(wsg_sdf_path);
  plant.Finalize();
  const auto& frame = plant.GetFrameByName(gripper_body_frame_name);
  const auto& gripper_body = plant.GetRigidBodyByName(frame.body().name());
  const auto& left_finger = plant.GetRigidBodyByName("left_finger");
  const auto& right_finger = plant.GetRigidBodyByName("right_finger");
  const auto& left_slider = plant.GetJointByName("left_finger_sliding_joint");
  const auto& right_slider = plant.GetJointByName("right_finger_sliding_joint");
  const SpatialInertia<double>& M_GGo_G =
      gripper_body.default_spatial_inertia();
  const SpatialInertia<double>& M_LLo_L = left_finger.default_spatial_inertia();
  const SpatialInertia<double>& M_RRo_R =
      right_finger.default_spatial_inertia();
  auto CalcFingerPoseInGripperFrame = [](const Joint<double>& slider) {
    // Pose of the joint's parent frame P (attached on gripper body G) in the
    // frame of the gripper G.
    const RigidTransform<double> X_GP(
        slider.frame_on_parent().GetFixedPoseInBodyFrame());
    // Pose of the joint's child frame C (attached on the slider's finger body)
    // in the frame of the slider's finger F.
    const RigidTransform<double> X_FC(
        slider.frame_on_child().GetFixedPoseInBodyFrame());
    // When the slider's translational dof is zero, then P coincides with C.
    // Therefore:
    const RigidTransform<double> X_GF = X_GP * X_FC.inverse();
    return X_GF;
  };
  // Pose of left finger L in gripper frame G when the slider's dof is zero.
  const RigidTransform<double> X_GL(CalcFingerPoseInGripperFrame(left_slider));
  // Pose of right finger R in gripper frame G when the slider's dof is zero.
  const RigidTransform<double> X_GR(CalcFingerPoseInGripperFrame(right_slider));
  // Helper to compute the spatial inertia of a finger F in about the gripper's
  // origin Go, expressed in G.
  auto CalcFingerSpatialInertiaInGripperFrame =
      [](const SpatialInertia<double>& M_FFo_F,
         const RigidTransform<double>& X_GF) {
        const auto M_FFo_G = M_FFo_F.ReExpress(X_GF.rotation());
        const auto p_FoGo_G = -X_GF.translation();
        const auto M_FGo_G = M_FFo_G.Shift(p_FoGo_G);
        return M_FGo_G;
      };
  // Shift and re-express in G frame the finger's spatial inertias.
  const auto M_LGo_G = CalcFingerSpatialInertiaInGripperFrame(M_LLo_L, X_GL);
  const auto M_RGo_G = CalcFingerSpatialInertiaInGripperFrame(M_RRo_R, X_GR);
  // With everything about the same point Go and expressed in the same frame G,
  // proceed to compose into composite body C:
  // TODO(amcastro-tri): Implement operator+() in SpatialInertia.
  SpatialInertia<double> M_CGo_G = M_GGo_G;
  M_CGo_G += M_LGo_G;
  M_CGo_G += M_RGo_G;
  return M_CGo_G;
}

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
ModelInstanceIndex AddAndWeldModelFrom(
    const std::string& model_path, const std::string& model_name,
    const Frame<T>& parent, const std::string& child_frame_name,
    const RigidTransform<double>& X_PC, MultibodyPlant<T>* plant) {
  DRAKE_THROW_UNLESS(!plant->HasModelInstanceNamed(model_name));

  Parser parser(plant);
  const ModelInstanceIndex new_model =
      parser.AddModelFromFile(model_path, model_name);
  const auto& child_frame = plant->GetFrameByName(child_frame_name, new_model);
  plant->WeldFrames(parent, child_frame, X_PC);
  return new_model;
}

}  // namespace internal

namespace drake {
namespace examples {
namespace manipulation_station {

template <typename T>
void CombinedIiwaWsg<T>::MakeIiwaControllerModel() {
  // Build the controller's version of the plant, which only contains the
  // IIWA and the equivalent inertia of the gripper.
  Parser parser(&controller_plant_);
  const auto controller_iiwa_model =
      parser.AddModelFromFile(iiwa_model_.model_path, "iiwa");

  controller_plant_.WeldFrames(
      controller_plant_.world_frame(),
      controller_plant_.GetFrameByName(iiwa_model_.child_frame->name(),
                                              controller_iiwa_model),
      iiwa_model_.X_PC);
  // Add a single body to represent the IIWA pendant's calibration of the
  // gripper.  The body of the WSG accounts for >90% of the total mass
  // (according to the sdf)... and we don't believe our inertia calibration
  // on the hardware to be so precise, so we simply ignore the inertia
  // contribution from the fingers here.
  const RigidBody<T>& wsg_equivalent =
      controller_plant_.AddRigidBody(
          "wsg_equivalent", controller_iiwa_model,
          ::internal::MakeCompositeGripperInertia(
              wsg_model_.model_path, wsg_model_.child_frame->name()));

  // TODO(siyuan.feng@tri.global): when we handle multiple IIWA and WSG, this
  // part need to deal with the parent's (iiwa's) model instance id.
  controller_plant_.WeldFrames(
      controller_plant_.GetFrameByName(wsg_model_.parent_frame->name(),
                                              controller_iiwa_model),
      wsg_equivalent.body_frame(), wsg_model_.X_PC);

  controller_plant_.template AddForceElement<UniformGravityFieldElement>();
  controller_plant_.set_name("controller_plant");
}

// Add default iiwa.
template <typename T>
void CombinedIiwaWsg<T>::AddRobotModelToMultibodyPlant() {
  // First add the Iiwa model.
  std::string iiwa_sdf_path;
  switch (collision_model_) {
    case IiwaCollisionModel::kNoCollision:
      iiwa_sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/iiwa_description/iiwa7/"
          "iiwa7_no_collision.sdf");
      break;
    case IiwaCollisionModel::kBoxCollision:
      iiwa_sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/iiwa_description/iiwa7/"
          "iiwa7_with_box_collision.sdf");
      break;
    default:
      throw std::domain_error("Unrecognized collision_model.");
  }
  const auto X_WI = RigidTransform<double>::Identity();
  auto iiwa_instance = ::internal::AddAndWeldModelFrom(
      iiwa_sdf_path, "iiwa", this->plant_->world_frame(), "iiwa_link_0",
      X_WI, this->plant_);
  RegisterIiwaControllerModel(
      iiwa_sdf_path, iiwa_instance, this->plant_->world_frame(),
      this->plant_->GetFrameByName("iiwa_link_0", iiwa_instance), X_WI);

  const std::string wsg_sdf_path = FindResourceOrThrow(
      "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf");
  const Frame<T>& link7 =
      this->plant_->GetFrameByName("iiwa_link_7", iiwa_model_.model_instance);
  const RigidTransform<double> X_7G(RollPitchYaw<double>(M_PI_2, 0, M_PI_2),
                                    Vector3d(0, 0, 0.114));
  auto wsg_instance = ::internal::AddAndWeldModelFrom(
      wsg_sdf_path, "gripper", link7, "body", X_7G, this->plant_);
  RegisterWsgControllerModel(wsg_sdf_path, wsg_instance, link7,
                             this->plant_->GetFrameByName("body", wsg_instance),
                             X_7G);
}

template <typename T>
void CombinedIiwaWsg<T>::RegisterIiwaControllerModel(
    const std::string& model_path,
    const ModelInstanceIndex iiwa_instance,
    const Frame<T>& parent_frame,
    const Frame<T>& child_frame,
    const RigidTransform<double>& X_PC) {
  // TODO(siyuan.feng@tri.global): We really only just need to make sure
  // the parent frame is a AnchoredFrame(i.e. there is a rigid kinematic path
  // from it to the world), and record that X_WP. However, the computation to
  // query X_WP given a partially constructed plant is not feasible at the
  // moment, so we are forcing the parent frame to be the world instead.
  DRAKE_THROW_UNLESS(parent_frame.name() == this->plant_->world_frame().name());

  iiwa_model_.model_path = model_path;
  iiwa_model_.parent_frame = &parent_frame;
  iiwa_model_.child_frame = &child_frame;
  iiwa_model_.X_PC = X_PC;

  iiwa_model_.model_instance = iiwa_instance;
}

template <typename T>
void CombinedIiwaWsg<T>::RegisterWsgControllerModel(
    const std::string& model_path,
    const ModelInstanceIndex wsg_instance,
    const Frame<T>& parent_frame,
    const Frame<T>& child_frame,
    const RigidTransform<double>& X_PC) {
  wsg_model_.model_path = model_path;
  wsg_model_.parent_frame = &parent_frame;
  wsg_model_.child_frame = &child_frame;
  wsg_model_.X_PC = X_PC;

  wsg_model_.model_instance = wsg_instance;
}

template <typename T>
VectorX<T> CombinedIiwaWsg<T>::GetManipulatorPositions(
    const Context<T>& diagram_context, const Diagram<T>& diagram) const {
  const auto& plant_context =
      diagram.GetSubsystemContext(*this->plant_, diagram_context);
  return this->plant_->GetPositions(plant_context, iiwa_model_.model_instance);
}

template <typename T>
void CombinedIiwaWsg<T>::SetManipulatorPositions(
    const Context<T>& diagram_context,
    const Eigen::Ref<const VectorX<T>>& q,
    const Diagram<T>& diagram,
    State<T>* diagram_state) const {
  const int num_iiwa_positions =
      this->plant_->num_positions(iiwa_model_.model_instance);
  DRAKE_DEMAND(diagram_state != nullptr);
  DRAKE_DEMAND(q.size() == num_iiwa_positions);
  auto& plant_context = diagram.GetSubsystemContext(
      *this->plant_, diagram_context);
  auto& plant_state = diagram.GetMutableSubsystemState(
      *this->plant_, diagram_state);
  this->plant_->SetPositions(plant_context, &plant_state,
      iiwa_model_.model_instance, q);

  // Set the position history in the state interpolator to match.
  const auto& state_from_position = dynamic_cast<
      const StateInterpolatorWithDiscreteDerivative<double>&>(
      diagram.GetSubsystemByName("desired_state_from_position"));
  state_from_position.set_initial_position(
      &diagram.GetMutableSubsystemState(state_from_position, diagram_state), q);
}

template <typename T>
VectorX<T> CombinedIiwaWsg<T>::GetManipulatorVelocities(
    const Context<T>& diagram_context, const Diagram<T>& diagram) const {
  const auto& plant_context =
      diagram.GetSubsystemContext(*this->plant_, diagram_context);
  return this->plant_->GetVelocities(plant_context, iiwa_model_.model_instance);
}

template <typename T>
void CombinedIiwaWsg<T>::SetManipulatorVelocities(
    const Context<T>& diagram_context,
    const Eigen::Ref<const VectorX<T>>& v,
    const Diagram<T>& diagram,
    State<T>* diagram_state) const {
  const int num_iiwa_velocities =
      this->plant_->num_velocities(iiwa_model_.model_instance);
  DRAKE_DEMAND(diagram_state != nullptr);
  DRAKE_DEMAND(v.size() == num_iiwa_velocities);
  auto& plant_context = diagram.GetSubsystemContext(
      *this->plant_, diagram_context);
  auto& plant_state = diagram.GetMutableSubsystemState(
      *this->plant_, diagram_state);
  this->plant_->SetVelocities(plant_context, &plant_state,
      iiwa_model_.model_instance, v);
}

template <typename T>
VectorX<T> CombinedIiwaWsg<T>::GetGripperPositions(
    const Context<T>& diagram_context, const Diagram<T>& diagram) const {
  const auto& plant_context =
      diagram.GetSubsystemContext(*this->plant_, diagram_context);

  Vector2<T> positions =
      this->plant_->GetPositions(plant_context, wsg_model_.model_instance);
  VectorX<T> result(1);
  result[0] = positions(1) - positions(0);
  return result;
}

template <typename T>
VectorX<T> CombinedIiwaWsg<T>::GetGripperVelocities(
    const Context<T>& diagram_context, const Diagram<T>& diagram) const {
  const auto& plant_context =
      diagram.GetSubsystemContext(*this->plant_, diagram_context);

  Vector2<T> velocities =
      this->plant_->GetVelocities(plant_context, wsg_model_.model_instance);
  VectorX<T> result(1);
  result[0] = velocities(1) - velocities(0);
  return result;
}

template <typename T>
void CombinedIiwaWsg<T>::SetGripperPositions(
    const Context<T>& diagram_context,
    const Eigen::Ref<const VectorX<T>>& q,
    const Diagram<T>& diagram,
    State<T>* diagram_state) const {
  DRAKE_DEMAND(diagram_state != nullptr);
  auto& plant_context = diagram.GetSubsystemContext(
      *this->plant_, diagram_context);
  auto& plant_state = diagram.GetMutableSubsystemState(
      *this->plant_, diagram_state);

  const Vector2<T> positions(-q[0] / 2, q[0] / 2);
  this->plant_->SetPositions(plant_context, &plant_state,
      wsg_model_.model_instance, positions);

  // Set the position history in the state interpolator to match.
  const auto& wsg_controller = dynamic_cast<
      const manipulation::schunk_wsg::SchunkWsgPositionController&>(
      diagram.GetSubsystemByName("wsg_controller"));
  wsg_controller.set_initial_position(
      &diagram.GetMutableSubsystemState(wsg_controller, diagram_state), q[0]);
}

template <typename T>
void CombinedIiwaWsg<T>::SetGripperPositionsToDefaultOpen(
    const Context<T>& diagram_context,
    const Diagram<T>& diagram,
    State<T>* diagram_state) const {
  VectorX<T> q_gripper_default_open(1);
  q_gripper_default_open[0] = { 0.1 };
  SetGripperPositions(
      diagram_context, q_gripper_default_open, diagram, diagram_state);
}

template <typename T>
void CombinedIiwaWsg<T>::SetGripperVelocities(
    const Context<T>& diagram_context,
    const Eigen::Ref<const VectorX<T>>& v,
    const Diagram<T>& diagram,
    State<T>* diagram_state) const {
  DRAKE_DEMAND(diagram_state != nullptr);
  auto& plant_context = diagram.GetSubsystemContext(
      *this->plant_, diagram_context);
  auto& plant_state = diagram.GetMutableSubsystemState(
      *this->plant_, diagram_state);

  const Vector2<T> velocities(-v[0] / 2, v[0] / 2);
  this->plant_->SetVelocities(plant_context, &plant_state,
      wsg_model_.model_instance, velocities);
}

template <typename T>
void CombinedIiwaWsg<T>::SetWsgGains(const double kp, const double kd) {
  DRAKE_THROW_UNLESS(!this->plant_->is_finalized());
  DRAKE_THROW_UNLESS(kp >= 0 && kd >= 0);
  wsg_kp_ = kp;
  wsg_kd_ = kd;
}

template <typename T>
void CombinedIiwaWsg<T>::BuildControlDiagram(
      DiagramBuilder<T>* builder) {
  const int num_iiwa_positions =
      this->plant_->num_positions(iiwa_model_.model_instance);
  DRAKE_THROW_UNLESS(num_iiwa_positions ==
                     this->plant_->num_velocities(iiwa_model_.model_instance));

  // Export the commanded positions via a PassThrough.
  auto iiwa_position =
      builder->template AddSystem<PassThrough>(num_iiwa_positions);
  builder->ExportInput(iiwa_position->get_input_port(), "iiwa_position");
  builder->ExportOutput(iiwa_position->get_output_port(),
                       "iiwa_position_commanded");

  // Export iiwa "state" outputs.
  {
    auto demux = builder->template AddSystem<Demultiplexer>(
        2 * num_iiwa_positions, num_iiwa_positions);
    builder->Connect(this->plant_->get_continuous_state_output_port(
        iiwa_model_.model_instance), demux->get_input_port(0));
    builder->ExportOutput(demux->get_output_port(0), "iiwa_position_measured");
    builder->ExportOutput(demux->get_output_port(1), "iiwa_velocity_estimated");

    builder->ExportOutput(this->plant_->get_continuous_state_output_port(
        iiwa_model_.model_instance), "iiwa_state_estimated");
  }

  MakeIiwaControllerModel();

  // Add the IIWA controller "stack".
  {
    controller_plant_.Finalize();

    auto check_gains = [](const VectorX<double>& gains, int size) {
      return (gains.size() == size) && (gains.array() >= 0).all();
    };

    // Set default gains if.
    if (iiwa_kp_.size() == 0) {
      iiwa_kp_ = VectorXd::Constant(num_iiwa_positions, 100);
    }
    DRAKE_THROW_UNLESS(check_gains(iiwa_kp_, num_iiwa_positions));

    if (iiwa_kd_.size() == 0) {
      iiwa_kd_.resize(num_iiwa_positions);
      for (int i = 0; i < num_iiwa_positions; i++) {
        // Critical damping gains.
        iiwa_kd_[i] = 2 * std::sqrt(iiwa_kp_[i]);
      }
    }
    DRAKE_THROW_UNLESS(check_gains(iiwa_kd_, num_iiwa_positions));

    if (iiwa_ki_.size() == 0) {
      iiwa_ki_ = VectorXd::Constant(num_iiwa_positions, 1);
    }
    DRAKE_THROW_UNLESS(check_gains(iiwa_ki_, num_iiwa_positions));

    // Add the inverse dynamics controller.
    auto iiwa_controller = builder->template AddSystem<
        systems::controllers::InverseDynamicsController>(
        controller_plant_, iiwa_kp_, iiwa_ki_, iiwa_kd_, false);
    iiwa_controller->set_name("iiwa_controller");
    builder->Connect(
        this->plant_->get_continuous_state_output_port(
            iiwa_model_.model_instance),
        iiwa_controller->get_input_port_estimated_state());

    // Add in feedforward torque.
    auto adder =
        builder->template AddSystem<Adder>(2, num_iiwa_positions);
    builder->Connect(iiwa_controller->get_output_port_control(),
                    adder->get_input_port(0));
    builder->ExportInput(adder->get_input_port(1), "iiwa_feedforward_torque");
    builder->Connect(
      adder->get_output_port(),
      this->plant_->get_actuation_input_port(iiwa_model_.model_instance));

    // Approximate desired state command from a discrete derivative of the
    // position command input port.
    auto desired_state_from_position = builder->template AddSystem<
        StateInterpolatorWithDiscreteDerivative>(
          num_iiwa_positions, this->plant_->time_step());
    desired_state_from_position->set_name("desired_state_from_position");
    builder->Connect(desired_state_from_position->get_output_port(),
                    iiwa_controller->get_input_port_desired_state());
    builder->Connect(iiwa_position->get_output_port(),
                    desired_state_from_position->get_input_port());

    // Export commanded torques:
    builder->ExportOutput(adder->get_output_port(), "iiwa_torque_commanded");
    builder->ExportOutput(adder->get_output_port(), "iiwa_torque_measured");
  }

  {
    auto wsg_controller = builder->template AddSystem<
        manipulation::schunk_wsg::SchunkWsgPositionController>(
        manipulation::schunk_wsg::kSchunkWsgLcmStatusPeriod, wsg_kp_, wsg_kd_);
    wsg_controller->set_name("wsg_controller");

    builder->Connect(
        wsg_controller->get_generalized_force_output_port(),
        this->plant_->get_actuation_input_port(wsg_model_.model_instance));
    builder->Connect(
        this->plant_->get_continuous_state_output_port(
            wsg_model_.model_instance),
        wsg_controller->get_state_input_port());

    builder->ExportInput(
        wsg_controller->get_desired_position_input_port(), "wsg_position");
    builder->ExportInput(
        wsg_controller->get_force_limit_input_port(), "wsg_force_limit");

    auto wsg_mbp_state_to_wsg_state = builder->template AddSystem(
        manipulation::schunk_wsg::MakeMultibodyStateToWsgStateSystem<double>());
    builder->Connect(
        this->plant_->get_continuous_state_output_port(
            wsg_model_.model_instance),
        wsg_mbp_state_to_wsg_state->get_input_port());

    builder->ExportOutput(
        wsg_mbp_state_to_wsg_state->get_output_port(), "wsg_state_measured");

    builder->ExportOutput(
        wsg_controller->get_grip_force_output_port(), "wsg_force_measured");
  }

  builder->ExportOutput(
      this->plant_->get_generalized_contact_forces_output_port(
          iiwa_model_.model_instance),
      "iiwa_torque_external");
}

template class CombinedIiwaWsg<double>;

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

