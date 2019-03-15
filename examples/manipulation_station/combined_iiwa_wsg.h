#pragma once

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace manipulation_station {

template <typename T>
class CombinedIiwaWsg : public RobotModel {
 public:
  void Finalize(systems::DiagramBuilder<T>* builder) override;


  /// Notifies the ManipulationStation that the IIWA robot model instance can
  /// be identified by @p iiwa_instance as well as necessary information to
  /// reload model for the internal controller's use. Assumes @p iiwa_instance
  /// has already been added to the MultibodyPlant.
  /// Note, the current implementation only allows @p parent_frame to be the
  /// world frame. The IIWA frame needs to directly contain @p child_frame.
  /// Only call this with custom IIWA models (i.e. not calling
  /// SetupDefaultStation()). Must be called before Finalize().
  /// @param model_path Full path to the model file.
  /// @param iiwa_instance Identifies the IIWA model.
  /// @param parent_frame Identifies frame P (the parent frame) in the
  /// MultibodyPlant that the IIWA model has been attached to.
  /// @param child_frame_name Identifies frame C (the child frame) in the IIWA
  /// model that is welded to frame P.
  /// @param X_PC Transformation between frame P and C.
  /// @throws If @p parent_frame is not the world frame.
  // TODO(siyuan.feng@tri.global): throws meaningful errors earlier here,
  // rather than in Finalize() if the arguments are inconsistent with the plant.
  // TODO(siyuan.feng@tri.global): remove the assumption that parent frame has
  // to be world.
  // TODO(siyuan.feng@tri.global): Some of these information should be
  // retrievable from the MultibodyPlant directly or MultibodyPlant should
  // provide partial tree cloning.
  void RegisterIiwaControllerModel(
      const std::string& model_path,
      const multibody::ModelInstanceIndex iiwa_instance,
      const multibody::Frame<T>& parent_frame,
      const multibody::Frame<T>& child_frame,
      const math::RigidTransform<double>& X_PC);

  /// Notifies the ManipulationStation that the WSG gripper model instance can
  /// be identified by @p wsg_instance, as well as necessary information to
  /// reload model for the internal controller's use. Assumes @p wsg_instance
  /// has already been added to the MultibodyPlant. The IIWA model needs to
  /// directly contain @p parent_frame, and the WSG model needs to directly
  /// contain @p child_frame.
  /// Only call this with custom WSG models (i.e. not calling
  /// SetupDefaultStation()). Must be called before Finalize().
  /// @param model_path Full path to the model file.
  /// @param wsg_instance Identifies the WSG model.
  /// @param parent_frame Identifies frame P (the parent frame) in the
  /// MultibodyPlant that the WSG model has been attached to. Has to be part
  /// of the IIWA model.
  /// @param child_frame Identifies frame C (the child frame) in the WSG
  /// model that is used welded to frame P.
  /// @param X_PC Transformation between frame P and C.
  // TODO(siyuan.feng@tri.global): Some of these information should be
  // retrievable from the MultibodyPlant directly or MultibodyPlant should
  // provide partial tree cloning.
  // TODO(siyuan.feng@tri.global): throws meaningful errors earlier here,
  // rather than in Finalize() if the arguments are inconsistent with the plant.
  void RegisterWsgControllerModel(
      const std::string& model_path,
      const multibody::ModelInstanceIndex wsg_instance,
      const multibody::Frame<T>& parent_frame,
      const multibody::Frame<T>& child_frame,
      const math::RigidTransform<double>& X_PC);

  /// Return a reference to the plant used by the inverse dynamics controller
  /// (which contains only a model of the iiwa + equivalent mass of the
  /// gripper).
  const multibody::MultibodyPlant<T>& get_controller_plant() const {
    return *owned_controller_plant_;
  }

  /// Get the number of joints in the IIWA (only -- does not include the
  /// gripper).
  int num_iiwa_joints() const { return 7; }

  /// Set the gains for the WSG controller.
  /// @throws exception if Finalize() has been called.
  void SetWsgGains(double kp, double kd);

  /// Set the position gains for the IIWA controller.
  /// @throws exception if Finalize() has been called.
  void SetIiwaPositionGains(const VectorX<double>& kp) {
    DRAKE_THROW_UNLESS(!plant_->is_finalized());
    iiwa_kp_ = kp;
  }

  /// Set the velocity gains for the IIWA controller.
  /// @throws exception if Finalize() has been called.
  void SetIiwaVelocityGains(const VectorX<double>& kd) {
    DRAKE_THROW_UNLESS(!plant_->is_finalized());
    iiwa_kd_ = kd;
  }

  /// Set the integral gains for the IIWA controller.
  /// @throws exception if Finalize() has been called.
  void SetIiwaIntegralGains(const VectorX<double>& ki) {
    DRAKE_THROW_UNLESS(!plant_->is_finalized());
    iiwa_ki_ = ki;
  }


  /// Convenience method for getting all of the joint angles of the Kuka IIWA.
  /// This does not include the gripper.
  VectorX<T> GetIiwaPosition(const systems::Context<T>& station_context) const;

  /// Convenience method for setting all of the joint angles of the Kuka IIWA.
  /// Also sets the position history in the velocity command generator.
  /// @p q must have size num_iiwa_joints().
  /// @pre `state` must be the systems::State<T> object contained in
  /// `station_context`.
  void SetIiwaPosition(const systems::Context<T>& station_context,
                       systems::State<T>* state,
                       const Eigen::Ref<const VectorX<T>>& q) const;

  /// Convenience method for setting all of the joint angles of the Kuka IIWA.
  /// Also sets the position history in the velocity command generator.
  /// @p q must have size num_iiwa_joints().
  void SetIiwaPosition(systems::Context<T>* station_context,
                       const Eigen::Ref<const VectorX<T>>& q) const {
    SetIiwaPosition(*station_context, &station_context->get_mutable_state(), q);
  }

  DRAKE_DEPRECATED("2019-04-01",
      "Prefer the version with the Context as the first argument.")
  void SetIiwaPosition(const Eigen::Ref<const VectorX<T>>& q,
                       systems::Context<T>* station_context) const {
    SetIiwaPosition(station_context, q);
  }

  /// Convenience method for getting all of the joint velocities of the Kuka
  // IIWA.  This does not include the gripper.
  VectorX<T> GetIiwaVelocity(const systems::Context<T>& station_context) const;

  /// Convenience method for setting all of the joint velocities of the Kuka
  /// IIWA. @v must have size num_iiwa_joints().
  /// @pre `state` must be the systems::State<T> object contained in
  /// `station_context`.
  void SetIiwaVelocity(const systems::Context<T>& station_context,
                       systems::State<T>* state,
                       const Eigen::Ref<const VectorX<T>>& v) const;

  /// Convenience method for setting all of the joint velocities of the Kuka
  /// IIWA. @v must have size num_iiwa_joints().
  void SetIiwaVelocity(systems::Context<T>* station_context,
                       const Eigen::Ref<const VectorX<T>>& v) const {
    SetIiwaVelocity(*station_context, &station_context->get_mutable_state(), v);
  }

  DRAKE_DEPRECATED("2019-04-01",
      "Prefer the version with the Context as the first argument.")
  void SetIiwaVelocity(const Eigen::Ref<const VectorX<T>>& v,
                       systems::Context<T>* station_context) const {
    SetIiwaVelocity(station_context, v);
  }

  /// Convenience method for getting the position of the Schunk WSG. Note
  /// that the WSG position is the signed distance between the two fingers
  /// (not the state of the fingers individually).
  T GetWsgPosition(const systems::Context<T>& station_context) const;

  /// Convenience method for getting the velocity of the Schunk WSG.
  T GetWsgVelocity(const systems::Context<T>& station_context) const;

  /// Convenience method for setting the position of the Schunk WSG. Also
  /// sets the position history in the velocity interpolator.  Note that the
  /// WSG position is the signed distance between the two fingers (not the
  /// state of the fingers individually).
  /// @pre `state` must be the systems::State<T> object contained in
  /// `station_context`.
  void SetWsgPosition(const systems::Context<T>& station_context,
                      systems::State<T>* state, const T& q) const;

  /// Convenience method for setting the position of the Schunk WSG. Also
  /// sets the position history in the velocity interpolator.  Note that the
  /// WSG position is the signed distance between the two fingers (not the
  /// state of the fingers individually).
  void SetWsgPosition(systems::Context<T>* station_context, const T& q) const {
    SetWsgPosition(*station_context, &station_context->get_mutable_state(), q);
  }

  DRAKE_DEPRECATED("2019-04-01",
      "Prefer the version with the Context as the first argument.")
  void SetWsgPosition(const T& q, systems::Context<T>* station_context) const {
    SetWsgPosition(station_context, q);
  }

  /// Convenience method for setting the velocity of the Schunk WSG.
  /// @pre `state` must be the systems::State<T> object contained in
  /// `station_context`.
  void SetWsgVelocity(const systems::Context<T>& station_context,
                      systems::State<T>* state, const T& v) const;

  /// Convenience method for setting the velocity of the Schunk WSG.
  void SetWsgVelocity(systems::Context<T>* station_context, const T& v) const {
    SetWsgVelocity(*station_context, &station_context->get_mutable_state(), v);
  }

  DRAKE_DEPRECATED("2019-04-01",
      "Prefer the version with the Context as the first argument.")
  void SetWsgVelocity(const T& v, systems::Context<T>* station_context) const {
    SetWsgVelocity(station_context, v);
  }

 private:
  // The MultibodyPlant holding the robot model (and possibly other models as
  // well).
  multibody::MultibodyPlant<T>* plant_{nullptr};

  // Assumes iiwa_model_info_ and wsg_model_info_ have already being populated.
  // Should only be called from Finalize().
  void MakeIiwaControllerModel();

  void AddDefaultIiwa(const IiwaCollisionModel collision_model);
  void AddDefaultWsg();

  std::unique_ptr<multibody::MultibodyPlant<T>> owned_controller_plant_;

  // Populated by RegisterIiwaControllerModel() and
  // RegisterWsgControllerModel().
  ModelInformation iiwa_model_;
  ModelInformation wsg_model_;

  // These are kp and kd gains for iiwa and wsg controllers.
  VectorX<double> iiwa_kp_;
  VectorX<double> iiwa_kd_;
  VectorX<double> iiwa_ki_;
  // TODO(siyuan.feng@tri.global): Need to tunes these better.
  double wsg_kp_{200};
  double wsg_kd_{5};
};

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

