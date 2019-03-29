#pragma once

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"

namespace drake {

namespace systems {
template <typename T>
class DiagramBuilder;
}

namespace multibody {
template <typename T>
class MultibodyPlant;
}

namespace examples {
namespace manipulation_station {

template <typename T>
class CombinedManipulatorAndGripperModel {
 public:
  CombinedManipulatorAndGripperModel(multibody::MultibodyPlant<T>* plant) :
      plant_(plant) {}
  virtual ~CombinedManipulatorAndGripperModel() {}

  // TODO(edrumwri) Remove this.
  /// Determines which manipulation station is simulated.
  enum class Setup { kNone, kDefault, kClutterClearing };

  /// TODO: Finish me.
  /// This function is called when the ManipulationStation...
  virtual void Finalize(const Setup setup,
      systems::DiagramBuilder<T>* builder) = 0;

  /// Gets the number of joints in the manipulator.
  virtual int num_manipulator_joints() const = 0;

  /// Gets the number of joints in the gripper.
  virtual int num_gripper_joints() const = 0;

  /// Gets the manipulator generalized positions.
  virtual VectorX<T> GetManipulatorPositions(
      const systems::Context<T>& robot_context) const = 0;

  /// Gets the gripper generalized positions.
  virtual VectorX<T> GetGripperPositions(
      const systems::Context<T>& robot_context) const = 0;

  /// Sets the manipulator generalized positions.
  virtual void SetManipulatorPositions(
    const systems::Context<T>& robot_context,
    const Eigen::Ref<const VectorX<T>>& q,
    systems::State<T>* state) const = 0;

  /// Sets the gripper generalized positions to the default "open" position.
  virtual void SetGripperPositionsToDefaultOpen(
    const systems::Context<T>& robot_context,
    systems::State<T>* state) const = 0;

  /// Sets the gripper generalized positions.
  virtual void SetGripperPositions(
    const systems::Context<T>& robot_context,
    const Eigen::Ref<const VectorX<T>>& q,
    systems::State<T>* state) const = 0;

  /// Gets the manipulator generalized velocities.
  virtual VectorX<T> GetManipulatorVelocities(
      const systems::Context<T>& robot_context) const = 0;

  /// Gets the gripper generalized velocities.
  virtual VectorX<T> GetGripperVelocities(
      const systems::Context<T>& robot_context) const = 0;

  /// Sets the manipulator generalized velocities.
  virtual void SetManipulatorVelocities(
    const systems::Context<T>& robot_context,
    const Eigen::Ref<const VectorX<T>>& v,
    systems::State<T>* state) const = 0;

  /// Sets the gripper generalized velocities.
  virtual void SetGripperVelocities(
    const systems::Context<T>& robot_context,
    const Eigen::Ref<const VectorX<T>>& v,
    systems::State<T>* state) const = 0;

  /// This function is called by the ManipulationStation to add the manipulator
  /// and gripper models to the plant.
  virtual void AddRobotModelToMultibodyPlant() = 0;

  /// This function is called by the ManipulationStation to build the
  /// control diagram.
  virtual void BuildControlDiagram(systems::DiagramBuilder<T>* builder) = 0;

 protected:
  // The MultibodyPlant holding the robot model (and possibly other models as
  // well).
  multibody::MultibodyPlant<T>* plant_{nullptr};
};

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

