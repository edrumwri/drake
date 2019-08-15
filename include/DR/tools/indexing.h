#pragma once

#include <map>
#include <string>

#include <drake/common/eigen_types.h>
#include <drake/multibody/plant/multibody_plant.h>

namespace DR {
/**
 @tparam T default drake scalar type
 @param mbp a const reference to the plant the will determine the indices of the joint position index aligned vector.
 @param name_value_map joint name to value map.
 @return drake::VectorX<T> the joint position index aligned vector with zeros for unspecified joint names.
 */
template <typename T>
drake::VectorX<T> CreatePositionIndexAlignedVector(const drake::multibody::MultibodyPlant<T>& mbp,
                                                   const std::map<std::string, T>& name_value_map) {
  drake::VectorX<T> output = drake::VectorX<T>::Zero(mbp.num_positions());
  for (const auto& name_value_pair : name_value_map) {
    const auto& joint = mbp.GetJointByName(name_value_pair.first);
    // TODO(samzapo): Implement this function for joints with multiple dofs.
    DR_DEMAND(joint.num_positions() == 1);
    int index = joint.position_start();
    output[index] = name_value_pair.second;
  }
  return output;
}

/**
 @tparam T default drake scalar type
 @param mbp a const reference to the plant the will determine the indices of the joint velocity index aligned vector.
 @param name_value_map joint name to value map.
 @return drake::VectorX<T> the joint velocity index aligned vector with zeros for unspecified joint names.
 */
template <typename T>
drake::VectorX<T> CreateVelocityIndexAlignedVector(const drake::multibody::MultibodyPlant<T>& mbp,
                                                   const std::map<std::string, T>& name_value_map) {
  drake::VectorX<T> output = drake::VectorX<T>::Zero(mbp.num_velocities());
  for (const auto& name_value_pair : name_value_map) {
    const auto& joint = mbp.GetJointByName(name_value_pair.first);
    // TODO(samzapo): Implement this function for joints with multiple dofs.
    DR_DEMAND(joint.num_velocities() == 1);
    int index = joint.velocity_start();
    output[index] = name_value_pair.second;
  }
  return output;
}
}  // namespace DR
