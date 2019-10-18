#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>

#include <DR/interfaces/task_space_impedance_controller.h>

namespace DR {

template <typename T>
class ChopstickImpedanceController : public TaskSpaceImpedanceController<T> {
 public:
  /// Constructs the impedance controller with a plant containing all multibodies in the environment (including the
  /// robot).
  /// @throws std::logic_error if the plant's actuation matrix is not a binary matrix.
  ChopstickImpedanceController(const drake::multibody::MultibodyPlant<T>* universal_plant,
                               const drake::multibody::MultibodyPlant<T>* robot_plant,
                               std::vector<const TaskSpaceGoal<T>*>&& task_space_goals)
      : TaskSpaceImpedanceController<T>(universal_plant, robot_plant, std::move(task_space_goals)) {}
};

}  // namespace DR
