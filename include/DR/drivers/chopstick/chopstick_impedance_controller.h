#pragma once

#include <memory>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>

#include <DR/interfaces/impedance_controller.h>

namespace DR {

template <typename T>
class ChopstickImpedanceController : public ImpedanceController<T> {
 public:
  /// Constructs the impedance controller with a plant containing all multibodies in the environment (including the
  /// robot).
  /// @throws std::logic_error if the plant's actuation matrix is not a binary matrix.
  ChopstickImpedanceController(
      const drake::multibody::MultibodyPlant<T>* all_plant) : ImpedanceController<T>(all_plant) {}

 private:
  virtual drake::VectorX<T> DoCalcPhi(const drake::systems::Context<T>&) const { return drake::VectorX<T>(0); }
  virtual drake::VectorX<T> DoCalcPhiDot(const drake::systems::Context<T>&) const { return drake::VectorX<T>(0); }
  virtual drake::MatrixX<T> DoCalcG(const drake::systems::Context<T>&) const {
    return drake::MatrixX<T>(0, this->all_plant().num_positions());
  }
  virtual drake::MatrixX<T> DoCalcK(const drake::systems::Context<T>&) const { return drake::MatrixX<T>(0, 0); }
  virtual drake::MatrixX<T> DoCalcC(const drake::systems::Context<T>&) const { return drake::MatrixX<T>(0, 0); }
};

}  // namespace DR
