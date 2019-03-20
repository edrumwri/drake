#pragma once

namespace drake {

namespace systems {
template <typename T>
class DiagramBuilder;
}

namespace examples {
namespace manipulation_station {

template <typename T>
class CombinedManipulatorAndGripperModel {
 public:
  /// This function is called when the ManipulationStation
  virtual void Finalize(systems::DiagramBuilder<T>* builder) = 0;
};

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

