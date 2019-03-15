#pragma once

#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace manipulation_station {

class CombinedManipulatorAndGripperModel {
 public:
  virtual void Finalize(systems::DiagramBuilder<T>* builder) = 0;
};

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

