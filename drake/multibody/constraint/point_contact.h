#pragma once

#include <Eigen/Core>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace constraint {

/// The descriptor for a point contact between two rigid bodies.
struct PointContact {
  /// Whether the bodies are considered to be sliding at the point or not. 
  bool sliding;

  /// The identifier used to locate the point of contact on the bodies.
  void* id;
};

}  // namespace constraint 
}  // namespace multibody
}  // namespace drake
