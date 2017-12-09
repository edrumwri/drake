#pragma once

#include <Eigen/Core>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace constraint {

enum class SlidingModeType {
  // The mode has not been set.
  kNotSet,

  // The bodies are sliding at the contact.
  kSliding,

  // The bodies are not sliding at the contact. 
  kNotSliding,

  // The bodies are transitioning from not sliding to sliding at the contact.
  kTransitioning,
};

/// The descriptor for a point contact between two rigid bodies.
struct PointContact {
  /// Whether the bodies are considered to be sliding at the point or not.
  SlidingModeType sliding_type{SlidingModeType::kNotSet};

  /// The identifier used to locate the point of contact on the bodies.
  void* id;
};

}  // namespace constraint 
}  // namespace multibody
}  // namespace drake
