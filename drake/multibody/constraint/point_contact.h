#pragma once

#include <Eigen/Core>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace constraint {

/// The descriptor for a point contact between two rigid bodies.
struct PointContact {
  bool sliding;

  void* id;
};

}  // namespace constraint 
}  // namespace multibody
}  // namespace drake
