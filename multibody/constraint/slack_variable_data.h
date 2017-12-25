#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace constraint {

/// Structure for holding slack variable data from constraint force / impact
/// impulse computation.
template <class T>
struct SlackVariableData {
  VectorX<T> N_slack;
  VectorX<T> L_slack;
};

}  // namespace constraint
}  // namespace multibody
}  // namespace drake
