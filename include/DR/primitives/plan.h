#pragma once

#include <drake/common/eigen_types.h>

namespace DR {

/// A virtual class that is the output of a PrimitiveBehavior. A Plan can leverage any representation (kinematics,
/// torques, even symbolic information) used to compute a control output. Plans are valid only over certain times. 
template <typename T>
class Plan {
 public:
  /// Evaluates the plan at the given time.
  virtual drake::VectorX<T> Evaluate(const T& time) const = 0;

  // TODO(edrumwri): Consider adding a generic Evaluate() method for allowing output of symbolic information.

  virtual const T& start_time() const = 0;
  virtual const T& end_time() const = 0;

  // TODO(edrumwri): Consider adding an is_valid(time) method.

  std::unique_ptr<Plan> Clone() const {
    // Just calls the DoClone() method by default.
    return DoClone();
  }

 protected:
  virtual std::unique_ptr<Plan> DoClone() const = 0;
};

}  // namespace DR

