#pragma once

namespace drake {
namespace geometry {

template <class T>
class Field {
 public:
  virtual ~Field() {}

  /// Evaluates the scalar field at the value `p_F`, where
  /// `F` is the frame that this field is defined with respect to.
  virtual T Evaluate(const Vector3<T>& p_F) const = 0;
};

}  // namespace geometry
}  // namespace drake