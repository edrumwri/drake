#pragma once

namespace drake {
namespace geometry {

template <class T>
class Tetrahedron {
 public:
  /// Evaluates the field defined over the domain of this tetrahedron, and
  /// sampled at the vertices, using interpolation.
  T EvaluateField(const Vector3<T>& p_F) const;
};

}  // namespace geometry
}  // namespace drake