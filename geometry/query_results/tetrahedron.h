#pragma once

namespace drake {
namespace geometry {

template <class T>
class Tetrahedron {
 public:
  /// Evaluates the scalar field defined over the domain of this tetrahedron,
  /// and sampled at the vertices, using interpolation at the value `p_F`, where
  /// `F` is the frame that this tetrahedron is attached to.
  T EvalField(const Vector3<T>& p_F) const;
};

}  // namespace geometry
}  // namespace drake