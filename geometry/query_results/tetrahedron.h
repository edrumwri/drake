#pragma once

namespace drake {
namespace geometry {

template <class T>
class Tetrahedron {
 public:
  T EvaluateField(const Vector3<T>& p) const;
};

}  // namespace geometry
}  // namespace drake