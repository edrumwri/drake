#pragma once

#include "drake/multibody/rigid_body_plant/triangle3.h"

namespace drake {
namespace multibody {

template <class T>
class Trimesh {
 public:
  Triangle3<T> transform(int index, const Isometry3<T>& wTb) const {
    DRAKE_DEMAND(index < tris_.size());
    const Triangle3<T>& t = tris_[index];
    return Triangle3<T>(wTb * t.a(), wTb * t.b(), wTb * t.c());
  }

  /// Gets the requisite triangle. Aborts on index out of bound error.
  const Triangle3<T>& triangle(int index) const {
    DRAKE_DEMAND(index < tris_.size() && index >= 0); 
    return tris_[index];
  } 
 
 private:
  // Vector of triangles.
  std::vector<Triangle3<T>> tris_;
};

}  // namespace multibody
}  // namespace drake

