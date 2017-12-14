#pragma once

#include "drake/multibody/rigid_body_plant/triangle.h"

namespace drake {
namespace multibody {

template <class T>
class Trimesh {
 public:
  Triangle<T> transform(int index, const Isometry3<T>& wTb) const {
    DRAKE_DEMAND(index < tris_.size());
    const Triangle<T>& t = tris_[index];
    return Triangle<T>(wTb * t.a(), wTb * t.b(), wTb * t.c());
  }
 
 private:

  // Vector of triangles.
  std::vector<Triangle<T>> tris_;
};

}  // namespace multibody
}  // namespace drake

