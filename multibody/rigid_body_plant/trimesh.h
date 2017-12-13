#pragma once

#include "drake/multibody/rigid_body_plant/triangle.h"

namespace drake {
namespace multibody {

template <class T>
class Trimesh {
 public:
 
 private:

  // Vector of triangles.
  std::vector<Triangle<T>> tris_;
};

}  // namespace multibody
}  // namespace drake

