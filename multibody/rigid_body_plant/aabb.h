#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_plant/triangle.h"

namespace drake {
namespace multibody {

template <class T>
class AABB {
 public:
  AABB(const Vector3<T>& minp, const Vector3<T>& maxp) :
      minp_(minp), maxp_(maxp) {
  }
 
  AABB(const Triangle3<T>& t) {
    using std::min;
    using std::max;

    for (int i = 0; i < 3; ++i) {
      minp_[i] = min(min(t.a()[i], t.b()[i]), t.c()[i]);
      maxp_[i] = max(max(t.a()[i], t.b()[i]), t.c()[i]);
    }
  }

  Vector3<T>& lower_bounds() { return minp_; } 
  Vector3<T>& upper_bounds() { return maxp_; }

  bool Intersects(const AABB<T>& a) const {
    for (int i = 0; i < 3; ++i) {
      if (maxp_[i] < a.minp_[i] || minp_[i] > a.maxp_[i])
        return false; 
    }
    return true;
  }

 private:
  /// The lower and upper corners of the AABB.
  Vector3<T> minp_, maxp_;
};

}  // namespace multibody
}  // namespace drake

