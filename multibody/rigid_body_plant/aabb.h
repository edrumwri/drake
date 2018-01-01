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
    Expand();
  }

  const Vector3<T>& lower_bounds() const { return minp_; }
  Vector3<T>& lower_bounds() { return minp_; } 
  const Vector3<T>& upper_bounds() const { return maxp_; }
  Vector3<T>& upper_bounds() { return maxp_; }

  T CalcDistance(const AABB<T>& a) const {
    using std::min;
    if (Intersects(a))
      return 0;
    Vector3<T> axes_dist;
    for (int i = 0; i < 3; ++i) {
      axes_dist[i] = min(a.minp_[i] - maxp_[i], minp_[i] - a.maxp_[i]);
      DRAKE_DEMAND(axes_dist[i] > 0);
    }

    return axes_dist.norm();
  }

  bool Intersects(const AABB<T>& a) const {
    for (int i = 0; i < 3; ++i) {
      if (maxp_[i] < a.minp_[i] || minp_[i] > a.maxp_[i])
        return false; 
    }
    return true;
  }

 private:
  // This function acts to expand the AABB such that the pass through problem
  // is eliminated for bounding boxes
  // TODO: This should not be necesary!
  void Expand() {
    minp_ -= Vector3<T>::Ones() * 1e-4;
    maxp_ += Vector3<T>::Ones() * 1e-4;
  }

  /// The lower and upper corners of the AABB.
  Vector3<T> minp_, maxp_;
};

}  // namespace multibody
}  // namespace drake

