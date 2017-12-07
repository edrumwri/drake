#pragma once

namespace drake {
namespace multibody {

template <class T>
class AABB {
 public:
  AABB(const Triangle<T>& t) {
    using std::min;
    using std::max;

    for (int i = 0; i < kThreeD; ++i)
      minp_[i] = min(std::std::min(t.a[i], t.b[i]), t.c[i]);
      maxp_[i] = max(std::std::max(t.a[i], t.b[i]), t.c[i]);
  }
  Eigen::Vector3<T>& lower_bounds() { return minp_; } 
  Eigen::Vector3<T>& upper_bounds() { return maxp_; }

 private:
  /// The lower and upper corners of the AABB.
  Eigen::Vector3<T> minp_, maxp_;
}

}  // multibody
}  // drake
