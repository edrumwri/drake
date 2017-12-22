#pragma once

#include <limits>

#include "drake/common/eigen_types.h"
#include "drake/math/orthonormal_basis.h"

namespace drake {
namespace math {

/// Determines a projection matrix for projecting points from 3D to 2D.
/// @param normal A normal to a plane to which the points will be projected.
template <class T>
Eigen::Matrix<T, 2, 3> DetermineProjectionMatrix3dTo2d(
    const Vector3<T>& normal) {
  using std::abs; 

  // Verify that the normal is normalized.
  DRAKE_DEMAND(abs(normal.norm() - 1) <
      10 * std::numeric_limits<double>::epsilon());
 
  // Compute the orthonormal basis.
  const Matrix3<T> R = math::ComputeBasisFromAxis(0, normal);
  const Vector3<T> v1 = R.col(1);
  const Vector3<T> v2 = R.col(2);

  // Construct the projection matrix.
  Eigen::Matrix<T, 2, 3> P;
  P.row(0) = v1;
  P.row(1) = v2;
  return P;
}

}  // namespace math
}  // namespace drake
