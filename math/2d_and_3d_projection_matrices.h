#pragma once

#include <iostream>
#include <limits>

#include "drake/common/eigen_types.h"
#include "drake/math/orthonormal_basis.h"

namespace drake {
namespace math {

/// Creates a matrix for projecting vectors in 3D to 2D using a normal to the
/// projection plane. Assuming that the normal were defined as: [0 1 0]', a
/// candidate projection matrix would be [ 1 0 0; 0 0 1 ]. Then the vector
/// [1 2 3]' would be projected to [1 3]'. Note that there are infinitely many
/// candidate projection matrices.
template <class T>
Eigen::Matrix<T, 2, 3> Compute3dTo2dProjectionMatrix(
    const Vector3<T>& normal) {
  // Compute an orthonormal basis with the normal in the last column.
  const int axis = 2;
  Matrix3<T> P = math::ComputeBasisFromAxis(axis, normal);
  return P.transpose().template block<2, 3>(0, 0);
}

/// Creates a matrix for projecting vectors in 2D to 3D using a 3D normal to a
/// 2D plane in which the 2D vector lives. Assuming that the normal were defined
/// as [0 1 0]', a candidate projection matrix would be [ 1 0; 0 0; 0 1 ]. Then
/// applying the projection matrix to the vector [ 1 3 ]' would yield the
/// vector [ 1 0 3 ]'. Note that there are infinitely many candidate projection
/// matrices. Also note that `v ≠ Compute2dTo3dProjectionMatrix() *`
/// `Compute3dTo2dProjectionMatrix() * v`, for some 3D vector v.
template <class T>
Eigen::Matrix<T, 3, 2> Compute2dTo3dProjectionMatrix(
    const Vector3<T>& normal) {
  // Compute an orthonormal basis with the normal in the last column.
  const int axis = 2;
  Matrix3<T> PT = math::ComputeBasisFromAxis(axis, normal);
  PT.col(axis).setZero();
  return PT.transpose().template block<3, 2>(0, 0);
}

}  // namespace math
}  // namespace drake
