#include "drake/math/2d_and_3d_projection_matrices.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace math {
namespace {

// Tests that projection works as expected.
GTEST_TEST(Projections, ProjectionFrom2dTo3d) {
  // Construct the vector.
  Vector3d v(1, 2, 3);

  // Set the normal.
  const Vector3d normal(0, 1, 0);

  // Set the projection matrix.
  Eigen::Matrix<double, 2, 3> P = math::Compute3dTo2dProjectionMatrix(normal);

  // Get the projected vector.
  const Vector2d Pv = P * v;

  // Manually construct a candidate projected vector.
  const Vector2d Pv_candidate(1, 3);

  // Check only that the lengths are identical.
  EXPECT_NEAR(Pv.norm(), Pv_candidate.norm(),
              10 * std::numeric_limits<double>::epsilon());
}

}  // namespace
}  // namespace math
}  // namespace drake
