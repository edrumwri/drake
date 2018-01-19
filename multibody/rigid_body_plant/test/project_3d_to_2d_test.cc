#include "drake/multibody/rigid_body_plant/project_3d_to_2d.h"

#include <gtest/gtest.h>

// Tests the utility function that projects points from 3D to 2D.
namespace drake {
namespace multibody {
namespace {

// Confirms that the data of a single contact info is appropriately copied both
// through the copy constructor as well as the assignment operator.
GTEST_TEST(Project3dTo2d, ProjectionLengths) {
  // Construct three normals: +x, +y, +z.
  const Vector3<double> x(1, 0, 0);
  const Vector3<double> y(0, 1, 0);
  const Vector3<double> z(0, 0, 1);

  // Construct three projection matrices.
  const Eigen::Matrix<double, 2, 3> Px = Determine3dTo2dProjectionMatrix(x);
  const Eigen::Matrix<double, 2, 3> Py = Determine3dTo2dProjectionMatrix(y);
  const Eigen::Matrix<double, 2, 3> Pz = Determine3dTo2dProjectionMatrix(z);

  // Project a single point three times and compare actual vector lengths vs.
  // lengths expected from the projection.
  const double zero_tol = 10 * std::numeric_limits<double>::epsilon();
  const Vector3<double> point(2, 3, 5);
  EXPECT_NEAR((Px * point).norm(), Vector2<double>(3, 5).norm(), zero_tol);
  EXPECT_NEAR((Py * point).norm(), Vector2<double>(2, 5).norm(), zero_tol);
  EXPECT_NEAR((Pz * point).norm(), Vector2<double>(2, 3).norm(), zero_tol);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
