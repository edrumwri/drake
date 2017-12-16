#include "drake/multibody/rigid_body_plant/triangle.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {

class Triangle3Test : public ::testing::Test {
 protected:
  const double tol_ = 10 * std::numeric_limits<double>::epsilon();
};

// Tests the ability to load an instance of a URDF model into a RigidBodyPlant.
TEST_F(Triangle3Test, CalcNormalTest) {
  Triangle3<double> t(Vector3<double>(10, 0, 0),
                      Vector3<double>(10, 1, 0),
                      Vector3<double>(0, 1, 0));
  Vector3<double> normal = t.CalcNormal();

  EXPECT_NEAR(normal[0], 0, tol_);
  EXPECT_NEAR(normal[1], 0, tol_);
  EXPECT_NEAR(normal[2], 1, tol_);
}

// Checks that the distance from a triangle to a line works properly when the
// line is parallel to the triangle.
TEST_F(Triangle3Test, CalcLineSquareDistanceParallel) {
  Triangle3<double> t(Vector3<double>(1, 0, 0),
                      Vector3<double>(1, 1, 0),
                      Vector3<double>(0, 1, 0));
  Vector3<double> origin(0, -1, 0);
  Vector3<double> dir(1, 0, 0);
  Vector3<double> closest_point_on_line, closest_point_on_tri;
  double t_line;
  double sq_dist = t.CalcSquareDistance(origin, dir, &closest_point_on_line,
                                        &closest_point_on_tri, &t_line);

  // Verify the distance.
  EXPECT_NEAR(sq_dist, 1.0, tol_);

  // Note: we do not check closest points here, because there are infinitely
  // many closest points on both the triangle and the line.
}

// Checks that the distance from a triangle to a line works properly when the
// line intersects the triangle.
TEST_F(Triangle3Test, CalcLineSquareDistanceIntersects) {
  Triangle3<double> t(Vector3<double>(1, 0, 0),
                      Vector3<double>(1, 1, 0),
                      Vector3<double>(0, 1, 0));
  Vector3<double> origin(0, 0, 0);
  Vector3<double> dir(0, -1, 0);
  Vector3<double> closest_point_on_line, closest_point_on_tri;
  double t_line;
  double sq_dist = t.CalcSquareDistance(origin, dir, &closest_point_on_line,
                                        &closest_point_on_tri, &t_line);

  // Verify the distance.
  EXPECT_NEAR(sq_dist, 0, tol_);

  // The two should intersect at (0, 1, 0)
  EXPECT_NEAR(closest_point_on_line[0], 0, tol_);
  EXPECT_NEAR(closest_point_on_tri[0], 0, tol_);
  EXPECT_NEAR(closest_point_on_line[1], 1, tol_);
  EXPECT_NEAR(closest_point_on_tri[1], 1, tol_);
  EXPECT_NEAR(closest_point_on_line[2], 0, tol_);
  EXPECT_NEAR(closest_point_on_tri[2], 0, tol_);

  // The line parameter should be -1.
  EXPECT_NEAR(t_line, -1, tol_);
}

// Tests the square distance from a line to a line segment, which are disjoint.
TEST_F(Triangle3Test, CalcLineSquareDistanceFromSegmentDisjoint) {
  // Set the line.
  Vector3<double> origin(0, 0, 0);
  Vector3<double> dir(0, 1, 0);

  // Set the line segment to have a unique closest point (at the endpoint).
  Vector3<double> ep1(1, 0, 0);
  Vector3<double> ep2(2, 0, 0);
  auto seg = std::make_pair(ep1, ep2);

  // Compute the square distance.
  Vector3<double> closest_point_on_line, closest_point_on_seg;
  double t_line, t_seg;
  double sq_dist = Triangle3<double>::CalcSquareDistance(
      origin, dir, seg, &closest_point_on_line, &closest_point_on_seg,
      &t_line, &t_seg);

  // Check the results.
  EXPECT_NEAR(sq_dist, 1.0, tol_);
  EXPECT_NEAR(closest_point_on_line[0], 0, tol_);
  EXPECT_NEAR(closest_point_on_line[1], 0, tol_);
  EXPECT_NEAR(closest_point_on_line[2], 0, tol_);
  EXPECT_NEAR(t_line, 0, tol_);
  EXPECT_NEAR(closest_point_on_seg[0], 1, tol_);
  EXPECT_NEAR(closest_point_on_seg[1], 0, tol_);
  EXPECT_NEAR(closest_point_on_seg[2], 0, tol_);
  EXPECT_NEAR(t_seg, 0, tol_);
}

// Tests the square distance from a line to a line segment that intersects.
TEST_F(Triangle3Test, CalcLineSquareDistanceFromSegmentIntersects) {
  // Set the line.
  Vector3<double> origin(0, 0, 0);
  Vector3<double> dir(0, 1, 0);

  // Set the line segment to have a unique closest point (at the endpoint).
  Vector3<double> ep1(-1, 0, 0);
  Vector3<double> ep2(1, 0, 0);
  auto seg = std::make_pair(ep1, ep2);

  // Compute the square distance.
  Vector3<double> closest_point_on_line, closest_point_on_seg;
  double t_line, t_seg;
  double sq_dist = Triangle3<double>::CalcSquareDistance(
      origin, dir, seg, &closest_point_on_line, &closest_point_on_seg,
      &t_line, &t_seg);

  // Check the results.
  EXPECT_NEAR(sq_dist, 0, tol_);
  EXPECT_NEAR(closest_point_on_line.norm(), 0, tol_);
  EXPECT_NEAR(closest_point_on_seg.norm(), 0, tol_);
  EXPECT_NEAR(t_line, 0, tol_);
  EXPECT_NEAR(t_seg, 0.5, tol_);
}
  
}  // namespace multibody 
}  // namespace drake
