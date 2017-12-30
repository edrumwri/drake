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
  const Vector3<double> a(10, 0, 0);
  const Vector3<double> b(10, 1, 0);
  const Vector3<double> c(0, 1, 0);
  Triangle3<double> t(&a, &b, &c);
  Vector3<double> normal = t.CalcNormal();

  EXPECT_NEAR(normal[0], 0, tol_);
  EXPECT_NEAR(normal[1], 0, tol_);
  EXPECT_NEAR(normal[2], 1, tol_);
}

// Checks that the distance from a triangle to a line works properly when the
// line is parallel to the triangle.
TEST_F(Triangle3Test, CalcLineSquareDistanceParallel) {
  const Vector3<double> a(1, 0, 0);
  const Vector3<double> b(1, 1, 0);
  const Vector3<double> c(0, 1, 0);
  Triangle3<double> t(&a, &b, &c);
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
  const Vector3<double> a(1, 0, 0);
  const Vector3<double> b(1, 1, 0);
  const Vector3<double> c(0, 1, 0);
  Triangle3<double> t(&a, &b, &c);
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
 
class Triangle2Test : public ::testing::Test {
 protected:
  const double tol_ = 10 * std::numeric_limits<double>::epsilon();
};

// Tests the CalcSignedDistance() function (triangle / single point version).
TEST_F(Triangle2Test, PointTriCalcSignedDistance) {
  const double sqrt_34 = std::sqrt(0.75);
  const Vector2<double> a(0, 0), b(1, 0), c(0.5, sqrt_34);
  const Triangle2<double> t(a, b, c);

  // Test one point inside and one point outside.
  const Vector2<double> q1(0.5, -1);
  const Vector2<double> q2(0.5, .1);

  EXPECT_NEAR(t.CalcSignedDistance(q1), 1, tol_);
  EXPECT_NEAR(t.CalcSignedDistance(q2), -.1, tol_);
}

// Tests the CalcSignedDistance() function (triangle / line segment version).
TEST_F(Triangle2Test, SegTriCalcSignedDistance) {
  const double sqrt_34 = std::sqrt(0.75);
  const Vector2<double> a(0, 0), b(1, 0), c(0.5, sqrt_34);
  const Triangle2<double> t(a, b, c);

  // Test the vertex case.
  EXPECT_NEAR(t.CalcSignedDistance(
    std::make_pair(Vector2<double>(0, 0), Vector2<double>(0, 1))), 0, tol_);

  // Test the edge case.
  EXPECT_NEAR(t.CalcSignedDistance(
    std::make_pair(Vector2<double>(0, -1), Vector2<double>(0, 1))), 0, tol_);

  // Test the edge overlap case.
  EXPECT_NEAR(t.CalcSignedDistance(
    std::make_pair(Vector2<double>(.2, 0), Vector2<double>(.5, 0))), 0, tol_);

  // Test the proper interior case.
  EXPECT_NEAR(t.CalcSignedDistance(std::make_pair(
    Vector2<double>(.49, .2), Vector2<double>(.51, .2))), -.2, tol_);

  // Test the fully outside case.
  EXPECT_NEAR(t.CalcSignedDistance(
    std::make_pair(Vector2<double>(.2, -1), Vector2<double>(.5, -1))), 1, tol_);

  // Test the "planar intersect" case.
  EXPECT_NEAR(t.CalcSignedDistance(std::make_pair(
    Vector2<double>(.5, .1), Vector2<double>(.5, -1))), -.1, tol_);
}

// Tests the CalcSignedDistance() function (triangle / triangle version).
TEST_F(Triangle2Test, TriTriCalcSignedDistance) {
  const double sqrt_34 = std::sqrt(0.75);
  const Vector2<double> a(0, 0), b(1, 0), c(0.5, sqrt_34);
  const Triangle2<double> t1(a, b, c);

  // Test intersecting triangles.
  EXPECT_NEAR(t1.CalcSignedDistance(t1), -std::sqrt(3)/2, tol_);
}

// Tests the CalcSignedDistance() function (line segment / segment version).
TEST_F(Triangle2Test, SegSegCalcSignedDistance) {
  // Test the intersecting case.
  auto seg1 = std::make_pair(Vector2<double>(0, 0), Vector2<double>(0, 1));
  auto seg2 = std::make_pair(Vector2<double>(-1, .25), Vector2<double>(1, .25));
  EXPECT_NEAR(Triangle2<double>::ApplySeparatingAxisTheorem(seg1, seg2), -.25,
      tol_);
}

TEST_F(Triangle2Test, SegLocation) {
  EXPECT_EQ(Triangle2<double>::DetermineSegLocation(0.0),
      Triangle2<double>::kSegOrigin);
  EXPECT_EQ(Triangle2<double>::DetermineSegLocation(1.0),
      Triangle2<double>::kSegEndpoint);
  EXPECT_EQ(Triangle2<double>::DetermineSegLocation(0.5),
      Triangle2<double>::kSegInterior);
  EXPECT_EQ(Triangle2<double>::DetermineSegLocation(-1.0),
      Triangle2<double>::kSegExterior);
  EXPECT_EQ(Triangle2<double>::DetermineSegLocation(2.0),
      Triangle2<double>::kSegExterior);
}

// Tests the IsBetween() function.
TEST_F(Triangle2Test, IsBetween) {
  // Set segment endpoints along the x-axis.
  Vector2<double> a(0, 0), b(1, 0);

  // Set query points.
  Vector2<double> q1(-1, 0), q2(2, 0), q3(0.5, 0);

  // Check non-collinear points.
  EXPECT_FALSE(Triangle2<double>::IsBetween(a, b, Vector2<double>(1, 1)));

  // Do the tests.
  EXPECT_TRUE(Triangle2<double>::IsBetween(a, b, q3));
  EXPECT_FALSE(Triangle2<double>::IsBetween(a, b, q1));
  EXPECT_FALSE(Triangle2<double>::IsBetween(a, b, q2));

  // Set segment endpoints along the y-axis.
  b = Vector2<double>(0, 1);

  // Reset query points.
  q1 = Vector2<double>(0, -1);
  q2 = Vector2<double>(0, 2);
  q3 = Vector2<double>(0, 0.5);

  // Do the tests.
  EXPECT_TRUE(Triangle2<double>::IsBetween(a, b, q3));
  EXPECT_FALSE(Triangle2<double>::IsBetween(a, b, q1));
  EXPECT_FALSE(Triangle2<double>::IsBetween(a, b, q2));
}

// Tests the DetermineLineParam() function.
TEST_F(Triangle2Test, DetermineLineParam) {
  const double tol = 10 * std::numeric_limits<double>::epsilon();

  // Set the origin, direction, and query points.
  const Vector2<double> origin(0, 0);
  const Vector2<double> dir(1, 0);
  const Vector2<double> q1(2, 0), q2(-2, 0);
  EXPECT_NEAR(Triangle2<double>::DetermineLineParam(origin, dir, q1), 2, tol);
  EXPECT_NEAR(Triangle2<double>::DetermineLineParam(origin, dir, q2), -2, tol);
}

TEST_F(Triangle2Test, CalcAreaSign) {
  const double tol = 10 * std::numeric_limits<double>::epsilon();
 
  // Set the two points.
  const Vector2<double> p1(0, 0), p2(1, 0);

  // Set the query points.
  const Vector2<double> q1(2, 0), q2(1, -1), q3(1, 1);

  // Check for expected results.
  EXPECT_EQ(Triangle2<double>::CalcAreaSign(p1, p2, q1, tol),
            Triangle2<double>::kOn);
  EXPECT_EQ(Triangle2<double>::CalcAreaSign(p1, p2, q2, tol),
            Triangle2<double>::kRight);
  EXPECT_EQ(Triangle2<double>::CalcAreaSign(p1, p2, q3, tol),
            Triangle2<double>::kLeft);
}

TEST_F(Triangle2Test, SegTriIntersection) {
  const Vector2<double> a(0, 0), b(1, 0), c(0, 1);
  const Triangle2<double> t(a, b, c);

  // Set query segments.
  auto s1 = std::make_pair(Vector2<double>(.1, .5), Vector2<double>(.1, .2));
  auto s2 = std::make_pair(Vector2<double>(0, -1), Vector2<double>(0, 0));
  auto s3 = std::make_pair(Vector2<double>(.1, 0), Vector2<double>(.1, -1));
  auto s4 = std::make_pair(Vector2<double>(.1, 0), Vector2<double>(.2, 0));
  auto s5 = std::make_pair(Vector2<double>(-1, .5), Vector2<double>(.1, .5));
  auto s6 = std::make_pair(Vector2<double>(0, -1), Vector2<double>(1, -1));

  // Do the tests.
  Vector2<double> isect, isect2;
  EXPECT_EQ(t.Intersect(s1, tol_, &isect, &isect2),
      Triangle2<double>::kSegTriInside);

  EXPECT_EQ(t.Intersect(s2, tol_, &isect, &isect2),
      Triangle2<double>::kSegTriVertex); 
  EXPECT_EQ(isect[0], 0);
  EXPECT_EQ(isect[1], 0);

  EXPECT_EQ(t.Intersect(s3, tol_, &isect, &isect2),
      Triangle2<double>::kSegTriEdge);
  EXPECT_EQ(isect[0], .1);
  EXPECT_EQ(isect[1], 0); 

  EXPECT_EQ(t.Intersect(s4, tol_, &isect, &isect2),
      Triangle2<double>::kSegTriEdgeOverlap);
  EXPECT_TRUE((isect[0] == .1 && isect2[0] == .2) ||
              (isect[0] == .2 && isect2[0] == .1));
  EXPECT_EQ(isect[1], 0);
  EXPECT_EQ(isect2[1], 0);

  EXPECT_EQ(t.Intersect(s5, tol_, &isect, &isect2),
      Triangle2<double>::kSegTriPlanarIntersect);
  EXPECT_TRUE((isect[0] == .1 && isect[1] == .5 &&
               isect2[0] == 0 && isect2[1] == 0.5) ||
              (isect2[0] == .1 && isect2[1] == .5 &&
              isect[0] == 0 && isect[1] == 0.5));

  EXPECT_EQ(t.Intersect(s6, tol_, &isect, &isect2),
      Triangle2<double>::kSegTriNoIntersect);
}

// Checks the line segment intersection algorithms.
TEST_F(Triangle2Test, SegSegIntersection) {
  auto seg1 = std::make_pair(Vector2<double>(0, 0), Vector2<double>(1, 0));

  // Check proper intersection.
  auto seg2 = std::make_pair(Vector2<double>(.5, .5), Vector2<double>(.5, -.5));
  Vector2<double> isect, isect2;
  EXPECT_EQ(Triangle2<double>::IntersectSegs(seg1, seg2, &isect, &isect2),
            Triangle2<double>::kSegSegIntersect);
  EXPECT_EQ(isect[0], .5);
  EXPECT_EQ(isect[1], 0);

  // Check vertex intersection.
  auto seg3 = std::make_pair(Vector2<double>(0, 1), Vector2<double>(0, -.1));
  EXPECT_EQ(Triangle2<double>::IntersectSegs(seg1, seg3, &isect, &isect2),
            Triangle2<double>::kSegSegVertex);
  EXPECT_EQ(isect[0], 0);
  EXPECT_EQ(isect[1], 0);

  // Check edge/edge intersection.
  auto seg4 = std::make_pair(Vector2<double>(-1, 0), Vector2<double>(.5, 0));
  EXPECT_EQ(Triangle2<double>::IntersectSegs(seg1, seg4, &isect, &isect2),
            Triangle2<double>::kSegSegEdge);
  EXPECT_TRUE((isect[0] == 0 && isect2[0] == .5) ||
              (isect2[0] == 0 && isect[0] == .5));
  EXPECT_EQ(isect[1], 0);
  EXPECT_EQ(isect2[1], 0);

  // Check no intersection.
  auto seg5 = std::make_pair(Vector2<double>(-1, 1), Vector2<double>(-1, -.1));
  EXPECT_EQ(Triangle2<double>::IntersectSegs(seg1, seg5, &isect, &isect2),
            Triangle2<double>::kSegSegNoIntersect);
}

TEST_F(Triangle2Test, ParallelSegSegIntersection) {
  auto seg1 = std::make_pair(Vector2<double>(0, 0), Vector2<double>(2, 0));
  auto seg2 = std::make_pair(Vector2<double>(1, 0), Vector2<double>(3, 0));
  auto seg3 = std::make_pair(Vector2<double>(3, 0), Vector2<double>(4, 0));
  Vector2<double> isect, isect2;
  EXPECT_EQ(Triangle2<double>::IntersectParallelSegs(
      seg1, seg2, &isect, &isect2), Triangle2<double>::kSegSegEdge);
  EXPECT_TRUE((isect[0] == 1 && isect2[0] == 2) ||
              (isect[0] == 2 && isect2[0] == 1));
  EXPECT_EQ(Triangle2<double>::IntersectParallelSegs(
      seg1, seg3, &isect, &isect2), Triangle2<double>::kSegSegNoIntersect);
}

// Checks intersection between two triangles in 2D.
TEST_F(Triangle2Test, TriTriIntersection) {
  Triangle2<double> tA(Vector2<double>(-.5, 0), Vector2<double>(.5, 0),
                       Vector2<double>(0, 1));
  Triangle2<double> tB(Vector2<double>(-.5, 2), Vector2<double>(.5, 2),
                       Vector2<double>(0, 3));
  Triangle2<double> tC(Vector2<double>(-.5, -1), Vector2<double>(.5, -1),
                       Vector2<double>(0, 0));
  Triangle2<double> tD(Vector2<double>(-.25, 0), Vector2<double>(0, -1),
                       Vector2<double>(.25, 0));
  Triangle2<double> tE(Vector2<double>(-.5, 1), Vector2<double>(0, 0),
                       Vector2<double>(.5, 1));

  // Should be no intersection between tA and tB.
  Vector2<double> intersections[6];
  EXPECT_EQ(tA.Intersect(tB, &intersections[0]), 0);

  // Should be exactly three intersections between tA and itself.
  EXPECT_EQ(tA.Intersect(tA, &intersections[0]), 3);

  // Should be exactly one intersection between tA and tC (one vertex from tC
  // intersects tA).
  EXPECT_EQ(tA.Intersect(tC, &intersections[0]), 1);
  EXPECT_EQ(intersections[0][0], 0);
  EXPECT_EQ(intersections[0][1], 0);

  // Should be two intersections between tA and tD (due to a shared edge).
  EXPECT_EQ(tA.Intersect(tD, &intersections[0]), 2);

  // Should be four intersections between tA and tE (the shape of intersection
  // looks like a diamond).
  EXPECT_EQ(tA.Intersect(tE, &intersections[0]), 4);
}

}  // namespace multibody 
}  // namespace drake
