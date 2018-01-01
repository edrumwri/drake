#pragma once

/// @file
/// Template method implementations for triangle.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "triangle.h"
namespace drake {
namespace multibody {

// Applies the separating axis theorem to points and axes.
template <class T>
template <class ForwardIterator, class RandomAccessIterator1,
    class RandomAccessIterator2>
T Triangle2<T>::ApplySeparatingAxisTheorem(
    ForwardIterator axes_begin, ForwardIterator axes_end,
    RandomAccessIterator1 points_A_begin, RandomAccessIterator1 points_A_end,
    RandomAccessIterator1 points_B_begin, RandomAccessIterator1 points_B_end,
    RandomAccessIterator2 projs_A_begin, RandomAccessIterator2 projs_B_begin) {
  using std::max;
  using std::min;

  // Get the number of points.
  const int num_points_A = std::distance(points_A_begin, points_A_end);
  const int num_points_B = std::distance(points_B_begin, points_B_end);

  // Set the separation to infinity initially.
  T separation = -std::numeric_limits<double>::infinity();

  // Determine the minimum overlap / maximum separation.
  for (auto axis = axes_begin; axis != axes_end; ++axis) {
    // Project all points from A.
    auto proj = projs_A_begin;
    auto point = points_A_begin;
    for (point = points_A_begin, proj = projs_A_begin; point != points_A_end; ++point)
      *proj++ = axis->dot(*point);

    // Project all points from B.
    for (point = points_B_begin, proj = projs_B_begin; point != points_B_end; ++point)
      *proj++ = axis->dot(*point);

    // Sort the projections.
    std::sort(projs_A_begin, projs_A_begin + num_points_A);
    std::sort(projs_B_begin, projs_B_begin + num_points_B);

    // Determine the greater distance.
    const T dist1 = projs_B_begin[0] - projs_A_begin[num_points_A - 1];
    const T dist2 = projs_A_begin[0] - projs_B_begin[num_points_B - 1];
    const T dist = max(dist1, dist2);
    separation = max(separation, dist);
  }

  return separation;
}

// Computes the signed distance between two line segments.
template <class T>
T Triangle2<T>::CalcSignedDistance(
    const std::pair<Vector2<T>, Vector2<T>>& seg1,
    const std::pair<Vector2<T>, Vector2<T>>& seg2) {
  return ApplySeparatingAxisTheorem(seg1, seg2);
}

// Applies the separating axis theorem to two line segments.
template <class T>
T Triangle2<T>::ApplySeparatingAxisTheorem(
    const std::pair<Vector2<T>, Vector2<T>>& seg1,
    const std::pair<Vector2<T>, Vector2<T>>& seg2) {
  // Construct a 90 degree rotation matrix.
  Eigen::Rotation2D<T> R(M_PI_2);

  // Determine the candidate axes.
  const int num_axes = 4;
  Vector2<T> axes[num_axes];
  axes[0] = seg1.second - seg1.first;
  axes[1] = seg2.second - seg2.first;
  axes[2] = R * axes[0];
  axes[3] = R * axes[1];

  // Normalize all axes.
  for (int i = 0; i < num_axes; ++i)
    axes[i].normalize();

  // Determine the set of points.
  const int num_points_A = 2;
  const int num_points_B = 2;
  const Vector2<T> pointsA[num_points_A] = { seg1.first, seg1.second };
  const Vector2<T> pointsB[num_points_B] = { seg2.first, seg2.second };
  T proj_A[num_points_A];
  T proj_B[num_points_B];

  return ApplySeparatingAxisTheorem(axes, axes + num_axes, pointsA,
                                    pointsA + num_points_A, pointsB,
                                    pointsB + num_points_B, proj_A, proj_B);
}

// Applies the separating axis theorem to the triangle and a line segment.
template <class T>
T Triangle2<T>::ApplySeparatingAxisTheorem(
    const std::pair<Vector2<T>, Vector2<T>>& seg) const {
  // Construct a 90 degree rotation matrix.
  Eigen::Rotation2D<T> R(M_PI_2);

  // Determine the candidate axes.
  const int num_axes = 8;
  Vector2<T> axes[num_axes];
  axes[0] = seg.second - seg.first;
  axes[1] = b() - a();
  axes[2] = c() - b();
  axes[3] = a() - c();
  for (int i = 0; i < 4; ++i)
    axes[i+4] = R * axes[i];

  // Normalize all axes.
  for (int i = 0; i < num_axes; ++i)
    axes[i].normalize();

  // Determine the set of points.
  const int num_points_A = 3;
  const int num_points_B = 2;
  const Vector2<T> pointsA[num_points_A] = { a(), b(), c() };
  const Vector2<T> pointsB[num_points_B] = { seg.first, seg.second };
  T proj_A[num_points_A];
  T proj_B[num_points_B];

  return ApplySeparatingAxisTheorem(axes, axes + num_axes, pointsA,
                                    pointsA + num_points_A, pointsB,
                                    pointsB + num_points_B, proj_A, proj_B);
}

// Applies the separating axis theorem to two triangles.
template <class T>
T Triangle2<T>::ApplySeparatingAxisTheorem(const Triangle2<T>& t) const {
  // Construct a 90 degree rotation matrix.
  Eigen::Rotation2D<T> R(M_PI_2);

  // Determine the candidate axes.
  const int num_axes = 12;
  Vector2<T> axes[num_axes];
  axes[0] = b() - a();
  axes[1] = c() - b();
  axes[2] = a() - c();
  axes[3] = t.b() - t.a();
  axes[4] = t.c() - t.b();
  axes[5] = t.a() - t.c();
  for (int i = 0; i < 6; ++i)
    axes[i+6] = R * axes[i];

  // Normalize all axes.
  for (int i = 0; i < num_axes; ++i)
    axes[i].normalize();

  // Determine the set of points.
  const int num_points_A = 3;
  const int num_points_B = 3;
  const Vector2<T> pointsA[num_points_A] = { a(), b(), c() };
  const Vector2<T> pointsB[num_points_B] = { t.a(), t.b(), t.c() };
  T proj_A[num_points_A];
  T proj_B[num_points_B];

  return ApplySeparatingAxisTheorem(axes, axes + num_axes, pointsA,
                                    pointsA + num_points_A, pointsB,
                                    pointsB + num_points_B, proj_A, proj_B);
}

// Computes the signed distance between this triangle and the given point.
template <class T>
T Triangle2<T>::CalcSignedDistance(const Vector2<T>& p) const {
  using std::min;
  using std::sqrt;

  if (PointInside(p)) {
    // Instantiate line segments.
    const auto ab = std::make_pair(a(), b());
    const auto bc = std::make_pair(b(), c());
    const auto ca = std::make_pair(c(), a());

    // Calculate the square distance from each line segment.
    Vector2<T> dummy;
    const T ab_dist = CalcSquareDistance(ab, p, &dummy);
    const T bc_dist = CalcSquareDistance(bc, p, &dummy);
    const T ca_dist = CalcSquareDistance(ca, p, &dummy);

    return -sqrt(min(ab_dist, min(bc_dist, ca_dist)));
  } else {
    const Vector3<T> a_3d(a()[0], a()[1], 0);
    const Vector3<T> b_3d(b()[0], b()[1], 0);
    const Vector3<T> c_3d(c()[0], c()[1], 0);
    const Triangle3<T> t(&a_3d, &b_3d, &c_3d);
    const Vector3<T> p_3d(p[0], p[1], 0);
    Vector3<T> dummy;
    return sqrt(t.CalcSquareDistance(p_3d, &dummy));
  }
}

// Computes distance between a point and a line segment in 2D.
template <class T>
T Triangle2<T>::CalcSquareDistance(const std::pair<Vector2<T>, Vector2<T>>& seg,
                                   const Vector2<T>& point,
                                   Vector2<T>* closest_point_on_seg) const {
  const auto v = seg.second - seg.first;
  const auto w = point - seg.first;
  const T c1 = w.dot(v);
  if (c1 <= 0) {
    *closest_point_on_seg = seg.first;
    return w.squaredNorm();
  }

  const T c2 = v.dot(v);
  if (c2 <= c1) {
    *closest_point_on_seg = seg.second;
    return (point - seg.second).squaredNorm();
  }

  const T t = c1 / c2;
  *closest_point_on_seg = seg.first + v * t;
  return (*closest_point_on_seg - point).squaredNorm();
}

// Computes closest points between two line segments in 2D.
template <class T>
T Triangle2<T>::CalcClosestPoints(
    const std::pair<Vector2<T>, Vector2<T>>& seg1,
    const std::pair<Vector2<T>, Vector2<T>>& seg2,
    Vector2<T>* p1,
    Vector2<T>* p2) {
  using std::abs;

  DRAKE_DEMAND(p1 && p2);

  // TODO: Compute a proper zero tolerance.
  const double zero_tol = std::numeric_limits<double>::epsilon();

  const auto u = seg1.second - seg1.first;
  const auto v = seg2.second - seg2.first;
  const auto w = seg1.first - seg2.first;
  const T a = u.dot(u);
  const T b = u.dot(v);
  const T c = v.dot(v);
  const T d = u.dot(w);
  const T e = v.dot(w);
  const T det = a*c - b*b;

  T s_denom = det, t_denom = det;
  T s_num, t_num;
  if (abs(det) < zero_tol) {
    // The lines are effectively parallel; force using the endpoint on seg1
    // to prevent possible division by zero later.
    s_num = 0;
    s_denom = 1;
    t_num = e;
    t_denom = c;
  } else {
    // Get the closest points on the infinite lines.
    s_num = (b*e - c*d);
    t_num = (a*e - b*d);

    // Clip s, if necessary.
    if (s_num < 0) {
      s_num = 0;
      t_num = e;
      t_denom = c;
    } else {
      if (s_num > s_denom) {
        s_num = s_denom;
        t_denom = e + b;
        t_denom = c;
      }
    }

    // Clip t, if necessary.
    if (t_num < 0) {
      t_num = 0;
      if (-d < 0) {
        s_num = 0;
      } else {
        if (-d > a) {
          s_num = s_denom;
        } else {
          s_num = -d;
          s_denom = a;
        }
      }
    } else {
      if (t_num > t_denom) {
        t_num = t_denom;
        if (-d + b < 0) {
          s_num = 0;
        } else {
          if (-d + b > a) {
            s_num = s_denom;
          } else {
            s_num = -d + b;
            s_denom = a;
          }
        }
      }
    }
  }

  // Do the division to get s and t.
  const T s = (abs(s_num) < zero_tol) ? 0 : s_num / s_denom;
  const T t = (abs(t_num) < zero_tol) ? 0 : t_num / t_denom;

  // Compute p1 and p2.
  *p1 = seg1.first + u * s;
  *p2 = seg2.first + v * t;

  return (*p1 - *p2).norm();
}

// Computes the signed distance between this triangle and the given segment.
template <class T>
T Triangle2<T>::CalcSignedDistance(
    const std::pair<Vector2<T>, Vector2<T>>& seg) const {
  using std::sqrt;

  return ApplySeparatingAxisTheorem(seg);
/*

  // TODO: Use a properly computed threshold.
  const T zero_tol = std::numeric_limits<double>::epsilon();

  // Compute the location between this triangle and the line segment.
  Vector2<T> isect0, isect1;
  SegTriIntersectType isect_type = Intersect(seg, zero_tol, &isect0, &isect1);
  switch (isect_type) {
    // All of these cases are kissing intersections.
    case SegTriIntersectType::kSegTriVertex:
    case SegTriIntersectType::kSegTriEdge:
    case SegTriIntersectType::kSegTriEdgeOverlap:
      return 0;

    // The segment is fully inside the triangle. Use the separating axis
    // theorem to determine the signed distance.
    case SegTriIntersectType::kSegTriInside:  {
      return ApplySeparatingAxisTheorem(seg);
    }

    case SegTriIntersectType::kSegTriNoIntersect:  {
      // Compute the square distance between the line segment and the triangle.
      const Vector3<T> a_3d(a()[0], a()[1], 0);
      const Vector3<T> b_3d(b()[0], b()[1], 0);
      const Vector3<T> c_3d(c()[0], c()[1], 0);
      const Triangle3<T> t(&a_3d, &b_3d, &c_3d);
      const Vector3<T> p1_3d(seg.first[0], seg.first[1], 0);
      const Vector3<T> p2_3d(seg.second[0], seg.second[1], 0);
      Vector3<T> dummy1, dummy2;
      return sqrt(t.CalcSquareDistance(std::make_pair(p1_3d, p2_3d),
                                       &dummy1, &dummy2));
    }

    // The segment is fully inside the triangle. Use the separating axis theorem
    // to determine the signed distance.
    case SegTriIntersectType::kSegTriPlanarIntersect:
      return ApplySeparatingAxisTheorem(seg);

    default:
      DRAKE_ABORT();
  }
  */
}

// Computes the signed distance between this triangle and the given triangle.
template <class T>
T Triangle2<T>::CalcSignedDistance(
    const Triangle2<T>& t) const {
  using std::sqrt;

  // Intersect the triangles.
  Vector2<T> isects[6];
  int num_intersections = Intersect(t, &isects[0]);

  // If the intersection is empty, the triangles are disjoint. Return the
  // Euclidean distance between the triangles.
  if (num_intersections == 0) {
    const Vector3 <T> a_3d(a()[0], a()[1], 0);
    const Vector3 <T> b_3d(b()[0], b()[1], 0);
    const Vector3 <T> c_3d(c()[0], c()[1], 0);
    const Vector3 <T> t_a_3d(t.a()[0], t.a()[1], 0);
    const Vector3 <T> t_b_3d(t.b()[0], t.b()[1], 0);
    const Vector3 <T> t_c_3d(t.c()[0], t.c()[1], 0);
    const Triangle3<T> this_3d(&a_3d, &b_3d, &c_3d);
    const Triangle3<T> t_3d(&t_a_3d, &t_b_3d, &t_c_3d);
    Vector3 <T> dummy1, dummy2;
    return sqrt(this_3d.CalcSquareDistance(t_3d, &dummy1, &dummy2));
  } else {
    return ApplySeparatingAxisTheorem(t);
  }
}

// Partial template specializations on three dimensional triangles follow.
template <class T>
Vector3<T> Triangle3<T>::CalcNormal() const {
  const Vector3<T> e1 = b() - a();
  const Vector3<T> e2 = c() - b();
  return e1.cross(e2).normalized();
}

/// Determines the distance between a triangle and a line and the closest
/// point on the line to the triangle.
/// @param origin the origin of the line
/// @param dir the direction of the line 
/// @param[out] closest_point_on_line the closest point on the line, on
///             return
/// @param[out] closest_point_on_tri the closest point on the triangle, on
///             return.
/// @param[out] The point on the line, defined origin + dir*t_line, on return.
/// @return the Euclidean distance between this triangle and the point.
/// \note code adapted from www.geometrictools.com 
template <class T>
T Triangle3<T>::CalcSquareDistance(
    const Vector3<T>& origin,
    const Vector3<T>& dir,
    Vector3<T>* closest_point_on_line,
    Vector3<T>* closest_point_on_tri,
    T* t_line) const {
  using std::abs;

  // TODO: replace this with a more informed tolerance.
  const double zero_tol = 10 * std::numeric_limits<double>::epsilon(); 

  // Test if line intersects triangle. If so, the squared distance is zero.
  Vector3<T> edge0 = b() - a();
  Vector3<T> edge1 = c() - a();
  Vector3<T> normal = CalcNormal();

  // get the direction and origin of the line segment
  T NdD = normal.dot(dir);
  if (abs(NdD) > zero_tol) {
    // The line and triangle are not parallel, so the line intersects
    // the plane of the triangle.
    Vector3<T> kDiff = origin - a();
    Matrix3<T> R = math::ComputeBasisFromAxis(0, dir);
    const Vector3<T> kU = R.col(1);
    const Vector3<T> kV = R.col(2);
    T UdE0 = kU.dot(edge0);
    T UdE1 = kU.dot(edge1);
    T UdDiff = kU.dot(kDiff);
    T VdE0 = kV.dot(edge0);
    T VdE1 = kV.dot(edge1);
    T VdDiff = kV.dot(kDiff);
    T inv_det = 1.0/(UdE0*VdE1 - UdE1*VdE0);

    // Barycentric coordinates for the point of intersection.
    T B1 = (VdE1*UdDiff - UdE1*VdDiff)*inv_det;
    T B2 = (UdE0*VdDiff - VdE0*UdDiff)*inv_det;
    T B0 = 1.0 - B1 - B2;

    if (B0 >= 0.0 && B1 >= 0.0 && B2 >= 0.0) {
      // Line parameter for the point of intersection.
      T DdE0 = dir.dot(edge0);
      T DdE1 = dir.dot(edge1);
      T DdDiff = dir.dot(kDiff);
      *t_line = B1*DdE0 + B2*DdE1 - DdDiff;

      // The intersection point is inside or on the triangle.
      *closest_point_on_line = origin + (*t_line)*dir;
      *closest_point_on_tri = a() + B1*edge0 + B2*edge1;
      return T(0);
    }
  }

  // Either (1) the line is not parallel to the triangle and the point of
  // intersection of the line and the plane of the triangle is outside the
  // triangle or (2) the line and triangle are parallel.  Regardless, the
  // closest point on the triangle is on an edge of the triangle.  Compare
  // the line to all three edges of the triangle.
  T square_dist = std::numeric_limits<T>::max();
  for (int i0 = 2, i1 = 0; i1 < 3; i0 = i1++) {
    // Construct the edge.
    std::pair<Vector3<T>, Vector3<T>> edge(
      get_vertex(i0), get_vertex(i1));

    // Compute the squared distance and closest points.
    T t, t_seg;
    Vector3<T> temp_point1, temp_point2;
    T edge_square_dist = CalcSquareDistance(
        origin, dir, edge, &temp_point1, &temp_point2, &t, &t_seg);
    if (edge_square_dist < square_dist) {
      *closest_point_on_line = temp_point1;
      *closest_point_on_tri = temp_point2;
      square_dist = edge_square_dist;
      *t_line = t;
    }
  }

  return square_dist;
}

/// Determines the distance between a line and a line segment.
/// @param[out] t_seg the value that corresponds to the closest point on the
///             line segment, such that closest_point_on_seg = seg.first +
///             (seg.second - seg.first)*t_seg 
template <class T>
T Triangle3<T>::CalcSquareDistance(
    const Vector3<T>& origin,
    const Vector3<T>& dir,
    const std::pair<Vector3<T>, Vector3<T>>& seg,
    Vector3<T>* closest_point_on_line,
    Vector3<T>* closest_point_on_seg,
    T* t_line,
    T* t_seg) {
  DRAKE_DEMAND(closest_point_on_line);
  DRAKE_DEMAND(closest_point_on_seg);
  DRAKE_DEMAND(t_line);
  DRAKE_DEMAND(t_seg);

  using std::abs;

  // TODO: come up with a better tolerance.
  const double zero_tol = 1e-8;

  // determine the origins of the segment
  const Vector3<T> seg_origin = (seg.first + seg.second) * 0.5;

  // determine the directions of the segment
  Vector3<T> seg_dir = seg.second - seg.first;
  const T seg_dir_len = seg_dir.norm();
  seg_dir /= seg_dir_len;

  // determine the extents of the segment
  const T seg_extent = seg_dir_len * 0.5;

  const Vector3<T> kDiff = origin - seg_origin;
  T A01 = -dir.dot(seg_dir);
  T B0 = kDiff.dot(dir);
  T C = kDiff.squaredNorm();
  T det = abs(1.0 - A01 * A01);
  T B1, S0, S1, sqr_dist, ext_det;

  if (det >= zero_tol) {
    // The line and segment are not parallel.
    B1 = -kDiff.dot(seg_dir);
    S1 = A01 * B0 - B1;
    ext_det = seg_extent * det;

    if (S1 >= -ext_det) {
      if (S1 <= ext_det) {
        // Two interior points are closest, one on the line and one
        // on the segment.
        T inv_det = (1.0) / det;
        S0 = (A01 * B1 - B0) * inv_det;
        S1 *= inv_det;
        sqr_dist = S0 * (S0 + A01 * S1 + 2 * B0) +
                   S1 * (A01 * S0 + S1 + 2 * B1) + C;
      } else {
        // The end point e1 of the segment and an interior point of
        // the line are closest.
        S1 = seg_extent;
        S0 = -(A01 * S1 + B0);
        sqr_dist = -S0 * S0 + S1 * (S1 + 2 * B1) + C;
      } 
    } else {
      // The end point e0 of the segment and an interior point of the
      // line are closest.
      S1 = -seg_extent;
      S0 = -(A01 * S1 + B0);
      sqr_dist = -S0 * S0 + S1 * (S1 + 2 * B1) + C;
    }
  } else {
    // The line and segment are parallel.  Choose the closest pair so that
    // one point is at segment origin.
    S1 = T(0);
    S0 = -B0;
    sqr_dist = B0 * S0 + C;
  }

  *closest_point_on_line = origin + S0*dir;
  *closest_point_on_seg = seg_origin + S1*seg_dir;
  *t_line = S0;

  // Determine parameter using formula:
  // seg.first + (seg.second - seg.first)*t_line = closest_point_on_seg
  // t_line = (closest_point_on_seg - seg.first)/(seg.second - seg.first).
  *t_seg = (*closest_point_on_seg - seg.first).norm() /
           (seg.second - seg.first).norm();

  return abs(sqr_dist);
}

/// Determines the distance between this triangle and a given point
/// @param point the query point
/// @param[out] sout the Barycentric coordinate s, where
///             a*s + b*t + c*(1-s-t) = p
/// @param[out] tout the Barycentric coordinate t 
/// @return the Euclidean distance between this triangle and the point
template <class T>
T Triangle3<T>::CalcSquareDistance(
    const Vector3<T>& point, T* sout, T* tout) const {
  using std::abs;

  // compute needed quantities
  const Vector3<T> k_diff = a() - point;
  const Vector3<T> E0 = b() - a();
  const Vector3<T> E1 = c() - a();
  T A00 = E0.squaredNorm();
  T A01 = E0.dot(E1);
  T A11 = E1.squaredNorm(); 
  T B0 = k_diff.dot(E0);
  T B1 = k_diff.dot(E1);
  T C = k_diff.squaredNorm();
  T det = abs(A00 * A11 - A01 * A01);
  T s = A01 * B1 - A11 * B0;
  T t = A01 * B0 - A00 * B1;
  T sqr_dist;

  if (s + t <= det) {
    if (s < 0) {
      if (t < 0) {
        // region 4
        if (B0 < 0) {
          t = 0.0;
          if (-B0 >= A00) {
            s = 1.0;
            sqr_dist = A00 + (2 * B0) + C;
          } else {
            s = -B0/A00;
            sqr_dist = (B0*s)+C;
          }
        } else {
          s = 0.0;
          if (B1 >= 0) {
            t = 0.0;
            sqr_dist = C;
          } else {
            if (-B1 >= A11) {
              t = 1.0;
              sqr_dist = A11 + (2 * B1) + C;
            } else {
              t = -B1 / A11;
              sqr_dist = (B1 * t) + C;
            }
          }  
        } 
      } else {
        // region 3
        s = 0;
        if (B1 >= 0) {
          t = 0;
          sqr_dist = C;
        } else {
          if (-B1 >= A11) {
            t = 1.0;
            sqr_dist = A11 + (2 * B1) + C;
          } else {
            t = -B1 / A11;
            sqr_dist = (B1 * t) + C;
          }
        }
      } 
    } else {
      if (t < 0) {
        // region 5
        t = 0;
        if (B0 >= 0) {
          s = 0;
          sqr_dist = C;
        } else {
          if (-B0 >= A00) {
            s = 1.0;
            sqr_dist = A00 + (2 * B0) + C;
          } else {
            s = -B0 / A00;
            sqr_dist = (B0 * s) + C;
          }
        }
      } else {
        // region 0
        const double inv_det = 1.0/det;
        s *= inv_det;
        t *= inv_det;
        sqr_dist = s * (A00 * s + A01 * t + (2 * B0)) +
                   t * (A01 * s + A11* t + (2 * B1)) + C;
      }
    }
  } else {
    T tmp0, tmp1, numer, denom;
    if (s < 0) {
      // region 2
    tmp0 = A01 + B0;
    tmp1 = A11 + B1;
    if (tmp1 > tmp0) {
      numer = tmp1 - tmp0;
      denom = A00 - 2 * A01 + A11;
      if (numer >= denom) {
        s = 1.0;
        t = 0.0;
        sqr_dist = A00 + (2 * B0) + C;
      } else {
        s = numer/denom;
        t = 1.0 - s;
        sqr_dist = s * (A00 * s + A01 * t + 2 * B0) +
                   t * (A01 * s + A11 * t + 2 * B1) + C;
      }
    } else {
      s = 0.0;
      if (tmp1 <= 0) {
        t = 1.0;
        sqr_dist = A11 + 2 * B1 + C;
      } else {
        if (B1 >= 0) {
          t = 0.0;
          sqr_dist = C;
        } else {
          t = -B1 / A11;
          sqr_dist = B1 * t + C;
        }
      }
    } 
  } else {
    if (t < 0) {
      // region 6
      tmp0 = A01 + B1;
      tmp1 = A00 + B0;
      if (tmp1 > tmp0) {
        numer = tmp1 - tmp0;
        denom = A00 - 2 * A01 + A11;
        if (numer >= denom) {
          t = 1.0;
          s = 0.0;
          sqr_dist = A11 + 2 * B1 + C;
        } else {
          t = numer / denom;
          s = 1.0 - t;
          sqr_dist = s * (A00 * s + A01 * t + 2 * B0) +
                     t * (A01 * s + A11 * t + 2 *B1) + C;
        }
      } else {
        t = 0.0;
        if (tmp1 <= 0) {
          s = 1.0;
          sqr_dist = A00 + 2 * B0+C;
        } else {
          if (B0 >= 0) {
            s = 0.0;
            sqr_dist = C;
          } else {
            s = -B0 / A00;
            sqr_dist = B0 * s + C;
          }
        }
      } 
    } else {
      // region 1
      numer = A11 + B1 - A01 - B0;
      if (numer <= 0) {
        s = 0.0;
        t = 1.0;
        sqr_dist = A11 + 2 * B1 + C;
        } else {
          denom = A00 - 2 * A01 + A11;
          if (numer >= denom) {
            s = 1.0;
            t = 0.0;
            sqr_dist = A00 + 2 * B0 + C;
          } else {
            s = numer / denom;
            t = 1.0 - s;
            sqr_dist =  s * (A00 * s + A01 * t + 2 * B0) +
                        t * (A01 * s + A11 * t + 2 * B1) + C;
          }
        }
      }
    } 
  }

  // Convert to standard formulation. 
  *sout = 1 - s - t;
  *tout = s;
  return sqr_dist;
}

/// Determines the signed distance between this triangle and a given point
/// @param query_point the query point
/// @param[out] closest_point the closest point on the triangle, on return.
/// @return the square distance between this triangle and the point
template <class T>
T Triangle3<T>::CalcSquareDistance(
    const Vector3<T>& query_point,
    Vector3<T>* closest_point) const {
  DRAKE_DEMAND(closest_point);

  // Get the closest point in barycentric coordinates.
  T s, t;
  T square_distance = CalcSquareDistance(query_point, &s, &t);

  // compute the closest point
  *closest_point = a()*s + b()*t + c()*(1-s-t);

  return square_distance;
}

/// Computes the closest distance between a triangle and a line segment.
template <class T>
T Triangle3<T>::CalcSquareDistance(
    const std::pair<Vector3<T>, Vector3<T>>& seg,
    Vector3<T>* closest_point_on_tri,
    Vector3<T>* closest_point_on_seg) const {
  // Compute the segment origin, direction, and extent.
  Vector3<T> origin = (seg.first + seg.second) * 0.5;
  Vector3<T> dir = seg.second - seg.first;
  T dir_len = dir.norm();
  T extent = dir_len * 0.5;
  dir /= dir_len;

  // Get the squared distance between a line containing the segment and
  // the triangle.
  T t_line;
  T square_dist = CalcSquareDistance(
      origin, dir, closest_point_on_seg, closest_point_on_tri, &t_line);

  if (t_line >= -extent) {
    if (t_line <= extent) {
      return square_dist;
    } else {
      *closest_point_on_seg = seg.second;
    }
  } else {
    *closest_point_on_seg = seg.first;
  }

  // If at this point, it is necessary to compute the closest point to the
  // triangle.
  return CalcSquareDistance(*closest_point_on_seg, closest_point_on_tri);
}

/// Computes the square distance between two triangles and returns closest
/// points on each.
/// @param point the query point
/// @param closest_point the closest point on the triangle is returned here
/// @return the squared Euclidean distance between this triangle and the point
/// @note code adapted from www.geometrictools.com 
template <class T>
T Triangle3<T>::CalcSquareDistance(
      const Triangle3<T>& t,
      Vector3<T>* closest_point_on_this,
      Vector3<T>* closest_point_on_t) const {
  Vector3<T> tmp1, tmp2;

  // Compare edges of t1 to the interior of t2
  T square_dist = std::numeric_limits<T>::max();
  std::pair<Vector3<T>, Vector3<T>> seg;
  for (int i0 = 2, i1 = 0; i1 < 3; i0 = i1++) {
    // Compute the distance between the triangle and the line segment.
    seg.first = get_vertex(i0);
    seg.second = get_vertex(i1);
    T square_seg_dist = t.CalcSquareDistance(seg, &tmp2, &tmp1);
    if (square_seg_dist < square_dist) {
      *closest_point_on_this = tmp2;
      *closest_point_on_t = tmp1;
      square_dist = square_seg_dist;
    }
  }

  // compare edges of t2 to the interior of t1
  for (int i0 = 2, i1 = 0; i1 < 3; i0 = i1++) {
    seg.first = t.get_vertex(i0);
    seg.second = t.get_vertex(i1);
    T square_seg_dist = CalcSquareDistance(seg, &tmp1, &tmp2);
    if (square_seg_dist < square_dist) {
      *closest_point_on_this = tmp1;
      *closest_point_on_t = tmp2;
      square_dist = square_seg_dist;
    }
  }

  return square_dist;
}

/// Projects a triangle to 2d, given a projection matrix.
template <class T>
Triangle2<T> Triangle3<T>::ProjectTo2d(
    const Eigen::Matrix<T, 2, 3>& P) const {
  // Create the new triangle.
  return Triangle2<T>(P * a(), P * b(), P * c());
}

/// Computes the sign of the area of two vectors with respect to an arbitrary
/// center.
template <class T>
typename Triangle2<T>::OrientationType Triangle2<T>::CalcAreaSign(
    const Vector2<T>& a, const Vector2<T>& b, const Vector2<T>& c, T tol) {
  using std::abs;

  DRAKE_DEMAND(tol >= 0);
  const T a1 = (b[0] - a[0]) * (c[1] - a[1]);
  const T a2 = (c[0] - a[0]) * (b[1] - a[1]);
  const T area = a1 - a2;
  if (abs(area) < tol)
    return Triangle2<T>::OrientationType::kOn;
  return (area > 0) ? Triangle2<T>::OrientationType::kLeft :
                      Triangle2<T>::OrientationType::kRight;
}

/// Determines whether a point is strictly inside or on a triangle.
template <class T>
bool Triangle2<T>::PointInside(const Vector2<T>& point) const {
  auto polygon_location = GetLocation(point);
  return (polygon_location != kPolygonOutside);
}

/// Determines the location of a point on the triangle.
/// Adapted from O'Rourke, p. 236.
template <class T>
typename Triangle2<T>::PolygonLocationType Triangle2<T>::GetLocation(
    const Vector2<T>& point) const {
  // TODO: Determine tolerance in a more principled manner.
  const T tol = 1e-8;

  // Get the signed areas.
  auto o1 = CalcAreaSign(point, a(), b(), tol);
  auto o2 = CalcAreaSign(point, b(), c(), tol);
  auto o3 = CalcAreaSign(point, c(), a(), tol);

  if ((o1 == kOn && o2 == kLeft && o3 == kLeft) ||
      (o2 == kOn && o1 == kLeft && o3 == kLeft) ||
      (o3 == kOn && o1 == kLeft && o2 == kLeft))
    return kPolygonOnEdge;

  if ((o1 == kOn && o2 == kRight && o3 == kLeft) ||
      (o2 == kOn && o1 == kRight && o3 == kLeft) ||
      (o3 == kOn && o1 == kRight && o2 == kLeft))
    return kPolygonOnEdge;
  
  if ((o1 == kLeft && o2 == kLeft && o3 == kLeft) ||
      (o1 == kRight && o2 == kRight && o3 == kRight))
    return kPolygonInside; 

  if ((o1 == kOn && o2 == kOn) ||
      (o1 == kOn && o3 == kOn) ||
      (o2 == kOn && o3 == kOn))
    return kPolygonOnVertex; 

  // Some cases are clearly impossible; we will handle those here.

  // We must assume that one of the areas is close to zero (right on the zero
  // side of our tolerance) - it could be the case where two or even all three
  // areas are off.
  if (o1 == kOn && o2 == kOn && o3 == kOn)
    return kPolygonOnVertex;  // Point is on a vertex of the triangle.

  // If none of the other cases apply, point must be outside of the triangle.
  return kPolygonOutside;
}

/// Determines the intersection between a line segment and triangle in 2D
/**
 * \param seg a line segment in 2D
 * \param tri a triangle in 2D
 * \param isect contains the point of intersection, if any (on return)
 * \param isect2 contains a second point of intersection, if the intersection
 *               type is kSegTriInside, kSegTriEdgeOverlap, or 
 *               kSegTriPlanarIntersect 
 *               (on return)
 * \return kSegTriInside (if the segment lies wholly within the triangle face
 *         kSegTriVertex (if an endpoint of the segment coincides with a vertex
 *         of the triangle), kSegTriEdge (if an endpoint of the segment is in
 *         the relative interior of an edge of the triangle),  
 *         kSegTriEdgeOverlap (if the segment overlaps one edge of the
 *         triangle [or vice versa]), kSegTriPlanarIntersect (if the
 *         segment lies partially within the face of the triangle), or
 *         kSegTriNoIntersect (if there is no intersection).
 * \note assumes triangle is oriented ccw 
 */
template <class T>
typename Triangle2<T>::SegTriIntersectType Triangle2<T>::Intersect(
    const std::pair<Vector2<T>, Vector2<T>>& seg, T tol,
    Vector2<T>* isect, Vector2<T>* isect2) const {
  DRAKE_DEMAND(isect);
  DRAKE_DEMAND(isect2);
  using std::min;
  using std::max;
  using std::abs;

  // get the two points
  const Vector2<T>& p = seg.first;
  const Vector2<T>& q = seg.second;

  // make segments out of the edges
  auto e1 = std::make_pair(a(), b());
  auto e2 = std::make_pair(b(), c());
  auto e3 = std::make_pair(c(), a());

  // get the orientations w.r.t. the edges
  OrientationType pe1 = CalcAreaSign(a(), b(), p, tol);
  OrientationType pe2 = CalcAreaSign(b(), c(), p, tol);
  OrientationType pe3 = CalcAreaSign(c(), a(), p, tol);
  OrientationType qe1 = CalcAreaSign(a(), b(), q, tol);
  OrientationType qe2 = CalcAreaSign(b(), c(), q, tol);
  OrientationType qe3 = CalcAreaSign(c(), a(), q, tol);

  // see whether points are outside (most common)
  if ((pe1 == kRight && qe1 == kRight) ||
      (pe2 == kRight && qe2 == kRight) ||
      (pe3 == kRight && qe3 == kRight))
    return kSegTriNoIntersect;

  // check for edge overlap
  if (pe1 == kOn && qe1 == kOn) {
    // do parallel segment / segment intersection
    const Vector2<T> dir = e1.second - e1.first;
    T t1 = DetermineLineParam(e1.first, dir, p);
    T t2 = DetermineLineParam(e1.first, dir, q);
    if (t2 < t1)
      std::swap(t1, t2); 
    t1 = max(t1, (T) 0.0);
    t2 = min(t2, (T) 1.0);
    if (abs(t1 - t2) < tol) {
      // vertex only
      *isect = e1.first + (e1.second - e1.first) * t1;
      return kSegTriVertex;
    } else {
      if (t2 > t1) {
        // Edge overlap.
        *isect = e1.first + dir*t1;
        *isect2 = e1.first + dir*t2;
        return kSegTriEdgeOverlap;
      } else {
        // No overlap.
        return kSegTriNoIntersect;
      }
    }
  }

  if (pe2 == kOn && qe2 == kOn) {
    // do parallel segment / segment intersection
    const Vector2<T> dir = e2.second - e2.first;
    T t1 = DetermineLineParam(e2.first, dir, p);
    T t2 = DetermineLineParam(e2.first, dir, q);
    if (t2 < t1)
      std::swap(t1, t2); 
    t1 = max(t1, (T) 0.0);
    t2 = min(t2, (T) 1.0);
    if (abs(t1 - t2) < tol) {
      // vertex only
      *isect = e2.first + (e2.second - e2.first) * t1;
      return kSegTriVertex;
    } else {
      if (t2 > t1) {
        // Edge overlap.
        *isect = e2.first + dir*t1;
        *isect2 = e2.first + dir*t2;
        return kSegTriEdgeOverlap;
      } else {
        // No overlap.
        return kSegTriNoIntersect;
      }
    }
  }

  if (pe3 == kOn && qe3 == kOn) {
    const Vector2<T> dir = e3.second - e3.first; 
    T t1 = DetermineLineParam(e3.first, dir, p);
    T t2 = DetermineLineParam(e3.first, dir, q);
    if (t2 < t1)
      std::swap(t1, t2); 
    t1 = max(t1, (T) 0.0);
    t2 = min(t2, (T) 1.0);
    if (abs(t1 - t2) < tol) {
      // vertex only
      *isect = e3.first + (e3.second - e3.first)*t1;
      return kSegTriVertex;
    } else {
      if (t2 > t1) {
        // Edge overlap.
        *isect = e3.first + dir * t1;
        *isect2 = e3.first + dir * t2;
        return kSegTriEdgeOverlap;
      } else {
        // no overlap
        return kSegTriNoIntersect;
      }
    }
  }

   // check for edge or vertex intersection with edge #1
  if (pe1 == kRight) {
    // if q is on edge 1, it may be on a vertex, an edge, or neither
    if (qe1 == kOn) {
      const Vector2<T> dir = e1.second - e1.first;
      T t = DetermineLineParam(e1.first, dir, q);
      SegLocationType feat = DetermineSegLocation(t);
      *isect = e1.first + dir*t;
      if (feat == kSegOrigin || feat == kSegEndpoint) {
        return kSegTriVertex;
      } else {
        if (feat == kSegInterior)
          return kSegTriEdge;
      }
    } else {
      // q *should* be to the left of e1; see whether q is on a vertex, an
      // edge, or inside the triangle 
      PolygonLocationType qloc = GetLocation(q);
      if (qloc == kPolygonOnVertex) {
        *isect2 = q;
        return kSegTriVertex;
      } else {
        if (qloc == kPolygonOnEdge) {
          *isect2 = q;
          return kSegTriEdge;
        } else {
          if (qloc == kPolygonInside) {
            // Intersect seg vs. e1. 
            SegSegIntersectType type = IntersectSegs(seg, e1, isect, isect2);
            *isect2 = q;
            std::swap(*isect, *isect2);
            switch (type) {
              case kSegSegIntersect:  
                return kSegTriPlanarIntersect;

              case kSegSegNoIntersect:
                break;

              // none of these should occur, so we'll return the points
              // of intersection and fudge the type 
              case kSegSegVertex:  
              case kSegSegEdge:
                return kSegTriPlanarIntersect;
            }
          }
        }
      }
    }
  } else {
    if (qe1 == kRight) {
      if (pe1 == kOn) {
        const Vector2 <T> dir = e1.second - e1.first;
        T t = DetermineLineParam(e1.first, dir, p);
        SegLocationType feat = DetermineSegLocation(t);
        *isect = e1.first + dir * t;
        if (feat == kSegOrigin || feat == kSegEndpoint) {
          return kSegTriVertex;
        } else {
          if (feat == kSegInterior)
            return kSegTriEdge;
        }
      } else {
        // p *should* be to the left of e1; see whether p is on a vertex, an
        // edge, or inside the triangle
        PolygonLocationType ploc = GetLocation(p);
        if (ploc == kPolygonOnVertex) {
          *isect2 = p;
          return kSegTriVertex;
        } else {
          if (ploc == kPolygonOnEdge) {
            *isect2 = p;
            return kSegTriEdge;
          } else {
            if (ploc == kPolygonInside) {
              // Intersect seg vs. e1.
              SegSegIntersectType type = IntersectSegs(seg, e1, isect, isect2);
              *isect2 = p;
              std::swap(*isect, *isect2);
              switch (type) {
                case kSegSegIntersect:return kSegTriPlanarIntersect;

                case kSegSegNoIntersect:break;

                  // None of these should occur, so we'll return the points
                  // of intersection and fudge the type.
                case kSegSegVertex:
                case kSegSegEdge:return kSegTriPlanarIntersect;
              }
            }
          }
        }
      }
    }
  }

  // check for edge or vertex intersection with edge #2
  if (pe2 == kRight) {
    if (qe2 == kOn) {
      const Vector2 <T> dir = e2.second - e2.first;
      T t = DetermineLineParam(e2.first, dir, q);
      SegLocationType feat = DetermineSegLocation(t);
      *isect = e2.first + dir * t;
      if (feat == kSegOrigin || feat == kSegEndpoint) {
        return kSegTriVertex;
      } else {
        if (feat == kSegInterior) {
          return kSegTriEdge;
        }
      }
    } else {
      // q *should* be to the left of e2; see whether q is on a vertex, an
      // edge, or inside the triangle
      PolygonLocationType qloc = GetLocation(q);
      if (qloc == kPolygonOnVertex) {
        *isect2 = q;
        return kSegTriVertex;
      } else {
        if (qloc == kPolygonOnEdge) {
          *isect2 = q;
          return kSegTriEdge;
        } else {
          if (qloc == kPolygonInside) {
            // intersect seg vs. e2
            SegSegIntersectType type =
                IntersectSegs(seg, e2, isect, isect2);
            *isect2 = q;
            std::swap(*isect, *isect2);
            switch (type) {
              case kSegSegIntersect:
                return kSegTriPlanarIntersect;

              case kSegSegNoIntersect:
                break;

                // None of these should occur, so we'll return the points
                // of intersection and fudge the type.
              case kSegSegVertex:
              case kSegSegEdge:
                return kSegTriPlanarIntersect;
            }
          }
        }
      }
    }
  } else {
    if (qe2 == kRight) {
      if (pe2 == kOn) {
        const Vector2 <T> dir = e2.second - e2.first;
        T t = DetermineLineParam(e2.first, dir, p);
        SegLocationType feat = DetermineSegLocation(t);
        *isect = e2.first + dir * t;
        if (feat == kSegOrigin || feat == kSegEndpoint) {
          return kSegTriVertex;
        } else {
          if (feat == kSegInterior)
            return kSegTriEdge;
        }
      } else {
        // p *should* be to the left of e2; see whether p is on a vertex, an
        // edge, or inside the triangle
        PolygonLocationType ploc = GetLocation(p);
        if (ploc == kPolygonOnVertex) {
          *isect2 = p;
          return kSegTriVertex;
        } else {
          if (ploc == kPolygonOnEdge) {
            *isect2 = p;
            return kSegTriEdge;
          } else {
            if (ploc == kPolygonInside) {
              // Intersect seg vs. e2.
              SegSegIntersectType type = IntersectSegs(seg, e2, isect, isect2);
              *isect2 = p;
              std::swap(*isect, *isect2);
              switch (type) {
                case kSegSegIntersect:return kSegTriPlanarIntersect;

                case kSegSegNoIntersect:return kSegTriEdge;

                  // None of these should occur, so we'll return the points
                  // of intersection and fudge the type
                case kSegSegVertex:
                case kSegSegEdge:return kSegTriPlanarIntersect;
              }
            }
          }
        }
      }
    }
  }

  // Check for edge or vertex intersection with edge #3
  if (pe3 == kRight) {
    if (qe3 == kOn) {
      const Vector2<T> dir = e3.second - e3.first;
      T t = DetermineLineParam(e3.first, dir, q);
      SegLocationType feat = DetermineSegLocation(t);
      *isect = e3.first + dir * t;
      if (feat == kSegOrigin || feat == kSegEndpoint) {
        return kSegTriVertex;
      } else {
        if (feat == kSegInterior)
          return kSegTriEdge;
      }
    } else {
      // q *should* be to the left of e3; see whether q is on a vertex, an
      // edge, or inside the triangle 
      PolygonLocationType qloc = GetLocation(q);
      if (qloc == kPolygonOnVertex) {
        *isect2 = q;
        return kSegTriVertex;
      } else {
        if (qloc == kPolygonOnEdge) {
          *isect2 = q;
          return kSegTriEdge;
        } else {
          if (qloc == kPolygonInside) {
            // intersect seg vs. e3
            SegSegIntersectType type = IntersectSegs(seg, e3, isect, isect2);
            *isect2 = q;
            std::swap(*isect, *isect2);
            switch (type) {
              case kSegSegIntersect:  
                return kSegTriPlanarIntersect;

              case kSegSegNoIntersect:
                return kSegTriEdge;

              // None of these should occur, so we'll return the points
              // of intersection and fudge the type. 
              case kSegSegVertex:  
              case kSegSegEdge:
                return kSegTriPlanarIntersect;
            }
          }
        }
      }
    }
  } else {
    if (qe3 == kRight) {
      if (pe3 == kOn) {
        const Vector2<T> dir = e3.second - e3.first;
        T t = DetermineLineParam(e3.first, dir, p);
        SegLocationType feat = DetermineSegLocation(t);
        *isect = e3.first + dir * t;
        if (feat == kSegOrigin || feat == kSegEndpoint) {
          return kSegTriVertex;
        } else {
          if (feat == kSegInterior)
            return kSegTriEdge;
        }
      } else {
        // p *should* be to the left of e3; see whether p is on a vertex, an
        // edge, or inside the triangle
        PolygonLocationType ploc = GetLocation(p);
        if (ploc == kPolygonOnVertex) {
          *isect2 = p;
          return kSegTriVertex;
        } else {
          if (ploc == kPolygonOnEdge) {
            *isect2 = p;
            return kSegTriEdge;
          } else {
            if (ploc == kPolygonInside) {
              // intersect seg vs. e3
              SegSegIntersectType type = IntersectSegs(seg, e3, isect, isect2);
              *isect2 = p;
              std::swap(*isect, *isect2);
              switch (type) {
                case kSegSegIntersect:return kSegTriPlanarIntersect;

                case kSegSegNoIntersect:return kSegTriEdge;

                  // None of these should occur, so we'll return the points
                  // of intersection and fudge the type.
                case kSegSegVertex:
                case kSegSegEdge:return kSegTriPlanarIntersect;
              }
            }
          }
        }
      }
    }
  }
 
  // if we're here, one of two cases has occurred: both points are inside
  // the triangle or the segment does not intersect the triangle; find out
  // which it is
  if (pe1 != kRight && pe2 != kRight && pe3 != kRight &&
      qe1 != kRight && qe2 != kRight && qe3 != kRight) { 
    *isect = p;
    *isect2 = q;
    return kSegTriInside;
  } else {
//    assert(in_tri(tri, p) == ePolygonOutside && in_tri(tri, q) == ePolygonOutside);
    return kSegTriNoIntersect;
  }
}

// Determines the location of a point on a line, determined by its line
// parameter.
template <class T>
typename Triangle2<T>::SegLocationType Triangle2<T>::DetermineSegLocation(T t) {
  using std::abs;
  using std::max;

  // Setup a reasonable tolerance.
  const double tol = 10 * std::numeric_limits<double>::epsilon();

  // See whether the point is on the origin.
  if (abs(t) < tol)
    return kSegOrigin;

  // See whether the point is on the second endpoint.
  if (abs(t - 1) < tol)
    return kSegEndpoint;

  // Point is either in the interior or the exterior.
  if (t > 0 && t < 1) {
    return kSegInterior;
  } else {
    return kSegExterior;
  }
}

// Determines the parameter of a point on a line.
template <class T>
T Triangle2<T>::DetermineLineParam(
    const Vector2<T>& origin, const Vector2<T>& dir, const Vector2<T>& point) {
  const T dir_norm = dir.norm();
  DRAKE_DEMAND(dir_norm > std::numeric_limits<double>::epsilon());

  auto sgn = [](T a, T b) {
    if (b > 0) {
      return a;
    } else {
      if (b < 0)
        return -a;
      else
        return T(0);
    }
  };

  return sgn((point - origin).norm() / dir_norm, (point - origin).dot(dir));
}

// Determines the two types of parallel intersection.
template <class T>
typename Triangle2<T>::SegSegIntersectType Triangle2<T>::IntersectParallelSegs(
    const std::pair<Vector2<T>, Vector2<T>>& seg1,
    const std::pair<Vector2<T>, Vector2<T>>& seg2,
    Vector2<T>* isect,
    Vector2<T>* isect2) {
  // TODO: Determine a tolerance in a principled manner.
  const T tol = 10 * std::numeric_limits<double>::epsilon();

  // Get the individual points.
  const Vector2<T>& pA = seg1.first;
  const Vector2<T>& pB = seg1.second;
  const Vector2<T>& qA = seg2.first;
  const Vector2<T>& qB = seg2.second;

  // Check for collinearity.
  if (CalcAreaSign(pA, pB, qA, tol) == kRight)
    return kSegSegNoIntersect;

  if (IsBetween(pA, pB, qA) && IsBetween(pA, pB, qB)) {
    *isect = qA;
    *isect2 = qB;
    return kSegSegEdge;
  }

  if (IsBetween(qA, qB, pA) && IsBetween(qA, qB, pB)) {
    *isect = pA;
    *isect2 = pB;
    return kSegSegEdge;
  }

  if (IsBetween(pA, pB, qA) && IsBetween(qA, qB, pB)) {
    *isect = qA;
    *isect2 = pB;
    return kSegSegEdge;
  }

  if (IsBetween(pA, pB, qA) && IsBetween(qA, qB, pA)) {
    *isect = qA;
    *isect2 = pA;
    return kSegSegEdge;
  }

  if (IsBetween(pA, pB, qB) && IsBetween(qA, qB, pB)) {
    *isect = qB;
    *isect2 = pB;
    return kSegSegEdge;
  }

  if (IsBetween(pA, pB, qB) && IsBetween(qA, qB, pA)) {
    *isect = qB;
    *isect2 = pA;
    return kSegSegEdge;
  }

  return kSegSegNoIntersect;
}

// Determines whether point c is between points [a, c].
template <class T>
bool Triangle2<T>::IsBetween(
    const Vector2<T>& a, const Vector2<T>& b, const Vector2<T>& c) {
  using std::abs;

  // TODO: Select a proper tolerance.
  const T tol = 10 * std::numeric_limits<double>::epsilon();

  // If the points are not collinear, quit.
  if (CalcAreaSign(a, b, c, tol) != kOn)
    return false;

  if (abs(a[0] - b[0]) > tol) {
    return (a[0] <= c[0] && c[0] <= b[0]) || (a[0] >= c[0] && c[0] >= b[0]);
  } else {
    return (a[1] <= c[1] && c[1] <= b[1]) || (a[1] >= c[1] && c[1] >= b[1]);
  }
}

// Intersects two line segments in 2D.
template <class T>
typename Triangle2<T>::SegSegIntersectType Triangle2<T>::IntersectSegs(
    const std::pair<Vector2<T>, Vector2<T>>& seg1,
    const std::pair<Vector2<T>, Vector2<T>>& seg2,
    Vector2<T>* isect,
    Vector2<T>* isect2) {
  using std::abs;

  DRAKE_DEMAND(isect);
  DRAKE_DEMAND(isect2);

  // TODO: Set the tolerance in a principled manner.
  const T tol = 10 * std::numeric_limits<double>::epsilon();

  // Set intersection type to 'none' initially.
  SegSegIntersectType type = kSegSegNoIntersect;

  // Get the individual points.
  const Vector2<T>& pA = seg1.first;
  const Vector2<T>& pB = seg1.second;
  const Vector2<T>& qA = seg2.first;
  const Vector2<T>& qB = seg2.second;

  // Compute the denominator.
  const T denom = pA[0] * (qB[1] - qA[1]) + pB[0] * (qA[1] - qB[1]) + 
      qB[0] * (pB[1] - pA[1]) + qA[0] * (pA[1] - pB[1]);
 
  // If the denominator is zero, the segments are parallel: handle separately.
  if (abs(denom) < tol)
    return IntersectParallelSegs(seg1, seg2, isect, isect2);

  // Calculate the numerator.
  T num = pA[0] * (qB[1] - qA[1]) + qA[0] * (pA[1] - qB[1]) +
      qB[0] * (qA[1] - pA[1]);

  if (abs(num) < tol || abs(num - denom) < tol)
    type = kSegSegVertex;
  const T s = num / denom;
  num = -(pA[0] * (qA[1] - pB[1]) + pB[0] * (pA[1] - qA[1]) +
      qA[0] * (pB[1] - pA[1]));
  if (abs(num) < tol || abs(num - denom) < tol)
    type = kSegSegVertex;

  const T t = num / denom;
  if (s > 0 && s < 1 && t > 0 && t < 1) {
    type = kSegSegIntersect;
  } else {
    if (s < 0 || s > 1 || t < 0 || t > 1)
      type = kSegSegNoIntersect;
  }
  
  *isect = pA + s * (pB - pA);
  return type;
}

/*
/// Intersects a triangle with another triangle in 2D.
template <class T>
int Triangle2<T>::Intersect(
    const Triangle2<T>& t, Vector2<T>* intersections) const {
  // Initialize the potential intersection with t.
  intersections[0] = t.a();
  intersections[1] = t.b();
  intersections[2] = t.c();
  int num_intersections = 3;

  // Clip against edges.
  for (int i1 = 2, i0 = 0; i0 < 3; i1 = i0, ++i0) {
    const Vector2<T> kN(get_vertex(i1)[1] - get_vertex(i0)[1],
                        get_vertex(i0)[0] - get_vertex(i1)[0]);
    const T fC = kN.dot(get_vertex(i1));
    ClipConvexPolygonAgainstLine(kN, fC, &num_intersections, intersections);
    if (num_intersections == 0)
      break; 
  }

  return num_intersections;
}

template <class T>
void Triangle2<T>::ClipConvexPolygonAgainstLine(
    const Vector2<T>& rkN, T fC, int* ri, Vector2<T>* isects) {
  // input vertices assumed to be ccw

  // test on which side of line the vertices are
  int iPositive = 0, iNegative = 0, iPIndex = -1;
  T afTest[6];
  for (int i=0; i< *ri; ++i)
  {
    afTest[i] = rkN.dot(isects[i]) - fC;
    if (afTest[i] >= 0)
    {
      iPositive++;
      if (iPIndex < 0)
        iPIndex = i;
    }
    else if (afTest[i] < 0)
     iNegative++;
  }

  if (iPositive > 0)
  {
    if (iNegative > 0)
    {
      // line transversely intersects polygon
      Vector2<T> akCV[6];
      int iCQuantity = 0;
      int iCur, iPrv;
      T fT;

      if (iPIndex > 0)
      {
        // first clip vertex on line
        iCur = iPIndex;
        iPrv = iCur-1;
        fT = afTest[iCur]/(afTest[iCur] - afTest[iPrv]);
        akCV[iCQuantity++] = isects[iCur]+fT*(isects[iPrv]-isects[iCur]);

        // vertices on positive side of line
        while (iCur < *ri && afTest[iCur] > 0)
          akCV[iCQuantity++] = isects[iCur++];

        // last clip vertex on line
        if (iCur < *ri)
          iPrv = iCur - 1;
        else
        {
          iCur = 0;
          iPrv = *ri - 1;
        }
        fT = afTest[iCur]/(afTest[iCur] - afTest[iPrv]);
        akCV[iCQuantity++] = isects[iCur]+fT*(isects[iPrv]-isects[iCur]);
      }
      else // iPIndex is 0
      {
        // vertices on positive side of line
        iCur = 0;
        while (iCur < *ri && afTest[iCur] > 0)
          akCV[iCQuantity++] = isects[iCur++];

        // last clip vertex on line
        iPrv = iCur-1;
        fT = afTest[iCur]/(afTest[iCur] - afTest[iPrv]);
        akCV[iCQuantity++] = isects[iCur]+fT*(isects[iPrv]-isects[iCur]);

        // skip vertices on negative side
        while (iCur < *ri && afTest[iCur] <= 0)
          iCur++;

        // first clip vertex on line
        if (iCur < *ri)
        {
          iPrv = iCur-1;
          fT = afTest[iCur]/(afTest[iCur] - afTest[iPrv]);
          akCV[iCQuantity++] = isects[iCur]+fT*(isects[iPrv]-isects[iCur]);

          // vertices on positive side of line
          while (iCur < *ri && afTest[iCur] > 0)
            akCV[iCQuantity++] = isects[iCur++];
        }
        else
        {
          // iCur = 0;
          iPrv = *ri - 1;
          fT = afTest[0]/(afTest[0] - afTest[iPrv]);
          akCV[iCQuantity++] = isects[0]+fT*(isects[iPrv]-isects[0]);
        }
      }

      *ri = iCQuantity;
      for (int i=0; i< iCQuantity; ++i)
        isects[i] = akCV[i];
    }
    // else polygon fully on positive side of line, nothing to do
  }
  else // polygon does not intersect positive side of line, clip all
    *ri = 0;
}
*/

/// Utility function for intersect_coplanar_tris()
/**
 * Taken from O'Rourke, p. 259.
 */
template <class T>
int Triangle2<T>::Advance(int a, int* aa, bool inside, const Vector2<T>& p, Vector2<T>* intersections, int* num_intersections) {
  const int num_vertices = 3;

  if (inside)
    intersections[(*num_intersections)++] = p;

  (*aa)++;
  return (a+1) % num_vertices;
}

/// Verifies that the 2D triangle is counter-clockwise.
template <class T>
bool Triangle2<T>::ccw() const {
  // TODO: Pick a proper tolerance here.
  const T tol = 10 * std::numeric_limits<double>::epsilon();

  return CalcAreaSign(a(), b(), c(), tol) != kRight;
}

/// @note Adapted from O'Rourke's convex polygon intersection algorithm.
///       algorithm.
template <class T>
int Triangle2<T>::Intersect(
    const Triangle2<T>& t, Vector2<T>* intersections) const {
  enum tInFlag { kPin, kQin, kUnknown };

  // TODO: Replace this with a proper tolerance.
  const T tol = 10 * std::numeric_limits<double>::epsilon();

  // Number of vertices is constant.
  const int num_vertices = 3;

  // now compute their intersections  
  int a = 0, b = 0, aa = 0, ba = 0;
  tInFlag inflag = kUnknown;
  bool first_point = true;
  Vector2<T> origin(0,0);
  int num_intersections = 0;

  do {
    int a1 = (a + num_vertices-1) % num_vertices;
    int b1 = (b + num_vertices-1) % num_vertices;

    const Vector2<T> AX = get_vertex(a) - get_vertex(a1);
    const Vector2<T> BX = t.get_vertex(b) - t.get_vertex(b1);

    // determine signs of cross-products
    OrientationType cross = CalcAreaSign(origin, AX, BX, tol);
    OrientationType aHB = CalcAreaSign(
        t.get_vertex(b1), t.get_vertex(b), get_vertex(a), tol);
    OrientationType bHA = CalcAreaSign(
        get_vertex(a1), get_vertex(a), t.get_vertex(b), tol);
    
    // if A and B intersect, update inflag
    Vector2<T> p, q;
    SegSegIntersectType type = IntersectSegs(
        std::make_pair(get_vertex(a1), get_vertex(a)),
        std::make_pair(t.get_vertex(b1), t.get_vertex(b)), &p, &q);
    if (type == kSegSegVertex || type == kSegSegIntersect) {
      if (inflag == kUnknown && first_point) {
        aa = ba = 0;
        first_point = false;
      }

      intersections[num_intersections++] = p;
      if (aHB == kLeft) {
        inflag = kPin;
      } else {
        if (bHA == kLeft)
          inflag = kQin;
      }
    }

    // --------- Advance rules --------------
    // special cases: O'Rourke p. 262
    // special case: A and B overlap and oppositely oriented; intersection
    // is only a line segment
    if ((type == kSegSegEdge) && AX.dot(BX) < 0) {
      intersections[num_intersections++] = p;
      intersections[num_intersections++] = q;
      return num_intersections;
    } else { 
      // special case: A and B are parallel and disjoint
      if ((cross == kOn) && (aHB == kRight) && (bHA == kRight)) {
        return num_intersections;
      } else {
        // special case: A and B are collinear
        if ((cross == kOn) && (aHB == kOn) && (bHA == kOn)) {
          // Advance but do not add point to intersecting polygon.
          if (inflag == kPin) {
            b = Advance(b, &ba, inflag == kQin, t.get_vertex(b), intersections, &num_intersections);
          } else {
            a = Advance(a, &aa, inflag == kPin, get_vertex(a), intersections, &num_intersections);
          }
        } else {
          // generic cases (continued from p. 258)
          if (cross == kOn || cross == kLeft) {
            if (bHA == kLeft) {
              a = Advance(a, &aa, inflag == kPin, get_vertex(a), intersections, &num_intersections);
            } else {
              b = Advance(b, &ba, inflag == kQin, t.get_vertex(b), intersections, &num_intersections);
            }
          } else {
            if (aHB == kLeft) {
              b = Advance(b, &ba, inflag == kQin, t.get_vertex(b), intersections, &num_intersections);
            } else {
              a = Advance(a, &aa, inflag == kPin, get_vertex(a), intersections, &num_intersections);
            }
          } 
        }
      }
    }
  } while (((aa < num_vertices) || (ba < num_vertices)) && (aa < 2*num_vertices) && (ba < 2*num_vertices));

  // deal with remaining special cases: P fully inside Q, Q fully inside P,
  // or P and Q do not intersect
  if (inflag == kUnknown) {
    // look for 'this' fully inside t.
    if (t.PointInside(this->a())) {
      intersections[0] = this->a();
      intersections[1] = this->b();
      intersections[2] = this->c();
      return 3;
    }

    // Look for t fully inside 'this'.
    if (PointInside(t.a())) {
      intersections[0] = t.a();
      intersections[1] = t.b();
      intersections[2] = t.c();
      return 3;
    }

    // If at this point, the polygons must not intersect.
    return 0;
  }

  // Remove duplicates.
  for (int i = 1; i < num_intersections; ++i) {
    if ((intersections[i] - intersections[i-1]).norm() < tol) {
      // Shift all intersections after i on to the left
      for (int j = i + 1; j < num_intersections ; ++j)
        intersections[j-1] = intersections[j];
      --i;
      --num_intersections;
    }
  }

  // Check whether the last point is a duplicate of the first.
  if (num_intersections > 1 &&
      (intersections[0] - intersections[num_intersections-1]).norm() < tol) {
    --num_intersections;
  }

  return num_intersections;
}

}  // namespace examples
}  // namespace drake
