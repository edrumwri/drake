#pragma once

#include <Eigen/Dense>

namespace drake {
namespace multibody {

template <class T>
class Triangle {
 public:
  Triangle(const Eigen::Vector3<T>& a,
     const Eigen::Vector3<T>& b,
     const Eigen::Vector3<T>& c) : a_(a), b_(b), c_(c) {
  }

  Eigen::Vector3<T> a() const { return a_; }
  Eigen::Vector3<T> b() const { return b_; }
  Eigen::Vector3<T> c() const { return c_; }

  Eigen::Vector3<T> CalcNormal() const {
    const Eigen::Vector3<T> e1 = b() - a();
    const Eigen::Vector3<T> e2 = c() - b();
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
  T CalcSquareDistance(
      const Eigen::Vector3<T>& origin,
      const Eigen::Vector3<T>& dir,
      Eigen::Vector3<T>* closest_point_on_line,
      Eigen::Vector3<T>* closest_point_on_tri,
      T* t_line) const {
    using std::abs;

    // TODO: replace this with a more informed tolerance.
    const double zero_tol = 10 * std::numeric_limits<double>::epsilon(); 

    // Test if line intersects triangle. If so, the squared distance is zero.
    Eigen::Vector3<T> edge0 = b() - a();
    Eigen::Vector3<T> edge1 = c() - a();
    Eigen::Vector3<T> normal = CalcNormal();

    // get the direction and origin of the line segment
    T NdD = normal.dot(dir);
    if (abs(NdD) > zero_tol) {
      // The line and triangle are not parallel, so the line intersects
      // the plane of the triangle.
      Eigen::Vector3<T> kDiff = origin - a();
      Eigen::Vector3<T> kU, kV;
      Eigen::Vector3<T>::determine_orthonormal_basis(dir, &kU, &kV);
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
        *closest_point_on_line = origin + t*dir;
        *closest_point_on_tri = a() + fB1*edge0 + fB2*edge1;
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
      std::pair<Eigen::Vector3<T>, Eigen::Vector3<T>> edge(
        get_vertex(i0), get_vertex(i1));

      // Compute the squared distance and closest points.
      T t, t_seg;
      Eigen::Vector3<T> temp_point1, temp_point2;
      T edge_square_dist = calc_sq_dist(
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

  /// Determines the distance between a line and a line segment
  T CalcSquareDistance(
      const Eigen::Vector3<T>& origin,
      const Eigen::Vector3<T>& dir,
      const std::pair<Eigen::Vector3<T>, Eigen::Vector3<T>>& seg,
      Eigen::Vector3<T>* closest_point_on_line,
      Eigen::Vector3<T>* closest_point_on_seg,
      T* t_line,
      T* t_seg) {
    DRAKE_DEMAND(closest_point_on_line);
    DRAKE_DEMAND(closest_point_on_seg);
    DRAKE_DEMAND(t_line);
    DRAKE_DEMAND(t_seg);

    using std::abs;

    // determine the origins of the segment
    const Eigen::Vector3<T> seg_origin = (seg.first + seg.second) * 0.5;

    // determine the directions of the segment
    Eigen::Vector3<T> seg_dir = seg.second - seg.first;
    const T seg_dir_len = seg_dir.norm();
    seg_dir /= seg_dir_len;

    // determine the extents of the segment
    const T seg_extent = seg_dir_len * 0.5;

    const Eigen::Vector3<T> kDiff = origin - seg_origin;
    T A01 = -dir.dot(seg_dir);
    T B0 = kDiff.dot(dir);
    T C = kDiff.norm_sq();
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
    t_line = S0;
    t_seg = S1;
    return abs(sqr_dist);
  }

  /// Determines the distance between this triangle and a given point
  /// @param point the query point
  /// @param[out] sout the Barycentric coordinate s, where
  ///             a*s + b*t + c*(1-s-t) = p
  /// @param[out] tout the Barycentric coordinate t 
  /// @return the Euclidean distance between this triangle and the point
  T CalcSquareDistance(const Eigen::Vector3<T>& point, T* sout, T* tout) const {
    using std::abs;

    // compute needed quantities
    const Eigen::Vector3<T> k_diff = a() - point;
    const Eigen::Vector3<T> E0 = b() - a();
    const Eigen::Vector3<T> E1 = c() - a();
    T A00 = E0.norm_sq();
    T A01 = E0.dot(E0, E1);
    T A11 = E1.norm_sq(); 
    T B0 = k_diff.dot(E0);
    T B1 = k_diff.dot(E1);
    T C = k_diff.norm_sq();
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
      }
    } else {
    // region 0
    const double inv_det = 1.0/det;
    s *= inv_det;
    t *= inv_det;
    sqr_dist = s * (A00 * s + A01 * t + (2 * B0)) +
               t * (A01 * s + A11* t + (2 * B1)) + C;
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
  T CalcSquareDistance(
      const Eigen::Vector3<T>& query_point,
      Eigen::Vector3<T>* closest_point) const {
    DRAKE_DEMAND(closest_point);

    // Get the closest point in barycentric coordinates.
    T s, t;
    T square_distance = CalcSquareDistance(query_point, &s, &t);

    // compute the closest point
    *closest_point = a()*s + b()*t + c()*(1-s-t);

    return square_distance;
  }

  /// Computes the closest distance between a triangle and a line segment.
  T Triangle<T>::CalcSquareDistance(
      const std::pair<Eigen::Vector3<T>, Eigen::Vector<T>>& seg,
      Eigen::Vector3<T>* closest_point_on_tri,
      Eigen::Vector3<T>* closest_point_on_seg) const {
    // Compute the segment origin, direction, and extent.
    Eigen::Vector3<T> origin = (seg.first + seg.second) * 0.5;
    Eigen::Vector3<T> dir = seg.second - seg.first;
    T dir_len = dir.norm();
    T extent = dir_len * 0.5;
    dir /= dir_len;

    // Get the squared distance between a line containing the segment and
    // the triangle.
    T t_line;
    T square_dist = CalcSquareDistance(
        tri, origin, dir, closest_point_on_seg, closest_point_on_tri, &t_line);

    if (t_line >= -extent) {
      if (t_line <= extent) {
        return square_dist;
      } else {
        closest_point_on_seg = seg.second;
      }
    } else {
      closest_point_on_seg = seg.first;
    }

    // If at this point, it is necessary to compute the closest point to the
    // triangle.
    return tri.CalcSquareDistance(closest_point_on_seg, closest_point_on_tri);
  }

  /// Determines the squared distance between two triangles and two closest
  /// points.
  T CalcSquareDistance(const Triangle<T>& t,
     Eigen::Vector3<T>* closest_point_on_this,
     Eigen::Vector3<T>* closest_point_on_t) const {
    DRAKE_DEMAND(closest_point_on_this);
    DRAKE_DEMAND(closest_point_on_t);
    Eigen::Vector3<T> temp_point1, temp_point2;

    // compare edges of t1 to the interior of t2
    T square_dist = std::numeric_limits<T>::max(), fSqrDistTmp;
    std::pair<Eigen::Vector3<T>> seg;
    for (int i0 = 2, i1 = 0; i1 < 3; i0 = i1++)
    {
      // compute the distance between the triangle and the line segment
      seg.first = get_vertex(i0);
      seg.second = get_vertex(i1);
      T seg_square_dist = t.CalcSquareDist(seg, &temp_point2, &temp_point1);
      if (seg_square_dist < square_dist)
      {
        *closest_point_on_this = temp_point2;
        *closest_point_on_t = temp_point1;
        square_dist = seg_square_dist;
      }
    }

    // compare edges of t2 to the interior of t1
    for (int i0 = 2, i1 = 0; i1 < 3; i0 = i1++)
    {
      seg.first = t.get_vertex(i0);
      seg.second = t.get_vertex(i1);
      T seg_square_dist = CalcSquareDist(seg, &temp_point1, &temp_point2);
      if (seg_square_dist < square_dist)
      {
        *closest_point_on_this = temp_point1;
        *closest_point_on_t = temp_point2;
        square_dist = seg_square_dist;
      }
    }

    return square_dist;
  }

  /// Computes the square distance between two triangles and returns closest
  /// points on each.
  /// @param point the query point
  /// @param closest_point the closest point on the triangle is returned here
  /// @return the squared Euclidean distance between this triangle and the point
  /// @note code adapted from www.geometrictools.com 
  T CalcSquareDistance(
      const Triangle& t,
      Eigen::Vector3<T>* closest_point_on_this,
      Eigen::Vector3<T>* closest_point_on_t) const {
  Eigen::Vector3<T> tmp1, tmp2;

  // compare edges of t1 to the interior of t2
  T square_dist = std::numeric_limits<T>::max();
  std::pair<Eigen::Vector3<T>, Eigen::Vector3<T>> seg;
  for (int i0 = 2, i1 = 0; i1 < 3; i0 = i1++) {
    // compute the distance between the triangle and the line segment
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

 private:
  Eigen::Vector3<T> a_, b_, c_;
};

}  // multibody
}  // drake
