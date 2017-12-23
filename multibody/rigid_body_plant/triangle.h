#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/math/orthonormal_basis.h"

namespace drake {
namespace multibody {

template <class T>
class Triangle2 {
 public:
  // Types of intersections between two line segments.
  enum SegSegIntersectType {
    kSegSegNoIntersect,
    kSegSegVertex,
    kSegSegEdge,
    kSegSegIntersect
  };

  // Types of intersections between a point and a triangle.
  enum SegTriIntersectType {
    kSegTriNoIntersect,
    kSegTriVertex,
    kSegTriEdge,
    kSegTriEdgeOverlap,
    kSegTriPlanarIntersect,
    kSegTriInside
  };


 private:
  // The orientation of a point with respect to a line in 2D.
  enum OrientationType {
    kLeft,
    kOn,
    kRight
  };

  // Locations on a line segment. 
  enum SegLocationType {
    kSegEndpoint,
    kSegOrigin,
    kSegInterior,
    kSegExterior
  };

  // Locations on a polygon.
  enum PolygonLocationType {
    kPolygonInside,
    kPolygonOutside,
    kPolygonOnVertex,
    kPolygonOnEdge
  };

 public:
  Triangle2(const Vector2<T>& a,
     const Vector2<T>& b,
     const Vector2<T>& c) : a_(a), b_(b), c_(c) {
    DRAKE_ASSERT(ccw());
  }

  const Vector2<T>& a() const { return a_; }
  const Vector2<T>& b() const { return b_; }
  const Vector2<T>& c() const { return c_; }

  std::pair<Vector2<T>, Vector2<T>> get_edge(int i) const {
    switch (i) {
      case 0: return std::make_pair(a_, b_);
      case 1: return std::make_pair(b_, c_);
      case 2: return std::make_pair(c_, a_);
      default:
        DRAKE_ABORT();
    }
  }

  const Vector2<T>& get_vertex(int i) const {
    switch (i) {
      case 0: return a_;
      case 1: return b_;
      case 2: return c_;
      default:
        DRAKE_ABORT();
    }
  }

  int Intersect(const Triangle2& t, Vector2<T>* intersections) const;
  bool PointInside(const Vector2<T>& point) const;
  PolygonLocationType GetLocation(const Vector2<T>& point) const;
  SegTriIntersectType Intersect(
      const std::pair<Vector2<T>, Vector2<T>>& seg, T tol,
      Vector2<T>* isect, Vector2<T>* isect2) const;
  static SegSegIntersectType IntersectSegs(
      const std::pair<Vector2<T>, Vector2<T>>& seg1,
      const std::pair<Vector2<T>, Vector2<T>>& seg2,
      Vector2<T>* isect, Vector2<T>* isect2);

 private:
  friend class Triangle2Test_IsBetween_Test;
  friend class Triangle2Test_DetermineLineParam_Test;
  friend class Triangle2Test_CalcAreaSign_Test;
  friend class Triangle2Test_SegLocation_Test;
  friend class Triangle2Test_SegTriIntersection_Test;
  friend class Triangle2Test_SegSegIntersection_Test;
  friend class Triangle2Test_ParallelSegSegIntersection_Test;
  friend class Triangle2Test_TriTriIntersection_Test;

  static int Advance(int a, int* aa, bool inside, const Vector2<T>& p, Vector2<T>* intersections, int* num_intersections);
  bool ccw() const;
  static bool IsBetween(
      const Vector2<T>& a, const Vector2<T>& b, const Vector2<T>& c);
  static T DetermineLineParam(
      const Vector2<T>& origin, const Vector2<T>& dir, const Vector2<T>& point);
  static OrientationType CalcAreaSign(
      const Vector2<T>& a, const Vector2<T>& b, const Vector2<T>& c, T tol);
  static SegLocationType DetermineSegLocation(T t);
  static SegSegIntersectType IntersectParallelSegs(
      const std::pair<Vector2<T>, Vector2<T>>& seg1,
      const std::pair<Vector2<T>, Vector2<T>>& seg2,
      Vector2<T>* isect, Vector2<T>* isect2);
  static void ClipConvexPolygonAgainstLine(
      const Vector2<T>& rkN, T fC, int* ri, Vector2<T>* isects);

  Vector2<T> a_, b_, c_;
};

template <class T>
class Triangle3 {
 public:
  Triangle3(const Vector3<T>* a,
     const Vector3<T>* b,
     const Vector3<T>* c) : a_(a), b_(b), c_(c) {
  }

  Triangle2<T> ProjectTo2d(const Eigen::Matrix<T, 2, 3>& P) const;
  const Vector3<T>& a() const { return *a_; }
  const Vector3<T>& b() const { return *b_; }
  const Vector3<T>& c() const { return *c_; }

  std::pair<Vector3<T>, Vector3<T>> get_edge(int i) const {
    switch (i) {
      case 0: return std::make_pair(*a_, *b_);
      case 1: return std::make_pair(*b_, *c_);
      case 2: return std::make_pair(*c_, *a_);
      default:
        DRAKE_ABORT();
    }
  }

  const Vector3<T>& get_vertex(int i) const {
    switch (i) {
      case 0: return *a_;
      case 1: return *b_;
      case 2: return *c_;
      default:
        DRAKE_ABORT();
    }
  }

  Vector3<T> CalcNormal() const;

  T CalcSquareDistance(
      const Vector3<T>& query_point,
      Vector3<T>* closest_point) const; 

  T CalcSquareDistance(
      const std::pair<Vector3<T>, Vector3<T>>& seg,
      Vector3<T>* closest_point_on_tri,
      Vector3<T>* closest_point_on_seg) const; 

  T CalcSquareDistance(
      const Triangle3<T>& t,
      Vector3<T>* closest_point_on_this,
      Vector3<T>* closest_point_on_t) const;


 private:
  friend class Triangle3Test_CalcLineSquareDistanceParallel_Test;
  friend class Triangle3Test_CalcLineSquareDistanceIntersects_Test;
  friend class Triangle3Test_CalcLineSquareDistanceFromSegmentDisjoint_Test;
  friend class Triangle3Test_CalcLineSquareDistanceFromSegmentIntersects_Test;

  T CalcSquareDistance(
      const Vector3<T>& origin,
      const Vector3<T>& dir,
      Vector3<T>* closest_point_on_line,
      Vector3<T>* closest_point_on_tri,
      T* t_line) const;

  static T CalcSquareDistance(
      const Vector3<T>& origin,
      const Vector3<T>& dir,
      const std::pair<Vector3<T>, Vector3<T>>& seg,
      Vector3<T>* closest_point_on_line,
      Vector3<T>* closest_point_on_seg,
      T* t_line,
      T* t_seg); 

  T CalcSquareDistance(
      const Vector3<T>& point, T* sout, T* tout) const;

  const Vector3<T>* a_{nullptr};
  const Vector3<T>* b_{nullptr};
  const Vector3<T>* c_{nullptr};
};

}  // multibody
}  // drake
