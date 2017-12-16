#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/math/orthonormal_basis.h"

namespace drake {
namespace multibody {

template <class T>
class Triangle2 {
 private:
  enum OrientationType {
    kLeft,
    kOn,
    kRight
  };

 public:
  Triangle2(const Vector2<T>& a,
     const Vector2<T>& b,
     const Vector2<T>& c) : a_(a), b_(b), c_(c) {
  }

  const Vector2<T>& a() const { return a_; }
  const Vector2<T>& b() const { return b_; }
  const Vector2<T>& c() const { return c_; }

  const Vector2<T>& get_vertex(int i) const {
    switch (i) {
      case 0: return a_;
      case 1: return b_;
      case 2: return c_;
      default:
        DRAKE_ABORT();
    }
  }

  bool PointIsInside(const Vector2<T>& point) const;

 private:
  static OrientationType CalcAreaSign(
      const Vector2<T>& a, const Vector2<T>& b, const Vector2<T>& c, T tol);

  Vector2<T> a_, b_, c_;
};

template <class T>
class Triangle3 {
 public:
  Triangle3(const Vector3<T>& a,
     const Vector3<T>& b,
     const Vector3<T>& c) : a_(a), b_(b), c_(c) {
  }

  Triangle2<T> ProjectTo2d(const Vector3<T>& normal) const;
  const Vector3<T>& a() const { return a_; }
  const Vector3<T>& b() const { return b_; }
  const Vector3<T>& c() const { return c_; }

  const Vector3<T>& get_vertex(int i) const {
    switch (i) {
      case 0: return a_;
      case 1: return b_;
      case 2: return c_;
      default:
        DRAKE_ABORT();
    }
  }

  Vector3<T> CalcNormal() const;

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

  Vector3<T> a_, b_, c_;
};

}  // multibody
}  // drake
