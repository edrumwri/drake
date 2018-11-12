#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_results/tetrahedron.h"
#include "drake/math/orthonormal_basis.h"

// TODO: Interface for GeometryWorld computing the contact surface.

namespace drake {
namespace geometry {

template <class T>
struct ContactSurfaceVertex {
  // The Cartesian location in space of the vertex.
  Vector3<T> location;
};

template <class T>
class ContactSurfaceFace {
 public:

  // TODO: vertices must be specified in the proper order so that the normal
  // and area is correct.
  ContactSurfaceFace(
      ContactSurfaceVertex<T>* vA,
      ContactSurfaceVertex<T>* vB,
      ContactSurfaceVertex<T>* vC,
      Tetrahedron<T>* tA,
      Tetrahedron<T>* tB) : vA_(vA), vB_(vB), vC_(vC), tA_(tA), tB_(tB) {
    using std::sqrt;

    // Compute the normal.
    normal_W_ = (*vB->location - *vA->location).cross(
        *vC->location - *vB->location);

    // Compute the area.
    const T s1 = (*vB->location - *vA->location).norm();
    const T s2 = (*vC->location - *vB->location).norm();
    const T s3 = (*vA->location - *vC->location).norm();
    const T sp = (s1 + s2 + s3) / 2;  // semiparameter.
    area_ = sqrt(sp*(sp - s1)*(sp - s2)*(sp - s3));

    // Compute the centroid.
    centroid_W_ = (vA->location + vB->location + vC->location)/3;
  }

  const Vector3<T> normal_W() const { return normal_W_; }
  const T area() const { return area_; }
  const Vector3<T> centroid_W() const { return centroid_W_; }
  const ContactSurfaceVertex<T>* vertex_A() const { return vA_; }
  const ContactSurfaceVertex<T>* vertex_B() const { return vB_; }
  const ContactSurfaceVertex<T>* vertex_C() const { return vC_; }
  const Tetrahedron<T>* tetrahedron_A() const { return tA_; }
  const Tetrahedron<T>* tetrahedron_B() const { return tB_; }

 protected:
  // TODO: Fill me in.
  Vector3<T> ConvertFromCartesianToBarycentricCoords(const Vector3<T>& p) const;

  // The vertices of the face.
  ContactSurfaceVertex<T>* vA_{nullptr};
  ContactSurfaceVertex<T>* vB_{nullptr};
  ContactSurfaceVertex<T>* vC_{nullptr};

  // The tetrahedra that the triangle was constructed from.
  Tetrahedron<T>* tA_{nullptr};
  Tetrahedron<T>* tB_{nullptr};

  // The normal, computed only once, expressed in the world frame.
  Vector3<T> normal_W_;

  // The area, computed only once.
  T area_;

  // The centroid, computed only once, which is defined as an offset expressed
  // in the world frame.
  Vector3<T> centroid_W_;
};

/// The contact surface computed by GeometryWorld.
template <class T>
class ContactSurface {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactSurface)
  ContactSurface(GeometryId A, GeometryId B) : id_A_(A), id_B_(B) {}
  const std::vector<ContactSurfaceFace<T>> triangles() const { return faces_; }
  GeometryId id_A() const { return id_A_; }
  GeometryId id_B() const { return id_B_; }

 private:
  /// The id of the first geometry in the contact.
  GeometryId id_A_;

  /// The id of the second geometry in the contact.
  GeometryId id_B_;

  /// Vertices comprising the contact surface.
  std::vector<ContactSurfaceVertex<T>> vertices_;

  /// Triangles comprising the contact surface.
  std::vector<ContactSurfaceFace<T>> faces_;
};

}  // namespace geometry
}  // namespace drake
