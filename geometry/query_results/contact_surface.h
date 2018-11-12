#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_results/tetrahedron.h"
#include "drake/math/orthonormal_basis.h"

namespace drake {
namespace geometry {

template <class T>
class ContactSurfaceFace;

/// A vertex in a ContactSurface.
template <class T>
class ContactSurfaceVertex {
  friend class ContactSurfaceFace<T>;

 public:
  /// The Cartesian location in space of the vertex.
  Vector3<T> location;

  /// Gets the face that this vertex belongs to.
  const ContactSurfaceFace<T>* face() const { return face_; }

 private:
  // The face this vertex belongs to.
  ContactSurfaceFace<T>* face_{nullptr};
};

/// A triangular face in a ContactSurface.
template <class T>
class ContactSurfaceFace {
 public:

  /// Constructs a ContactSurfaceFace using the specified ContactSurfaceVertex
  /// objects and tetrahedra. The vertices must be specified such that the
  /// operation (vB - vA) x (vC - vB) yields a vector pointing toward Body A for
  /// contacting bodies A and B.
  ContactSurfaceFace(
      const ContactSurfaceVertex<T>& vA,
      const ContactSurfaceVertex<T>& vB,
      const ContactSurfaceVertex<T>& vC,
      const Tetrahedron<T>* tA,
      const Tetrahedron<T>* tB) : vA_(vA), vB_(vB), vC_(vC), tA_(tA), tB_(tB) {
    using std::sqrt;

    // Compute the normal.
    normal_W_ = (vB.location - vA.location).cross(
        vC.location - vB.location).normalized();

    // Compute the area.
    const T s1 = (vB.location - vA.location).norm();
    const T s2 = (vC.location - vB.location).norm();
    const T s3 = (vA.location - vC.location).norm();
    const T sp = (s1 + s2 + s3) / 2;  // semiparameter.
    area_ = sqrt(sp*(sp - s1)*(sp - s2)*(sp - s3));

    // Compute the centroid.
    centroid_W_ = (vA.location + vB.location + vC.location)/3;

    // Store this in the vertices.
    vA_.face_ = this;
    vB_.face_ = this;
    vC_.face_ = this;
  }

  /// Gets the normal to this surface expressed in the global frame and
  /// with orientation determined using the ordering of the three vertices
  /// specified on object construction.
  const Vector3<T> normal_W() const { return normal_W_; }

  /// Gets the area of this triangle.
  const T area() const { return area_; }

  /// Gets the centroid of this triangle, which is defined as an offset vector
  /// expressed in the global frame.
  const Vector3<T> centroid_W() const { return centroid_W_; }

  /// Gets the first vertex passed in at construction.
  const ContactSurfaceVertex<T>& vertex_A() const { return vA_; }

  /// Gets the second vertex passed in at construction.
  const ContactSurfaceVertex<T>& vertex_B() const { return vB_; }

  /// Gets the third vertex passed in at construction.
  const ContactSurfaceVertex<T>& vertex_C() const { return vC_; }

  /// Gets the first tetrahedron passed in at construction, which corresponds
  /// to geometry from Body A, from which the contact surface was computed.
  const Tetrahedron<T>* tetrahedron_A() const { return tA_; }

  /// Gets the second tetrahedron passed in at construction, which corresponds
  /// to geometry from Body B, from which the contact surface was computed.
  const Tetrahedron<T>* tetrahedron_B() const { return tB_; }

 protected:
  // TODO: Fill me in.
  Vector3<T> ConvertFromCartesianToBarycentricCoords(const Vector3<T>& p) const;

  // The vertices of the face.
  ContactSurfaceVertex<T> vA_;
  ContactSurfaceVertex<T> vB_;
  ContactSurfaceVertex<T> vC_;

  // The tetrahedra that the triangle was constructed from.
  const Tetrahedron<T>* tA_{nullptr};
  const Tetrahedron<T>* tB_{nullptr};

  // The normal, computed only once, expressed in the world frame.
  Vector3<T> normal_W_;

  // The area, computed only once.
  T area_;

  // The centroid, computed only once, which is defined as an offset expressed
  // in the world frame.
  Vector3<T> centroid_W_;
};

/// The contact surface computed by GeometryWorld.
template <class FaceType>
class ContactSurfaceType {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactSurfaceType)
  ContactSurfaceType(
      GeometryId A, GeometryId B, std::vector<FaceType>& faces) :
      id_A_(A), id_B_(B), faces_(faces) {}
  const std::vector<FaceType> triangles() const { return faces_; }
  GeometryId id_A() const { return id_A_; }
  GeometryId id_B() const { return id_B_; }

 private:
  /// The id of the first geometry in the contact.
  GeometryId id_A_;

  /// The id of the second geometry in the contact.
  GeometryId id_B_;

  /// Triangles comprising the contact surface.
  std::vector<FaceType> faces_;
};

template <class T>
using ContactSurface = ContactSurfaceType<ContactSurfaceFace<T>>;

}  // namespace geometry
}  // namespace drake
