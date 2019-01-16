#pragma once

#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_results/field.h"
#include "drake/math/orthonormal_basis.h"

namespace drake {
namespace geometry {

template <class T>
class ContactSurfaceFace;

template <class T>
class Field;

/// A vertex in a ContactSurface.
template <class T>
class ContactSurfaceVertex {
  friend class ContactSurfaceFace<T>;

 public:
  ContactSurfaceVertex() {}
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactSurfaceVertex)

  /// The Cartesian location of the vertex, as an offset vector expressed in
  /// the global frame.
  Vector3<T> location_w;

  /// Gets the face that this vertex belongs to.
  const ContactSurfaceFace<T>* face() const { return face_; }

 private:
  // The face this vertex belongs to.
  ContactSurfaceFace<T>* face_{nullptr};
};

/// An oriented triangular face in a ContactSurface.
template <class T>
class ContactSurfaceFace {
 public:
  /// Constructs a ContactSurfaceFace using the specified ContactSurfaceVertex
  /// objects and field pointers. The vertices must be specified such that the
  /// operation (vB - vA) x (vC - vB) yields a vector pointing toward Body A for
  /// contacting bodies A and B.
  ContactSurfaceFace(
      std::unique_ptr<ContactSurfaceVertex<T>> vA,
      std::unique_ptr<ContactSurfaceVertex<T>> vB,
      std::unique_ptr<ContactSurfaceVertex<T>> vC,
      const Field<T>* fA,
      const Field<T>* fB) : vA_(std::move(vA)), vB_(std::move(vB)),
          vC_(std::move(vC)), fA_(fA), fB_(fB) {
    Initialize();
  }

  ContactSurfaceFace(const ContactSurfaceFace& f) { operator=(f); }
  ContactSurfaceFace& operator=(const ContactSurfaceFace<T>& f) {
    // Create vertices.
    vA_ = std::make_unique<ContactSurfaceVertex<T>>(*f.vA_);
    vB_ = std::make_unique<ContactSurfaceVertex<T>>(*f.vB_);
    vC_ = std::make_unique<ContactSurfaceVertex<T>>(*f.vC_);

    // Copy field pointers.
    fA_ = f.fA_;
    fB_ = f.fB_;

    // Complete construction.
    Initialize();
    return *this;
  }

  ContactSurfaceFace(ContactSurfaceFace&&) = default;
  ContactSurfaceFace& operator=(ContactSurfaceFace&&) = default;

  /// Gets the normal to this surface expressed in the global frame and
  /// with orientation determined using the ordering of the three vertices
  /// specified on object construction.
  const Vector3<T> normal_w() const { return normal_w_; }

  /// Gets the area of this triangle.
  const T area() const { return area_; }

  /// Gets the centroid of this triangle, which is defined as an offset vector
  /// expressed in the global frame.
  const Vector3<T> centroid_w() const { return centroid_w_; }

  /// Gets a copy of the first vertex passed in at construction.
  const ContactSurfaceVertex<T>& vertex_A() const { return *vA_; }

  /// Gets  a copy of the second vertex passed in at construction.
  const ContactSurfaceVertex<T>& vertex_B() const { return *vB_; }

  /// Gets  a copy of the third vertex passed in at construction.
  const ContactSurfaceVertex<T>& vertex_C() const { return *vC_; }

  /// Gets the first field passed in at construction, which corresponds
  /// to geometry from Body A, from which the contact surface was computed.
  const Field<T>* field_A() const { return fA_; }

  /// Gets the second field passed in at construction, which corresponds
  /// to geometry from Body B, from which the contact surface was computed.
  const Field<T>* field_B() const { return fB_; }

 protected:
  // Computes useful quantities and vectors and sets the face pointers in the
  // stored vertex to `this`.
  void Initialize() {
    using std::sqrt;

    // Compute the normal.
    const ContactSurfaceVertex<T>& vA = vertex_A();
    const ContactSurfaceVertex<T>& vB = vertex_B();
    const ContactSurfaceVertex<T>& vC = vertex_C();

    normal_w_ = (vB.location_w - vA.location_w).cross(
        vC.location_w - vB.location_w).normalized();

    // Compute the area.
    const T s1 = (vB.location_w - vA.location_w).norm();
    const T s2 = (vC.location_w - vB.location_w).norm();
    const T s3 = (vA.location_w - vC.location_w).norm();
    const T sp = (s1 + s2 + s3) / 2;  // semiparameter.
    area_ = sqrt(sp * (sp - s1) * (sp - s2) * (sp - s3));

    // Compute the centroid.
    centroid_w_ = (vA.location_w + vB.location_w + vC.location_w)/3;

    // Store this in the vertices.
    vA_->face_ = this;
    vB_->face_ = this;
    vC_->face_ = this;
  }

  // TODO: Fill me in.
  Vector3<T> ConvertFromCartesianToBarycentricCoords(const Vector3<T>& p) const;

  // The vertices of the face.
  std::unique_ptr<ContactSurfaceVertex<T>> vA_;
  std::unique_ptr<ContactSurfaceVertex<T>> vB_;
  std::unique_ptr<ContactSurfaceVertex<T>> vC_;

  // The field that the triangle was constructed from.
  const Field<T>* fA_{nullptr};
  const Field<T>* fB_{nullptr};

  // The normal, computed on construction, expressed in the world frame.
  Vector3<T> normal_w_;

  // The area, computed on construction.
  T area_;

  // The centroid, computed on construction, which is defined as an offset
  // expressed in the world frame.
  Vector3<T> centroid_w_;
};

/// The contact surface computed by the geometry system.
template <class FaceType>
class ContactSurfaceType {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactSurfaceType)
  ContactSurfaceType(
      GeometryId A, GeometryId B, const std::vector<FaceType>& faces) :
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
