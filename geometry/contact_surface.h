#pragma once

template <class T>
struct ContactSurfaceVertex {
  // The Cartesian location in space of the vertex.
  Eigen::Vector3<T> location;

  // The traction at the vertex.
  Eigen::Vector3<T> traction;

  // The slip velocity at the vertex.
  Eigen::Vector2<T> slip_velocity;
};

template <class T>
class ContactSurfaceFace {
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
    normal_ = (*vB->location - *vA->location).cross(
        *vC->location - *vB->location);

    // Compute the area.
    const T s1 = (*vB->location - *vA->location).norm();
    const T s2 = (*vC->location - *vB->location).norm();
    const T s3 = (*vA->location - *vC->location).norm();
    const T sp = (s1 + s2 + s3) / 2;  // semiparameter.
    area_ = sqrt(sp*(sp - s1)*(sp - s2)*(sp - s3));
  }

  // TODO: Re-evaluate: do we still need to store values at vertices.

  // TODO: Calculates traction at a point.
  T CalculateTraction(const Vector3<T>& p) const {

  }

  // TODO: Fill me in.
  T EvaluatePressure(const Vector3<T>& p) const {

  }

  // TODO: Fill me in.
  T EvaluateSlipVelocity(const Vector3<T>& p) const {

  }

  const Vector3<T> normal() const { return normal_; }
  const T area() const { return area_; }
  const ContactSurfaceVertex<T>* vertex_A() const { return vA_; }
  const ContactSurfaceVertex<T>* vertex_B() const { return vB_; }
  const ContactSurfaceVertex<T>* vertex_C() const { return vC_; }
  const Tetrahedron<T>* tetrahedron_A() const { return tA_; }
  const Tetrahedron<T>* tetrahedron_B() const { return tB_; }

 private:
  // TODO: Fill me in.
  Vector3<T> ConvertFromCartesianToBarycentricCoords(const Vector3<T>& p);

  // The vertices of the face.
  ContactSurfaceVertex<T>* vA_{nullptr};
  ContactSurfaceVertex<T>* vB_{nullptr};
  ContactSurfaceVertex<T>* vC_{nullptr};

  // The tetrahedra that the triangle was constructed from.
  Tetrahedron<T>* tA_{nullptr};
  Tetrahedron<T>* tB_{nullptr};

  // The normal, computed only once.
  const Vector3<T> normal_;

  // The area, computed only once.
  const T area_;
};

template <class T>
class ContactSurface {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactSurface)

  /// The id of the first geometry in the contact.
  GeometryId id_A;

  /// The id of the second geometry in the contact.
  GeometryId id_B;

  /// Vertices comprising the contact surface.
  std::vector<std::unique_ptr<ContactSurfaceVertex<T>>> vertices_;

  /// Triangles comprising the contact surface.
  std::vector<std::unique_ptr<ContactSurfaceFace<T>>> faces_;
};