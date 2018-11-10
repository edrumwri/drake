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

  // Evaluates the pressure at a point using interpolation.
  T EvaluatePressure(const Vector3<T>& p) const {

  }

  // Evaluates the slip velocity at a point using interpolation.
  T EvaluateSlipVelocity(const Vector3<T>& p) const {

  }

  // Gets the specified vertex
  ContactSurfaceVertex* vertex(int i) const {
    switch (i) {
      case 0: return vA_;
      case 1: return vB_;
      case 2: return vC_;
      default:
        DRAKE_ABORT();
    }
  }

  // Integrates the traction vectors over the surface of the triangle.
  Vector3<T> IntegrateTraction(
      std::function<T(const Vector3<T>&)> pressure_function,
      std::function<Vector2<T>(const Vector3<T>&)> slip_velocity_function)
  const {
  }

  Vector3<T> IntegrateTractionSimple() const {
    // The tolerance below which contact is assumed to be not-sliding.
    const double slip_tol = std::numeric_limits<double>::epsilon();

    // Construct a matrix for projecting two-dimensional vectors in the plane
    // orthogonal to the contact normal to 3D.
    const Matrix<3, 2, T> P = Get2DTo3DProjectionMatrix();

    // Get the area of the contact surface triangle.
    const T triangle_area = area();

    // Evaluate the pressure distribution at the triangle centroid.
    const T pressure = EvaluatePressure(centroid);

    // Get the contact normal from the contact surface triangle and expressed
    // in the global frame using the convention that the normal points toward
    // Body A.
    const Vector3<T>& nhat_W = normal();

    // Compute the normal force, expressed in the global frame.
    const Vector3<T> fN_W = nhat_W * pressure * area;

    // Get the slip velocity at the centroid.
    const Vector2<T> slip_vel_W = EvaluateSlipVelocity(centroid);

    // Get the direction of slip.
    const T slip_speed = slip_vel_W.norm();

    // Determine the slip direction expressed in the global frame.
    const Vector3<T> slip_dir_W = (slip_speed > slip_tol) ?
                                  P * (slip_vel_W / slip_speed) :
                                  Vector3<T>::Zero();

    // Compute the frictional force.
    const Vector3<T> fF_W = (slip_speed > slip_tol) ?
                            mu_coulomb_ * pressure * -slip_dir_W :
                            Vector3<T>::Zero();

    // Increment the traction vector integral.
    return fN_W + fF_W;
  }

  // Constructs a matrix for projecting two-dimensional vectors in the plane
  // orthogonal to the contact normal to 3D.
  Matrix<3, 2, T> Get2DTo3DProjectionMatrix() const {
    const int axis = 2;
    Matrix3<T> PT = math::ComputeBasisFromAxis(axis, normal);
    PT.col(axis).setZero();
    return PT.transpose().block<3, 2>(0, 0);
  }

  T mu_coulomb_{0.0};        // The coefficient of friction between the bodies.
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