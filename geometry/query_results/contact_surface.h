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

  // Note: the values below have been evaluated at the vertex to permit simple
  // approximation of the true fields using interpolation.
  /// The pressure evaluated at this vertex.
  T pressure;

  // The traction evaluated at this vertex.
  Vector3<T> traction;

  // The slip velocity at the vertex.
  Vector2<T> slip_velocity;
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

  // Evaluates the traction at a point using interpolation over the values
  // defined at the vertices.
  T CalculateTraction(const Vector3<T>& p) const {
    const Vector3<T> u = ConvertFromCartesianToBarycentricCoords(p);
    return u[0] * vA_->traction + u[1] * vB_->traction +
        u[2] * vC_->traction;
  }

  // Evaluates the pressure at a point using interpolation over the values
  // defined at the vertices.
  T EvaluatePressure(const Vector3<T>& p) const {
    const Vector3<T> u = ConvertFromCartesianToBarycentricCoords(p);
    return u[0] * vA_->pressure + u[1] * vB_->pressure + u[2] * vC_->pressure;
  }

  // Evaluates the slip velocity at a point using interpolation over the values
  // defined at the vertices.
  Vector2<T> EvaluateSlipVelocity(const Vector3<T>& p) const {
    const Vector3<T> u = ConvertFromCartesianToBarycentricCoords(p);
    return u[0] * vA_->slip_velocity + u[1] * vB_->slip_velocity +
        u[2] * vC_->slip_velocity;
  }

  // Integrates the traction vectors over the surface of the triangle.
  // Note: geometry world provides the first argument (presumably
  // through a lambda function; HydrostaticContactModel provides the second.
  Vector3<T> IntegrateTraction(
      std::function<T(const Vector3<T>&)> pressure_function,
      std::function<Vector2<T>(const Vector3<T>&)> slip_velocity_function)
  const {
    // TODO: Implement this if I want a higher order quadrature method.
    DRAKE_ABORT();
    return Vector3<T>::Zero();
  }

  // Integrates the traction vectors over the surface of the triangle using a
  // simple quadrature rule.
  Vector3<T> IntegrateTractionSimple() const {
    // The tolerance below which contact is assumed to be not-sliding.
    const double slip_tol = std::numeric_limits<double>::epsilon();

    // Get the contact normal from the contact surface triangle and expressed
    // in the global frame using the convention that the normal points toward
    // Body A.
    const Vector3<T>& nhat_W = normal_W();

    // Construct a matrix for projecting two-dimensional vectors in the plane
    // orthogonal to the contact normal to 3D.
    const Eigen::Matrix<T, 3, 2> P = Get2DTo3DProjectionMatrix(nhat_W);

    // Evaluate the pressure distribution at the triangle centroid.
    const T pressure = EvaluatePressure(centroid_W_);

    // Compute the normal force, expressed in the global frame.
    const Vector3<T> fN_W = nhat_W * pressure * area();

    // Get the slip velocity at the centroid.
    const Vector2<T> slip_vel_W = EvaluateSlipVelocity(centroid_W_);

    // Get the direction of slip.
    const T slip_speed = slip_vel_W.norm();

    // Determine the slip direction expressed in the global frame.
    const Vector3<T> zeros_3 = Vector3<T>::Zero();
    const Vector3<T> slip_dir_W = (slip_speed > slip_tol) ?
                                  P * (slip_vel_W / slip_speed) :
                                  zeros_3;

    // Compute the frictional force.
    const Vector3<T> fF_W = (slip_speed > slip_tol) ?
                            (mu_coulomb_ * pressure * -slip_dir_W) :
                            zeros_3;

    // Increment the traction vector integral.
    return fN_W + fF_W;
  }

  // Constructs a matrix for projecting two-dimensional vectors in the plane
  // orthogonal to the contact normal to 3D.
  Eigen::Matrix<T, 3, 2> Get2DTo3DProjectionMatrix(
      const Vector3<T>& normal) const {
    const int axis = 2;
    Matrix3<T> PT = math::ComputeBasisFromAxis(axis, normal);
    PT.col(axis).setZero();
    return PT.transpose().template block<3, 2>(0, 0);
  }

  T mu_coulomb_{0.0};        // The coefficient of friction between the bodies.
  const Vector3<T> normal_W() const { return normal_W_; }
  const T area() const { return area_; }
  const Vector3<T> centroid_W() const { return centroid_W_; }
  const ContactSurfaceVertex<T>* vertex_A() const { return vA_; }
  const ContactSurfaceVertex<T>* vertex_B() const { return vB_; }
  const ContactSurfaceVertex<T>* vertex_C() const { return vC_; }
  const Tetrahedron<T>* tetrahedron_A() const { return tA_; }
  const Tetrahedron<T>* tetrahedron_B() const { return tB_; }

 private:
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
  const Vector3<T> normal_W_;

  // The area, computed only once.
  const T area_;

  // The centroid, computed only once, which is defined as an offset expressed
  // in the world frame.
  const Vector3<T> centroid_W_;
};

/// The contact surface computed by GeometryWorld.
template <class T>
class ContactSurface {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactSurface)
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
