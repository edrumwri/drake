#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/orthonormal_basis.h"
#include "drake/math/2d_and_3d_projection_matrices.h"

namespace drake {
namespace multibody {

/// A ContactSurfaceVertex augmented with samples taken from respective fields
/// at the vertices.
template <class T>
class AugmentedContactSurfaceVertex :
    public geometry::ContactSurfaceVertex<T> {
 public:
  AugmentedContactSurfaceVertex() {}
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AugmentedContactSurfaceVertex)

  // Note: the values below have been evaluated at the vertex to permit simple
  // approximation of the true fields using interpolation.
  /// The pressure evaluated at this vertex.
  T pressure;

  // The traction evaluated at this vertex.
  Vector3<T> traction;

  // The slip velocity at the vertex.
  Vector2<T> slip_velocity;
};

/// A ContactSurfaceFace augmented with methods to evaluate various fields
/// defined over the domain of the triangle using linear interpolation.
template <class T>
class AugmentedContactSurfaceFace :
    public geometry::ContactSurfaceFace<T> {
 public:
  AugmentedContactSurfaceFace(
      std::unique_ptr<AugmentedContactSurfaceVertex<T>> vA,
      std::unique_ptr<AugmentedContactSurfaceVertex<T>> vB,
      std::unique_ptr<AugmentedContactSurfaceVertex<T>> vC,
      const geometry::Field<T>* fieldA,
      const geometry::Field<T>* fieldB) :
      geometry::ContactSurfaceFace<T>(
          std::move(vA), std::move(vB), std::move(vC), fieldA, fieldB) {
  }

  AugmentedContactSurfaceFace(const AugmentedContactSurfaceFace& f) :
      geometry::ContactSurfaceFace<T>(f) {
    operator=(f);
  }

  AugmentedContactSurfaceFace& operator=(
      const AugmentedContactSurfaceFace<T>& f) {
    // Create vertices.
    auto& vertex_A = static_cast<const AugmentedContactSurfaceVertex<T>&>(
        f.vertex_A());
    auto& vertex_B = static_cast<const AugmentedContactSurfaceVertex<T>&>(
        f.vertex_B());
    auto& vertex_C = static_cast<const AugmentedContactSurfaceVertex<T>&>(
        f.vertex_C());
    this->vA_ = std::make_unique<AugmentedContactSurfaceVertex<T>>(vertex_A);
    this->vB_ = std::make_unique<AugmentedContactSurfaceVertex<T>>(vertex_B);
    this->vC_ = std::make_unique<AugmentedContactSurfaceVertex<T>>(vertex_C);

    // Copy field pointers.
    this->fA_ = f.fA_;
    this->fB_ = f.fB_;

    // Complete construction.
    this->Initialize();
    return *this;
  }

  AugmentedContactSurfaceFace(AugmentedContactSurfaceFace&&) = default;
  AugmentedContactSurfaceFace& operator=(
      AugmentedContactSurfaceFace&&) = default;

  // Evaluates the traction at a point using interpolation over the values
  // defined at the vertices.
  T CalculateTraction(const Vector3<T>& p) const {
    const Vector3<T> u = this->ConvertFromCartesianToBarycentricCoords(p);
    auto& vertex_A = static_cast<const AugmentedContactSurfaceVertex<T>&>(
        this->vertex_A());
    auto& vertex_B = static_cast<const AugmentedContactSurfaceVertex<T>&>(
        this->vertex_B());
    auto& vertex_C = static_cast<const AugmentedContactSurfaceVertex<T>&>(
        this->vertex_C());
    return u[0] * vertex_A.traction +
        u[1] * vertex_B.traction +
        u[2] * vertex_C.traction;
  }

  // Evaluates the pressure at a point using interpolation over the values
  // defined at the vertices.
  T EvaluatePressure(const Vector3<T>& p) const {
    const Vector3<T> u = this->ConvertFromCartesianToBarycentricCoords(p);
    auto& vertex_A = static_cast<const AugmentedContactSurfaceVertex<T>&>(
        this->vertex_A());
    auto& vertex_B = static_cast<const AugmentedContactSurfaceVertex<T>&>(
        this->vertex_B());
    auto& vertex_C = static_cast<const AugmentedContactSurfaceVertex<T>&>(
        this->vertex_C());
    return u[0] * vertex_A.pressure +
        u[1] * vertex_B.pressure +
        u[2] * vertex_C.pressure;
  }

  // Evaluates the slip velocity at a point using interpolation over the values
  // defined at the vertices.
  Vector2<T> EvaluateSlipVelocity(const Vector3<T>& p) const {
    const Vector3<T> u = this->ConvertFromCartesianToBarycentricCoords(p);
    auto& vertex_A = static_cast<const AugmentedContactSurfaceVertex<T>&>(
        this->vertex_A());
    auto& vertex_B = static_cast<const AugmentedContactSurfaceVertex<T>&>(
        this->vertex_B());
    auto& vertex_C = static_cast<const AugmentedContactSurfaceVertex<T>&>(
        this->vertex_C());
    return u[0] * vertex_A.slip_velocity +
        u[1] * vertex_B.slip_velocity +
        u[2] * vertex_C.slip_velocity;
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

  // NOTE: Consider eliminating this function as it is not being used.
  // Integrates the traction vectors over the surface of the triangle using a
  // simple quadrature rule.
  Vector3<T> IntegrateTractionSimple(double mu_dynamic) const {
    // The tolerance below which contact is assumed to be not-sliding.
    const double slip_tol = std::numeric_limits<double>::epsilon();

    // Get the contact normal from the contact surface triangle and expressed
    // in the global frame using the convention that the normal points toward
    // Body A.
    const Vector3<T>& nhat_W = this->normal_W();

    // Construct a matrix for projecting two-dimensional vectors in the plane
    // orthogonal to the contact normal to 3D.
    const Eigen::Matrix<T, 3, 2> P = math::Compute2dTo3dProjectionMatrix(
        nhat_W);

    // Evaluate the pressure distribution at the triangle centroid.
    const T pressure = EvaluatePressure(this->centroid_W());

    // Compute the normal force, expressed in the global frame.
    const Vector3<T> fN_W = nhat_W * pressure * this->area();

    // Get the slip velocity at the centroid.
    const Vector2<T> slip_vel_W = EvaluateSlipVelocity(this->centroid_W());

    // Get the direction of slip.
    const T slip_speed = slip_vel_W.norm();

    // Determine the slip direction expressed in the global frame.
    const Vector3<T> zeros_3 = Vector3<T>::Zero();
    const Vector3<T> slip_dir_W = (slip_speed > slip_tol) ?
                                  P * (slip_vel_W / slip_speed) :
                                  zeros_3;

    // Compute the frictional force.
    const Vector3<T> fF_W = (slip_speed > slip_tol) ?
                            (mu_dynamic * pressure * -slip_dir_W) :
                            zeros_3;

    // Increment the traction vector integral.
    return fN_W + fF_W;
  }
};

/// A geometry::ContactSurface<T> that has been augmented with field information
/// and permits integration of the fields over the domain of the contact
/// surface.
template <class T>
using AugmentedContactSurface =
    geometry::ContactSurfaceType<AugmentedContactSurfaceFace<T>>;

}  // namespace multibody
}  // namespace drake
