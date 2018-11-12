#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/orthonormal_basis.h"

namespace drake {
namespace multibody {

template <class T>
struct AugmentedContactSurfaceVertex : 
    public geometry::ContactSurfaceVertex<T> {
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
class AugmentedContactSurfaceFace :
    public geometry::ContactSurfaceFace<T> {
 public:

  // TODO: vertices must be specified in the proper order so that the normal
  // and area is correct.
  AugmentedContactSurfaceFace(
      AugmentedContactSurfaceVertex<T>* vA,
      AugmentedContactSurfaceVertex<T>* vB,
      AugmentedContactSurfaceVertex<T>* vC,
      const geometry::Tetrahedron<T>* tA,
      const geometry::Tetrahedron<T>* tB) :
      geometry::ContactSurfaceFace<T>(vA, vB, vC, tA, tB) {
  }

  // Evaluates the traction at a point using interpolation over the values
  // defined at the vertices.
  T CalculateTraction(const Vector3<T>& p) const {
    const Vector3<T> u = this->ConvertFromCartesianToBarycentricCoords(p);
    auto vertex_A = static_cast<const AugmentedContactSurfaceVertex<T>*>(
        this->vertex_A());
    auto vertex_B = static_cast<const AugmentedContactSurfaceVertex<T>*>(
        this->vertex_B());
    auto vertex_C = static_cast<const AugmentedContactSurfaceVertex<T>*>(
        this->vertex_C());
    return u[0] * vertex_A->traction +
        u[1] * vertex_B->traction +
        u[2] * vertex_C->traction;
  }

  // Evaluates the pressure at a point using interpolation over the values
  // defined at the vertices.
  T EvaluatePressure(const Vector3<T>& p) const {
    const Vector3<T> u = this->ConvertFromCartesianToBarycentricCoords(p);
    auto vertex_A = static_cast<const AugmentedContactSurfaceVertex<T>*>(
        this->vertex_A());
    auto vertex_B = static_cast<const AugmentedContactSurfaceVertex<T>*>(
        this->vertex_B());
    auto vertex_C = static_cast<const AugmentedContactSurfaceVertex<T>*>(
        this->vertex_C());
    return u[0] * vertex_A->pressure +
        u[1] * vertex_B->pressure +
        u[2] * vertex_C->pressure;
  }

  // Evaluates the slip velocity at a point using interpolation over the values
  // defined at the vertices.
  Vector2<T> EvaluateSlipVelocity(const Vector3<T>& p) const {
    const Vector3<T> u = this->ConvertFromCartesianToBarycentricCoords(p);
    auto vertex_A = static_cast<const AugmentedContactSurfaceVertex<T>*>(
        this->vertex_A());
    auto vertex_B = static_cast<const AugmentedContactSurfaceVertex<T>*>(
        this->vertex_B());
    auto vertex_C = static_cast<const AugmentedContactSurfaceVertex<T>*>(
        this->vertex_C());
    return u[0] * vertex_A->slip_velocity +
        u[1] * vertex_B->slip_velocity +
        u[2] * vertex_C->slip_velocity;
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
    const Vector3<T>& nhat_W = this->normal_W();

    // Construct a matrix for projecting two-dimensional vectors in the plane
    // orthogonal to the contact normal to 3D.
    const Eigen::Matrix<T, 3, 2> P = Get2DTo3DProjectionMatrix(nhat_W);

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

 private:
  T mu_coulomb_{0.0};        // The coefficient of friction between the bodies.
};

/// A geometry::ContactSurface<T> that has been augmented with field information
/// and permits integration of the fields over the domain of the contact
/// surface.
template <class T>
using AugmentedContactSurface =
    geometry::ContactSurfaceType<AugmentedContactSurfaceFace<T>,
                       AugmentedContactSurfaceVertex<T>>;

/*
template <class T>
class AugmentedContactSurface {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AugmentedContactSurface)
  AugmentedContactSurface(
      geometry::GeometryId A, geometry::GeometryId B) : id_A_(A), id_B_(B) {}
  const std::vector<AugmentedContactSurfaceFace<T>> triangles()
      const { return faces_; }
  geometry::GeometryId id_A() const { return id_A_; }
  geometry::GeometryId id_B() const { return id_B_; }

 private:
  /// The id of the first geometry in the contact.
  const geometry::GeometryId id_A_;

  /// The id of the second geometry in the contact.
  const geometry::GeometryId id_B_;

  /// Vertices comprising the contact surface.
  std::vector<AugmentedContactSurfaceVertex<T>> vertices_;

  /// Triangles comprising the contact surface.
  std::vector<AugmentedContactSurfaceFace<T>> faces_;
};
*/
}  // namespace multibody
}  // namespace drake
