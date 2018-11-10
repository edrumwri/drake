#include "drake/multibody/contact_model/hydrostatic_contact_model.h"

#include "drake/math/orthonormal_basis.h"

// TODO: Use a different way to index the fields than string?

template <class T>
void HydrostaticContactModel::ComputePressureDistribution(
    const Context<T>& context,
    const Field<T, T>& pressure_field,
    geometry::ContactSurface<T>* contact_surface) const {

  // Iterate over every triangle in the contact surface.
  for (auto& tri : contact_surface->faces()) {
    // Create a pressure field over the surface of the triangle.
    tri.AddField<T, T>(&pressure_field, "pressure");
  }
}

// @pre pressure distribution, slip velocity has been computed
template <class T>
VectorX<T> HydrostaticContactModel::ComputeGeneralizedForces(
      const Context<T>& context,
      const std::vector<geometry::ContactSurfaces<T>>& contact_surfaces) const {
  // Note: we use a very simple algorithm that computes the stress at the center
  // of a triangle (using the already computed pressure distribution) and
  // integrates that stress over the surface of the triangle using a very simple
  // quadrature formula: the net force on the body (due to deformation of over
  // that particular triangle) is computed as the area of the contact triangle
  // times the stress at the center of the triangle. I call this vector the
  //  "normal force". The moment on the core is computed using this force
  // vector (with the moment arm being the vector from the rigid core to the
  // triangle centroid). The sliding frictional force is computed using the
  // velocity at the triangle centroid and the normal force.

  // Start with a zero generalized force vector.
  VectorX<T> gf = VectorX<T>::Zero(tree.num_velocities());

  // Iterate over all contact surfaces.
  for (const auto& surface : contact_surfaces) {
    // Iterate over every triangle in the contact surface.
    for (const auto& triangle : surface.get_triangles()) {
      // Get the centroid of the contact surface triangle.
      const Vector3<T>& centroid = triangle.get_centroid();

      // Get the area of the contact surface triangle.
      const T triangle_area = triangle.get_area();

      // Evaluate the pressure distribution at the triangle centroid.
      const T pressure = triangle.EvaluatePressure(centroid);

      // Get the contact normal from the contact surface triangle and expressed
      // in the global frame using the convention that the normal points toward
      // Body A.
      const Vector3<T>& normal = triangle.normal();

      // Compute the normal force, expressed in the global frame.
      const Vector3<T> fN_W = normal * pressure;

      // Evaluate the slip velocity field at the triangle centroid.
      const Vector2<T> slip_vel_W = triangle.EvaluateSlipVelocity(centroid);

      // The tolerance below which contact is assumed to be not-sticking.
      const double slip_tol = std::numeric_limits<double>::epsilon();

      // Get the direction of slip.
      const T slip_speed = slip_vel_W.norm();

      // Construct a matrix for projecting two-dimensional vectors in the plane
      // orthogonal to the contact normal to 3D.
      const int axis = 2;
      Matrix3<T> PT = math::ComputeBasisFromAxis(axis, normal);
      PT.col(axis).setZero();
      const Matrix<3, 2, T> P = PT.transpose().block<3, 2>(0, 0);

      // TODO: Get the coefficient of friction.

      // Determine the slip direction expressed in the global frame.
      const Vector3<T> slip_dir_W = (slip_speed > slip_tol) ?
                                    P * (slip_vel_W / slip_speed) :
                                    Vector3<T>::Zero();

      // Compute the frictional force.
      const Vector3<T> fF_W = (slip_speed > slip_tol) ?
                              mu_coulomb * pressure * -slip_dir_W :
                              Vector3<T>::Zero();

      // TODO:
      // Get the Jacobian matrix for applying a force at the centroid of the
      // contact surface.

      // Update the generalized force vector to account for the effect of
      // applying the force at the contact surface on both bodies.
      gf += J.transpose() * (fN_W + fF_W);
    }
  }

}
