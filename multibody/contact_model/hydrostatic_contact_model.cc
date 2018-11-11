#include "drake/multibody/contact_model/hydrostatic_contact_model.h"

#include "drake/geometry/geometry_ids.h"
#include "drake/math/orthonormal_basis.h"

// TODO: Consider whether this should really be a call or some methods in MBT.
// TODO: Need a means to compute the pressure field.
// TODO: How would this class be cloned / scalar copy constructed, assuming
// that it maintains pointers to multibody plant and multibody plant maintains
// a pointer to it.

// (Perhaps no longer relevant)
// TODO: Use a different way to index the fields than string?

namespace drake {
namespace multibody {

// This method is used to calculate the slip velocity for populating the
// ContactSurfaceVertex field samples. It can also be used to enable
// accurate quadrature over triangles as well.
template <class T>
Vector2<T> HydrostaticContactModel<T>::CalcSlipVelocityAtPoint(
    const systems::Context<T>& multibody_plant_context,
    const MatrixX<T>& J_Wp,
    const Vector3<T>& normal_W) const  {
  // Get the multibody velocity.
  Eigen::VectorBlock<const VectorX<T>> v = plant_->GetVelocities(
      multibody_plant_context);

  // Compute a projection matrix from 3D to 2D.
  const int kZAxisIndex = 2;
  Matrix3<T> R_WP = math::ComputeBasisFromAxis(kZAxisIndex, normal_W);
  const Eigen::Matrix<T, 2, 3> P = R_WP.template block<2, 3>(0, 0);

  // Calculate the slip velocity.
  return P * J_Wp.bottomRows(3) * v;
}

template <class T>
MatrixX<T> HydrostaticContactModel<T>::CalcContactPointJacobian(
    const systems::Context<T>& multibody_plant_context,
    const Vector3<T>& point_W,
    const Body<T>& body_A,
    const Body<T>& body_B) const  {
  const int num_spatial_dim = 6;

  // TODO: Convert point_W to point_A and point_B frames.

  // Get the geometric Jacobian for the velocity of the point
  // as moving with Body A.
  MatrixX<T> J_WAp(num_spatial_dim, plant_->num_velocities());
  plant_->tree().CalcFrameGeometricJacobianExpressedInWorld(
      multibody_plant_context, body_A.body_frame(), point_A, &J_WAp);

  // Get the geometric Jacobian for the velocity of the point
  // as moving with Body B.
  MatrixX<T> J_WBp(num_spatial_dim, plant_->num_velocities());
  plant_->tree().CalcFrameGeometricJacobianExpressedInWorld(
      multibody_plant_context, body_B.body_frame(), point_B, &J_WBp);

  // Compute the Jacobian.
  return J_WAp - J_WBp;
}

// @pre pressure distribution, slip velocity has been computed
template <class T>
VectorX<T> HydrostaticContactModel<T>::ComputeGeneralizedForces(
      const systems::Context<T>& multibody_plant_context,
      const geometry::SceneGraphInspector<T>& inspector,
      const std::vector<geometry::ContactSurface<T>>& contact_surfaces) const {
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
  VectorX<T> gf = VectorX<T>::Zero(plant_->num_velocities());

  // Iterate over all contact surfaces.
  for (const auto& surface : contact_surfaces) {
    // Get the two bodies.
    geometry::GeometryId geometry_A_id = surface.id_A();
    geometry::GeometryId geometry_B_id = surface.id_B();
    auto frame_A_id = inspector.GetFrameId(geometry_A_id);
    auto frame_B_id = inspector.GetFrameId(geometry_B_id);
    const Body<T>& body_A = *plant_->GetBodyFromFrameId(frame_A_id);
    const Body<T>& body_B = *plant_->GetBodyFromFrameId(frame_B_id);

    // Iterate over every triangle in the contact surface.
    for (const auto& triangle : surface.triangles()) {
      // Get the Jacobian matrix for applying a force at the centroid of the
      // contact surface.
      const MatrixX<T> J_Wp =  CalcContactPointJacobian(
          multibody_plant_context, triangle.centroid_W(), body_A, body_B);

      // Set the spatial force using the force on the bottom and moment on the
      // top in order to make it compatible with the Jacobian matrix.
      Eigen::Matrix<T, 6, 1> f_spatial;
      f_spatial.head(3).setZero();
      f_spatial.tail(3) = triangle.IntegrateTractionSimple();

      // Update the generalized force vector to account for the effect of
      // applying the force at the contact surface on both bodies.
      gf += J_Wp.transpose() * f_spatial;
    }
  }

  return gf;
}

}  // namespace multibody
}  // namespace drake


template class drake::multibody::HydrostaticContactModel<double>;