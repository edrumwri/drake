#include "drake/multibody/contact_model/hydrostatic_contact_model.h"

template <class T>
void HydrostaticContactModel::ComputePressureDistribution(
    const Context<T>& context,
    const std::vector<geometry::ContactSurfaces<T>>& contact_surfaces) const {
  // Get the strain fields for each geometry.

  // Iterate over each contact surface.

  // Compute the centroid of the contact surface polygon.

  // Compute the area of the contact surface polygon.

  // Get the normal to the contact surface polygon.

  // Get the values of the two strain fields at the centroid.

  // 
}

// @pre pressure distribution, slip velocity has been computed
template <class T>
VectorX<T> HydrostaticContactModel::ComputeGeneralizedForces(
      const Context<T>& context,
      const std::vector<geometry::ContactSurfaces<T>>& contact_surfaces) const {
  // Note: we use a very simple algorithm that computes the stress at the center
  // of a polygon (using the already computed pressure distribution) and
  // integrates that stress over the surface of the polygon using a very simple
  // quadrature formula: the net force on the body (due to deformation of over
  // that particular polygon) is computed as the area of the contact polygon
  // times the stress at the center of the polygon. I call this vector the
  //  "normal force". The moment on the core is computed using this force
  // vector (with the moment arm being the vector from the rigid core to the
  // polygon centroid). The sliding frictional force is computed using the
  // velocity at the polygon centroid and the normal force.

  // Start with a zero generalized force vector.

  // Iterate over each contact surface.

    // Compute the centroid of the contact surface polygon.

    // Compute the area of the contact surface polygon.

    // Evaluate the pressure distribution at the polygon centroid.

    // Get the contact normal from the contact surface polygon.

    // Compute the normal force.

    // Evaluate the slip velocity field at the polygon centroid.

    // Compute the frictional force.

    // Update the generalized force vector to account for the effect of applying
    // the force at the contact surface on both bodies.
}

