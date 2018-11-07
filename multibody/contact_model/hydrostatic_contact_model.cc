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

}

