#include "drake/multibody/contact_model/hydrostatic_contact_model.h"

#include "drake/geometry/geometry_ids.h"
#include "drake/math/orthonormal_basis.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

// TODO: Replace copies with moves in AugmentedContactSurface.
// TODO: Check all class documentation.
// TODO: Add contact outputs for surface alone and surface + fields
// TODO: Ensure that everything works with caching.
// TODO: Enable GetVelocities.
// TODO: Convert point_W to point_A and point_B frames in CalcContactPointJacobianForHydrostaticModel
// TODO: Need a means to compute the pressure field.

namespace drake {
namespace multibody {


}  // namespace multibody
}  // namespace drake


template class drake::multibody::HydrostaticContactModel<double>;