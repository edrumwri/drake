#include "drake/multibody/contact_model/hydrostatic_contact_model.h"

#include "drake/geometry/geometry_ids.h"
#include "drake/math/orthonormal_basis.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

// TODO: Ensure that everything works with caching.
// TODO: Enable GetVelocities.
// TODO: Convert point_W to point_A and point_B frames in CalcContactPointJacobianForHydrostaticModel
// TODO: Need a means to compute the pressure field.
// TODO: How would this class be cloned / scalar copy constructed, assuming
// that it maintains pointers to multibody plant and multibody plant maintains
// a pointer to it.

// (Perhaps no longer relevant)
// TODO: Use a different way to index the fields than string?

namespace drake {
namespace multibody {


}  // namespace multibody
}  // namespace drake


template class drake::multibody::HydrostaticContactModel<double>;