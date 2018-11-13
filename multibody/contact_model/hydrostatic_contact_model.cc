#include "drake/multibody/contact_model/hydrostatic_contact_model.h"

#include "drake/geometry/geometry_ids.h"
#include "drake/math/orthonormal_basis.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

// TODO: Add tests for 2d / 3d projection matrices
// TODO: Replace copies with moves in AugmentedContactSurface.
// TODO: Enable GetVelocities.
// TODO: Implement barycentric coordinate calculation.
namespace drake {
namespace multibody {


}  // namespace multibody
}  // namespace drake


template class drake::multibody::HydrostaticContactModel<double>;