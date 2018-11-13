#include "drake/multibody/contact_model/hydrostatic_contact_model.h"

#include "drake/geometry/geometry_ids.h"
#include "drake/math/orthonormal_basis.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

// TODO: Replace copies with moves in AugmentedContactSurface.
// TODO: Implement barycentric coordinate calculation.
namespace drake {
namespace multibody {


}  // namespace multibody
}  // namespace drake


template class drake::multibody::HydrostaticContactModel<double>;