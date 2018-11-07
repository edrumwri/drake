#pragma once

#include "drake/multibody/multibody_tree/rigid_body.h"

namespace drake {
namespace multibody {

template <class T>
class HydrostaticContactModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HydrostaticContactModel)
  HydrostaticContactModel(const MultibodyTree<T>& tree);

  VectorX<T> ComputeGeneralizedForces(
      const Context<T>& context,
      const std::vector<geometry::ContactSurfaces<T>>& contact_surfaces) const;
};

}  // namespace multibody
}  // namespace drake
