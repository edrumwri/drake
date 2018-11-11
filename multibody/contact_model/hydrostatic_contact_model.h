#pragma once

#include "drake/common/eigen_types.h"
#include "drake/geometry/contact_surface.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/systems/framework/context.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

namespace drake {
namespace multibody {

template <class T>
class HydrostaticContactModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HydrostaticContactModel)
  HydrostaticContactModel(const multibody_plant::MultibodyPlant<T>& plant);

  Vector2<T> CalcSlipVelocityAtPoint(
      const systems::Context<T>& multibody_plant_context,
      const MatrixX<T>& J_Wp,
      const Vector3<T>& normal_W) const;

    VectorX<T> ComputeGeneralizedForces(
      const systems::Context<T>& context,
      const geometry::SceneGraphInspector<T>& inspector,
      const std::vector<geometry::ContactSurface<T>>& contact_surfaces) const;

 private:
  MatrixX<T> CalcContactPointJacobian(
      const systems::Context<T>& multibody_plant_context,
      const Vector3<T>& point_W,
      const Body<T>& body_A,
      const Body<T>& frame_B) const;

    multibody_plant::MultibodyPlant<T>* plant_{nullptr};
};

}  // namespace multibody
}  // namespace drake
