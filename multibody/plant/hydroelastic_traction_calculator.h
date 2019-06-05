#pragma once

#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/math/spatial_velocity.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

template <class T>
class MultibodyPlant;

namespace internal {

// Data structure for storing quantities used repeatedly in the hydroelastic
// traction calculations.
template <typename T>
class HydroelasticTractionCalculatorData {
 public:
  HydroelasticTractionCalculatorData() {}
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HydroelasticTractionCalculatorData)

  // @param context a pointer to the MultibodyPlant context that will maintained
  //        for the life of this object.
  // @param surface a pointer to the contact surface computed by the
  //        hydroelastic contact model that will be maintained for the life of
  //        this object.
  // @param plant a pointer to the the plant used to compute the data for the
  //        traction calculations that will be maintained for the life of this
  //        object.
  HydroelasticTractionCalculatorData(
      const systems::Context<T>* context,
      const geometry::ContactSurface<T>* surface,
      const MultibodyPlant<T>* plant);

  // Gets the context for the MultibodyPlant.
  const systems::Context<T>& context() const {
    DRAKE_DEMAND(context_);
    return *context_;
  }

  // Gets the ContactSurface passed to the data structure on construction.
  const geometry::ContactSurface<T>& surface() const {
    DRAKE_DEMAND(surface_);
    return *surface_;
  }

  // Gets the pose from Body A (the body that Geometry M in the contact surface
  // is affixed to) relative to the world frame.
  const math::RigidTransform<T>& X_WA() const { return X_WA_; }

  // Gets the pose from Body B (the body that Geometry N in the contact surface
  // is affixed to) relative to the world frame.
  const math::RigidTransform<T>& X_WB() const { return X_WB_; }

  // Gets the pose from Geometry N in the contact surface to the world frame.
  const math::RigidTransform<T>& X_WM() const { return X_WM_; }

  // Gets the spatial velocity of Body A (the body that Geometry M in the
  // contact surface is affixed to) at the origin of A's frame, measured
  // in the world frame and expressed in the world frame.
  const SpatialVelocity<T>& V_WA() const { return V_WA_; }

  // Gets the spatial velocity of Body B (the body that Geometry N in the
  // contact surface is affixed to) at the origin of B's frame, measured
  // in the world frame and expressed in the world frame.
  const SpatialVelocity<T>& V_WB() const { return V_WB_; }

  /**
   Convenience function for getting the pose of a geometry relative to the world
   frame.
   @param geometry_id the id of the requisite geometry.
   @pre geometry_id has been registered with the MultibodyPlant `this` was
        constructed with.
   */
  const math::RigidTransform<T> GetGeometryTransformationToWorld(
      geometry::GeometryId geometry_id) const;

 private:
  const systems::Context<T>* context_{nullptr};
  const geometry::ContactSurface<T>* surface_{nullptr};
  const MultibodyPlant<T>* plant_{nullptr};
  math::RigidTransform<T> X_WM_;
  math::RigidTransform<T> X_WA_;
  math::RigidTransform<T> X_WB_;
  SpatialVelocity<T> V_WA_;
  SpatialVelocity<T> V_WB_;
};
}  // namespace internal

/**
 A class for computing the spatial forces on rigid bodies in a MultibodyPlant
 as a function of the hydroelastic contact model.
 */
template <typename T>
class HydroelasticTractionCalculator {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HydroelasticTractionCalculator)
  explicit HydroelasticTractionCalculator(
      MultibodyPlant<T>* plant) : plant_(plant) {}

  /**
   Gets the regularization parameter used for friction (in m/s). The closer
   that this parameter is to zero, the closer that the regularized friction
   model will approximate Coulomb friction.
   */
  double regularization_scalar() const { return vslip_regularizer_; }

 private:
  // To allow GTEST to test private functions.
  friend class MultibodyPlantHydroelasticTractionTests;

  Vector3<T> CalcTractionAtPoint(
      const internal::HydroelasticTractionCalculatorData<T>& data,
      geometry::SurfaceFaceIndex face_index,
      const typename geometry::SurfaceMesh<T>::Barycentric&
          Q_barycentric_M, double dissipation, double mu_coulomb,
      Vector3<T>* p_WQ) const;

  void ComputeSpatialForcesAtBodyOriginsFromTraction(
      const internal::HydroelasticTractionCalculatorData<T>& data,
      const Vector3<T>& p_WQ,
      const Vector3<T>& traction_Q_W,
      multibody::SpatialForce<T>* F_Mo_W,
      multibody::SpatialForce<T>* F_No_W) const;

  Vector3<T> CalcContactPoint(
      const geometry::ContactSurface<T>& surface,
      geometry::SurfaceFaceIndex face_index,
      const typename geometry::SurfaceMesh<T>::Barycentric& r_barycentric_M,
      const math::RigidTransform<T>& X_WM) const;

  MultibodyPlant<T>* plant_{nullptr};

  // The parameter (in m/s) for regularizing the Coulomb friction model.
  double vslip_regularizer_{1e-6};
};

}  // namespace multibody
}  // namespace drake

// TODO(edrumwri) instantiate these on SymbolicExpression when they no longer
// causes a linker error complaining about an unresolved symbol in SceneGraph.
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::internal::HydroelasticTractionCalculatorData)
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::HydroelasticTractionCalculator)
