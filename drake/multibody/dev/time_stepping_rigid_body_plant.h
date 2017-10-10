#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {

/// This class provides a System interface around a multibody dynamics model
/// of the world represented by a RigidBodyTree, implemented as a first order
/// discretization of rigid body dynamics and constraint equations, without
/// stepping to event times.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
/// @ingroup rigid_body_systems
template <typename T>
class TimeSteppingRigidBodyPlant : public RigidBodyPlant<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TimeSteppingRigidBodyPlant)

  /// Instantiates a %TimeSteppingRigidBodyPlant from a Multi-Body Dynamics
  /// (MBD) model of the world in `tree`.  `tree` must not be `nullptr`.
  ///
  /// @param[in] tree the dynamic model to use with this plant.
  /// @param[in] timestep a strictly positive, floating point value specifying
  /// the update period of the model (in seconds).
  /// @throws std::logic_error when timestep is non-positive.
  TimeSteppingRigidBodyPlant(std::unique_ptr<const RigidBodyTree<T>> tree,
                          double timestep);

  /// Sets the default stiffness parameter. Aborts if negative. Default value
  /// is 1000.
  void set_default_stiffness(double stiffness) {
    DRAKE_DEMAND(stiffness >= 0);
    default_stiffness_ = stiffness;
  }

  /// Sets the default damping parameter. Aborts if negative. Default value is
  /// 100.
  void set_default_damping(double damping) {
    DRAKE_DEMAND(damping >= 0);
    default_damping_ = damping;
  }

  /// Sets the default friction coefficient. Aborts if negative. Default
  /// value is 0.1.
  void set_default_friction_coefficient(double mu) {
    DRAKE_DEMAND(mu >= 0);
    mu_ = mu;
  }

 private:
  void DoCalcDiscreteVariableUpdates(const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<double>*>&,
      DiscreteValues<T>* updates) const override;

  // Pointer to the class that performs all constraint computations.
  multibody::constraint::ConstraintSolver<T> constraint_solver_;

 private:
  // Structure for storing joint limit data for time stepping.
  struct JointLimit {
    // The index for the joint limit.
    int v_index{-1};

    // Whether the limit is a lower limit or upper limit.
    bool lower_limit{false};

    // Gets the "error", meaning the amount over the limit (if the error is
    // positive) or under the limit (if the error is negative). Negative error
    // is not error per se, but rather a way to limit the movement of a joint
    // by the step into the future.
    T error{0};
  };

  bool CalcContactStiffnessDampingAndMu(
      const drake::multibody::collision::PointPair& contact,
      double* stiffness,
      double* damping,
      double* mu) const;
  Vector3<T> CalcRelTranslationalVelocity(
      const KinematicsCache<T>& kcache, int body_a_index, int body_b_index,
      const Vector3<T>& p_W) const;
  void UpdateGeneralizedForce(
      const KinematicsCache<T>& kcache, int body_a_index, int body_b_index,
      const Vector3<T>& p, const Vector3<T>& f, VectorX<T>* gf) const;
  VectorX<T> N_mult(
      const std::vector<drake::multibody::collision::PointPair>& contacts,
      const VectorX<T>& q,
      const VectorX<T>& v) const;
  VectorX<T> N_transpose_mult(
      const std::vector<drake::multibody::collision::PointPair>& contacts,
      const KinematicsCache<T>& kcache,
      const VectorX<T>& f) const;
  VectorX<T> F_mult(
      const std::vector<drake::multibody::collision::PointPair>& contacts,
      const VectorX<T>& q,
      const VectorX<T>& v) const;
  VectorX<T> F_transpose_mult(
      const std::vector<drake::multibody::collision::PointPair>& contacts,
      const KinematicsCache<T>& kcache,
      const VectorX<T>& f) const;

  // The coefficient of friction. Must be non-negative.
  double mu_{0.1};

  // Half of the number of edges in the friction cone approximation for
  // contacts in 3D. Must be no fewer than 2 (equates to a friction pyramid).
  int half_cone_edges_{2};

  // The default stiffness value, in the range [0, infinity]. This value will
  // be applied to all constraints that are not otherwise provided a stiffness.
  // Low stiffnesses are less likely to cause "popping" effects.
  double default_stiffness_{1e10};

  // The default damping value, in the range [0, infinity]. This value will
  // be applied to all constraints that are not otherwise provided a damping
  // value.
  double default_damping_{1e7};
};

}  // namespace systems
}  // namespace drake
