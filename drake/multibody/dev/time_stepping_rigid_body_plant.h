#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/constraint/constraint_solver.h"

namespace drake {
namespace systems {

/// This class provides a System interface around a multibody dynamics model
/// of the world represented by a RigidBodyTree.
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
  TimeSteppingRigidBodyPlant(std::unique_ptr<const RigidBodyTree<T>> tree,
                          double timestep);

  /// The default Coulomb friction coefficient.
  double mu_{0.1};

  /// Sets the ERP parameter. Aborts if not in the range [0,1]. Default value
  /// is 0.1.
  void set_erp(double erp) { DRAKE_DEMAND(erp >= 0 && erp <= 1); erp_ = erp; }

  /// Sets the CFM parameter. Aborts if negative. Default value is 1e-12.
  void set_cfm(double cfm) { DRAKE_DEMAND(cfm >= 0); cfm_ = cfm; }

 protected:
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

  void CalcContactStiffnessAndDamping(
      const drake::multibody::collision::PointPair& contact,
      double* stiffness,
      double* damping) const;
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
      const VectorX<T>& q,
      const VectorX<T>& v,
      const VectorX<T>& f) const;
  VectorX<T> F_mult(
      const std::vector<drake::multibody::collision::PointPair>& contacts,
      const VectorX<T>& q,
      const VectorX<T>& v) const;
  VectorX<T> F_transpose_mult(
      const std::vector<drake::multibody::collision::PointPair>& contacts,
      const VectorX<T>& q,
      const VectorX<T>& v,
      const VectorX<T>& f) const;

  // Half of the number of edges in the friction cone approximation for
  // contacts in 3D. Must be no fewer than 2 (equates to a friction pyramid).
  int half_cone_edges_{2};

  // The "error reduction parameter" (ERP), first seen in CM Labs' Vortex and
  // Open Dynamics Engine, and formulated from [Lacoursiere 2007], which
  // determines how rapidly constraint errors are corrected. ERP values of zero
  // indicate constraint errors will not be corrected, while ERP values of one
  // indicate constraint errors will be corrected at every time step (ERP values
  // outside of the range [0,1] are invalid, and will cause assertion failures).
  // Since Lacoursiere's constraint stabilization process assumes that the
  // constraint function is approximately linear in position, values of ERP
  // smaller than unity are generally recommended. A generally safe value of 0.1
  // is the the default, and can be increased as desired to mitigate constraint
  // error.
  double erp_{0.1};

  // The "constraint force mixing" (CFM) parameter, first seen in CM Labs'
  // Vortex and Open Dynamics Engine, and formulated from [Lacoursiere 2007],
  // which determines how constraints are "softened" (allowed to become
  // violated), which generally provides numerical robustness along with a
  // reduction in constraint stiffness (particularly useful for contact). CFM
  // values of zero yield no softening, while CFM values of infinity yield
  // constraints that are completely unenforced. Typical values for CFM
  // lie in the range [1e-12, 1e-6], but this range is just a rough guideline.
  double cfm_{1e-12};
};

}  // namespace systems
}  // namespace drake
