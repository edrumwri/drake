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

  // TODO(SeanCurtis-TRI): Link to documentation explaining these parameters
  // in detail.  To come in a subsequent PR.
  /// Sets only the parameters for *normal* contact.  This is a convenience
  /// function to allow for more targeted parameter tuning.
  void set_normal_contact_parameters(double penetration_stiffness,
                                     double dissipation);

  /// Sets only the parameters for *friction* contact.  This is a convenience
  /// function to allow for more targeted parameter tuning.
  void set_friction_contact_parameters(double static_friction_coef,
                                       double dynamic_friction_coef,
                                       double v_stiction_tolerance);

 protected:
  void DoCalcDiscreteVariableUpdates(const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<double>*>&,
      DiscreteValues<T>* updates) const override;

  // Pointer to the class that encapsulates all the rigid constraint
  // computations.
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

  // Structure for storing contact data for time stepping.
  struct ContactData {
    /// Model Id of Body A participating in the contact.
    int idA{0};

    /// Model Id of Body B participating in the contact.
    int idB{0};

    /// Point of contact on the surface of Body A, expressed in A's frame.
    Vector3<T> ptA;

    /// Point of contact on the surface of Body B, expressed in B's frame.
    Vector3<T> ptB;

    /// Coefficient of friction at the point of contact. A small amount of
    /// friction is employed by default.
    double mu{0.1};

    /// Outward-pointing normal on body B, expressed in the world frame. On
    /// Body A it points in the opposite direction.
    Vector3<T> normal;

    /// The number of edges in the friction cone approximation. Must be a
    /// multiple of two *and* must be at least two.
    int num_cone_edges{2};

    // Gets the "error", meaning the amount of interpenetration (if the error
    // is positive) or separation distance (if the error is negative). Negative
    // error is not error per se, but rather a way to limit the relative motion
    // of two bodies not yet in contact by the step into the future.
    T error{0};
  };

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
