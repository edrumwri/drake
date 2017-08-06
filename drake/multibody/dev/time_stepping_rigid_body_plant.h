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
#include "drake/multibody/rigid_constraint/rigid_constraint_solver.h"

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
  const std::unique_ptr<multibody::rigid_constraint::RigidConstraintModel<T>>
      constraint_model_;
};

}  // namespace systems
}  // namespace drake
