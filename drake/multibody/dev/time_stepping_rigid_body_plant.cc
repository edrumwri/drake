#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/solvers/mathematical_program.h"

using std::make_unique;
using std::move;
using std::runtime_error;
using std::string;
using std::unique_ptr;
using std::vector;

using drake::multibody::collision::ElementId;
using drake::multibody::rigid_constraint::RigidConstraintModel;

namespace drake {
namespace systems {

template <typename T>
TimeSteppingRigidBodyPlant<T>::TimeSteppingRigidBodyPlant(std::unique_ptr<const RigidBodyTree<T>> tree,
                                  double timestep)
    : RigidBodyPlant<T>(tree, timestep), rigid_constraint_model_(
    std::make_unique<RigidConstraintModel<T>>()) {

  // Verify that the time-step is strictly positive.
}

template <typename T>
void TimeSteppingRigidBodyPlant<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>&,
    drake::systems::DiscreteValues<T>* updates) const {
  static_assert(std::is_same<double, T>::value,
                "Only support templating on double for now");
  if (timestep_ == 0.0) return;

  VectorX<T> u = EvaluateActuatorInputs(context);

  const int nq = get_num_positions();
  const int nv = get_num_velocities();
  const int num_actuators = get_num_actuators();

  // Get the system state.
  auto x = context.get_discrete_state(0)->get_value();
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  auto kinsol = tree_->doKinematics(q, v);

  // Get the generalized inertia matrix and set up the inertia solve function.
  auto H = tree_->massMatrix(kinsol);
  

  // There are no external wrenches, but it is a required argument in
  // dynamicsBiasTerm().
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;

  // right_hand_side is the right hand side of the system's equations:
  //   right_hand_side = B*u - C(q,v)
  VectorX<T> right_hand_side =
      -tree_->dynamicsBiasTerm(kinsol, no_external_wrenches);
  if (num_actuators > 0) right_hand_side += tree_->B * u;

  // Determine the set of contact points corresponding to the current q.

  // Determine the set of tangent spanning directions at each point of contact.

  // Determine the contact Jacobians corresponding to each projected direction
  // at each point of contact.

  // Set the joint limits.

  // Set the constraint violations.

  // Integrate the forces into the velocity.

  // Set the rigid impact problem data.

  // Solve the rigid impact problem.

  // TODO(russt): Handle joint limits.
  // TODO(russt): Handle contact constraints.

  // Add H*(vn - v)/h = right_hand_side
  prog.AddLinearEqualityConstraint(H / timestep_,
                                   H * v / timestep_ + right_hand_side, vn);

  VectorX<T> xn(get_num_states());
  const auto& vn_sol = prog.GetSolution(vn);

  // qn = q + h*qdot.
  xn << q + timestep_ * tree_->transformVelocityToQDot(kinsol, vn_sol), vn_sol;
  updates->get_mutable_vector(0)->SetFromVector(xn);
}

// Explicitly instantiates on the most common scalar types.
template class TimeSteppingRigidBodyPlant<double>;

}  // namespace systems
}  // namespace drake
