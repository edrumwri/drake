#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/SparseCholesky>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/constraint/constraint_problem_data.h"
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
TimeSteppingRigidBodyPlant<T>::TimeSteppingRigidBodyPlant(
    std::unique_ptr<const RigidBodyTree<T>> tree, double timestep)
    : RigidBodyPlant<T>(tree, timestep), rigid_constraint_model_(
    std::make_unique<RigidConstraintModel<T>>()) {

  // Verify that the time-step is strictly positive.
  if (timestep <= 0.0) {
    throw std::logic_error("TimeSteppingRigidBodyPlant requires a positive "
                               "integration time step.");
  }
}

template <typename T>
void TimeSteppingRigidBodyPlant<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>&,
    drake::systems::DiscreteValues<T>* updates) const {
  using std::abs;
  std::vector<JointLimit> limits;

  static_assert(std::is_same<double, T>::value,
                "Only support templating on double for now");
  DRAKE_DEMAND(timestep_ > 0.0);

  VectorX<T> u = EvaluateActuatorInputs(context);

  const int nq = get_num_positions();
  const int nv = get_num_velocities();
  const int num_actuators = get_num_actuators();

  // Initialize the velocity problem data.
  drake::multibody::constraint::ConstraintVelProblemData<T> data(nv);

  // Get the system state.
  auto x = context.get_discrete_state(0)->get_value();
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  auto kinsol = tree_->doKinematics(q, v);

  // Get the generalized inertia matrix and set up the inertia solve function.
  auto H = tree_->massMatrix(kinsol);

  // Get H as a sparse matrix.
  std::vector<Eigen::Triplet<double>> coefficients;
  Eigen::SparseMatrix<double> sparseH(H.rows(), H.cols());
  const double zero_tol = 1e-12;
  for (int i = 0; i < H.rows(); ++i) {
    for (int j = 0; j < H.cols(); ++j) {
      if (abs(H(i,j)) > zero_tol) {
        const T& value = H(i,j);
        coefficients.push_back(Eigen::Triplet<double>(i, j, value));
        coefficients.push_back(Eigen::Triplet<double>(j, i, value));
      }
    }
  }
  sparseH.setFromTriplets(coefficients.begin(), coefficients.end());

  // Compute the LDLT factorization, which will be used by the solver.
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt(sparseH);
  DRAKE_DEMAND(ldlt.info() == Eigen::Success);

  // Set the inertia solver.
  data->solve_inertia = [this, &ldlt](const MatrixX<T>& m) {
    return ldlt.solve(m);
  };

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

  // Set the joint range of motion limits.
  for (auto const& b : tree_->bodies) {
    if (!b->has_parent_body()) continue;
    auto const& joint = b->getJoint();

    // Joint limit forces are only implemented for single-axis joints.
    if (joint.get_num_positions() == 1 && joint.get_num_velocities() == 1) {
      const T qmin = joint.getJointLimitMin()(0);
      const T qmax = joint.getJointLimitMax()(0);
      DRAKE_DEMAND(qmin < qmax);

      // Get the current joint position and velocity.
      const T& qjoint = q(b->get_position_start_index());
      const T& vjoint = v(b->get_velocity_start_index());

      // See whether the *current* joint velocity might lead to a limit
      // violation.
      if (qjoint < qmin || qjoint + vjoint*timestep_ < qmin) {
        // Institute a lower limit.
        limits.push_back(JointLimit());
        limits.back().v_index = b->get_velocity_start_index();
        limits.back().error = (qjoint - qmin)/timestep_ * erp_;
        limits.back().lower_limit = true;
      }
      if (qjoint > qmax || qjoint + vjoint*timestep_ > qmax) {
        // Institute an upper limit.
        limits.push_back(JointLimit());
        limits.back().v_index = b->get_velocity_start_index();
        limits.back().error = (qmax - qjoint)/timestep_ * erp_;
        limits.back().lower_limit = false;
      }
    }
  }

  // Set the number of generic unilateral constraint functions.
  data->num_limit_constraints = limits.size();

  // Set the constraint Jacobian transpose operator.
  data->L_mult = [this, &limits](const VectorX<T>& v) -> VectorX<T> {
    VectorX<T> result(limits.size());
    for (int i = 0; static_cast<size_t>(i) < limits.size(); ++i) {
      const int index = limits[i].v_index;
      result[i] = (limits[i].lower_limit) ? v[index] : -v[index];
    }
    return result;
  };
  data->L_transpose_mult = [this, &v, &limits](const VectorX<T>& lambda) {
    VectorX<T> result(v.size());
    for (int i = 0; static_cast<size_t>(i) < limits.size(); ++i) {
      const int index = limits[i].v_index;
      result[index] = (limits[i].lower_limit) ? lambda[i] : -lambda[i];
    }
    return result;
  };

  // Set the constraint error.

  // Integrate the forces into the velocity.
  data.v = v + right_hand_side * timestep_;

  // Solve the rigid impact problem.
  VectorX<T> vnew, cf;
  constraint_solver_.SolveImpactProblem(cfm_, data, &cf);
  constraint_solver_.ComputeGeneralizedVelocityChange(data, cf, &vnew);
  vnew += v;

  // qn = q + h*qdot.
  VectorX<T> xn(get_num_states());
  xn << q + timestep_ * tree_->transformVelocityToQDot(kinsol, vnew), vnew;
  updates->get_mutable_vector(0)->SetFromVector(xn);
}

// Explicitly instantiates on the most common scalar types.
template class TimeSteppingRigidBodyPlant<double>;

}  // namespace systems
}  // namespace drake
