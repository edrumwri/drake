#include "drake/multibody/dev/time_stepping_rigid_body_plant.h"

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
using drake::multibody::constraint::ConstraintSolver;

namespace drake {
namespace systems {

template <typename T>
TimeSteppingRigidBodyPlant<T>::TimeSteppingRigidBodyPlant(
    std::unique_ptr<const RigidBodyTree<T>> tree, double timestep)
    : RigidBodyPlant<T>(std::move(tree), timestep) {

  // Verify that the time-step is strictly positive.
  if (timestep <= 0.0) {
    throw std::logic_error("TimeSteppingRigidBodyPlant requires a positive "
                               "integration time step.");
  }
}

// Gets A's translational velocity relative to B's translational velocity at a
// point common to the two rigid bodies.
// @param p_W The point of contact (defined in the world frame).
// @returns the relative velocity at p_W expressed in the world frame.
template <class T>
Vector3<T> TimeSteppingRigidBodyPlant<T>::CalcRelTranslationalVelocity(
    const KinematicsCache<T>& kcache, const RigidBody* A, const RigidBody* B,
    const Vector3<T>& p_W) const {
  const auto& tree = this->get_tree();

  // TODO(edrumwri): Convert this method to avoid Jacobian computation.

  // Get the two body indices.
  const int body_a_index = A->get_body_index();
  const int body_b_index = B->get_body_index();

  // The reported point on A's surface (As) in the world frame (W).
  const Vector3<T> p_WAs =
      kcache.get_element(body_a_index).transform_to_world * pair.ptA;

  // The reported point on B's surface (Bs) in the world frame (W).
  const Vector3<T> p_WBs =
      kinsol.get_element(body_b_index).transform_to_world * pair.ptB;

  // The contact point in A's frame.
  const auto X_AW = kcache.get_element(body_a_index)
      .transform_to_world.inverse(Eigen::Isometry);
  const Vector3<T> p_A = X_AW * p_W;

  // The contact point in B's frame.
  const auto X_BW = kcache.get_element(body_b_index)
      .transform_to_world.inverse(Eigen::Isometry);
  const Vector3<T> p_B = X_BW * p_W;

  // Get the Jacobian matrices.
  const auto JA =
      tree.transformPointsJacobian(kcache, p_A, body_a_index, 0, false);
  const auto JB =
      tree.transformPointsJacobian(kcache, p_B, body_b_index, 0, false);

  // Compute the relative velocity in the world frame.
  return (JA - JB) * kcache.getV();
}

// Evaluates the relative velocities between two bodies projected along the
// contact normals.
template <class T>
VectorX<T> TimeSteppingRigidBodyPlant<T>::N_mult(
    const std::vector<ContactData>& contacts,
    const VectorX<T>& q,
    const VectorX<T>& v) const {
  const auto& tree = this->get_tree();
  auto kinsol = tree.doKinematics(q, v);

  // Create a result vector.
  VectorX<T> result(contacts.size());

  // Loop through all contacts.
  for (int i = 0; static_cast<size_t>(i) < contacts.size(); ++i) {
    // The *relative* velocity of the contact point in A relative to that in
    // B.
    const auto v_W = CalcRelTranslationalVelocity(kinsol, p_W);

    // Get the projected normal velocity
    result[i] = v_W.dot(contacts[i].normal);
  }

  return result;
}

// Evaluates the relative velocities between two bodies projected along the
// contact tangent directions.
template <class T>
VectorX<T> TimeSteppingRigidBodyPlant<T>::F_mult(
    const std::vector<ContactData>& contacts,
    const VectorX<T>& q,
    const VectorX<T>& v) const {
  using std::cos;
  using std::sin;

  const auto& tree = this->get_tree();
  std::vector<Vector3<T>> basis_vecs;
  auto kinsol = tree.doKinematics(q, v);

  // Create a result vector.
  VectorX<T> result(contacts.size());

  // Loop through all contacts.
  for (int i = 0, k = 0; static_cast<size_t>(i) < contacts.size(); ++i) {
    // The *relative* velocity of the contact point in A relative to that in
    // B.
    const auto v_W = CalcRelTranslationalVelocity(kinsol, p_W);

    // Compute an orthonormal basis.

    // Set spanning tangent directions.
    basis_vecs.resize(contacts[i].num_contact_edges);
    if (contacts[i].num_contact_edges == 2) {
      // Special case: pyramid friction.
      basis_vecs.front() = tan1_dir;
      basis_vecs.back() = tan2_dir;
    } else {
      for (int j = 0; j < contacts[i].num_cone_edges; ++j) {
        double theta = M_PI * j / (contacts[i].num_cone_edges - 1);
        basis_vecs[i] = tan1_dir * cos(theta) + tan2_dir * sin(theta);
      }
    }

    // Loop over the spanning tangent directions.
    for (int j = 0; j < basis_vecs.size(); ++j) {
      // Get the projected tangent velocity.
      result[k++] = v_W.dot(basis_vecs[j]);
    }
  }

  return result;
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
  // Get the time step.
  double dt = this->get_time_step();
  DRAKE_DEMAND(dt > 0.0);

  VectorX<T> u = this->EvaluateActuatorInputs(context);

  const int nq = this->get_num_positions();
  const int nv = this->get_num_velocities();
  const int num_actuators = this->get_num_actuators();

  // Initialize the velocity problem data.
  drake::multibody::constraint::ConstraintVelProblemData<T> data(nv);

  // Get the rigid body tree.
  const auto& tree = this->get_tree();

  // Get the system state.
  auto x = context.get_discrete_state(0)->get_value();
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  auto kinsol = tree.doKinematics(q, v);

  // Get the generalized inertia matrix and set up the inertia solve function.
  auto H = tree.massMatrix(kinsol);

  // TODO: Get H as a vector of block matrices.
  std::vector<Eigen::Ref<MatrixX<T>>> H_blocks;
  for (int i = 0; i < tree.get_num_model_instances(); ++i) {
    std::vector<const RigidBody<T>*> bodies = tree.FindModelInstanceBodies(i);

    // Determine the generalized velocity starting index and number of values
    // for this body.
    int gv_start = -1, nv = 0;
    for (int j = 0; static_cast<size_t>(j) < bodies.size(); ++j) {
      const RigidBody<T>& body_j = *bodies[j];
      if (body_j.has_parent_body()) {
        int v_start_j = body_j.get_velocity_start_index();
        int nv_j = body_j.getJoint().get_num_velocities();
        gv_start = min(gv_start, v_start_j);
        nv += nv_j;
      }
    }

    // Get the corresponding block of H.
    H_blocks.push_back(H.block(gv_start, gv_start, nv, nv));
  }

  // Compute the LDLT factorizations, which will be used by the solver.
  std::vector<Eigen::LDLT<Eigen::Matrix<T>>> ldlt(H_blocks.size());
  for (int i = 0; static_cast<size_t>(i) < H_blocks.size(); ++i) {
    ldlt[i].compute(H_blocks[i]);
    DRAKE_DEMAND(ldlt[i].info() == Eigen::Success);
  }

  // Set the inertia matrix solver.
  data->solve_inertia = [this, &ldlt](const MatrixX<T>& m) {
    DRAKE_DEMAND(m.rows() == v.size());
    MatrixX<T> result(v.size(), m.cols());
    for (int i = 0, start = 0; static_cast<size_t>(i) < H_blocks.size(); ++i) {
      result.block(start, 0, H_blocks[i].rows(), m.cols()) =
          ldlt[i].solve(m.block(start, 0, H_blocks[i].rows(), m.cols()))
      start += H_blocks[i].rows();
    }
    return result;
  };

  // There are no external wrenches, but it is a required argument in
  // dynamicsBiasTerm().
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;

  // right_hand_side is the right hand side of the system's equations:
  //   right_hand_side = B*u - C(q,v)
  VectorX<T> right_hand_side =
      -tree.dynamicsBiasTerm(kinsol, no_external_wrenches);
  if (num_actuators > 0) right_hand_side += tree.B * u;

  // TODO(edrumwri): Complete me!
  // Determine the set of contact points corresponding to the current q.
  std::vector<ContactData> contacts;

  // Determine the set of tangent spanning directions at each point of contact.

  // Determine the contact Jacobians corresponding to each projected direction
  // at each point of contact.

  // Set the joint range of motion limits.
  for (auto const& b : tree.bodies) {
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
      if (qjoint < qmin || qjoint + vjoint * dt < qmin) {
        // Institute a lower limit.
        limits.push_back(JointLimit());
        limits.back().v_index = b->get_velocity_start_index();
        limits.back().error = (qjoint - qmin) / dt * erp_;
        limits.back().lower_limit = true;
      }
      if (qjoint > qmax || qjoint + vjoint * dt > qmax) {
        // Institute an upper limit.
        limits.push_back(JointLimit());
        limits.back().v_index = b->get_velocity_start_index();
        limits.back().error = (qmax - qjoint) / dt * erp_;
        limits.back().lower_limit = false;
      }
    }
  }

  // Set the number of generic unilateral constraint functions.
  data.num_limit_constraints = limits.size();

  // Set up the N multiplication operator.
  data.N_mult = [this, &contacts, &q](const VectorX<T>& w) -> VectorX<T> {
    return N_mult(contacts, q, w);
  };

  // Set up the F multiplication operator.
  data.F_mult = [this, &contacts, &q](const VectorX<T>& w) -> VectorX<T> {
    return F_mult(contacts, q, w);
  };

  /*
  // Set the constraint Jacobian transpose operator.
  data.L_mult = [this, &limits](const VectorX<T>& v) -> VectorX<T> {
    VectorX<T> result(limits.size());
    for (int i = 0; static_cast<size_t>(i) < limits.size(); ++i) {
      const int index = limits[i].v_index;
      result[i] = (limits[i].lower_limit) ? v[index] : -v[index];
    }
    return result;
  };
  data.L_transpose_mult = [this, &v, &limits](const VectorX<T>& lambda) {
    VectorX<T> result(v.size());
    for (int i = 0; static_cast<size_t>(i) < limits.size(); ++i) {
      const int index = limits[i].v_index;
      result[index] = (limits[i].lower_limit) ? lambda[i] : -lambda[i];
    }
    return result;
  };
*/
  // Set the constraint error.

  // Integrate the forces into the velocity.
  data.v = v + right_hand_side * dt;

  // Solve the rigid impact problem.
  VectorX<T> vnew, cf;
  constraint_solver_.SolveImpactProblem(cfm_, data, &cf);
  constraint_solver_.ComputeGeneralizedVelocityChange(data, cf, &vnew);
  vnew += v;

  // qn = q + h*qdot.
  VectorX<T> xn(this->get_num_states());
  xn << q + dt * tree.transformVelocityToQDot(kinsol, vnew), vnew;
  updates->get_mutable_vector(0)->SetFromVector(xn);
}

// Explicitly instantiates on the most common scalar types.
template class TimeSteppingRigidBodyPlant<double>;

}  // namespace systems
}  // namespace drake
