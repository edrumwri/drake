#include "drake/multibody/dev/time_stepping_rigid_body_plant.h"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Cholesky>

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

  static_assert(std::is_same<double, T>::value,
                "Only support templating on double for now");

  // Verify that the time-step is strictly positive.
  if (timestep <= 0.0) {
    throw std::logic_error("TimeSteppingRigidBodyPlant requires a positive "
                               "integration time step.");
  }

  // Schedule the time stepping.
  this->DeclarePeriodicDiscreteUpdate(timestep);
}

// Gets A's translational velocity relative to B's translational velocity at a
// point common to the two rigid bodies.
// @param p_W The point of contact (defined in the world frame).
// @returns the relative velocity at p_W expressed in the world frame.
template <class T>
Vector3<T> TimeSteppingRigidBodyPlant<T>::CalcRelTranslationalVelocity(
    const KinematicsCache<T>& kcache, int body_a_index, int body_b_index,
    const Vector3<T>& p_W) const {
  const auto& tree = this->get_rigid_body_tree();

  // TODO(edrumwri): Convert this method to avoid Jacobian computation using
  // RigidBodyTree::CalcBodySpatialVelocityInWorldFrame().

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

// Updates a generalized force from a force of f (expressed in the world frame)
// applied at point p (defined in the global frame).
template <class T>
void TimeSteppingRigidBodyPlant<T>::UpdateGeneralizedForce(
    const KinematicsCache<T>& kcache, int body_a_index, int body_b_index,
    const Vector3<T>& p_W, const Vector3<T>& f, VectorX<T>* gf) const {
  const auto& tree = this->get_rigid_body_tree();

  // TODO(edrumwri): Convert this method to avoid Jacobian computation using
  // RigidBodyTree::dynamicsBiasTerm().

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

  // Compute the Jacobian transpose times the force, and use it to update gf.
  (*gf) += (JA - JB).transpose() * f;
}

// Evaluates the relative velocities between two bodies projected along the
// contact normals.
template <class T>
VectorX<T> TimeSteppingRigidBodyPlant<T>::N_mult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const KinematicsCache<T>& kcache) const {
  // Create a result vector.
  VectorX<T> result(contacts.size());

  // Loop through all contacts.
  for (int i = 0; static_cast<size_t>(i) < contacts.size(); ++i) {
    // Get the two body indices.
    const int body_a_index = contacts[i].elementA->get_body()->get_body_index();
    const int body_b_index = contacts[i].elementB->get_body()->get_body_index();

    // The reported point on A's surface (As) in the world frame (W).
    const Vector3<T> p_WAs =
        kcache.get_element(body_a_index).transform_to_world * contacts[i].ptA;

    // The reported point on B's surface (Bs) in the world frame (W).
    const Vector3<T> p_WBs =
        kcache.get_element(body_b_index).transform_to_world * contacts[i].ptB;

    // Get the point of contact in the world frame.
    const Vector3<T> p_W = (p_WAs + p_WBs) * 0.5;

    // The *relative* velocity of the contact point in A relative to that in
    // B.
    const auto v_W = CalcRelTranslationalVelocity(kcache, body_a_index,
                                                  body_b_index, p_W);

    // Get the projected normal velocity
    result[i] = v_W.dot(contacts[i].normal);
  }

  return result;
}

// Applies forces along the contact normals at the contact points and gets the
// effect out on the generalized forces.
template <class T>
VectorX<T> TimeSteppingRigidBodyPlant<T>::N_transpose_mult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const KinematicsCache<T>& kcache,
    const VectorX<T>& f) const {
  // Create a result vector.
  VectorX<T> result = VectorX<T>::Zero(kcache.getV().size());

  // Loop through all contacts.
  for (int i = 0; static_cast<size_t>(i) < contacts.size(); ++i) {
    // Get the two body indices.
    const int body_a_index = contacts[i].elementA->get_body()->get_body_index();
    const int body_b_index = contacts[i].elementB->get_body()->get_body_index();

    // The reported point on A's surface (As) in the world frame (W).
    const Vector3<T> p_WAs =
        kcache.get_element(body_a_index).transform_to_world * contacts[i].ptA;

    // The reported point on B's surface (Bs) in the world frame (W).
    const Vector3<T> p_WBs =
        kcache.get_element(body_b_index).transform_to_world * contacts[i].ptB;

    // Get the point of contact in the world frame.
    const Vector3<T> p_W = (p_WAs + p_WBs) * 0.5;

    // Get the contribution to the generalized force from a force of the
    // specified normal applied at this point.
    UpdateGeneralizedForce(kcache, body_a_index, body_b_index, p_W,
                           contacts[i].normal * f[i], &result);
  }

  return result;
}

// Evaluates the relative velocities between two bodies projected along the
// contact tangent directions.
template <class T>
VectorX<T> TimeSteppingRigidBodyPlant<T>::F_mult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const KinematicsCache<T>& kcache) const {
  using std::cos;
  using std::sin;
  std::vector<Vector3<T>> basis_vecs;

  // Create a result vector.
  VectorX<T> result(contacts.size() * half_cone_edges_);

  // Loop through all contacts.
  for (int i = 0, k = 0; static_cast<size_t>(i) < contacts.size(); ++i) {
    // Get the two body indices.
    const int body_a_index = contacts[i].elementA->get_body()->get_body_index();
    const int body_b_index = contacts[i].elementB->get_body()->get_body_index();

    // The reported point on A's surface (As) in the world frame (W).
    const Vector3<T> p_WAs =
        kcache.get_element(body_a_index).transform_to_world * contacts[i].ptA;

    // The reported point on B's surface (Bs) in the world frame (W).
    const Vector3<T> p_WBs =
        kcache.get_element(body_b_index).transform_to_world * contacts[i].ptB;

    // Get the point of contact in the world frame.
    const Vector3<T> p_W = (p_WAs + p_WBs) * 0.5;

    // The *relative* velocity of the contact point in A relative to that in
    // B.
    const auto v_W = CalcRelTranslationalVelocity(kcache, body_a_index,
                                                  body_b_index, p_W);

    // Compute an orthonormal basis.
    Vector3<T> tan1_dir, tan2_dir;
    CalcOrthonormalBasis(contacts[i].normal, &tan1_dir, &tan2_dir);

    // Set spanning tangent directions.
    basis_vecs.resize(half_cone_edges_);
    if (half_cone_edges_ == 2) {
      // Special case: pyramid friction.
      basis_vecs.front() = tan1_dir;
      basis_vecs.back() = tan2_dir;
    } else {
      for (int j = 0; j < half_cone_edges_; ++j) {
        double theta = M_PI * j / ((double) half_cone_edges_ - 1);
        basis_vecs[j] = tan1_dir * cos(theta) + tan2_dir * sin(theta);
      }
    }

    // Loop over the spanning tangent directions.
    for (int j = 0; j < static_cast<int>(basis_vecs.size()); ++j) {
      // Get the projected tangent velocity.
      result[k++] = v_W.dot(basis_vecs[j]);
    }
  }

  return result;
}

// Applies a force at the contact spanning directions at all contacts and gets
// the effect out on the generalized forces.
template <class T>
VectorX<T> TimeSteppingRigidBodyPlant<T>::F_transpose_mult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const KinematicsCache<T>& kcache,
    const VectorX<T>& f) const {
  std::vector<Vector3<T>> basis_vecs;

  // Create a result vector.
  VectorX<T> result = VectorX<T>::Zero(kcache.getV().size());

  // Loop through all contacts.
  for (int i = 0, k = 0; static_cast<size_t>(i) < contacts.size(); ++i) {
    // Get the two body indices.
    const int body_a_index = contacts[i].elementA->get_body()->get_body_index();
    const int body_b_index = contacts[i].elementB->get_body()->get_body_index();

    // The reported point on A's surface (As) in the world frame (W).
    const Vector3 <T> p_WAs =
        kcache.get_element(body_a_index).transform_to_world * contacts[i].ptA;

    // The reported point on B's surface (Bs) in the world frame (W).
    const Vector3 <T> p_WBs =
        kcache.get_element(body_b_index).transform_to_world * contacts[i].ptB;

    // Get the point of contact in the world frame.
    const Vector3 <T> p_W = (p_WAs + p_WBs) * 0.5;

    // Compute an orthonormal basis.
    Vector3 <T> tan1_dir, tan2_dir;
    CalcOrthonormalBasis(contacts[i].normal, &tan1_dir, &tan2_dir);

    // Set spanning tangent directions.
    basis_vecs.resize(half_cone_edges_);
    if (half_cone_edges_ == 2) {
      // Special case: pyramid friction.
      basis_vecs.front() = tan1_dir;
      basis_vecs.back() = tan2_dir;
    } else {
      for (int j = 0; j < half_cone_edges_; ++j) {
        double theta = M_PI * j / ((double) half_cone_edges_ - 1);
        basis_vecs[j] = tan1_dir * cos(theta) + tan2_dir * sin(theta);
      }
    }

    // Get the contribution to the generalized force from a force of the
    // specified normal applied at this point.
    for (int j = 0; j < static_cast<int>(basis_vecs.size()); ++j) {
      UpdateGeneralizedForce(kcache, body_a_index, body_b_index, p_W,
                             basis_vecs[j] * f[k++], &result);
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
  const auto& tree = this->get_rigid_body_tree();

  // Get the system state.
  auto x = context.get_discrete_state(0)->get_value();
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  auto kcache = tree.doKinematics(q, v);

  // Get the generalized inertia matrix and set up the inertia solve function.
  auto H = tree.massMatrix(kcache);

  // Compute the LDLT factorizations, which will be used by the solver.
  Eigen::LDLT<MatrixX<T>> ldlt(H);
  DRAKE_DEMAND(ldlt.info() == Eigen::Success);

  // Set the inertia matrix solver.
  data.solve_inertia = [this, &ldlt](const MatrixX<T>& m) {
    return ldlt.solve(m);
  };

  // There are no external wrenches, but it is a required argument in
  // dynamicsBiasTerm().
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;

  // right_hand_side is the right hand side of the system's equations:
  //   right_hand_side = B*u - C(q,v)
  VectorX<T> right_hand_side =
      -tree.dynamicsBiasTerm(kcache, no_external_wrenches);
  if (num_actuators > 0) right_hand_side += tree.B * u;

  // Determine the set of contact points corresponding to the current q.
  std::vector<drake::multibody::collision::PointPair> contacts =
      const_cast<RigidBodyTree<T>*>(&tree)->ComputeMaximumDepthCollisionPoints(
          kcache, true);

  // Verify the friction directions are set correctly.
  DRAKE_DEMAND(half_cone_edges_ >= 2);

  // Set the coefficients of friction.
  data.mu.resize(contacts.size());
  data.r.resize(contacts.size());
  for (int i = 0; i < data.mu.rows(); ++i) {
    data.mu[i] = mu_;
    data.r[i] = half_cone_edges_;
  }
  const int total_friction_cone_edges = std::accumulate(
      data.r.begin(), data.r.end(), 0);

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
        limits.back().error = (qjoint - qmin);
        limits.back().lower_limit = true;
      }
      if (qjoint > qmax || qjoint + vjoint * dt > qmax) {
        // Institute an upper limit.
        limits.push_back(JointLimit());
        limits.back().v_index = b->get_velocity_start_index();
        limits.back().error = (qmax - qjoint);
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

  // Set up the N' multiplication operator.
  data.N_transpose_mult = [this, &contacts, &q, &v](const VectorX<T>& f) ->
      VectorX<T> {
    return N_transpose_mult(contacts, q, v, f);
  };

  // Set up the F multiplication operator.
  data.F_mult = [this, &contacts, &q](const VectorX<T>& w) -> VectorX<T> {
    return F_mult(contacts, q, w);
  };

  // Set up the N' multiplication operator.
  data.F_transpose_mult = [this, &contacts, &q, &v](const VectorX<T>& f) ->
      VectorX<T> {
    return F_transpose_mult(contacts, q, v, f);
  };

  // Set the constraint Jacobian transpose operator.
  data.L_mult = [this, &limits](const VectorX<T>& w) -> VectorX<T> {
    VectorX<T> result(limits.size());
    for (int i = 0; static_cast<size_t>(i) < limits.size(); ++i) {
      const int index = limits[i].v_index;
      result[i] = (limits[i].lower_limit) ? w[index] : -w[index];
    }
    return result;
  };

  data.L_transpose_mult = [this, &v, &limits](const VectorX<T>& lambda) {
    VectorX<T> result = VectorX<T>::Zero(v.size());
    for (int i = 0; static_cast<size_t>(i) < limits.size(); ++i) {
      const int index = limits[i].v_index;
      result[index] = (limits[i].lower_limit) ? lambda[i] : -lambda[i];
    }
    return result;
  };

  // Set the stabilization terms.
  data.kN.resize(contacts.size());
  for (int i = 0; i < static_cast<int>(contacts.size()); ++i) {
    double stiffness, damping;
    CalcStiffnessAndDamping(contacts[i], &stiffness, &damping);
    const double denom = dt * stiffness + damping;
    double contact_cfm = 1.0 / denom;
    double contact_erp = dt * stiffness / denom;
    data.kN[i] = contact_erp * contacts[i].distance / dt;
  }
  data.kF.setZero(total_friction_cone_edges);
  data.kL.resize(limits.size());
  for (int i = 0; i < static_cast<int>(limits.size()); ++i)
    data.kL[i] = erp_ * limits[i].error / dt;

  // Integrate the forces into the velocity.
  data.v = v + data.solve_inertia(right_hand_side) * dt;

  // Solve the rigid impact problem.
  VectorX<T> vnew, cf;
  constraint_solver_.SolveImpactProblem(cfm_, data, &cf);
  constraint_solver_.ComputeGeneralizedVelocityChange(data, cf, &vnew);
  vnew += data.v;
std::cout << "N * vnew: " << data.N_mult(vnew).transpose() << std::endl;
std::cout << "F * vnew: " << data.F_mult(vnew).transpose() << std::endl;

  // qn = q + dt*qdot.
  VectorX<T> xn(this->get_num_states());
  xn << q + dt * tree.transformVelocityToQDot(kcache, vnew), vnew;
std::cout << "force: " << right_hand_side.transpose() << std::endl;
std::cout << "old velocity: " << v.transpose() << std::endl;
std::cout << "new velocity: " << vnew.transpose() << std::endl;
std::cout << "new configuration: " << (q + dt * tree.transformVelocityToQDot(kcache, vnew)).transpose() << std::endl;
  updates->get_mutable_vector(0)->SetFromVector(xn);
}

// Explicitly instantiates on the most common scalar types.
template class TimeSteppingRigidBodyPlant<double>;

}  // namespace systems
}  // namespace drake
