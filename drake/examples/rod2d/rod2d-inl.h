#pragma once

// @file
// Template method implementations for rod2d.h.
// Most users should only include that file, not this one.
// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/examples/rod2d/rod2d.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"

// TODO(edrumwri,sherm1) This code is currently written out mostly in scalars
// but should be done in 2D vectors instead to make it more compact, easier to
// understand, and easier to relate to our 3D code.
namespace drake {
namespace examples {
namespace rod2d {

template <typename T>
Rod2D<T>::Rod2D(SimulationType simulation_type, double dt) :
    simulation_type_(simulation_type), dt_(dt) {
  // Verify that the simulation approach is either piecewise DAE or
  // compliant ODE.
  if (simulation_type == SimulationType::kTimeStepping) {
    if (dt <= 0.0)
      throw std::logic_error("Time stepping approach must be constructed using"
                                 " strictly positive step size.");

    // Time stepping approach requires three position variables and
    // three velocity variables, all discrete, and periodic update.
    this->DeclareDiscreteUpdatePeriodSec(dt);
    this->DeclareDiscreteState(6);
  } else {
    if (dt != 0)
      throw std::logic_error("Piecewise DAE and compliant approaches must be "
                                 "constructed using zero step size.");

    // Both piecewise DAE and compliant approach require six continuous
    // variables.
    this->DeclareContinuousState(3, 3, 0);  // q, v, z
  }

  this->DeclareInputPort(systems::kVectorValued, 3);
  this->DeclareOutputPort(systems::kVectorValued, 6);
}

template <class T>
std::vector<RigidContact>& Rod2D<T>::get_contacts(
    systems::State<T>* state) const {
  return state->get_mutable_abstract_state()
      ->get_mutable_value(0)
      .template GetMutableValue<std::vector<RigidContact>>();
}

template <class T>
const std::vector<RigidContact>& Rod2D<T>::get_contacts(
    const systems::State<T>& state) const {
  return state.get_abstract_state()->get_value(0).
      template GetValue<std::vector<RigidContact>>();
}

// Models any impacts for the piecewise-DAE based system.
// @param[in,out] state the pre-impact state on entry, the post-impact state
//                on return.
// @param[out] Nvplus if non-null, contains the post-impact velocities projected
//             along the normal contact directions on return.
// @param[out] Fvplus if non-null, contains the post-impact velocities projected
//             along the tangential contact directions on return.
// @param[out] zero_tol if non-null, contains the determined zero tolerance
//             on return.
template <class T>
void Rod2D<T>::ModelImpact(systems::State<T>* state,
                           VectorX<T>* Nvplus,
                           VectorX<T>* Fvplus,
                           T* zero_tol) const {
  DRAKE_DEMAND(state);

  // Get state variables.
  const VectorX<T> q = state->get_continuous_state()->
      get_generalized_position().CopyToVector();
  systems::VectorBase<T>* qdot = state->get_mutable_continuous_state()->
      get_mutable_generalized_velocity();

  // Initialize the problem data.
  RigidContactVelProblemData<T> problem_data;

  // Get the points in contact.
  std::vector<RigidContact>& contacts = get_contacts(state);
  for (size_t i = 0; i < contacts.size(); ++i) {
    if (contacts[i].state != RigidContact::ContactState::kNotContacting)
      problem_data.active_contacts.push_back(i);
  }
  const int nc = problem_data.active_contacts.size();
  DRAKE_DEMAND(nc > 0);

  // Get the coefficients of friction for each point of contact.
  VectorX<T> mu(nc);
  for (size_t i = 0, j = 0; i < contacts.size(); ++i) {
    if (contacts[i].state == RigidContact::ContactState::kNotContacting)
      continue;
    mu(j++) = contacts[i].mu;
  }

  // Total number of friction directions = number of friction directions
  // per contact * number of contacts. Because this problem is two dimensional,
  // no polygonalization of a friction cone is necessary. However, the LCP
  // variables can only assume positive values, so the negation of the tangent
  // direction permits obtaining the same effect.
  const int nk = 2 * nc;

  // Problem matrices and vectors are mildly adapted from:
  // M. Anitescu and F. Potra. Formulating Dynamic Multi-Rigid Body Contact
  // Problems as Solvable Linear Complementarity Problems. Nonlinear Dynamics,
  // 14, 1997.

  // Construct the inverse generalized inertia matrix computed about the
  // center of mass of the rod and expressed in the world frame.
  Matrix3<T> iM;
  iM << 1.0/mass_, 0,         0,
      0,         1.0/mass_, 0,
      0,         0,         1.0/J_;

  // Form the generalized velocity vector.
  Vector3<T> v = qdot->CopyToVector();

  // Compute the Jacobian matrices.
  FormRigidContactVelJacobians(*state, &problem_data);

  // Create aliases to N and F.
  MatrixX<T>& N = problem_data.N;
  MatrixX<T>& F = problem_data.F;
  
  // Look for the opportunity to skip out on building the LCP matrix.
  VectorX<T> Nv = N * v;
  if (Nv.minCoeff() >= 0) {
    if (Nvplus)
      *Nvplus = Nv;
    if (Fvplus)
      *Fvplus = F * v;
    if (zero_tol)
      zero_tol = 0;

    return;
  }

  // Construct a matrix similar to E in Anitscu and Potra 1997. This matrix
  // will be used to specify the constraints 0 ≤ μ⋅fN - E⋅fF ⊥ λ ≥ 0 and
  // 0 ≤ e⋅λ + F⋅v ⊥ fF ≥ 0.
  MatrixX<T> E = MatrixX<T>::Zero(nk, nc);
  const int num_tangent_dirs = get_num_tangent_directions_per_contact();
  for (int i = 0, j = 0; i < nc; ++i) {
    E.col(i).segment(j, num_tangent_dirs).setOnes();
    j += num_tangent_dirs;
  }

  // Construct the LCP matrix. First do the "normal contact direction" rows.
  const int nvars = nk + 2*nc;
  MatrixX<T> MM;
  MM.resize(nvars, nvars);
  MM.block(0, 0, nc, nc) = N * iM * N.transpose();
  MM.block(0, nc, nc, nc) = N * iM * F.transpose();
  MM.block(0, nc*2, nc, nc) = -MM.block(0, nc, nc, nc);
  MM.block(0, nc+nk, nc, nc).setZero();

  // Now construct the un-negated tangent contact direction rows (everything
  // but last block column).
  MM.block(nc, 0, nc, nc) = F * iM * N.transpose();
  MM.block(nc, nc, nc, nc) = F * iM * F.transpose();
  MM.block(nc, nc*2, nc, nc) = -MM.block(nc, nc, nc, nc);

  // Now construct the negated tangent contact direction rows (everything but
  // last block column). These negated tangent contact directions allow the
  // LCP to compute forces applied along the negative x-axis.
  MM.block(nc*2, 0, nc, nc + nk) = -MM.block(nc, 0, nc, nc + nk);

  // Construct the last block column for the last set of rows (see Anitescu and
  // Potra, 1997).
  MM.block(nc, nc+nk, nk, nc) = E;

  // Construct the last two rows, which provide the friction "cone" constraint.
  MM.block(nc+nk, 0, nc, nc) = Eigen::DiagonalMatrix<T, Eigen::Dynamic>(mu);
  MM.block(nc+nk, nc, nc, nk) = -E.transpose();
  MM.block(nc+nk, nc+nk, nc, nc).setZero();

  // Construct the LCP vector.
  Eigen::Matrix<T, Eigen::Dynamic, 1> qq;
  qq.resize(nvars, 1);
  qq.segment(0, nc) = N * v;
  qq.segment(nc, nc) = F * v;
  qq.segment(2*nc, nc) = -qq.segment(nc, nc);
  qq.segment(nc + nk, nc).setZero();

  // Regularize the LCP matrix: this is essentially Tikhonov Regularization.
  // Cottle et al. show that any linear complementarity problem is solvable
  // for sufficiently large cfm.
  // R. Cottle, J.-S. Pang, and R. Stone. The Linear Complementarity Problem.
  // Academic Press, 1992.
  const double cfm = get_cfm();
  MM += MatrixX<T>::Identity(nvars, nvars) * cfm;

  // Solve the LCP.
  VectorX<T> zz;
  bool success;
  if (zero_tol) {
    *zero_tol = lcp_.ComputeZeroTolerance(MM);
    success = lcp_.SolveLcpLemke(MM, qq, &zz, -1, *zero_tol);
  } else {
    success = lcp_.SolveLcpLemke(MM, qq, &zz);
  }
  DRAKE_DEMAND(success);

  // Obtain the normal and frictional contact forces.
  VectorX<T> fN = zz.segment(0, nc);
  VectorX<T> fF_pos = zz.segment(nc, nc);
  VectorX<T> fF_neg = zz.segment(nc*2, nc);

  // Compute the new velocity and update it.
  VectorX<T> vplus = v + iM * (N.transpose()*fN +
      F.transpose()*(fF_pos - fF_neg));
  qdot->SetFromVector(vplus);

  // Compute normal and tangent velocities *only if desired*.
  if (Nvplus)
    *Nvplus = N * vplus;
  if (Fvplus)
    *Fvplus = F * vplus;
}

// Models any impacts for the piecewise-DAE based system and determines
// the active set at the velocity-level (meaning that a point of contact
// will be de-activated when contacting bodies are separating at that point
// and that sliding and non-sliding are determined "freshly", i.e., without
// any consideration of existing contact mode).
// @param Nvplus the contact velocity projected along the normal contact
//               directions.
// @param Nvplus the contact velocity projected along the tangent contact
//               directions.
// @param zero_tol the tolerance with which the linear complementarity problem/
//                 linear algebra problem was solved.
template <class T>
void Rod2D<T>::DetermineVelLevelActiveSet(systems::State<T>* state,
                                          const VectorX<T>& Nvplus,
                                          const VectorX<T>& Fvplus,
                                          const T& zero_tol) const {
  // Get the set of contacts.
  std::vector<RigidContact>& contacts = get_contacts(state);

  // Examine contacts.
  for (size_t i = 0, contact_index = 0; i < contacts.size(); ++i) {
    if (contacts[i].state == RigidContact::ContactState::kNotContacting)
      continue;

    // Contact will only be made inactive if it the bodies are separating at
    // that point.
    if (abs(Nvplus[contact_index]) > 10 * zero_tol) {
      contacts[i].state = RigidContact::ContactState::kNotContacting;
    } else {
      // It's conceivable that no impulsive force was applied but the contact
      // is still active. Either way, check to see whether the contact is
      // sliding or not-sliding.
      if (abs(Fvplus[contact_index]) > 10 * zero_tol)
        contacts[i].state = RigidContact::ContactState::kContactingAndSliding;
      else
        contacts[i].state =
            RigidContact::ContactState::kContactingWithoutSliding;
    }

    contact_index++;
  }
}

template <class T>
void Rod2D<T>::DoCalcUnrestrictedUpdate(const systems::Context<T>& context,
                                        systems::State<T>* state) const {
  // Get the potential contacts.
  std::vector<RigidContact>& contacts = get_contacts(state);

  // Get necessary state variables.
  const VectorX<T> q = context.get_continuous_state()->
      get_generalized_position().CopyToVector();
  const VectorX<T> v = context.get_continuous_state()->
      get_generalized_velocity().CopyToVector();
  const T& theta = q(2);
  const T& thetadot = v(2);

  // Form the rod frame to world frame rotation matrix and the center-of-mass
  // vector.
  const Eigen::Rotation2D<T> R(theta);
  const Vector2<T> x(q(0), q(1));
  const Vector2<T> xdot(v(0), v(1));
  const Matrix2<T> Rdot = get_rotation_matrix_derivative(context.get_state());

  // Check whether a new point of contact is becoming active.
  for (size_t i = 0; i < contacts.size(); ++i) {
    // If the contact is already active, ignore it.
    if (contacts[i].state != RigidContact::ContactState::kNotContacting)
      continue;

    // Contact is not active; see whether the distance between the two bodies
    // at that contact is zero (or less). If so, activate the contact.
    // (1) Transform the contact into the global frame.
    const Vector2<T> p = x + R * contacts[i].u.segment(0,2);

    // (2) Compute the signed distance between the contact and the half-space.
    const double sdist = p(1);

    // (3) If signed distance is not positive; make the contact active.
    // Arbitrarily select the sliding mode because this new contact must be
    // triggering an impact, from which the sliding velocity will be determined
    // anew.
    if (sdist <= 0) {
      contacts[i].state = RigidContact::ContactState::kContactingAndSliding;
      DRAKE_DEMAND(v(1) + Rdot.row(1).dot(contacts[i].u.segment(0,2)) * thetadot < 0);
    }
  }

  // Check whether the rod is undergoing an impact.
  if (IsImpacting(context)) {
    // Rod IS undergoing an impact. Model the impact problem (which also
    // redetermines contact modes).
    T zero_tol;
    VectorX<T> Nvplus, Fvplus;
    ModelImpact(state, &Nvplus, &Fvplus, &zero_tol);
    DetermineVelLevelActiveSet(state, Nvplus, Fvplus, zero_tol);
  }

  // The active set must now be redetermined using the derivative at the current
  // time. Only active contacts are processed, and whether the contact is
  // sliding or not is determined by that contact's corresponding abstract
  // state variable.
  DetermineAccelLevelActiveSet(context, state);
}

// Performs the operation A * Xᵀ, where some rows of X are zero.
// This method aims to reduce computation during FormSustainedContactLCP().
// @param indices the indices of the non-zero rows of X
template <class T>
MatrixX<T> Rod2D<T>::MultTranspose(const MatrixX<T>& A, const MatrixX<T>& X,
                                   const std::vector<int>& indices) const {
  DRAKE_ASSERT(A.cols() == X.cols());

  // NOTE: It is not clear that skipping the zero rows is actually that much
  // more efficient than Eigen multiplying the entire, semi-sparse matrix,
  // unless the number of columns of X is large.
  MatrixX<T> result = MatrixX<T>::Zero(A.rows(), X.rows());
  for (int i = 0; i < A.rows(); ++i) {
    for (size_t j = 0; j < indices.size(); ++j)
      result(i,indices[j]) = A.row(i).dot(X.row(indices[j]));
  }

  return result;
}

// Performs the operation A + X * diag(scale), where some rows of X are zero.
// This method aims to reduce computation during FormSustainedContactLCP().
// @param indices the indices of the non-zero rows of X
template <class T>
MatrixX<T> Rod2D<T>::AddScaledRightTerm(const MatrixX<T>& A,
                                        const VectorX<T>& scale,
                                        const MatrixX<T>& X,
                                        const std::vector<int>& indices) const {
  DRAKE_ASSERT(A.cols() == X.cols());
  MatrixX<T> result = A;
  for (size_t i = 0; i < indices.size(); ++i)
    result.row(indices[i]) += scale(i) * X.row(i);
  return result;
}

// Gets the indices of the vectors of sliding contacts and non-sliding contacts.
// The indices of active contacts can be determined by taking the union of
// these mutually exclusive sets.
// @note sliding_contacts and non_sliding_contacts are cleared immediately
//       after entry to this function.
template <class T>
void Rod2D<T>::GetContactVectors(const std::vector<RigidContact>& contacts,
                                 std::vector<int>* sliding_contacts,
                                 std::vector<int>* non_sliding_contacts) {
  sliding_contacts->clear();
  non_sliding_contacts->clear();

  for (size_t i = 0, contact_index = 0; i < contacts.size(); ++i) {
    if (contacts[i].state ==
        RigidContact::ContactState::kContactingAndSliding) {
      sliding_contacts->push_back(contact_index++);
    } else {
      if (contacts[i].state ==
          RigidContact::ContactState::kContactingWithoutSliding)
        non_sliding_contacts->push_back(contact_index++);
    }
  }
}

// Computes problem data necessary to form the sustained contact linear
// complementarity problem or the sustained contact linear system.
template <class T>
void Rod2D<T>::InitRigidContactAccelProblemData(
    const systems::Context<T>& context,
    RigidContactAccelProblemData<T>* problem_data) const {
  DRAKE_DEMAND(problem_data);

  // Get the sets of sliding and non-sliding contacts.
  const std::vector<RigidContact>& contacts = get_contacts(context.get_state());
  GetContactVectors(contacts, &problem_data->sliding_contacts,
                    &problem_data->non_sliding_contacts);

  // Get numbers of types of contacts.
  const int num_sliding = problem_data->sliding_contacts.size();
  const int num_non_sliding = problem_data->non_sliding_contacts.size();

  // Get the coefficients of friction for each contact.
  problem_data->mu_sliding.resize(num_sliding);
  problem_data->mu_non_sliding.resize(num_non_sliding);
  for (size_t i = 0, sliding_index, non_sliding_index = 0;
       i < contacts.size(); ++i) {
    if (contacts[i].state ==
        RigidContact::ContactState::kContactingWithoutSliding) {
      problem_data->mu_non_sliding(non_sliding_index) = contacts[i].mu;
      ++non_sliding_index;
    } else {
      if (contacts[i].state ==
          RigidContact::ContactState::kContactingAndSliding) {
        problem_data->mu_sliding(non_sliding_index) = contacts[i].mu;
        ++sliding_index;
      }
    }
  }
}

// Computes the external forces on the rod
template <class T>
Vector3<T> Rod2D<T>::ComputeExternalForces(
    const systems::Context<T>& context) const {
  // Compute the external forces (expressed in the world frame).
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);
  const Vector3<T> fgrav(0, mass_ * get_gravitational_acceleration(), 0);
  const Vector3<T> fapplied = input.segment(0, 3);
  return fgrav + fapplied;
}

// Computes the Jacobian matrices used for rigid contact constraints at the
// velocity level.
template <class T>
void Rod2D<T>::FormRigidContactVelJacobians(
    const systems::State<T>& state,
    RigidContactVelProblemData<T>* problem_data) const {
  // Get state variables.
  const VectorX<T> q = state.get_continuous_state()->
      get_generalized_position().CopyToVector();
  const VectorX<T> v = state.get_continuous_state()->
      get_generalized_velocity().CopyToVector();

  // Three generalized velocity variables.
  const int ngv = 3;

  // Get total contacts.
  const std::vector<RigidContact>& contacts = get_contacts(state);

  // Get numbers of active contacts.
  const int nc = problem_data->active_contacts.size();

  // Get the transformation of vectors from the rod frame to the
  // world frame.
  const T& theta = q[2];
  Eigen::Rotation2D<T> R(theta);

  // Set the origin of the center of mass (in the world frame) and its velocity.
  const Vector2<T> x(q[0], q[1]);
  const Vector2<T> xdot(v[0], v[1]);

  // Set up the contact normal and tangent (friction) direction Jacobian
  // matrices and their time derivatives. These take the form:
  //     | 0 1 n1 |        | 1 0 f1 |       | 1 0 f1 |
  // N = | 0 1 n2 |    F = | 1 0 f2 |   Q = | 1 0 f2 |
  // where n1, n2/f1, f2 are the moment arm induced by applying the
  // force at the given contact point along the normal/tangent direction.
  // F is for non-sliding contacts and Q is for sliding contacts.
  problem_data->N.resize(nc, ngv);
  problem_data->F.resize(nc, ngv);
  for (size_t i = 0, j = 0; i < contacts.size(); ++i) {
    if (contacts[i].state == RigidContact::ContactState::kNotContacting)
      continue;

    // Transform the contact point to the world frame.
    const Vector2<T> p = x + R * contacts[i].u.segment(0,2);

    // Horizontal component of normal Jacobian is always zero and vertical
    // component is always one.
    problem_data->N(j, 0) = 0;
    problem_data->N(j, 1) = 1;
    problem_data->N(j, 2) = p[0] - q[0];

    problem_data->F(j, 0) = 1;
    problem_data->F(j, 1) = 0;
    problem_data->F(j, 2) = -(p[1] - q[1]);

    // Update the normal index.
    j++;
  }
}

// Computes the Jacobian matrices used for rigid contact constraints at the
// acceleration level
template <class T>
void Rod2D<T>::FormRigidContactAccelJacobians(
    const systems::State<T>& state,
    RigidContactAccelProblemData<T>* problem_data) const {
  // Get state variables.
  const VectorX<T> q = state.get_continuous_state()->
      get_generalized_position().CopyToVector();
  const VectorX<T> v = state.get_continuous_state()->
      get_generalized_velocity().CopyToVector();

  // Three generalized velocity variables.
  const int ngv = 3;

  // Get total contacts.
  const std::vector<RigidContact>& contacts = get_contacts(state);

  // Get numbers of types of contacts.
  const int num_sliding = problem_data->sliding_contacts.size();
  const int num_non_sliding = problem_data->non_sliding_contacts.size();
  const int nc = num_sliding + num_non_sliding;

  // Get the transformation of vectors from the rod frame to the
  // world frame.
  const T& theta = q[2];
  Eigen::Rotation2D<T> R(theta);

  // Set the origin of the center of mass (in the world frame) and its velocity.
  const Vector2<T> x(q[0], q[1]);
  const Vector2<T> xdot(v[0], v[1]);

  // Get the time derivative of the rotation matrix.
  const Matrix2<T> Rdot = get_rotation_matrix_derivative(state);
  const T& thetadot = v[2];

  // Set up the contact normal and tangent (friction) direction Jacobian
  // matrices and their time derivatives. These take the form:
  //     | 0 1 n1 |        | 1 0 f1 |       | 1 0 f1 |
  // N = | 0 1 n2 |    F = | 1 0 f2 |   Q = | 1 0 f2 |
  // where n1, n2/f1, f2 are the moment arm induced by applying the
  // force at the given contact point along the normal/tangent direction.
  // F is for non-sliding contacts and Q is for sliding contacts.
  problem_data->N.resize(nc, ngv);
  problem_data->F.resize(num_non_sliding, ngv);
  problem_data->Q.resize(num_sliding, ngv);
  problem_data->Ndot.resize(nc, ngv);
  problem_data->Fdot.resize(num_non_sliding, ngv);
  for (size_t i = 0, j = 0, r = 0, s = 0; i < contacts.size(); ++i) {
    if (contacts[i].state == RigidContact::ContactState::kNotContacting)
      continue;

    // Transform the contact point to the world frame.
    const Vector2<T> p = x + R * contacts[i].u.segment(0,2);

    // Get the velocity at the contact point.
    const Vector2<T> pdot = xdot + Rdot * contacts[i].u.segment(0,2) * thetadot;

    // Horizontal component of normal Jacobian is always zero and vertical
    // component is always one.
    problem_data->N(j, 0) = 0;
    problem_data->N(j, 1) = 1;
    problem_data->N(j, 2) = p[0] - q[0];

    // Time derivative of normal Jacobian is even simpler.
    problem_data->Ndot(j, 0) = 0;
    problem_data->Ndot(j, 1) = 0;
    problem_data->Ndot(j, 2) = pdot[0] - v[0];

    if (contacts[i].state ==
        RigidContact::ContactState::kContactingWithoutSliding) {

      problem_data->F(r, 0) = 1;
      problem_data->F(r, 1) = 0;
      problem_data->F(r, 2) = -(p[1] - q[1]);

      // Time derivative of tangent Jacobian is also simple.
      problem_data->Fdot(r, 0) = 0;
      problem_data->Fdot(r, 1) = 0;
      problem_data->Fdot(r, 2) = -(pdot[1] - v[1]);
      // Update r.
      ++r;
    } else {
      // Tangent direction for a sliding contact points along the direction of
      // sliding.
      problem_data->Q(s, 0) = (pdot[0] > 0) ? 1 : -1;
      problem_data->Q(s, 1) = 0;
      problem_data->Q(s, 2) = -problem_data->Q(s,0)*(p[1] - q[1]);

      // Update s.
      ++s;
    }

    // Update the normal index.
    j++;
  }
}

// Forms the system of linear equations used to compute the accelerations during
// ODE evaluations.
template <class T>
void Rod2D<T>::FormSustainedContactLinearSystem(
    const systems::Context<T>& context,
    const RigidContactAccelProblemData<T>& problem_data,
    MatrixX<T>* MM, VectorX<T>* qq) const {
  using std::abs;

  DRAKE_DEMAND(MM);
  DRAKE_DEMAND(qq);

  // Get numbers of types of contacts.
  const int num_sliding = problem_data.sliding_contacts.size();
  const int num_non_sliding = problem_data.non_sliding_contacts.size();
  const int nc = num_sliding + num_non_sliding;

  // Total number of friction directions = number of friction directions
  // per contact * number of contacts. Because this problem is two dimensional,
  // no polygonalization of a friction cone is necessary. However, the LCP
  // variables can only assume positive values, so the negation of the tangent
  // direction permits obtaining the same effect.
  const int nk = get_num_tangent_directions_per_contact() * num_non_sliding;
  const int half_nk = nk / 2;

  // Problem matrices and vectors are mildly adapted from:
  // M. Anitescu and F. Potra. Formulating Dynamic Multi-Rigid Body Contact
  // Problems as Solvable Linear Complementarity Problems. Nonlinear Dynamics,
  // 14, 1997.

  // Gets the external force on the system.
  const Vector3<T> fext = ComputeExternalForces(context);

  // Construct the inverse generalized inertia matrix computed about the
  // center of mass of the rod and expressed in the world frame.
  Matrix3<T> iM;
  iM << 1.0/mass_, 0,         0,
      0,         1.0/mass_, 0,
      0,         0,         1.0/J_;

  // Alias matrices / vectors to make accessing them more wieldy.
  const MatrixX<T>& N = problem_data.N;
  const MatrixX<T>& Ndot = problem_data.Ndot;
  const MatrixX<T>& F = problem_data.F;
  const MatrixX<T>& Fdot = problem_data.Fdot;

  // Construct the matrix:
  // N⋅M⁻¹⋅(Nᵀ - μQᵀ)  N⋅M⁻¹⋅Fᵀ
  // F⋅M⁻¹⋅Nᵀ          F⋅M⁻¹⋅Dᵀ
  const int nvars = nc + nk;
  MM->resize(nvars, nvars);
  MM->block(0, 0, nc, nc) = N * iM * problem_data.N_minus_mu_Q.transpose();
  MM->block(0, nc, nc, half_nk) = N * (problem_data.iM_x_FT);

  // Now construct the tangent contact direction rows for non-sliding contacts.
  MM->block(nc, 0, half_nk, nc) = F * iM * N.transpose();
  MM->block(nc, nc, half_nk, half_nk) = F * iM * F.transpose();

  // Construct the vector:
  // N⋅M⁻¹⋅fext + dN/dt⋅v
  // F⋅M⁻¹⋅fext + dF/dt⋅v
  const VectorX<T> v = context.get_continuous_state()->
      get_generalized_velocity().CopyToVector();
  qq->resize(nvars, 1);
  qq->segment(0, nc) = N * iM * fext + Ndot * v;
  qq->segment(nc, half_nk) = F * iM * fext + Fdot * v;
}

// Forms the LCP matrix and vector, which is used to determine the active
// set of constraints at the acceleration-level.
template <class T>
void Rod2D<T>::FormSustainedContactLCP(const systems::Context<T>& context,
    const RigidContactAccelProblemData<T>& problem_data,
    MatrixX<T>* MM, Eigen::Matrix<T, Eigen::Dynamic, 1>* qq) const {
  using std::abs;

  DRAKE_DEMAND(MM);
  DRAKE_DEMAND(qq);

  // Get numbers of types of contacts.
  const int num_sliding = problem_data.sliding_contacts.size();
  const int num_non_sliding = problem_data.non_sliding_contacts.size();
  const int nc = num_sliding + num_non_sliding;

  // Total number of friction directions = number of friction directions
  // per contact * number of contacts. Because this problem is two dimensional,
  // no polygonalization of a friction cone is necessary. However, the LCP
  // variables can only assume positive values, so the negation of the tangent
  // direction permits obtaining the same effect.
  const int nk = get_num_tangent_directions_per_contact() * num_non_sliding;
  const int half_nk = nk / 2;

  // Problem matrices and vectors are mildly adapted from:
  // M. Anitescu and F. Potra. Formulating Dynamic Multi-Rigid Body Contact
  // Problems as Solvable Linear Complementarity Problems. Nonlinear Dynamics,
  // 14, 1997.

  // Gets the external force on the system.
  const Vector3<T> fext = ComputeExternalForces(context);

  // Construct the inverse generalized inertia matrix computed about the
  // center of mass of the rod and expressed in the world frame.
  Matrix3<T> iM;
  iM << 1.0/mass_, 0,         0,
      0,         1.0/mass_, 0,
      0,         0,         1.0/J_;

  // Alias matrices / vectors to make accessing them more wieldy.
  const MatrixX<T>& N = problem_data.N;
  const MatrixX<T>& Ndot = problem_data.Ndot;
  const MatrixX<T>& F = problem_data.F;
  const MatrixX<T>& Fdot = problem_data.Fdot;
  const VectorX<T>& mu_non_sliding = problem_data.mu_non_sliding;

  // Construct a matrix similar to E in Anitscu and Potra 1997. This matrix
  // will be used to specify the constraints 0 ≤ μ⋅fN - E⋅fF ⊥ λ ≥ 0 and
  // 0 ≤ e⋅λ + F⋅dv/dt ⊥ fF ≥ 0.
  MatrixX<T> E = MatrixX<T>::Zero(nk, num_non_sliding);
  const int num_tangent_dirs = get_num_tangent_directions_per_contact();
  for (int i = 0, j = 0; i < num_non_sliding; ++i) {
    E.col(i).segment(j, num_tangent_dirs).setOnes();
    j += num_tangent_dirs;
  }

  // Construct the LCP matrix. First do the "normal contact direction" rows:
  // N⋅M⁻¹⋅(Nᵀ - μQᵀ)  N⋅M⁻¹⋅Dᵀ  0
  // D⋅M⁻¹⋅Nᵀ          D⋅M⁻¹⋅Dᵀ  E
  // μ                 -Eᵀ       0
  // where D = [F -F]
  const int nvars = nc + nk + num_non_sliding;
  MM->resize(nvars, nvars);
  MM->block(0, 0, nc, nc) = N * iM * problem_data.N_minus_mu_Q.transpose();
  MM->block(0, nc, nc, half_nk) = N * (problem_data.iM_x_FT);
  MM->block(0, nc + half_nk, nc, half_nk) = -MM->block(0, nc, nc, half_nk);
  MM->block(0, nc + nk, num_non_sliding, num_non_sliding).setZero();

  // Now construct the un-negated tangent contact direction rows (everything
  // but last block column).
  MM->block(nc, 0, half_nk, nc) = F * iM * N.transpose();
  MM->block(nc, nc, half_nk, half_nk) = F * iM * F.transpose();
  MM->block(nc, nc + half_nk, half_nk, half_nk) =
       -MM->block(nc, nc, half_nk, half_nk);

  // Now construct the negated tangent contact direction rows (everything but
  // last block column). These negated tangent contact directions allow the
  // LCP to compute forces applied along the negative x-axis.
  MM->block(nc + half_nk, 0, half_nk, nc + nk) =
       -MM->block(nc, 0, half_nk, nc + nk);

  // Construct the last block column for the last set of rows (see Anitescu and
  // Potra, 1997).
  MM->block(nc, nc + nk, nk, num_non_sliding) = E;

  // Construct the last two rows, which provide the friction "cone" constraint.
  MM->block(nc + nk, 0, num_non_sliding, num_non_sliding) =
      Eigen::DiagonalMatrix<T, Eigen::Dynamic>(mu_non_sliding);
  MM->block(nc + nk, nc, num_non_sliding, nk) = -E.transpose();
  MM->block(nc + nk, nc+nk, num_non_sliding, num_non_sliding).setZero();

  // Construct the LCP vector:
  // N⋅M⁻¹⋅fext + dN/dt⋅v
  // D⋅M⁻¹⋅fext + dD/dt⋅v
  // 0
  // where, as above, D is defined as [F -F]
  const VectorX<T> v = context.get_continuous_state()->
      get_generalized_velocity().CopyToVector();
  qq->resize(nvars, 1);
  qq->segment(0, nc) = N * iM * fext + Ndot * v;
  qq->segment(nc, half_nk) = F * iM * fext + Fdot * v;
  qq->segment(nc + half_nk, half_nk) = -qq->segment(nc, half_nk);
  qq->segment(nc + nk, num_non_sliding).setZero();
}

// Determines the active set at the acceleration level. Namely, this method
// determines when bodies may be accelerating away at a point of contact
// or when a contact is transitioning from sticking to sliding.
template <class T>
void Rod2D<T>::DetermineAccelLevelActiveSet(
    const systems::Context<T>& context, systems::State<T>* state) const {

  // Get the mutable contacts.
  std::vector<RigidContact>& contacts = get_contacts(state);

  // Populate problem data.
  RigidContactAccelProblemData<T> problem_data;
  InitRigidContactAccelProblemData(context, &problem_data);
  const std::vector<int>& sliding_contacts = problem_data.sliding_contacts;
  const std::vector<int>& non_sliding_contacts =
      problem_data.non_sliding_contacts;

  // Get numbers of friction directions and types of contacts.
  const int num_sliding = sliding_contacts.size();
  const int num_non_sliding = non_sliding_contacts.size();
  const int nc = num_sliding + num_non_sliding;
  const int nk = get_num_tangent_directions_per_contact() * num_non_sliding;

  // Compute Jacobian matrices.
  FormRigidContactAccelJacobians(*state, &problem_data);

  // Alias vectors and Jacobian matrices.
  const MatrixX<T>& N = problem_data.N;
  const MatrixX<T>& Q = problem_data.Q;
  const MatrixX<T>& F = problem_data.F;
  const VectorX<T>& mu_sliding = problem_data.mu_sliding;

  // Construct the inverse generalized inertia matrix computed about the
  // center of mass of the rod and expressed in the world frame.
  Matrix3<T> iM;
  iM << 1.0/mass_, 0,         0,
      0,         1.0/mass_, 0,
      0,         0,         1.0/J_;

  // Precompute using indices.
  problem_data.N_minus_mu_Q = AddScaledRightTerm(N, -mu_sliding, Q,
                              sliding_contacts);
  problem_data.iM_x_FT = MultTranspose(iM, F, non_sliding_contacts);

  // Form the linear complementarity problem.
  MatrixX<T> MM;
  Eigen::Matrix<T, Eigen::Dynamic, 1> qq;
  FormSustainedContactLCP(context, problem_data, &MM, &qq);

  // Regularize the LCP matrix: this is essentially Tikhonov Regularization.
  // Cottle et al. show that any linear complementarity problem is solvable
  // for sufficiently large cfm.
  // R. Cottle, J.-S. Pang, and R. Stone. The Linear Complementarity Problem.
  // Academic Press, 1992.
  const double cfm = get_cfm();
  MM += MatrixX<T>::Identity(MM.rows(), MM.rows()) * cfm;

  // Get the zero tolerance for solving the LCP.
  const T zero_tol = lcp_.ComputeZeroTolerance(MM);

  // Solve the LCP and compute the values of the slack variables.
  VectorX<T> zz;
  bool success = lcp_.SolveLcpLemke(MM, qq, &zz, -1, zero_tol);
  VectorX<T> ww = MM * zz + qq;

  // NOTE: This LCP might not be solvable due to inconsistent configurations.
  // Check the answer and throw a runtime error if it's no good.
  if (!success || zz.minCoeff() < -10*zero_tol ||
      ww.minCoeff() < -10*zero_tol || abs(zz.dot(ww)) > 10*zero_tol)
    throw std::runtime_error("Unable to solve LCP- it may be unsolvable.");

  // Obtain the normal and frictional contact forces and the value of λ, which
  // will yield the residual tangential accelerations unable to be eliminated
  // for the non-sliding contacts. If λ = 0, then all contacts will remain
  // non-sliding.
  const auto fN = zz.segment(0, nc);
  const auto lambda = zz.segment(nc+nk, num_non_sliding);

  // Now determine what constraints should be active during the interval.
  // Two possibilities exist for checking whether the contact should become
  // inactive over the interval. The normal force may be zero, though this check
  // must account for the rounding error that accumulates during the pivoting
  // process. A safer check will examine the magnitudes of both the slack
  // variable (which should be non-negative if the constraint is inactive) and
  // the normal force; however, we must consider the case where both the
  // normal force and the slack variable are zero. As an example, assume that
  // 1e-8 is the magnitude of the normal force and 1e-12 is the value of the
  // slack variable. Should this contact be considered to be active or inactive?
  // What about 2e-12 and 1e-12, respectively? These examples will lead us to
  // consider a contact to be active if the signed normal force is greater than
  // one order of magnitude larger than the slack variable magnitude.
  for (size_t i = 0, contact_index = 0, non_sliding_index = 0;
       i < contacts.size(); ++i) {
    if (contacts[i].state == RigidContact::ContactState::kNotContacting)
      continue;

    // Contact is currently active; see whether to make it inactive.
    if (fN[contact_index] < abs(ww[contact_index])) {
      // If the contact is currently not sliding, must update the lambda index.
      if (contacts[i].state ==
          RigidContact::ContactState::kContactingWithoutSliding)
        ++non_sliding_index;

      contacts[i].state = RigidContact::ContactState::kNotContacting;
    } else {
      // Contact cannot transition from sliding to not-sliding here. That must
      // happen in the DetermineVelLevelActiveSet() method. However, contact
      // can transition from not-sliding to sliding if the appropriate
      // slack variable is strictly positive.
      if (contacts[i].state ==
          RigidContact::ContactState::kContactingWithoutSliding) {
        if (lambda[non_sliding_index] > zero_tol)
          contacts[i].state = RigidContact::ContactState::kContactingAndSliding;
        ++non_sliding_index;
      }
    }

    // Update contact index.
    ++contact_index;
  }
}

/*
template <class T>
T Rod2D<T>::CalcSignedDistance(const systems::Context<T>& context) const {
  using std::sin;
  using std::cos;
  using std::min;

  // Verify the system is simulated using piecewise DAE.
  DRAKE_DEMAND(get_simulation_type() ==
      Rod2D<T>::SimulationType::kPiecewiseDAE);

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);

  // Get the two rod endpoints.
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  int k1 = 1;
  int k2 = -1;
  const Vector2<T> ep1 = CalcRodEndpoint(x, y, k1, ctheta, stheta,
                                         get_rod_half_length());
  const Vector2<T> ep2 = CalcRodEndpoint(x, y, k2, ctheta, stheta,
                                         get_rod_half_length());

  return min(ep1[1], ep2[1]);
}

template <class T>
T Rod2D<T>::CalcEndpointDistance(const systems::Context<T>& context) const {
  using std::sin;

  // Verify the system is simulated using piecewise DAE.
  DRAKE_DEMAND(get_simulation_type() ==
      Rod2D<T>::SimulationType::kPiecewiseDAE);

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T stheta = sin(theta);

  // Get the abstract variables that determine the current system mode and
  // the endpoint in contact.
  const Mode mode = context.template get_abstract_state<Mode>(0);
  DRAKE_DEMAND(mode == Mode::kSlidingSingleContact ||
               mode == Mode::kStickingSingleContact);
  const int k = get_k(context);

  // Get the vertical position of the other rod endpoint.
  const int k2 = -k;
  return y + k2 * stheta * get_rod_half_length();
}

template <class T>
T Rod2D<T>::CalcNormalAccelWithoutContactForces(const systems::Context<T>&
                                                     context) const {
  DRAKE_ASSERT_VOID(this->CheckValidContext(context));
  using std::sin;
  using std::cos;

  // Verify the system is simulated using piecewise DAE.
  DRAKE_DEMAND(get_simulation_type() ==
               Rod2D<T>::SimulationType::kPiecewiseDAE);

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& theta = state.GetAtIndex(2);
  const T& thetadot = state.GetAtIndex(5);

  // Get the abstract variables that determine the current system mode and
  // the endpoint in contact.
  const Mode mode = context.template get_abstract_state<Mode>(0);
  DRAKE_DEMAND(mode != Mode::kBallisticMotion);
  const int k = get_k(context);
  const T stheta = sin(theta);
  const T ctheta = cos(theta);

  // Get the external force.
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);
  const Vector3<T> fapplied = input.segment(0, 3);
  const T& fY = fapplied(1);
  const T& tau = fapplied(2);

  // Compute the normal acceleration at the point of contact (cyddot),
  // *assuming zero contact force*. This equation comes from the kinematics of
  // the rod:
  // cy = y + k * half_rod_length * sin(θ)  [rod endpoint vertical location]
  // dcy/dt = dy/dt + k * half_rod_length * cos(θ) * dθ/dt
  // d²cy/dt² = d²y/dt² - k * half_rod_length * sin(θ) * (dθ/dt)² +
  //            k * half_rod_length * cos(θ) * d²θ/dt²
  const T yddot = get_gravitational_acceleration() + fY/get_rod_mass();
  const T thetaddot = tau/get_rod_moment_of_inertia();
  T cyddot = yddot -
      k * half_length_ * stheta * thetadot * thetadot +
      k * half_length_ * ctheta * thetaddot;

  return cyddot;
}

template <class T>
T Rod2D<T>::CalcSlidingDot(const systems::Context<T>& context) const {
  using std::sin;

  // Verify the system is simulated using piecewise DAE.
  DRAKE_DEMAND(get_simulation_type() ==
      Rod2D<T>::SimulationType::kPiecewiseDAE);

  // Verify rod is undergoing sliding contact.
  const Mode mode = context.template get_abstract_state<Mode>(0);
  DRAKE_DEMAND(mode == Mode::kSlidingSingleContact ||
               mode == Mode::kSlidingTwoContacts);

  // Get the point of contact.
  const int k = get_k(context);

  // Get the relevant parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);
  const T& thetadot = state.GetAtIndex(5);

  // Compute the velocity at the point of contact
  const T stheta = sin(theta);
  const T half_rod_length = get_rod_half_length();
  const T xcdot = xdot - k * stheta * half_rod_length * thetadot;
  return xcdot;
}

template <class T>
T Rod2D<T>::CalcStickingFrictionForceSlack(const systems::Context<T>& context)
                                               const {
  using std::abs;

  // Compute the contact forces, assuming sticking contact.
  const Vector2<T> cf = CalcStickingContactForces(context);
  const T &fN = cf(0);
  const T &fF = cf(1);

  // Compute the difference between how much force *can* be applied to effect
  // sticking and how much force needs to be applied to effect sticking.
  const double mu = get_mu_coulomb();
  return mu * fN - abs(fF);
}
*/

/// Gets the integer variable 'k' used to determine the point of contact
/// indicated by the current mode.
/// @throws std::logic_error if this is a time-stepping system (implying that
///         modes are unused) or modes indicate no point is in contact.
/// @returns the value -1 to indicate the bottom of the rod (when theta = pi/2),
///          +1 to indicate the top of the rod (when theta = pi/2), or 0 to
///          indicate the mode where both endpoints of the rod are contacting
///          the halfspace.
template <class T>
int Rod2D<T>::get_k(const systems::Context<T>& context) const {
  if (simulation_type_ != SimulationType::kPiecewiseDAE)
    throw std::logic_error("'k' is only valid for piecewise DAE approach.");

  const std::vector<RigidContact>& contacts = get_contacts(context.get_state());
  DRAKE_DEMAND(contacts.size() == 2);
  if (contacts.front().state != RigidContact::ContactState::kNotContacting) {
    if (contacts.back().state != RigidContact::ContactState::kNotContacting) {
      return 0;
    } else {
      return -1;
    }
  } else {
      if (contacts.back().state != RigidContact::ContactState::kNotContacting)
        return 1;
  }

  throw std::logic_error("No modes are active.");
}

template <class T>
Vector2<T> Rod2D<T>::CalcRodEndpoint(const T& x, const T& y, const int k,
                                     const T& ctheta, const T& stheta,
                                     const double half_rod_len) {
  const T cx = x + k * ctheta * half_rod_len;
  const T cy = y + k * stheta * half_rod_len;
  return Vector2<T>(cx, cy);
}

template <class T>
Vector2<T> Rod2D<T>::CalcCoincidentRodPointVelocity(
    const Vector2<T>& p_WRo, const Vector2<T>& v_WRo,
    const T& w_WR,  // aka thetadot
    const Vector2<T>& p_WC) {
  const Vector2<T> p_RoC_W = p_WC - p_WRo;  // Vector from R origin to C, in W.
  const Vector2<T> v_WRc = v_WRo + w_cross_r(w_WR, p_RoC_W);
  return v_WRc;
}

template <class T>
int Rod2D<T>::DetermineNumWitnessFunctions(const systems::Context<T>&
context) const {
  return 0;
}

template <typename T>
void Rod2D<T>::DoCalcOutput(const systems::Context<T>& context,
                               systems::SystemOutput<T>* output) const {
  // Obtain the structure we need to write into.
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

  // Output port value is just the continuous state.
  output_vector->get_mutable_value() =
      context.get_continuous_state()->CopyToVector();
}

/// Integrates the Rod 2D example forward in time using a
/// half-explicit time stepping scheme.
template <class T>
void Rod2D<T>::DoCalcDiscreteVariableUpdates(
                           const systems::Context<T>& context,
                           systems::DiscreteState<T>* discrete_state) const {
  // Set ERP (error reduction parameter) and CFM (constraint force mixing)
  // to make this problem "mostly rigid" and with rapid stabilization. These
  // parameters are described in the Open Dynamics Engine user manual (see
  // http://ode.org/ode-latest-userguide.html#sec_3_8 titled "Soft constraint
  // and constraint force mixing (CFM)") as well as in a presentation by
  // Erin Catto at the 2011 Game Developers Conference (Soft Constraints:
  // Reinventing the Spring,
  // http://box2d.org/files/GDC2011/GDC2011_Catto_Erin_Soft_Constraints.pdf).
  const double erp = get_erp();
  const double cfm = get_cfm();

  // Get the necessary state variables.
  const systems::BasicVector<T>& state = *context.get_discrete_state(0);
  const auto& q = state.get_value().template segment<3>(0);
  Vector3<T> v = state.get_value().template segment<3>(3);
  const T& x = q(0);
  const T& y = q(1);
  const T& theta = q(2);

  // Get the inputs.
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);

  // Compute the two rod vertical endpoint locations.
  const T stheta = sin(theta), ctheta = cos(theta);

  // Three generalized velocity variables.
  const int ngv = 3;

  // Two contact points, corresponding to the two rod endpoints, are always
  // used, regardless of whether any part of the rod is in contact with the
  // halfspace. This practice is standard in time stepping approaches with
  // constraint stabilization. See:
  // M. Anitescu and G. Hart. A Constraint-Stabilized Time-Stepping Approach
  // for Rigid Multibody Dynamics with Joints, Contact, and Friction. Intl.
  // J. for Numerical Methods in Engr., 60(14), 2004.
  const int nc = 2;

  // Find left and right end point locations.
  const Vector2<T> left =
      CalcRodEndpoint(x, y, -1, ctheta, stheta, half_length_);
  const Vector2<T> right =
      CalcRodEndpoint(x, y,  1, ctheta, stheta, half_length_);

  // Total number of friction directions = number of friction directions
  // per contact * number of contacts. Because this problem is two dimensional,
  // no polygonalization of a friction cone is necessary. However, the LCP
  // variables can only assume positive values, so the negation of the tangent
  // direction permits obtaining the same effect.
  const int nk = 2 * nc;

  // Problem matrices and vectors are mildly adapted from:
  // M. Anitescu and F. Potra. Formulating Dynamic Multi-Rigid Body Contact
  // Problems as Solvable Linear Complementarity Problems. Nonlinear Dynamics,
  // 14, 1997.

  // Construct the inverse generalized inertia matrix computed about the
  // center of mass of the rod and expressed in the world frame.
  Matrix3<T> iM;
  iM << 1.0/mass_, 0,         0,
        0,         1.0/mass_, 0,
        0,         0,         1.0/J_;

  // Update the generalized velocity vector with discretized external forces
  // (expressed in the world frame).
  const Vector3<T> fgrav(0, mass_ * get_gravitational_acceleration(), 0);
  const Vector3<T> fapplied = input.segment(0, 3);
  v += dt_ * iM * (fgrav + fapplied);

  // Set up the contact normal and tangent (friction) direction Jacobian
  // matrices. These take the form:
  //     | 0 1 n1 |        | 1 0 f1 |
  // N = | 0 1 n2 |    F = | 1 0 f2 |
  // where n1, n2/f1, f2 are the moment arm induced by applying the
  // force at the given contact point along the normal/tangent direction.
  Eigen::Matrix<T, nc, ngv> N, F;
  N(0, 0) = N(1, 0) = 0;
  N(0, 1) = N(1, 1) = 1;
  N(0, 2) = (left[0]  - x);
  N(1, 2) = (right[0] - x);
  F(0, 0) = F(1, 0) = 1;
  F(0, 1) = F(1, 1) = 0;
  F(0, 2) = -(left[1]  - y);
  F(1, 2) = -(right[1] - y);

  // Construct a matrix similar to E in Anitscu and Potra 1997. This matrix
  // will yield mu*fN - E*fF = 0, or, equivalently:
  // mu*fN₁ - fF₁⁺ - fF₁⁻ ≥ 0
  // mu*fN₂ - fF₂⁺ - fF₂⁻ ≥ 0
  Eigen::Matrix<T, nk, nc> E;
  E.col(0) << 1, 0, 1, 0;
  E.col(1) << 0, 1, 0, 1;

  // Construct the LCP matrix. First do the "normal contact direction" rows.
  Eigen::Matrix<T, 8, 8> MM;
  MM.template block<2, 2>(0, 0) = N * iM * N.transpose();
  MM.template block<2, 2>(0, 2) = N * iM * F.transpose();
  MM.template block<2, 2>(0, 4) = -MM.template block<2, 2>(0, 2);
  MM.template block<2, 2>(0, 6).setZero();

  // Now construct the un-negated tangent contact direction rows (everything
  // but last block column).
  MM.template block<2, 2>(2, 0) = F * iM * N.transpose();
  MM.template block<2, 2>(2, 2) = F * iM * F.transpose();
  MM.template block<2, 2>(2, 4) = -MM.template block<2, 2>(2, 2);

  // Now construct the negated tangent contact direction rows (everything but
  // last block column). These negated tangent contact directions allow the
  // LCP to compute forces applied along the negative x-axis.
  MM.template block<2, 6>(4, 0) = -MM.template block<2, 6>(2, 0);

  // Construct the last block column for the last set of rows (see Anitescu and
  // Potra, 1997).
  MM.template block<4, 2>(2, 6) = E;

  // Construct the last two rows, which provide the friction "cone" constraint.
  MM.template block<2, 2>(6, 0) = Matrix2<T>::Identity() * get_mu_coulomb();
  MM.template block<2, 4>(6, 2) = -E.transpose();
  MM.template block<2, 2>(6, 6).setZero();

  // Construct the LCP vector.
  Eigen::Matrix<T, 8, 1> qq(8);
  qq.segment(0, 2) = N * v;
  qq(0) += erp * left[1]/dt_;
  qq(1) += erp * right[1]/dt_;
  qq.segment(2, 2) = F * v;
  qq.segment(4, 2) = -qq.segment(2, 2);
  qq.template segment(6, 2).setZero();

  // Regularize the LCP matrix: this is essentially Tikhonov Regularization.
  // Cottle et al. show that any linear complementarity problem is solvable
  // for sufficiently large cfm.
  // R. Cottle, J.-S. Pang, and R. Stone. The Linear Complementarity Problem.
  // Academic Press, 1992.
  MM += Eigen::Matrix<T, 8, 8>::Identity() * cfm;

  // Solve the LCP.
  VectorX<T> zz;
  bool success = lcp_.SolveLcpLemke(MM, qq, &zz);
  DRAKE_DEMAND(success);

  // Obtain the normal and frictional contact forces.
  VectorX<T> fN = zz.segment(0, 2);
  VectorX<T> fF_pos = zz.segment(2, 2);
  VectorX<T> fF_neg = zz.segment(4, 2);

  // Compute the new velocity. Note that external forces have already been
  // incorporated into v.
  VectorX<T> vplus = v + iM * (N.transpose()*fN + F.transpose()*fF_pos -
                               F.transpose()*fF_neg);

  // Compute the new position using explicit Euler integration.
  VectorX<T> qplus = q + vplus*dt_;

  // Set the new discrete state.
  systems::BasicVector<T>* new_state = discrete_state->
      get_mutable_discrete_state(0);
  new_state->get_mutable_value().segment(0, 3) = qplus;
  new_state->get_mutable_value().segment(3, 3) = vplus;
}

// Sets the velocity derivatives for the rod, given contact forces.
template <class T>
void Rod2D<T>::SetAccelerations(const systems::Context<T>& context,
                                   systems::VectorBase<T>* const f,
                                   const T& fN, const T& fF,
                                   const T& cx, const T& cy) const {
  using std::abs;

  // Get the inputs.
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);

  // Get necessary state variables.
  const T& x = context.get_continuous_state_vector().GetAtIndex(0);
  const T& y = context.get_continuous_state_vector().GetAtIndex(1);
  const T& theta = context.get_continuous_state_vector().GetAtIndex(2);
  const T& thetadot = context.get_continuous_state_vector().GetAtIndex(5);

  // Retrieve the external forces.
  const Vector3<T> fapplied = input.segment(0, 3);

  // Compute the derivatives
  const T xddot = (fapplied(0) + fF) / mass_;
  const T yddot = (fapplied(1) + fN) / mass_ + get_gravitational_acceleration();
  const T thetaddot = ((cx - x) * fN - (cy - y) * fF + fapplied(2)) / J_;

  // Set the derivatives.
  f->SetAtIndex(3, xddot);
  f->SetAtIndex(4, yddot);
  f->SetAtIndex(5, thetaddot);

  // Get constants for checking accelerations.
  const double mu = mu_;
  const T ctheta = cos(theta), stheta = sin(theta);
  const int k = get_k(context);
  const double h = half_length_;

  // Verify that the vertical acceleration at the point of contact is zero
  // (i.e., cyddot = 0).
  const T cyddot =
      yddot + h * k * (ctheta * thetaddot - stheta * thetadot * thetadot);

  DRAKE_DEMAND(abs(cyddot) < 10 * std::numeric_limits<double>::epsilon());

  // If the force is within the friction cone, verify that the horizontal
  // acceleration at the point of contact is zero (i.e., cxddot = 0).
  if (fN * mu > abs(fF)) {
    const T cxddot =
        xddot + h * k * (-stheta * thetaddot - ctheta * thetadot * thetadot);

    DRAKE_DEMAND(abs(cxddot) < 10 * std::numeric_limits<double>::epsilon());
  }
}

// Sets the velocity derivatives for the rod, given contact forces at the two
// rod endpoints.
template <class T>
void Rod2D<T>::SetAccelerations(const systems::Context<T>& context,
                                systems::VectorBase<T>* const f,
                                const Vector2<T>& fN, const Vector2<T>& fF,
                                const Vector2<T>& ca, const Vector2<T>& cb)
                                  const {
  using std::abs;

  // Get the inputs.
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);

  // Get necessary state variables.
  const T& x = context.get_continuous_state_vector().GetAtIndex(0);
  const T& y = context.get_continuous_state_vector().GetAtIndex(1);
  const T& theta = context.get_continuous_state_vector().GetAtIndex(2);
  const T& thetadot = context.get_continuous_state_vector().GetAtIndex(5);

  // Retrieve the external forces.
  const Vector3<T> fapplied = input.segment(0, 3);

  // Compute the derivatives
  const T xddot = (fapplied(0) + fF[0] + fF[1]) / mass_;
  const T yddot = (fapplied(1) + fN[0] + fN[1]) / mass_ +
      get_gravitational_acceleration();
  const T thetaddot = ((ca[0] - x) * fN[0] - (ca[1] - y) * fF[0] +
                        (cb[0] - x) * fN[1] - (cb[1] - y) * fF[1] +
                        fapplied(2)) / J_;

  // Set the derivatives.
  f->SetAtIndex(3, xddot);
  f->SetAtIndex(4, yddot);
  f->SetAtIndex(5, thetaddot);

  // Get constants for checking accelerations.
  const double mu = mu_;
  const T ctheta = cos(theta), stheta = sin(theta);
  const double h = half_length_;

  // Verify that the vertical acceleration at both points of contact are zero
  // (i.e., cyddot = 0).
  const T cyddot0 =
      yddot + h * 1 * (ctheta * thetaddot - stheta * thetadot * thetadot);
  const T cyddot1 =
      yddot + h * -1 * (ctheta * thetaddot - stheta * thetadot * thetadot);

  DRAKE_DEMAND(abs(cyddot0) < 10 * get_cfm());
  DRAKE_DEMAND(abs(cyddot1) < 10 * get_cfm());

  // If the force is within the friction cone, verify that the horizontal
  // acceleration at the point of contact is zero (i.e., cxddot = 0).
  if (fN[0] * mu > abs(fF[0]) && fN[1] * mu > abs(fF[1])) {
    const T cxddot0 =
        xddot + h * 1 * (-stheta * thetaddot - ctheta * thetadot * thetadot);
    const T cxddot1 =
        xddot + h * -1 * (-stheta * thetaddot - ctheta * thetadot * thetadot);

    DRAKE_DEMAND(abs(cxddot0) < 10 * get_cfm());
    DRAKE_DEMAND(abs(cxddot1) < 10 * get_cfm());
  }
}

// Computes the contact forces for the case of zero sliding velocity for a
// single point of contact, assuming that the tangential acceleration at that
// point will be zero (i.e., cxddot = 0). This function solves for these forces.
// NOTE: put another way, this function computes the forces corresponding to
// the dynamics of a single mode (sticking-single-contact) in the hybrid
// dynamical system model of this system.
//
// Equations were determined by issuing the following command in Mathematica:
// cx[t_] := x[t] + k*Cos[theta[t]]*(r/2)
// cy[t_] := y[t] + k*Sin[theta[t]]*(r/2)
// Solve[{0 == D[D[cy[t], t], t],
//        D[D[y[t], t], t] == (fN + fY)/mass + g,
//        D[D[x[t], t], t] == (fF + fX)/mass,
//        J*D[D[theta[t], t], t] == (cx[t]-x[t])*fN - (cy[t]-y[t])*fF + tau,
//        0 == D[D[cx[t], t], t]},
//       { fN, fF, D[D[y[t], t], t], D[D[x[t], t], t],
//          D[D[theta[t], t], t] } ]
// where theta is the counter-clockwise angle the rod makes with the
// x-axis; fN and fF are contact normal and frictional forces; [fX fY] are
// arbitrary external forces (expressed in the world frame) applied at the
// center-of-mass of the rod; tau is an arbitrary external torque (expressed
// in the world frame) that should contain any moments due to any forces applied
// away from the center-of-mass plus any pure torques; g is the
// acceleration due to gravity, and (hopefully) all other variables are
// self-explanatory.
//
// The first two equations above are the formula
// for the point of contact. The next equation requires that the
// vertical acceleration be zero. The fourth and fifth equations
// describe the horizontal and vertical accelerations at the center
// of mass of the rod. The sixth equation yields the moment from
// the contact forces. The last equation specifies that the horizontal
// acceleration at the point of contact be zero.
// @returns a Vector2 with the first element giving the normal force (along
//          the positive y-direction) and the second element giving the
//          tangential force (along the positive x-direction).
template <class T>
Vector2<T> Rod2D<T>::CalcStickingContactForces(
    const systems::Context<T>& context) const {
  // Get necessary state variables.
  const T& theta = context.get_continuous_state_vector().GetAtIndex(2);
  const T& thetadot = context.get_continuous_state_vector().GetAtIndex(5);

  // Precompute quantities that will be used repeatedly.
  const T ctheta = cos(theta), stheta = sin(theta);
  const int k = get_k(context);

  // Get the inputs.
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);

  // Set named Mathematica constants.
  const double mass = mass_;
  const double r = 2 * half_length_;
  const double g = get_gravitational_acceleration();
  const double J = J_;
  const double fX = input(0);
  const double fY = input(1);
  const double tau = input(2);
  const T fN = (-(1/(8*J +
      2 * k * k * mass * r * r)) * (8 * fY * J + 8 * g * J * mass +
      fY * k * k * mass * r * r +  g * k * k * mass * mass * r * r +
      4 * k * mass * r * tau * ctheta -
      fY * k * k * mass * r * r * cos(2 * theta) -
      g * k * k * mass * mass * r * r *cos(2 * theta) +
      fX * k * k * mass * r * r * sin(2 * theta) -
      k * mass * r * (4 * J + k * k * mass * r * r)*
          stheta * thetadot * thetadot));

  const T fF = (-(1/(8 * J + 2 * k * k * mass * r * r)) *
         (8 * fX * J + fX * k * k * mass * r * r +
          fX * k * k * mass* r * r * cos(2 * theta)-
          4 * k * mass * r * tau * stheta +
          fY * k * k * mass * r * r * sin(2 * theta)+
          g * k * k * mass * mass * r * r * sin(2 * theta) -
          k * mass * r * (4 * J + k * k * mass * r * r) *
          ctheta * thetadot * thetadot));

  return Vector2<T>(fN, fF);
}

// Computes the contact forces for the case of nonzero sliding velocity at
// two points of contact. NOTE: put another way, this function computes the
// forces corresponding to the dynamics of a single mode (sliding-two-contacts)
// in the hybrid dynamical system model of this system. Equations governing the
// dynamics in this mode are:
//
// (1) M⋅dv/dt = fext + NᵀfN - μFᵀfN
// (2) N⋅dv/dt + dN/dt⋅v ≥ 0
// (3) fN ≥ 0
// (4) (N⋅dv/dt + dN/dt⋅v)ᵀ⋅fN = 0
// where M is the 3x3 generalized inertia matrix, v is the generalized velocity
// vector, fext is the generalized external force vector, μ is the coefficient
// of friction, N ∈ ℝⁿˣ³ is the Jacobian matrix transforming generalized
// velocities to velocities along the normal component of the n contact frames,
// and F ∈ ℝⁿˣ³ is the Jacobian matrix transforming generalized velocities to
// velocities along the _directions of sliding_ of the n contact frames. As in
// other functions, we assume that fN ∈ ℝⁿ are the magnitudes of forces applied
// along the contact normals and fF ∈ ℝⁿ are the magnitude of forces applied
// *against the directions of sliding*.
//
// From Equation 1:
//   M⋅dv/dt = fext + NᵀfN - μFᵀfN
//   dv/dt = M⁻¹(fext + NᵀfN - μFᵀfN)
// Equation 2 then can be reformulated as:
//   N⋅M⁻¹(fext + NᵀfN - μFᵀfN) + dN/dt⋅v ≥ 0
// Therefore, the LCP matrix is:
//   N⋅M⁻¹⋅(Nᵀ - μFᵀ)
// and the LCP vector is:
//   N⋅M⁻¹⋅fext + dN/dt⋅v
template <class T>
void Rod2D<T>::CalcTwoContactSlidingForces(
    const systems::Context<T>& context, Vector2<T>* fN, Vector2<T>* fF) const {
  // Get the necessary state variables.
  const VectorX<T> state = context.get_continuous_state_vector().
      CopyToVector();
  const auto& q = state.template segment<3>(0);
  const auto& v = state.template segment<3>(3);
  const T& x = q(0);
  const T& y = q(1);
  const T& theta = q(2);
  const T& xdot = v(0);
  const T& ydot = v(1);
  const T& thetadot = v(2);

  // Function name is predicated on two contact points.
  const int nc = 2;

  // Verify arguments.
  DRAKE_DEMAND(fN && fN->size() == nc);
  DRAKE_DEMAND(fF && fF->size() == nc);

  // Get the coefficient of friction.
  const double mu = get_mu_coulomb();

  // Get the inputs.
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);

  // Compute the two rod vertical endpoint locations.
  const T stheta = sin(theta), ctheta = cos(theta);

  // Three generalized velocity variables.
  const int ngv = 3;

  // Find left and right end point locations.
  const Vector2<T> left =
      CalcRodEndpoint(x, y, -1, ctheta, stheta, half_length_);
  const Vector2<T> right =
      CalcRodEndpoint(x, y, 1, ctheta, stheta, half_length_);

  // Compute the velocities at the end points.
  const Vector2<T> v_WRo(xdot, ydot);
  const T w_WR(thetadot);
  const Vector2<T> p_WRo(x, y);
  const Vector2<T> leftdot = CalcCoincidentRodPointVelocity(p_WRo, v_WRo, w_WR, left);
  const Vector2<T> rightdot = CalcCoincidentRodPointVelocity(p_WRo, v_WRo, w_WR, right);

  // Construct the inverse generalized inertia matrix computed about the
  // center of mass of the rod and expressed in the world frame.
  Matrix3<T> iM;
  iM << 1.0 / mass_, 0, 0,
      0, 1.0 / mass_, 0,
      0, 0, 1.0 / J_;

  // Compute the external forces (expressed in the world frame).
  const Vector3<T> fgrav(0, mass_ * get_gravitational_acceleration(), 0);
  const Vector3<T> fapplied = input.segment(0, 3);
  const Vector3<T> fext = fgrav + fapplied;

  // Verify that the two directions of sliding are identical.
  double sliding_sign_left = (leftdot[0] > 0) ? 1 : -1;
  double sliding_sign_right = (rightdot[0] > 0) ? 1 : -1;
  DRAKE_DEMAND(sliding_sign_left * sliding_sign_right > 0);

  // Set up the contact normal and tangent (friction) direction Jacobian
  // matrices. These take the form:
  //     | 0 1 n1 |        | 1 0 f1 |
  // N = | 0 1 n2 |    F = | 1 0 f2 |
  // where n1, n2/f1, f2 are the moment arm induced by applying the
  // force at the given contact point along the normal/tangent direction.
  Eigen::Matrix<T, nc, ngv> N, F;
  N(0, 0) = N(1, 0) = 0;
  N(0, 1) = N(1, 1) = 1;
  N(0, 2) = (left[0] - x);
  N(1, 2) = (right[0] - x);
  F(0, 0) = sliding_sign_left;
  F(1, 0) = sliding_sign_right;
  F(0, 1) = F(1, 1) = 0;
  F(0, 2) = -sliding_sign_left*(left[1] - y);
  F(1, 2) = -sliding_sign_right*(right[1] - y);

  // Compute dN/dt.
  Eigen::Matrix<T, nc, ngv> Ndot;
  Ndot(0, 0) = Ndot(1, 0) = 0;
  Ndot(0, 1) = Ndot(1, 1) = 0;
  Ndot(0, 2) = (leftdot[0] - xdot);
  Ndot(1, 2) = (rightdot[0] - xdot);

  // Form the vector in the 2-dimensional linear complementarity problem.
  Vector2<T> qq;
  qq = N * iM * fext + Ndot * v;

  // Form the 2x2 linear complementarity problem.
  Matrix2<T> MM;
  MM = N * iM * (N.transpose() - mu * F.transpose());

  // Attempt to solve the LCP. For μ = 0, the LCP is guaranteed to have a
  // solution, and several algorithms (e.g., Dantzig's Principle Pivoting
  // Method) are capable of solving it in expected polynomial time. For μ > 0,
  // the LCP might not possess a solution. We use Lemke's Algorithm, which is
  // capable of solving "harder" LCPs (i.e., more classes of matrices) and
  // then verify the solution if success is reported.

  // Set a zero tolerance for solving the LCP.
  const double eps = std::numeric_limits<double>::epsilon();
  const double zero_tol = std::sqrt(eps);

  // Solve the LCP.
  VectorX<T> zz, ww;
  bool success = lcp_.SolveLcpLemke(MM, qq, &zz, zero_tol);
  ww = MM * zz + qq;

  // If not successful, throw an exception.
  if (!success || zz.minCoeff() < -zero_tol || ww.minCoeff() < -zero_tol ||
      std::abs(zz.dot(ww)) > zero_tol)
    throw std::runtime_error("Unable to solve LCP- it may be unsolvable.");

  // Obtain the normal and frictional contact forces.
  *fN = zz;
  *fF = (*fN) * -mu;
}

// Computes the contact forces for the case of zero sliding velocity at two
// points of contact. NOTE: put another way, this function computes the forces
// corresponding to the dynamics of a single mode (nonsliding-two-contacts) in
// the hybrid dynamical system model of this system. Equations governing the
// dynamics in this mode are:
//
// (1) M⋅dv/dt = fext + NᵀfN + DᵀfD
// (2) N⋅dv/dt + dN/dt⋅v ≥ 0
// (3) fN ≥ 0
// (4) (N⋅dv/dt + dN/dt⋅v)ᵀ⋅fN = 0
// (5) 0 ≤ Eλ + D⋅dv/dt + dD/dt v  ⊥  fD ≥ 0
// (6) 0 ≤ (μ⋅fN - fD)  ⊥  λ ≥ 0
// where M is the 3x3 generalized inertia matrix, v is the generalized velocity
// vector, fext is the generalized external force vector, μ is the coefficient
// of friction, N ∈ ℝⁿˣ³ is the Jacobian matrix transforming generalized
// velocities to velocities along the normal component of the n contact frames,
// D ∈ ℝ²ⁿˣ³ is the Jacobian matrix transforming generalized velocities to
// velocities along the positive and negative tangent direction at each point of
// contact, E is a block matrix of ones, λ is roughly interpretable as the
// magnitudes of remaining tangential acceleration at each point of contact
// after contact forces are applied, fN ∈ ℝⁿ are the magnitudes of forces
// applied along the contact normals, and fD ∈ ℝ²ⁿ are the magnitude of
// frictional forces applied along the positive and negative tangent directions
// at each point of contact.
//
// Complementarity conditions (5) and (6) were inspired directly from
// corresponding conditions in [Anitescu and Potra, 1997]. In the case that
// the frictional force lies strictly within the friction cone, (μ⋅fN - fD) > 0
// in Equation 6, implying both that λ = 0 and- since fD ≥ 0, μ⋅fN = fD and
// D⋅dv/dt + dD/dt v = 0 (i.e., there can be no tangential acceleration). In the
// case that the friction force is insufficient to prevent sliding,
// D⋅dv/dt + dD/dt v ≠ 0, which implies that λ > 0. It
// should also be evident that when the frictional force pushes along +x, any
// remaining tangential acceleration must be pointed along -x (and vice versa).
template <class T>
void Rod2D<T>::CalcTwoContactNoSlidingForces(
    const systems::Context<T>& context, Vector2<T>* fN, Vector2<T>* fF) const {
  // Get the necessary state variables.
  // Get the necessary state variables.
  const VectorX<T> state = context.get_continuous_state_vector().
      CopyToVector();
  const auto& q = state.template segment<3>(0);
  const auto& v = state.template segment<3>(3);
  const T& x = q(0);
  const T& y = q(1);
  const T& theta = q(2);
  const T& xdot = v(0);
  const T& ydot = v(1);
  const T& thetadot = v(2);
  
  // Function name is predicated on two contact points.
  const int nc = 2;

  // Verify arguments.
  DRAKE_DEMAND(fN && fN->size() == nc);
  DRAKE_DEMAND(fF && fF->size() == nc);

  // Compute the two rod vertical endpoint locations.
  const T stheta = sin(theta), ctheta = cos(theta);

  // Three generalized velocity variables.
  const int ngv = 3;

  // Find left and right end point locations.
  const Vector2<T> left = CalcRodEndpoint(x, y, -1, ctheta, stheta, half_length_);
  const Vector2<T> right = CalcRodEndpoint(x, y, 1, ctheta, stheta, half_length_);

  // Compute the velocities at the end points.
  const Vector2<T> v_WRo(xdot, ydot);
  const T w_WR(thetadot);
  const Vector2<T> p_WRo(x, y);
  const Vector2<T> leftdot = CalcCoincidentRodPointVelocity(p_WRo, v_WRo, w_WR, left);
  const Vector2<T> rightdot = CalcCoincidentRodPointVelocity(p_WRo, v_WRo, w_WR, right);

  // Construct the inverse generalized inertia matrix computed about the
  // center of mass of the rod and expressed in the world frame.
  Matrix3<T> iM;
  iM << 1.0 / mass_, 0, 0,
      0, 1.0 / mass_, 0,
      0, 0, 1.0 / J_;

  // Compute the external forces (expressed in the world frame).
  const Vector3<T> fext = ComputeExternalForces(context);

  // Set up the contact normal and tangent (friction) direction Jacobian
  // matrices. These take the form:
  //     | 0 1 n1 |        | 1 0 f1 |
  // N = | 0 1 n2 |    F = | 1 0 f2 |
  // where n1, n2/f1, f2 are the moment arm induced by applying the
  // force at the given contact point along the normal/tangent direction.
  Eigen::Matrix<T, nc, ngv> N, F;
  N(0, 0) = N(1, 0) = 0;
  N(0, 1) = N(1, 1) = 1;
  N(0, 2) = (left[0] - x);
  N(1, 2) = (right[0] - x);
  F(0, 0) = 1;
  F(1, 0) = 1;
  F(0, 1) = F(1, 1) = 0;
  F(0, 2) = -(left[1] - y);
  F(1, 2) = -(right[1] - y);

  // Compute D.
  Eigen::Matrix<T, 2*nc, ngv> D;
  D.template block<nc, ngv>(0, 0) = F;
  D.template block<nc, ngv>(nc, 0) = -F;

  // Compute dN/dt.
  Eigen::Matrix<T, nc, ngv> Ndot;
  Ndot(0, 0) = Ndot(1, 0) = 0;
  Ndot(0, 1) = Ndot(1, 1) = 0;
  Ndot(0, 2) = (leftdot[0] - xdot);
  Ndot(1, 2) = (rightdot[0] - xdot);

  // Compute dF/dt.
  Eigen::Matrix<T, nc, ngv> Fdot;
  Fdot(0, 0) = Fdot(1, 0) = 0;
  Fdot(0, 1) = Fdot(1, 1) = 0;
  Fdot(0, 2) = -(leftdot[1] - ydot);
  Fdot(1, 2) = -(rightdot[1] - ydot);

  // Compute dD/dt.
  Eigen::Matrix<T, 2*nc, ngv> Ddot;
  Ddot.template block<nc, ngv>(0, 0) = Fdot;
  Ddot.template block<nc, ngv>(nc, 0) = -Fdot;

  // Form the vector in the linear complementarity problem.
  Eigen::Matrix<T, 4*nc, 1> qq;
  qq.template segment<2>(0) = N * iM * fext + Ndot * v;
  qq.template segment<4>(2) = D * iM * fext + Ddot * v;
  qq.template segment<2>(nc*3).setZero();

  // Construct E.
  const int nk = 2 * nc;
  Eigen::Matrix<T, nk, nc> E;
  E.col(0) << 1, 0, 1, 0;
  E.col(1) << 0, 1, 0, 1;

  // Form the linear complementarity problem matrix.
  Eigen::Matrix<T, 4*nc, 4*nc> MM;

  // Do the first block of rows.
  MM.template block<2, 2>(0, 0) = N * iM * N.transpose();
  MM.template block<2, 4>(0, 2) = N * iM * D.transpose();
  MM.template block<2, 2>(0, 6).setZero();

  // Do the second block of rows.
  MM.template block<4, 2>(2, 0) = D * iM * N.transpose();
  MM.template block<4, 4>(2, 2) = D * iM * D.transpose();
  MM.template block<4, 2>(2, 6) = E;

  // Construct the last two rows, which provide the friction "cone" constraint.
  MM.template block<2, 2>(6, 0) = Matrix2<T>::Identity() * get_mu_coulomb();
  MM.template block<2, 4>(6, 2) = -E.transpose();
  MM.template block<2, 2>(6, 6).setZero();

  // Do some very small regularization on the matrix to match the time
  // stepping solution and help ensure a robust solution.
  const double cfm = get_cfm();
  MM += Eigen::Matrix<T, 8, 8>::Identity() * cfm;

  // Solve the LCP. It has the same form of that in [Anitescu and Potra, 1997];
  // by following the same proof as in that reference, it is clear that this
  // matrix is copositive and hence provably solvable by Lemke's Algorithm.
  VectorX<T> zz, ww;
  bool success = lcp_.SolveLcpLemke(MM, qq, &zz);
  DRAKE_DEMAND(success);

  // Obtain the normal and frictional contact forces.
  *fN = zz.template segment<2>(0);
  *fF = zz.template segment<2>(2) - zz.template segment<2>(4);
}

// This is a smooth approximation to a step function. Input x goes from 0 to 1;
// output goes 0 to 1 but smoothed with an S-shaped quintic with first and
// second derivatives zero at both ends.
template <class T>
T Rod2D<T>::step5(const T& x) {
  DRAKE_ASSERT(0 <= x && x <= 1);
  const T x3 = x * x * x;
  return x3 * (10 + x * (6 * x - 15));  // 10x^3-15x^4+6x^5
}

// This function models Stribeck dry friction as a C² continuous quintic spline
// with 3 segments that is used to calculate an effective coefficient of
// friction mu_stribeck. That is the composite coefficient of friction that we
// use to scale the normal force to produce the friction force.
//
// We are given the friction coefficients mu_s (static) and mu_d (dynamic), and
// a dimensionless slip speed s>=0, then calculate:
//     mu_stribeck =
//      (a) s=0..1: smooth interpolation from 0 to mu_s
//      (b) s=1..3: smooth interpolation from mu_s down to mu_d (Stribeck)
//      (c) s=3..Inf mu_d
//
// s must be non-dimensionalized by taking the actual slip speed and dividing by
// the stiction slip velocity tolerance.
//
// mu_stribeck is zero at s=0 with 1st deriv (slope) zero and 2nd deriv
// (curvature) 0. At large speeds s>>0 the value is mu_d, again with zero slope
// and curvature. That makes mu_stribeck(s) C² continuous for all s>=0.
//
// The resulting curve looks like this:
//
//     mu_s   ***
//           *    *
//           *     *
//           *      *
//     mu_d  *        *  *    *    *    *   slope, curvature = 0 at Inf
//          *
//          *
//         *
//  0  * *   slope, curvature = 0 at 0
//
//     |      |           |
//   s=0      1           3
//
template <class T>
T Rod2D<T>::CalcMuStribeck(const T& mu_s, const T& mu_d, const T& s) {
  DRAKE_ASSERT(mu_s >= 0 && mu_d >= 0 && s >= 0);
  T mu_stribeck;
  if (s >= 3)
    mu_stribeck = mu_d;  // sliding
  else if (s >= 1)
    mu_stribeck = mu_s - (mu_s - mu_d) * step5((s - 1) / 2);  // Stribeck
  else
    mu_stribeck = mu_s * step5(s);  // 0 <= s < 1 (stiction)
  return mu_stribeck;
}

// Finds the locations of the two rod endpoints and generates contact forces at
// either or both end points (depending on contact condition) using a linear
// stiffness model, Hunt and Crossley normal dissipation, and a Stribeck
// friction model. No forces are applied anywhere else along the rod. The force
// is returned as the spatial force F_Ro_W (described in Rod2D class comments),
// represented as (fx,fy,τ).
//
// Note that we are attributing all of the compliance to the *halfspace*, not
// the *rod*, so that forces will be applied to the same points of the rod as is
// done in the rigid contact models. That makes comparison of the models easier.
//
// For each end point, let h be the penetration depth (in the -y direction) and
// v the slip speed along x. Then
//   fK =  k*h                   normal force due to stiffness
//   fD =  fK*d*hdot             normal force due to dissipation
//   fN =  max(fK + fD, 0)       total normal force (>= 0)
//   fF = -mu(|v|)*fN*sign(v)    friction force
//   f  = fN + fF                total force
//
// Here mu(v) is a Stribeck function that is zero at zero slip speed, and
// reaches a maximum mu_s at the stiction speed tolerance. mu_s is the
// static coefficient of friction.
//
// Note: we return the spatial force in a Vector3 but it is not a vector
// quantity.
template <class T>
Vector3<T> Rod2D<T>::CalcCompliantContactForces(
    const systems::Context<T>& context) const {
  // Depends on continuous state being available.
  DRAKE_DEMAND(simulation_type_ == SimulationType::kCompliant);

  using std::abs;
  using std::max;

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const Vector2<T> p_WRo(state.GetAtIndex(0), state.GetAtIndex(1));
  const T theta = state.GetAtIndex(2);
  const Vector2<T> v_WRo(state.GetAtIndex(3), state.GetAtIndex(4));
  const T w_WR(state.GetAtIndex(5));

  const T ctheta = cos(theta), stheta = sin(theta);

  // Find left and right end point locations.
  Vector2<T> p_WP[2] = {
      CalcRodEndpoint(p_WRo[0], p_WRo[1], -1, ctheta, stheta, half_length_),
      CalcRodEndpoint(p_WRo[0], p_WRo[1], 1, ctheta, stheta, half_length_)};

  // Calculate the net spatial force to apply at rod origin from the individual
  // contact forces at the endpoints.
  Vector3<T> F_Ro_W(0, 0, 0);  // Accumulate contact forces here.
  for (int i = 0; i < 2; ++i) {
    const Vector2<T>& p_WC = p_WP[i];  // Get contact point C location in World.

    // Calculate penetration depth h along -y; negative means separated.
    const T h = -p_WC[1];
    if (h > 0) {  // This point is in contact.
      const Vector2<T> v_WRc =  // Velocity of rod point coincident with C.
          CalcCoincidentRodPointVelocity(p_WRo, v_WRo, w_WR, p_WC);
      const T hdot = -v_WRc[1];  // Penetration rate in -y.
      const T v = v_WRc[0];      // Slip velocity in +x.
      const int sign_v = v < 0 ? -1 : 1;
      const T fK = get_stiffness() * h;  // See method comment above.
      const T fD = fK * get_dissipation() * hdot;
      const T fN = max(fK + fD, T(0));
      const T mu = CalcMuStribeck(get_mu_static(), get_mu_coulomb(),
                                  abs(v) / get_stiction_speed_tolerance());
      const T fF = -mu * fN * T(sign_v);

      // Find the point Rc of the rod that is coincident with the contact point
      // C, measured from Ro but expressed in W.
      const Vector2<T> p_RRc_W = p_WC - p_WRo;
      const Vector2<T> f_Rc(fF, fN);  // The force to apply at Rc.

      const T t_R = cross2(p_RRc_W, f_Rc);  // τ=r X f.
      F_Ro_W += Vector3<T>(fF, fN, t_R);
    }
  }
  return F_Ro_W;  // A 2-vector & scalar, not really a 3-vector.
}

// Computes the accelerations of the rod center of mass for the case of the rod
// contacting the surface at exactly one point and with sliding velocity. NOTE:
// put another way, this function computes the accelerations corresponding to
// the dynamics of a single mode (sliding-single-contact) in the hybrid
// dynamical system model of this system.
template <class T>
void Rod2D<T>::CalcAccelerationsOneContactSliding(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::abs;
  using std::max;

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);
  const T& thetadot = state.GetAtIndex(5);

  // Obtain the structure we need to write into.
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();
  DRAKE_ASSERT(f != nullptr);

  // The two endpoints of the rod are located at (x,y) + R(theta)*[0,r/2] and
  // (x,y) + R(theta)*[0,-r/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and r is designated as the rod length. Thus, the heights of
  // the rod endpoints are y + sin(theta)*r/2 and y - sin(theta)*r/2.
  const T ctheta = cos(theta), stheta = sin(theta);
  const int k = get_k(context);

  // Determine the point of contact (cx, cy).
  const Vector2<T> c = CalcRodEndpoint(x, y, k, ctheta, stheta,
                                       half_length_);
  const T& cx = c[0];
  const T& cy = c[1];

  // Compute the horizontal velocity at the point of contact.
  const T cxdot = xdot - k * stheta * half_length_ * thetadot;

  // Get the inputs.
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);
  const double fX = input(0);
  const double fY = input(1);
  const double tau = input(2);

  // Compute the normal acceleration at the point of contact (cyddot),
  // *assuming zero contact force*. This equation comes from the kinematics of
  // the rod:
  // cy = y + k * half_rod_length * sin(θ)  [rod endpoint vertical location]
  // dcy/dt = dy/dt + k * half_rod_length * cos(θ) * dθ/dt
  // d²cy/dt² = d²y/dt² - k * half_rod_length * sin(θ) * (dθ/dt)² +
  //                      k * half_rod_length * cos(θ) * d²θ/dt²
  const T yddot = get_gravitational_acceleration() + fY/mass_;
  const T thetaddot = tau/J_;
  T cyddot = yddot -
      k * half_length_ * stheta * thetadot * thetadot +
      k * half_length_ * ctheta * thetaddot;

  // If the normal acceleration is non-negative, no contact forces need be
  // applied.
  const T zero_tol = 10 * std::numeric_limits<double>::epsilon() *
      max(T(1), thetadot) * max(T(1), thetaddot);
  if (cyddot > -zero_tol) {
    f->SetAtIndex(3, fX / mass_);
    f->SetAtIndex(4, fY / mass_ + g_);
    f->SetAtIndex(5, tau / J_);
    return;
  }

  // These equations were determined by issuing the following
  // commands in Mathematica:
  // cx[t_] := x[t] + k*Cos[theta[t]]*(r/2)
  // cy[t_] := y[t] + k*Sin[theta[t]]*(r/2)
  // Solve[{0 == D[D[cy[t], t], t],
  //        D[D[y[t], t], t] == (fN+fY)/mass + g,
  //        D[D[x[t], t], t] == (fF+fX)/mass,
  //        J*D[D[theta[t], t], t] == (cx[t]-x[t])*fN - (cy[t]-y[t])*fF + tau,
  //        fF == -sgn_cxdot*mu*fN},
  // { fN, fF, D[D[y[t], t], t], D[D[x[t], t], t], D[D[theta[t], t], t]}]
  // where theta is the counter-clockwise angle the rod makes with the
  // x-axis; 'r' is the length of the rod; fN and fF are normal and
  // frictional forces, respectively; [fX fY] are arbitrary
  // external forces (expressed in the world frame) applied at the
  // center-of-mass of the rod; tau is an arbitrary external torque (expressed
  // in the world frame) that should contain any moments due to any forces
  // applied away from the center-of-mass plus any pure torques; sgn_cxdot is
  // the sign function applied to the horizontal contact velocity; g is the
  // acceleration due to gravity, and (hopefully) all other variables are
  // self-explanatory. The first two equations above are the formula
  // for the point of contact. The next equation requires that the
  // vertical acceleration be zero. The fourth and fifth equations
  // describe the horizontal and vertical accelerations at the center
  // of mass of the rod. The sixth equation yields the moment from
  // the contact forces. The last equation corresponds to the relationship
  // between normal and frictional forces (dictated by the Coulomb
  // friction model).
  const double J = J_;
  const double mass = mass_;
  const double mu = mu_;
  const int sgn_cxdot = (cxdot > 0) ? 1 : -1;
  const double g = get_gravitational_acceleration();
  const double r = 2 * half_length_;
  const T fN = -((2 * (2 * J * (fY + g * mass) + k * mass * r * tau * ctheta -
       J * k * mass * r * stheta * thetadot * thetadot))/
       (4 * J + k * k * mass * r * r * ctheta * ctheta +
       k * k * mass * mu * r * r * ctheta * sgn_cxdot * stheta));

  // Check for inconsistent configuration.
  if (fN < 0)
    throw std::runtime_error("Inconsistent configuration detected.");

  // Now that normal force is computed, set the acceleration.
  const T fF = -sgn_cxdot * mu_ * fN;
  f->SetAtIndex(3, (fF + fX) / mass_);
  f->SetAtIndex(4, (fN + fY) / mass_ + get_gravitational_acceleration());
  f->SetAtIndex(5, ((cx - x) * fN - (cy - y) * fF + tau) / J);

  // Lines below currently unused but are occasionally helpful for
  // debugging.
  //        const T xddot = f->GetAtIndex(3);
  //        const T cxddot = xddot + r*k*(-stheta*thetaddot -
  //                                        +ctheta*thetadot*thetadot)/2;
}

// Computes the accelerations of the rod center of mass for the case of the rod
// contacting the surface at exactly one point and without any sliding velocity.
// NOTE: put another way, this function computes the accelerations corresponding
// to the dynamics of a single mode (non-sliding-single-contact, resulting in
// either sticking or a transition to sliding) in the hybrid
// dynamical system model of this system.
template <class T>
void Rod2D<T>::CalcAccelerationsOneContactNoSliding(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::abs;

  // Obtain the structure we need to write into.
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();
  DRAKE_ASSERT(f != nullptr);

  // Get necessary state variables.
  const T& x = context.get_continuous_state_vector().GetAtIndex(0);
  const T& y = context.get_continuous_state_vector().GetAtIndex(1);
  const T& theta = context.get_continuous_state_vector().GetAtIndex(2);
  const T& thetadot = context.get_continuous_state_vector().GetAtIndex(5);

  // Get the contact point.
  const int k = get_k(context);
  const T ctheta = cos(theta), stheta = sin(theta);
  const Vector2<T> c = CalcRodEndpoint(x, y, k, ctheta, stheta,
                                       half_length_);
  const T& cx = c[0];
  const T& cy = c[1];

  // Compute the contact forces, assuming sticking contact.
  const Vector2<T> cf = CalcStickingContactForces(context);
  const T& fN = cf(0);
  const T& fF = cf(1);

  // Sanity check that normal force is non-negative.
  DRAKE_DEMAND(fN >= 0);

  // Get the inputs.
  Vector3<T> fext = ComputeExternalForces(context);
  const double fX = fext(0);
  const double fY = fext(1);
  const double tau = fext(2);

  // Recompute fF if it does not lie within the friction cone.
  // Constrain F such that it lies on the edge of the friction cone.
  const double mu = get_mu_coulomb();
  if (abs(fF) > mu * fN) {
    // Set named Mathematica constants.
    const double mass = get_rod_mass();
    const double r = 2 * get_rod_half_length();
    const double J = get_rod_moment_of_inertia();
    const double g = get_gravitational_acceleration();

    // Pick the solution that minimizes the tangential acceleration toward
    // obeying the principle of maximum dissipation, which states that the
    // friction forces should be those that maximize the dot product
    // between slip velocity and frictional force. This particular solution was
    // obtained by solving for zero normal acceleration with the frictional
    // force pointing either possible direction (indicated by d, meaning
    // positive x-axis and negative x-axis).
    // cx[t_] := x[t] + k*Cos[theta[t]]*(r/2)
    // cy[t_] := y[t] + k*Sin[theta[t]]*(r/2)
    // Solve[{0 == D[D[cy[t], t], t],
    //       D[D[y[t], t], t] == (N + fY)/mass + g,
    //       D[D[x[t], t], t] == (F + fX)/mass,
    // J*D[D[theta[t], t], t] == (cx[t] - x[t])*N - (cy[t] - y[t])*F + tau,
    //       F == -d*mu*N},
   // {N, F, D[D[y[t], t], t], D[D[x[t], t], t], D[D[theta[t], t], t]}]
    auto calc_force = [=](int d) {
      const T N = (-2 * (2 * J * (fY + g * mass) + k * mass * r * tau * ctheta -
                   J * k * mass * r * stheta * thetadot * thetadot)/
                   (4 * J + k * k * mass * r * r * ctheta * ctheta +
                    d* k * k * mass * mu * r * r * ctheta * stheta));
      const T F = -d * mu * N;
      return Vector2<T>(N, F);
    };
    Vector2<T> s1 = calc_force(+1);
    Vector2<T> s2 = calc_force(-1);

    // Get the candidate normal and tangent forces.
    const T fN1 = s1(0);
    const T fN2 = s2(0);
    const T fF1 = s1(1);
    const T fF2 = s2(1);
    DRAKE_ASSERT(fN1 > -10*std::numeric_limits<double>::epsilon());
    DRAKE_ASSERT(fN2 > -10*std::numeric_limits<double>::epsilon());

    // Calculate candidate tangential accelerations.
    auto calc_tan_accel = [=](int d, const T N, const T F) {
      const T thetaddot = ((cx - x) * N - (cy - y) * F + tau) / J;
      return (F + fX) / mass +
          r * k * (-stheta * thetaddot - ctheta * thetadot * thetadot) / 2;
    };

    // Compute two tangential acceleration candidates.
    const T cxddot1 = calc_tan_accel(+1, fN1, fF1);
    const T cxddot2 = calc_tan_accel(-1, fN2, fF2);

    // Pick the one that is smaller in magnitude.
    if (abs(cxddot1) < abs(cxddot2)) {
      SetAccelerations(context, f, fN1, fF1, cx, cy);
    } else {
      SetAccelerations(context, f, fN2, fF2, cx, cy);
    }
  } else {
    // Friction force is within the friction cone.
    SetAccelerations(context, f, fN, fF, cx, cy);
  }
}

// Computes the accelerations of the rod center of mass for the case of the rod
// contacting the surface at more than one point. NOTE: put another way, this
// function computes the accelerations corresponding to the dynamics of one
// of two modes- sliding-two-contacts or non-sliding-two-contacts- in the hybrid
// dynamical system model of this system.
template <class T>
void Rod2D<T>::CalcAccelerationsTwoContact(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::abs;

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);

  // Obtain the structure we need to write into.
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();
  DRAKE_ASSERT(f != nullptr);

  // Get the two points of contact.
  const int k = get_k(context);
  const T ctheta = cos(theta), stheta = sin(theta);
  const Vector2<T> c1 = CalcRodEndpoint(x, y, k, ctheta, stheta,
                                        half_length_);
  const Vector2<T> c2 = CalcRodEndpoint(x, y, k, ctheta, stheta,
                                        half_length_);

  // Call the appropriate contact force computation method.
  Vector2<T> fN, fF;
  if (abs(xdot) < std::numeric_limits<double>::epsilon()) {
    CalcTwoContactNoSlidingForces(context, &fN, &fF);
  } else {
    CalcTwoContactSlidingForces(context, &fN, &fF);
  }

  SetAccelerations(context, f, fN, fF, c1, c2);
}

// Returns the time derivative of the 2x2 rotation matrix Rᶻ(θ) that transforms
// vectors in the Rod Frame to vectors in the world frame. Note that dRᶻ(θ)/dt
// is equal to Ṙᶻ(θ)⋅dθ/dt, but that this function only returns Ṙᶻ(θ).
template <class T>
Matrix2<T> Rod2D<T>::get_rotation_matrix_derivative(
    const systems::State<T>& state) const {
  using std::sin;
  using std::cos;

  const T& theta = (*state.get_continuous_state())[2];
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  Matrix2<T> Rdot;
  Rdot << -stheta, ctheta, -ctheta, -stheta;
  return Rdot;
}

template <class T>
bool Rod2D<T>::IsImpacting(const systems::Context<T>& context) const {
  using std::abs;
  using std::max;

  // Get the contact states.
  const std::vector<RigidContact>& contacts = get_contacts(context.get_state());

  // Get state data necessary to compute the point of contact.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& xdot = state.GetAtIndex(3);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // Compute the zero tolerance.
  const T zero_tol = std::numeric_limits<double>::epsilon() * max(10.0,
                         max(abs(xdot), max(abs(ydot), abs(thetadot))));

  // Set the velocity of the center of mass.
  const Vector2<T> v(xdot, ydot);

  // Get the time derivative of the rotation matrix.
  const Matrix2<T> Rdot = get_rotation_matrix_derivative(context.get_state());

  // Loop through all points of contact.
  for (size_t i = 0; i < contacts.size(); ++i) {
    if (contacts[i].state == RigidContact::ContactState::kNotContacting)
      continue;

    // Compute the translational velocity at the point of contact.
    VectorX<T> pdot = v + Rdot * contacts[i].u.segment(0,2) * thetadot;

    // Look for impact.
    const T rvel = pdot[1];
    if (rvel < -zero_tol)
      return true;
  }

  // No impact was found.
  return false;
}

// Computes the accelerations of the rod center of mass for the rod, both
// in compliant contact and contact-free.
template <typename T>
void Rod2D<T>::CalcAccelerationsCompliantContactAndBallistic(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Obtain the structure we need to write into (ds=d/dt state).
  systems::VectorBase<T>* const ds = derivatives->get_mutable_vector();

  // Get external applied force (a spatial force at Ro, in W).
  const int port_index = 0;
  const auto Fext_Ro_W = this->EvalEigenVectorInput(context, port_index);

  // Calculate contact forces (also spatial force at Ro, in W).
  const Vector3<T> Fc_Ro_W = CalcCompliantContactForces(context);
  const Vector3<T> F_Ro_W = Fc_Ro_W + Fext_Ro_W;  // Total force.

  // Second three derivative components are acceleration due to gravity,
  // contact forces, and non-gravitational, non-contact external forces.
  ds->SetAtIndex(3, F_Ro_W[0]/mass_);
  ds->SetAtIndex(4, F_Ro_W[1]/mass_ + get_gravitational_acceleration());
  ds->SetAtIndex(5, F_Ro_W[2]/J_);
}

// Computes the accelerations of the rod center of mass for the case of
// ballistic motion.
template <typename T>
void Rod2D<T>::CalcAccelerationsBallistic(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Obtain the structure we need to write into (ds=d/dt state).
  systems::VectorBase<T>* const ds = derivatives->get_mutable_vector();

  // Get the inputs.
  const int port_index = 0;
  const auto f_input = this->EvalEigenVectorInput(context, port_index);

  // Second three derivative components are acceleration due to gravity and
  // external forces.
  ds->SetAtIndex(3, f_input(0)/mass_);
  ds->SetAtIndex(4, f_input(1)/mass_ + get_gravitational_acceleration());
  ds->SetAtIndex(5, f_input(2)/J_);
}

template <typename T>
void Rod2D<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::sin;
  using std::cos;
  using std::abs;

  // Don't compute any derivatives if this is the time stepping system.
  if (simulation_type_ == SimulationType::kTimeStepping) {
    DRAKE_ASSERT(derivatives->size() == 0);
    return;
  }

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& v = context.get_continuous_state()->
      get_generalized_velocity();
  const T& xdot = v.GetAtIndex(0);
  const T& ydot = v.GetAtIndex(1);
  const T& thetadot = v.GetAtIndex(2);

  // First three derivative components are xdot, ydot, thetadot.
  derivatives->get_mutable_generalized_position()->SetFromVector(
      Vector3<T>(xdot, ydot, thetadot));

  // Compute the velocity derivatives (accelerations).
  if (simulation_type_ == SimulationType::kCompliant) {
    return CalcAccelerationsCompliantContactAndBallistic(context, derivatives);
  } else {
    // (Piecewise DAE approach follows).

    // Compute the external forces (expressed in the world frame).
    const Vector3<T> fext = ComputeExternalForces(context);

    // Populate problem data.
    RigidContactAccelProblemData<T> problem_data;
    InitRigidContactAccelProblemData(context, &problem_data);
    const std::vector<int>& sliding_contacts = problem_data.sliding_contacts;
    const std::vector<int>& non_sliding_contacts =
        problem_data.non_sliding_contacts;

    // Get numbers of friction directions and types of contacts.
    const int num_sliding = sliding_contacts.size();
    const int num_non_sliding = non_sliding_contacts.size();
    const int nc = num_sliding + num_non_sliding;
    const int nk = get_num_tangent_directions_per_contact() * num_non_sliding;
    const int half_nk = nk / 2;

    // Compute Jacobian matrices.
    FormRigidContactAccelJacobians(context.get_state(), &problem_data);

    // Alias vectors and Jacobian matrices.
    const MatrixX<T>& N = problem_data.N;
    const MatrixX<T>& Q = problem_data.Q;
    const MatrixX<T>& F = problem_data.F;
    const VectorX<T>& mu_sliding = problem_data.mu_sliding;

    // Construct the inverse generalized inertia matrix computed about the
    // center of mass of the rod and expressed in the world frame.
    Matrix3<T> iM;
    iM << 1.0/mass_, 0,         0,
        0,         1.0/mass_, 0,
        0,         0,         1.0/J_;

    // Alias matrices.
    MatrixX<T>& N_minus_mu_Q = problem_data.N_minus_mu_Q;
    MatrixX<T>& iM_x_FT = problem_data.iM_x_FT;

    // Precompute using indices.
    N_minus_mu_Q = AddScaledRightTerm(N, -mu_sliding, Q, sliding_contacts);
    iM_x_FT = MultTranspose(iM, F, non_sliding_contacts);

    // Formulate and solve the linear equation 0 = MM⋅zz + qq for zz.
    MatrixX<T> MM;
    VectorX<T> qq;
    FormSustainedContactLinearSystem(context, problem_data, &MM, &qq);

    // Solve the linear system. The LCP used in the active set method computed
    // an (invertible) basis to attain its solution, which means that MM should
    // be invertible. However, the LCP matrix was regularized (which may have
    // allowed an otherwise unsolvable LCP to be unsolvable in addition to its
    // other function, that of minimizing the effect of pivoting error on the
    // ability of the solver to find a solution on known solvable problems). For
    // this reason, we will also employ regularization here.
    if (qq.size() > 0) {
      // NOTE: this branch is necessary b/c Eigen currently handles zero-sized
      // matrices in a dumb way - QR factorization asserts on such a matrix.
      const double cfm = get_cfm();
      MM += MatrixX<T>::Identity(MM.rows(), MM.rows()) * cfm;
      QR_.compute(MM);
      VectorX<T> zz = QR_.solve(-qq);

      // Get the contact forces.
      const auto fN = zz.segment(0, nc);
      const auto fF = zz.segment(nc, half_nk);

      // Compute and save the acceleration at the center-of-mass.
      VectorX<T> a = iM * (fext + N_minus_mu_Q.transpose() * fN) + iM_x_FT * fF;
      derivatives->get_mutable_generalized_velocity()->SetFromVector(a);

      // TODO(edrumwri): Verify non-negativity of zz or save the constraint
      // multipliers?
    } else {
      derivatives->get_mutable_generalized_velocity()->SetFromVector(iM * fext);
    }
  }
}

/// Allocates the abstract state (for piecewise DAE systems).
template <typename T>
std::unique_ptr<systems::AbstractValues> Rod2D<T>::AllocateAbstractState()
    const {
  if (simulation_type_ == SimulationType::kPiecewiseDAE) {
    // Piecewise DAE approach needs two abstract variables: one mode and one
    // contact point indicator.
    std::vector<std::unique_ptr<systems::AbstractValue>> abstract_data;
    abstract_data.push_back(
        std::make_unique<systems::Value<std::vector<RigidContact>>>());

    return std::make_unique<systems::AbstractValues>(std::move(abstract_data));
  } else {
    // Time stepping and compliant approaches need no abstract variables.
    return std::make_unique<systems::AbstractValues>();
  }
}

/// Sets the rod to a 45 degree angle with the halfspace and positions the rod
/// such that it and the halfspace are touching at exactly one point of contact.
template <typename T>
void Rod2D<T>::SetDefaultState(const systems::Context<T>& context,
                                  systems::State<T>* state) const {
  using std::sqrt;
  using std::sin;

  // Initial state corresponds to an inconsistent configuration for piecewise
  // DAE.
  const double half_len = get_rod_half_length();
  VectorX<T> x0(6);
  const double r22 = sqrt(2) / 2;
  x0 << half_len * r22, half_len * r22, M_PI / 4.0, -1, 0, 0;  // Initial state.
  if (simulation_type_ == SimulationType::kTimeStepping) {
    state->get_mutable_discrete_state()->get_mutable_discrete_state(0)->
        SetFromVector(x0);
  } else {
    // Continuous variables.
    state->get_mutable_continuous_state()->SetFromVector(x0);

    // Set abstract variables for piecewise DAE approach.
    if (simulation_type_ == SimulationType::kPiecewiseDAE) {
      // Initialize the vector of contact points.
      std::vector<RigidContact>& contacts = get_contacts(state);

      // Indicate that the rod is in the single contact sliding mode.
      contacts.resize(2);

      // First point is Rl in Rod Frame (see class documentation); in the
      // default configuration, it contacts the half-space and is sliding.
      contacts[0].state = RigidContact::ContactState::kContactingAndSliding;
      contacts[0].mu = get_mu_coulomb();
      contacts[0].u = Eigen::Vector3d(-get_rod_half_length(), 0, 0);

      // Second point is Rr in Rod Frame. It is not initially contacting
      // the half-space.
      contacts[1].state = RigidContact::ContactState::kNotContacting;
      contacts[1].mu = get_mu_coulomb();
      contacts[1].u = Eigen::Vector3d(get_rod_half_length(), 0, 0);
    }
  }
}

}  // namespace rod2d
}  // namespace examples
}  // namespace drake
