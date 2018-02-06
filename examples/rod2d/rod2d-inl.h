#pragma once

// @file
// Template method implementations for rod2d.h.
// Most users should only include that file, not this one.
// For background, see http://drake.mit.edu/cxx_inl.html.

/* clang-format off to disable clang-format-includes */
#include "drake/examples/rod2d/rod2d.h"
/* clang-format on */

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
Rod2D<T>::Rod2D(SimulationType simulation_type, double dt)
    : simulation_type_(simulation_type), dt_(dt) {
  // Verify that the simulation approach is either piecewise DAE or
  // compliant ODE.
  if (simulation_type == SimulationType::kTimeStepping) {
    if (dt <= 0.0)
      throw std::logic_error(
          "Time stepping approach must be constructed using"
          " strictly positive step size.");

    // Time stepping approach requires three position variables and
    // three velocity variables, all discrete, and periodic update.
    this->DeclarePeriodicDiscreteUpdate(dt);
    this->DeclareDiscreteState(6);
  } else {
    if (dt != 0)
      throw std::logic_error(
          "Piecewise DAE and compliant approaches must be "
          "constructed using zero step size.");

    // Both piecewise DAE and compliant approach require six continuous
    // variables.
    this->DeclareContinuousState(Rod2dStateVector<T>(), 3, 3, 0);  // q, v, z
  }

  this->DeclareInputPort(systems::kVectorValued, 3);
  state_output_port_ = &this->DeclareVectorOutputPort(
      systems::BasicVector<T>(6), &Rod2D::CopyStateOut);
  pose_output_port_ = &this->DeclareVectorOutputPort(&Rod2D::CopyPoseOut);
  contact_force_output_port_ = &this->DeclareVectorOutputPort(
      systems::BasicVector<T>(3), &Rod2D::ComputeAndCopyContactForceOut);

  // Create the contact candidates.
  SetContactCandidates();
}

template <class T>
void Rod2D<T>::InitializeAbstractStateFromContinuousState(
    double sliding_velocity_threshold,
    systems::State<T>* state) const {
  using std::abs;

  if (simulation_type_ != Rod2D<T>::SimulationType::kPiecewiseDAE)
    throw std::logic_error("Simulation type is not kPiecewiseDAE");

  // Find both endpoint locations.
  const Vector2<T> left = CalcContactLocationInWorldFrame(*state, kLeft);
  const Vector2<T> right = CalcContactLocationInWorldFrame(*state, kRight);

  if (left[1] <= 0) {
    const Vector2<T> left_vel = CalcContactVelocity(*state, kLeft);
    const multibody::constraint::SlidingModeType sliding = 
        ((abs(left_vel[0]) > sliding_velocity_threshold)) ? 
        multibody::constraint::SlidingModeType::kSliding :
        multibody::constraint::SlidingModeType::kNotSliding;
    if (right[1] <= 0) {
      SetBothEndpointsContacting(state, sliding);
    } else {
      SetOneEndpointContacting(kLeft, state, sliding);
    }
  } else {
    if (right[1] <= 0) {
      const Vector2<T> right_vel = CalcContactVelocity(*state, kRight);
      const multibody::constraint::SlidingModeType sliding = 
          ((abs(right_vel[0]) > sliding_velocity_threshold)) ? 
          multibody::constraint::SlidingModeType::kSliding :
          multibody::constraint::SlidingModeType::kNotSliding;
      SetOneEndpointContacting(kRight, state, sliding);
    } else {
      SetBallisticMode(state); 
    }
  }
} 

template <class T>
void Rod2D<T>::SetBallisticMode(systems::State<T>* state) const {
  // Remove all contacts from force calculations.
  std::vector<multibody::constraint::PointContact>& contacts =
      get_contacts_used_in_force_calculations(state);
  contacts.clear();

  // Disable all witness functions except the signed distance one.
  GetSignedDistanceWitness(kLeft, *state)->set_enabled(true);
  GetSignedDistanceWitness(kRight, *state)->set_enabled(true);
  GetNormalVelWitness(kLeft, *state)->set_enabled(false);
  GetNormalVelWitness(kRight, *state)->set_enabled(false);
  GetNormalAccelWitness(kLeft, *state)->set_enabled(false);
  GetNormalAccelWitness(kRight, *state)->set_enabled(false);
  GetPosSlidingWitness(kLeft, *state)->set_enabled(false);
  GetPosSlidingWitness(kRight, *state)->set_enabled(false);
  GetNegSlidingWitness(kLeft, *state)->set_enabled(false);
  GetNegSlidingWitness(kRight, *state)->set_enabled(false);
  GetNormalForceWitness(kLeft, *state)->set_enabled(false);
  GetNormalForceWitness(kRight, *state)->set_enabled(false);
  GetStickingFrictionForceSlackWitness(kLeft, *state)->
      set_enabled(false);
  GetStickingFrictionForceSlackWitness(kRight, *state)->
      set_enabled(false);
}

template <class T>
void Rod2D<T>::SetOneEndpointContacting(
    int index,
    systems::State<T>* state,
    multibody::constraint::SlidingModeType sliding_type) const {
  // Set the contact to sliding property appropriately. 
  std::vector<multibody::constraint::PointContact>& contacts =
      get_contacts_used_in_force_calculations(state);
  contacts.resize(1);
  contacts.front().sliding_type = sliding_type; 
  contacts.front().id = reinterpret_cast<void*>(index);

  // The signed distance witnesses should always be enabled.
  GetSignedDistanceWitness(kLeft, *state)->set_enabled(true);
  GetSignedDistanceWitness(kRight, *state)->set_enabled(true);

  // Get the other index.
  DRAKE_DEMAND(index == 0 || index == 1);
  const int other = (index == 0) ? 1 : 0;

  // Enable and disable proper witnesses for sliding / non-sliding contact. 
  if (sliding_type == multibody::constraint::SlidingModeType::kSliding) {
    // Enable these witnesses.
    GetPosSlidingWitness(index, *state)->set_enabled(true);
    GetNegSlidingWitness(index, *state)->set_enabled(true);

    // Disable these witnesses.
    GetStickingFrictionForceSlackWitness(index, *state)->
        set_enabled(false);
  } else {
    // Enable these witnesses.
    GetStickingFrictionForceSlackWitness(index, *state)->
        set_enabled(true);

    // Disable these witnesses. Note that the sliding witnesses are disabled
    // at this time- they have to be enabled when the sticking friction
    // witness is triggered.
    GetPosSlidingWitness(index, *state)->set_enabled(false);
    GetNegSlidingWitness(index, *state)->set_enabled(false);
  }

  // Disable all witness functions (except the signed distance one) for the
  // right endpoint.
  GetNormalVelWitness(other, *state)->set_enabled(false);
  GetNormalAccelWitness(other, *state)->set_enabled(false);
  GetPosSlidingWitness(other, *state)->set_enabled(false);
  GetNegSlidingWitness(other, *state)->set_enabled(false);
  GetNormalForceWitness(other, *state)->set_enabled(false);
  GetStickingFrictionForceSlackWitness(other, *state)->
      set_enabled(false);

  // The following witnesses will be disabled by default. 
  GetNormalVelWitness(index, *state)->set_enabled(false);
  GetNormalAccelWitness(index, *state)->set_enabled(false);

  // We need to enable the normal force witness.
  GetNormalForceWitness(index, *state)->set_enabled(true);
}

template <class T>
void Rod2D<T>::SetBothEndpointsContacting(
    systems::State<T>* state,
    multibody::constraint::SlidingModeType sliding_type) const {
  // Set the contacts to sliding property appropriately. 
  std::vector<multibody::constraint::PointContact>& contacts =
      get_contacts_used_in_force_calculations(state);
  contacts.resize(2);
  contacts.front().sliding_type = sliding_type;
  contacts.back().sliding_type = sliding_type;
  contacts.front().id = reinterpret_cast<void*>(kLeft);
  contacts.back().id = reinterpret_cast<void*>(kRight);

  // The signed distance witnesses should always be enabled.
  GetSignedDistanceWitness(kLeft, *state)->set_enabled(true);
  GetSignedDistanceWitness(kRight, *state)->set_enabled(true);

  // Enable and disable proper witnesses for sliding / non-sliding contact. 
  switch (sliding_type) {
    case multibody::constraint::SlidingModeType::kSliding:
      // Enable these witnesses.
      GetPosSlidingWitness(kLeft, *state)->set_enabled(true);
      GetPosSlidingWitness(kRight, *state)->set_enabled(true);
      GetNegSlidingWitness(kLeft, *state)->set_enabled(true);
      GetNegSlidingWitness(kRight, *state)->set_enabled(true);

      // Disable these witnesses.
      GetStickingFrictionForceSlackWitness(kLeft, *state)->
          set_enabled(false);
      GetStickingFrictionForceSlackWitness(kRight, *state)->
          set_enabled(false);
      break;

    case multibody::constraint::SlidingModeType::kNotSliding:
      // Enable these witnesses.
      GetStickingFrictionForceSlackWitness(kLeft, *state)->
          set_enabled(true);
      GetStickingFrictionForceSlackWitness(kRight, *state)->
          set_enabled(true);

      // Disable these witnesses.
      GetPosSlidingWitness(kLeft, *state)->set_enabled(false);
      GetPosSlidingWitness(kRight, *state)->set_enabled(false);
      GetNegSlidingWitness(kLeft, *state)->set_enabled(false);
      GetNegSlidingWitness(kRight, *state)->set_enabled(false);
      break;

    case multibody::constraint::SlidingModeType::kTransitioning:
      // Enable these witnesses.
      GetPosSlidingWitness(kLeft, *state)->set_enabled(true);
      GetPosSlidingWitness(kRight, *state)->set_enabled(true);
      GetNegSlidingWitness(kLeft, *state)->set_enabled(true);
      GetNegSlidingWitness(kRight, *state)->set_enabled(true);

      // Disable these witnesses.
      GetStickingFrictionForceSlackWitness(kLeft, *state)->
          set_enabled(false);
      GetStickingFrictionForceSlackWitness(kRight, *state)->
          set_enabled(false);
      break;

    case multibody::constraint::SlidingModeType::kNotSet:
      DRAKE_ABORT();
  }

  // Enable the normal force witnesses. 
  GetNormalForceWitness(kLeft, *state)->set_enabled(true);
  GetNormalForceWitness(kRight, *state)->set_enabled(true);

  // The following witnesses will be disabled by default. 
  GetNormalVelWitness(kLeft, *state)->set_enabled(false);
  GetNormalVelWitness(kRight, *state)->set_enabled(false);
  GetNormalAccelWitness(kLeft, *state)->set_enabled(false);
  GetNormalAccelWitness(kRight, *state)->set_enabled(false);
}

// Computes the external forces on the rod.
// @returns a three dimensional vector corresponding to the forces acting at
//          the center-of-mass of the rod (first two components) and the
//          last corresponding to the torque acting on the rod (final
//          component). All forces and torques are expressed in the world
//          frame.
template <class T>
Vector3<T> Rod2D<T>::ComputeExternalForces(
    const systems::Context<T>& context) const {
  // Compute the external forces (expressed in the world frame).
  const int port_index = 0;
  const VectorX<T> input = this->EvalEigenVectorInput(context, port_index);
  const Vector3<T> fgrav(0, mass_ * get_gravitational_acceleration(), 0);
  const Vector3<T> fapplied = input.segment(0, 3);
  return fgrav + fapplied;
}

template <class T>
std::vector<multibody::constraint::PointContact>&
    Rod2D<T>::get_contacts_used_in_force_calculations(
    systems::State<T>* state) const {
  return state->get_mutable_abstract_state()
      .get_mutable_value(kContactAbstractIndex)
      .template GetMutableValue<std::vector<multibody::constraint::PointContact>>();
}

template <class T>
const std::vector<multibody::constraint::PointContact>&
    Rod2D<T>::get_contacts_used_in_force_calculations(
    const systems::State<T>& state) const {
  return state.get_abstract_state()
      .get_value(kContactAbstractIndex).
      template GetValue<std::vector<multibody::constraint::PointContact>>();
}

template <class T>
Vector2<T> Rod2D<T>::CalcRodEndpoint(const T& x, const T& y, int k,
                                     const T& ctheta, const T& stheta,
                                     double half_rod_len) {
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

/// Solves MX = B for X, where M is the generalized inertia matrix of the rod
/// and is defined in the following manner:<pre>
/// M = | m 0 0 |
///     | 0 m 0 |
///     | 0 0 J | </pre>
/// where `m` is the mass of the rod and `J` is its moment of inertia.
template <class T>
MatrixX<T> Rod2D<T>::solve_inertia(const MatrixX<T>& B) const {
  const T inv_mass = 1.0 / get_rod_mass();
  const T inv_J = 1.0 / get_rod_moment_of_inertia();
  Matrix3<T> iM;
  iM << inv_mass, 0,        0,
        0,       inv_mass, 0,
        0,       0,        inv_J;
  return iM * B;
}

/// The velocity tolerance for sliding.
template <class T>
T Rod2D<T>::GetSlidingVelocityTolerance() const {
  // TODO(edrumwri): Change this coarse tolerance, which is only guaranteed to
  // be reasonable for rod locations near the world origin and rod velocities
  // of unit magnitude, to a computed tolerance that can account for these
  // assumptions being violated.
  return 100 * std::numeric_limits<double>::epsilon();
}

// Gets the time derivative of a 2D rotation matrix.
template <class T>
Matrix2<T> Rod2D<T>::GetRotationMatrixDerivative(T theta, T thetadot) {
  using std::cos;
  using std::sin;
  const T cth = cos(theta), sth = sin(theta);
  Matrix2<T> Rdot;
  Rdot << -sth, -cth, cth, -sth;
  return Rdot * thetadot;
}

// Gets the second time derivative of a 2D rotation matrix.
template <class T>
Matrix2<T> Rod2D<T>::GetRotationMatrix2ndDerivative(T theta, T thetaddot) {
  using std::cos;
  using std::sin;
  const T cth = cos(theta), sth = sin(theta);
  Matrix2<T> Rddot;
  Rddot << -cth, sth, -sth, -cth;
  return Rddot * thetaddot;
}

template <class T>
void Rod2D<T>::GetContactPoints(const systems::Context<T>& context,
                                std::vector<Vector2<T>>* points) const {
  using std::cos;
  using std::sin;

  DRAKE_DEMAND(points);
  DRAKE_DEMAND(points->empty());

  const Vector3<T> q = GetRodConfig(context);
  const T& x = q[0];
  const T& y = q[1];
  T cth = cos(q[2]), sth = sin(q[2]);

  // Get the two rod endpoint locations.
  const T half_len = get_rod_half_length();
  const Vector2<T> pa = CalcRodEndpoint(x, y, -1, cth, sth, half_len);
  const Vector2<T> pb = CalcRodEndpoint(x, y, +1, cth, sth, half_len);

  // If an endpoint touches the ground, add it as a contact point.
  if (pa[1] <= 0)
    points->push_back(pa);
  if (pb[1] <= 0)
    points->push_back(pb);
}

template <class T>
void Rod2D<T>::GetContactPointsTangentVelocities(
    const systems::Context<T>& context,
    const systems::State<T>& state,
    const std::vector<Vector2<T>>& points, std::vector<T>* vels) const {
  DRAKE_DEMAND(vels);
  const Vector3<T> q = GetRodConfig(state);
  const Vector3<T> v = GetRodVelocity(state);

  // Get necessary quantities.
  const Vector2<T> p_WRo = q.segment(0, 2);
  const Vector2<T> v_WRo = v.segment(0, 2);
  const T& w_WR = v[2];

  vels->resize(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    (*vels)[i] = CalcCoincidentRodPointVelocity(p_WRo, v_WRo, w_WR,
                                                points[i])[0];
  }
}

// Gets the row of a contact Jacobian matrix, given a point of contact, @p p,
// and projection direction (unit vector), @p dir. Aborts if @p dir is not a
// unit vector.
template <class T>
Vector3<T> Rod2D<T>::GetJacobianRow(const systems::Context<T>& context,
                                    const Vector2<T>& p,
                                    const Vector2<T>& dir) const {
  using std::abs;
  const double eps = 10 * std::numeric_limits<double>::epsilon();
  DRAKE_DEMAND(abs(dir.norm() - 1) < eps);

  // Get rod configuration variables.
  const Vector3<T> q = GetRodConfig(context);

  // Compute cross product of the moment arm (expressed in the world frame)
  // and the direction.
  const Vector2<T> r(p[0] - q[0], p[1] - q[1]);
  return Vector3<T>(dir[0], dir[1], cross2(r, dir));
}

// Gets the time derivative of a row of a contact Jacobian matrix, given a
// point of contact, @p p, and projection direction (unit vector), @p dir.
// Aborts if @p dir is not a unit vector.
template <class T>
Vector3<T> Rod2D<T>::GetJacobianDotRow(const systems::Context<T>& context,
                                       const Vector2<T>& p,
                                       const Vector2<T>& dir) const {
  using std::abs;
  const double eps = 10 * std::numeric_limits<double>::epsilon();
  DRAKE_DEMAND(abs(dir.norm() - 1) < eps);

  // Get rod state variables.
  const Vector3<T> q = GetRodConfig(context);
  const Vector3<T> v = GetRodVelocity(context);

  // Get the transformation of vectors from the rod frame to the
  // world frame and its time derivative.
  const T& theta = q[2];
  Eigen::Rotation2D<T> R(theta);

  // Get the vector from the rod center-of-mass to the contact point,
  // expressed in the rod frame.
  const Vector2<T> x = q.segment(0, 2);
  const Vector2<T> u = R.inverse() * (p - x);

  // Compute the translational velocity of the contact point.
  const Vector2<T> xdot = v.segment(0, 2);
  const T& thetadot = v[2];
  const Matrix2<T> Rdot = GetRotationMatrixDerivative(theta, thetadot);
  const Vector2<T> pdot = xdot + Rdot * u;

  // Compute cross product of the time derivative of the moment arm (expressed
  // in the world frame) and the direction.
  const Vector2<T> rdot(pdot[0] - v[0], pdot[1] - v[1]);
  return Vector3<T>(0, 0, cross2(rdot, dir));
}

template <class T>
Matrix2<T> Rod2D<T>::GetSlidingContactFrameToWorldTransform(
    const T& xaxis_velocity) const {
  // Note: the contact normal for the rod with the horizontal plane always
  // points along the world +y axis; the sliding tangent vector points along
  // either the +/-x (world) axis. The << operator populates the matrix row by
  // row, so
  // R_WC = | 0  ±1 |
  //        | 1   0 |
  // indicating that the contact normal direction (the +x axis in the contact
  // frame) is +y in the world frame and the contact tangent direction (more
  // precisely, the direction of sliding, the +y axis in the contact frame) is
  // ±x in the world frame.
  DRAKE_DEMAND(xaxis_velocity != 0);
  Matrix2<T> R_WC;
  // R_WC's elements match how they appear lexically: one row per line.
  R_WC << 0, ((xaxis_velocity > 0) ? 1 : -1),
          1, 0;
  return R_WC;
}

template <class T>
Matrix2<T> Rod2D<T>::GetNonSlidingContactFrameToWorldTransform() const {
  // Note: the contact normal for the rod with the horizontal plane always
  // points along the world +y axis; the non-sliding tangent vector always
  // points along the world +x axis. The << operator populates the matrix row by
  // row, so
  // R_WC = | 0 1 |
  //        | 1 0 |
  // indicating that the contact normal direction is (the +x axis in the contact
  // frame) is +y in the world frame and the contact tangent direction (the +y
  // axis in the contact frame) is +x in the world frame.
  Matrix2<T> R_WC;
  // R_WC's elements match how they appear lexically: one row per line.
  R_WC << 0, 1,
          1, 0;
  return R_WC;
}

template <class T>
void Rod2D<T>::CalcConstraintProblemData(
    const systems::Context<T>& context,
    const systems::State<T>& state,
    multibody::constraint::ConstraintAccelProblemData<T>* data)
    const {
  using std::abs;

  // Get the current configuration of the system.
  const VectorX<T> q = state.get_continuous_state().
      get_generalized_position().CopyToVector();

  // Get the contacts.
  const auto& contacts = get_contacts_used_in_force_calculations(state);

  // Create the vector of contact points.
  std::vector<Vector2<T>> points;
  for (int i = 0; i < static_cast<int>(contacts.size()); ++i) {
    // Get the point in the world frame.
    const long index = reinterpret_cast<long>(contacts[i].id);
    const Vector2<T> p0 = GetPointInWorldFrame(q, contact_candidates_[index]);
    points.push_back(p0);
  }

  // Determine the tangent velocities at the contact points.
  std::vector<T> tangent_vels;
  GetContactPointsTangentVelocities(context, state, points, &tangent_vels);

  // Do not use the complementarity problem solver by default.
  bool use_complementarity_problem_solver = false;

  // If any contacts are transitioning from sticking to sliding- the
  // complementarity problem solver is activated.
  for (int i = 0; i < static_cast<int>(contacts.size()); ++i) {
    if (contacts[i].sliding_type ==
        multibody::constraint::SlidingModeType::kTransitioning)
      use_complementarity_problem_solver = true;
  }

  // Set tangent velocities appropriately.
  for (int i = 0; i < static_cast<int>(contacts.size()); ++i) {
    // Check for a transitioning contact.
    const int contact_index = reinterpret_cast<long>(contacts[i].id);
    if (contacts[i].sliding_type ==
        multibody::constraint::SlidingModeType::kTransitioning) {
      tangent_vels[i] = 0.0;
    } else {
      if (GetStickingFrictionForceSlackWitness(
          contact_index, state)->is_enabled())   {
        tangent_vels[i] = 0.0;
      }
    }
  }

  // Call the primary method.
  CalcConstraintProblemData(context, points, tangent_vels, data);

  // Set the complementarity problem solver.
  data->use_complementarity_problem_solver = use_complementarity_problem_solver;
}

template <class T>
void Rod2D<T>::CalcConstraintProblemData(
    const systems::Context<T>& context,
    const std::vector<Vector2<T>>& points,
    const std::vector<T>& tangent_vels,
    multibody::constraint::ConstraintAccelProblemData<T>* data)
    const {
  using std::abs;
  DRAKE_DEMAND(data);
  DRAKE_DEMAND(points.size() == tangent_vels.size());

  // Set the inertia solver.
  data->solve_inertia = [this](const MatrixX<T>& m) {
    return solve_inertia(m);
  };

  // The normal and tangent spanning direction are unique.
  const Matrix2<T> R_wc = GetNonSlidingContactFrameToWorldTransform();
  const Vector2<T> contact_normal = R_wc.col(0);
  const Vector2<T> contact_tangent = R_wc.col(1);

  // Verify contact normal and tangent directions are as we expect.
  DRAKE_ASSERT(abs(contact_normal.dot(Vector2<T>(0, 1)) - 1) <
      std::numeric_limits<double>::epsilon());
  DRAKE_ASSERT(abs(contact_tangent.dot(Vector2<T>(1, 0)) - 1) <
      std::numeric_limits<double>::epsilon());

  // Get the set of contact points.
  const int nc = points.size();

  // Get the set of tangent velocities.
  const T sliding_vel_tol = GetSlidingVelocityTolerance();

  // Set sliding and non-sliding contacts.
  std::vector<int>& non_sliding_contacts = data->non_sliding_contacts;
  std::vector<int>& sliding_contacts = data->sliding_contacts;
  non_sliding_contacts.clear();
  sliding_contacts.clear();
  for (int i = 0; i < nc; ++i) {
    if (abs(tangent_vels[i]) < sliding_vel_tol) {
      non_sliding_contacts.push_back(i);
    } else {
      sliding_contacts.push_back(i);
    }
  }

  // Designate sliding and non-sliding contacts.
  const int num_sliding = sliding_contacts.size();
  const int num_non_sliding = non_sliding_contacts.size();

  // Set sliding and non-sliding friction coefficients.
  data->mu_sliding.setOnes(num_sliding) *= get_mu_coulomb();
  data->mu_non_sliding.setOnes(num_non_sliding) *= get_mu_static();

  // Set spanning friction cone directions (set to unity, because rod is 2D).
  data->r.resize(num_non_sliding);
  for (int i = 0; i < num_non_sliding; ++i)
    data->r[i] = 1;

  // Form the normal contact Jacobian (N).
  const int ngc = 3;        // Number of generalized coordinates / velocities.
  MatrixX<T> N(nc, ngc);
  for (int i = 0; i < nc; ++i)
    N.row(i) = GetJacobianRow(context, points[i], contact_normal);
  data->N_mult = [N](const VectorX<T>& w) -> VectorX<T> { return N * w; };

  // Form Ndot (time derivative of N) and compute Ndot * v.
  MatrixX<T> Ndot(nc, ngc);
  for (int i = 0; i < nc; ++i)
    Ndot.row(i) =  GetJacobianDotRow(context, points[i], contact_normal);
  const Vector3<T> v = GetRodVelocity(context);
  data->kN = Ndot * v;
  data->gammaN.setZero(nc);

  // Form the tangent directions contact Jacobian (F), its time derivative
  // (Fdot), and compute Fdot * v.
  const int nr = std::accumulate(data->r.begin(), data->r.end(), 0);
  MatrixX<T> F = MatrixX<T>::Zero(nr, ngc);
  MatrixX<T> Fdot = MatrixX<T>::Zero(nr, ngc);
  for (int i = 0; i < static_cast<int>(non_sliding_contacts.size()); ++i) {
    const int contact_index = non_sliding_contacts[i];
    F.row(i) = GetJacobianRow(context, points[contact_index], contact_tangent);
    Fdot.row(i) = GetJacobianDotRow(
        context, points[contact_index], contact_tangent);
  }
  data->kF = Fdot * v;
  data->F_mult = [F](const VectorX<T>& w) -> VectorX<T> { return F * w; };
  data->F_transpose_mult = [F](const VectorX<T>& w) -> VectorX<T> {
    return F.transpose() * w;
  };
  data->gammaF.setZero(nr);
  data->gammaE.setZero(non_sliding_contacts.size());

  // Form N - mu*Q (Q is sliding contact direction Jacobian).
  MatrixX<T> N_minus_mu_Q = N;
  Vector3<T> Qrow;
  for (int i = 0; i < static_cast<int>(sliding_contacts.size()); ++i) {
    const int contact_index = sliding_contacts[i];
    Matrix2<T> sliding_contract_frame = GetSlidingContactFrameToWorldTransform(
        tangent_vels[contact_index]);
    const auto& sliding_dir = sliding_contract_frame.col(1);
    Qrow = GetJacobianRow(context, points[contact_index], sliding_dir);
    N_minus_mu_Q.row(contact_index) -= data->mu_sliding[i] * Qrow;
  }
  data->N_minus_muQ_transpose_mult = [N_minus_mu_Q](const VectorX<T>& w) ->
      VectorX<T> { return N_minus_mu_Q.transpose() * w; };

  data->kL.resize(0);
  data->gammaL.resize(0);

  // Set external force vector.
  data->tau = ComputeExternalForces(context);
}

template <class T>
void Rod2D<T>::CalcImpactProblemData(
    const systems::Context<T>& context,
    const std::vector<Vector2<T>>& points,
    multibody::constraint::ConstraintVelProblemData<T>* data) const {
  using std::abs;
  DRAKE_DEMAND(data);

  // Setup the generalized inertia matrix.
  Matrix3<T> M;
  M << mass_, 0,     0,
       0,     mass_, 0,
       0,     0,     J_;

  // Get the generalized velocity.
  data->Mv = M * context.get_continuous_state().get_generalized_velocity().
      CopyToVector();

  // Set the inertia solver.
  data->solve_inertia = [this](const MatrixX<T>& m) {
    return solve_inertia(m);
  };

  // The normal and tangent spanning direction are unique for the rod undergoing
  // impact (i.e., unlike with non-impacting rigid contact equations, the
  // frame does not change depending on sliding velocity direction).
  const Matrix2<T> non_sliding_contact_frame =
      GetNonSlidingContactFrameToWorldTransform();
  const Vector2<T> contact_normal = non_sliding_contact_frame.col(0);
  const Vector2<T> contact_tan = non_sliding_contact_frame.col(1);

  // Get the set of contact points.
  const int num_contacts = points.size();

  // Set sliding and non-sliding friction coefficients.
  data->mu.setOnes(num_contacts) *= get_mu_coulomb();

  // Set spanning friction cone directions (set to unity, because rod is 2D).
  data->r.resize(num_contacts);
  for (int i = 0; i < num_contacts; ++i)
    data->r[i] = 1;

  // Form the normal contact Jacobian (N).
  const int num_generalized_coordinates = 3;
  MatrixX<T> N(num_contacts, num_generalized_coordinates);
  for (int i = 0; i < num_contacts; ++i)
    N.row(i) = GetJacobianRow(context, points[i], contact_normal);
  data->N_mult = [N](const VectorX<T>& w) -> VectorX<T> { return N * w; };
  data->N_transpose_mult = [N](const VectorX<T>& w) -> VectorX<T> {
    return N.transpose() * w; };
  data->kN.setZero(num_contacts);
  data->gammaN.setZero(num_contacts);

  // Form the tangent directions contact Jacobian (F).
  const int nr = std::accumulate(data->r.begin(), data->r.end(), 0);
  MatrixX<T> F(nr, num_generalized_coordinates);
  for (int i = 0; i < num_contacts; ++i) {
    F.row(i) = GetJacobianRow(context, points[i], contact_tan);
  }
  data->F_mult = [F](const VectorX<T>& w) -> VectorX<T> { return F * w; };
  data->F_transpose_mult = [F](const VectorX<T>& w) -> VectorX<T> {
    return F.transpose() * w;
  };
  data->kF.setZero(nr);
  data->gammaF.setZero(nr);
  data->gammaE.setZero(num_contacts);

  data->kL.resize(0);
  data->gammaL.resize(0);
}

// Models any impacts for the piecewise-DAE based system.
// @param[in,out] state the pre-impact state on entry, the post-impact state
//                on return.
template <class T>
void Rod2D<T>::ModelImpact(
    const systems::Context<T>& context, systems::State<T>* state) const {
  DRAKE_DEMAND(state);

  // Get the current configuration and velocity of the system.
  const VectorX<T> q = state->get_continuous_state().
      get_generalized_position().CopyToVector();
  systems::VectorBase<T>& qdot = state->get_mutable_continuous_state().
      get_mutable_generalized_velocity();

  // Determine the vector of contact points using the active witness functions.
  std::vector<Vector2<T>> points;
  for (int i = 0; i < static_cast<int>(contact_candidates_.size()); ++i) {
    if (GetNormalForceWitness(i, *state)->is_enabled() ||
        GetNormalVelWitness(i, *state)->is_enabled() ||
        GetNormalAccelWitness(i, *state)->is_enabled()) {
      points.push_back(GetPointInWorldFrame(q, i));
    }
  }

  // Call the primary method.
  const int ngv = 3;
  multibody::constraint::ConstraintVelProblemData<T> problem_data(ngv);
  CalcImpactProblemData(context, points, &problem_data);

  // Solve the impact problem and update the state.
  VectorX<T> cf, vplus;
  solver_.SolveImpactProblem(problem_data, &cf);
  solver_.ComputeGeneralizedVelocityChange(problem_data, cf, &vplus);
  vplus += qdot.CopyToVector();

  // Update qdot.
  qdot.SetFromVector(vplus);
}

template <class T>
int Rod2D<T>::GetContactArrayIndex(
    const systems::State<T>& state,
    int contact_candidate_index) const {
  // Get the vector of contacts.
  const std::vector<multibody::constraint::PointContact>& contacts =
      get_contacts_used_in_force_calculations(state);

  for (int i = 0; i < static_cast<int>(contacts.size()); ++i) {
    if (reinterpret_cast<long>(contacts[i].id) == contact_candidate_index)
      return i;
  }

  return -1; 
}

// Gets the row of a contact Jacobian matrix, given a point of contact, @p p,
// and projection direction (unit vector), @p dir. Aborts if @p dir is not a
// unit vector.
template <class T>
RodWitnessFunction<T>* Rod2D<T>::GetSignedDistanceWitness(
    int contact_index, const systems::State<T>& state) const {
  auto witnesses = state.get_abstract_state()
      .get_value(kSignedDistanceWitnessAbstractIndex)
      .template GetValue<std::vector<RodWitnessFunction<T>*>>();
  for (int i = 0; i < static_cast<int>(witnesses.size()); ++i)
    if (witnesses[i]->get_contact_index() == contact_index)
      return witnesses[i];

  throw std::logic_error("Missing signed distance witness.");
  return nullptr;
}

template <class T>
RodWitnessFunction<T>* Rod2D<T>::GetNormalAccelWitness(
    int contact_index, const systems::State<T>& state) const {
  auto witnesses = state.get_abstract_state()
      .get_value(kNormalAccelWitnessAbstractIndex)
      .template GetValue<std::vector<RodWitnessFunction<T>*>>();
  for (int i = 0; i < static_cast<int>(witnesses.size()); ++i)
    if (witnesses[i]->get_contact_index() == contact_index)
      return witnesses[i];

  throw std::logic_error("Missing normal acceleration witness.");
  return nullptr;
}

template <class T>
RodWitnessFunction<T>* Rod2D<T>::GetNormalVelWitness(
    int contact_index, const systems::State<T>& state) const {
  auto witnesses = state.get_abstract_state()
      .get_value(kNormalVelWitnessAbstractIndex)
      .template GetValue<std::vector<RodWitnessFunction<T>*>>();
  for (int i = 0; i < static_cast<int>(witnesses.size()); ++i)
    if (witnesses[i]->get_contact_index() == contact_index)
      return witnesses[i];

  throw std::logic_error("Missing normal velocity witness.");
  return nullptr;
}

/// The velocity tolerance for sliding.
template <class T>
RodWitnessFunction<T>* Rod2D<T>::GetStickingFrictionForceSlackWitness(
    int contact_index, const systems::State<T>& state) const {
  auto witnesses = state.get_abstract_state()
      .get_value(kStickingFrictionForceSlackWitnessAbstractIndex)
      .template GetValue<std::vector<RodWitnessFunction<T>*>>();
  for (int i = 0; i < static_cast<int>(witnesses.size()); ++i)
    if (witnesses[i]->get_contact_index() == contact_index)
      return witnesses[i];

  throw std::logic_error("Sticking friction force slack witness.");
  return nullptr;
}

template <class T>
RodWitnessFunction<T>* Rod2D<T>::GetNormalForceWitness(
    int contact_index, const systems::State<T>& state) const {
  auto witnesses = state.get_abstract_state()
      .get_value(kNormalForceWitnessAbstractIndex)
      .template GetValue<std::vector<RodWitnessFunction<T>*>>();
  for (int i = 0; i < static_cast<int>(witnesses.size()); ++i)
    if (witnesses[i]->get_contact_index() == contact_index)
      return witnesses[i];

  throw std::logic_error("Missing normal force witness.");
  return nullptr;
}

template <class T>
RodWitnessFunction<T>* Rod2D<T>::GetPosSlidingWitness(
    int contact_index, const systems::State<T>& state) const {
  auto witnesses = state.get_abstract_state()
      .get_value(kPosSlidingWitnessAbstractIndex)
      .template GetValue<std::vector<RodWitnessFunction<T>*>>();
  for (int i = 0; i < static_cast<int>(witnesses.size()); ++i)
    if (witnesses[i]->get_contact_index() == contact_index)
      return witnesses[i];

  throw std::logic_error("Missing positive sliding witness.");
  return nullptr;
}

template <class T>
RodWitnessFunction<T>* Rod2D<T>::GetNegSlidingWitness(
    int contact_index, const systems::State<T>& state) const {
  auto witnesses = state.get_abstract_state()
      .get_value(kNegSlidingWitnessAbstractIndex)
      .template GetValue<std::vector<RodWitnessFunction<T>*>>();
  for (int i = 0; i < static_cast<int>(witnesses.size()); ++i)
    if (witnesses[i]->get_contact_index() == contact_index)
      return witnesses[i];

  throw std::logic_error("Missing negative sliding witness.");
  return nullptr;
}

template <class T>
void Rod2D<T>::DoCalcUnrestrictedUpdate(
    const systems::Context<T>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<T>*>& events,
    systems::State<T>* state) const {
  using std::abs;

  // Get the current configuration of the system.
  const VectorX<T> q = state->get_continuous_state().
      get_generalized_position().CopyToVector();

  // Copy the state in the context into the state.
  state->CopyFrom(context.get_state());

  // Get the vector of contacts.
  std::vector<multibody::constraint::PointContact>& contacts =
      get_contacts_used_in_force_calculations(state);

  // Indicate there are no impacts.
  bool impacting = false;

  // Indicate that sliding, not-sliding, and separating states do not need to
  // be redetermined. Note that redetermination requires solving a
  // complementarity problem.
  bool redetermine_modes = false;

  // The vector of contacts to be removed from the force set.
  std::vector<int> contacts_to_remove;

  // Process all events.
  for (int i = 0; i < static_cast<int>(events.size()); ++i) {
    DRAKE_DEMAND(events[i]->has_attribute());

    // Get the attribute as a RodWitnessFunction type.
    auto const_witness = events[i]->get_attribute()->
        template GetValue<const RodWitnessFunction<T>*>();
    auto witness = const_cast<RodWitnessFunction<T>*>(const_witness);

    // Get the contact (candidate) index.
    const int contact_index = witness->get_contact_index();

    // Get the index of the contact in the contacts array.
    const int contact_array_index = GetContactArrayIndex(*state, contact_index);

    switch (witness->get_witness_function_type()) {
      case RodWitnessFunction<T>::WitnessType::kSignedDistance:  {
        // If the witness is a signed distance witness, the point should either
        // no longer be tracked or it should trigger
        // an impact.
        if (GetNormalVelWitness(contact_index, *state)->is_enabled() ||
            GetNormalAccelWitness(contact_index, *state)->is_enabled() ||
            GetNormalForceWitness(contact_index, *state)->is_enabled()) {
          // If the point is part of the force calculations, disable that.
          if (contact_array_index >= 0)
            contacts_to_remove.push_back(contact_array_index);

          // Deactivate all but signed distance function witness.
          GetNormalForceWitness(contact_index, *state)->set_enabled(false);
          GetNormalAccelWitness(contact_index, *state)->set_enabled(false);
          GetNormalVelWitness(contact_index, *state)->set_enabled(false);
          GetStickingFrictionForceSlackWitness(contact_index, *state)->
              set_enabled(false);
          GetPosSlidingWitness(contact_index, *state)->set_enabled(false);
          GetNegSlidingWitness(contact_index, *state)->set_enabled(false);
        } else {
          // Indicate that an impact is occurring. Activate one of
          // { NormalAccelWitness, NormalVelWitness } arbitrarily (lets the
          // impact approach know that this point of contact is being tracked,
          // but that contact is not part of force calculations).
          GetNormalVelWitness(contact_index, *state)->set_enabled(true);
          impacting = true;
        }
        break;
      }

      case RodWitnessFunction<T>::WitnessType::kNormalVel:  {
        // Once the normal velocity of a tracked contact (but one that force
        // is not being applied to) has decreased to zero, the contact needs to
        // re-enter the force calculations.

        // First disable the normal velocity witness.
        witness->set_enabled(false);

        // Add the contact.
        AddContactToForceCalculationSet(contact_index, context, state);

        // Redetermine the modes.
        redetermine_modes = true;

        break;
      }

      case RodWitnessFunction<T>::WitnessType::kNormalAccel:  {
        // Since the normal acceleration of a tracked contact (but one that
        // force is not being applied to) has decreased to zero, the contact
        // either needs to re-enter the set of force calculations again- if the
        // normal velocity at that contact is non-positive- or the
        // NormalVel witness for that contact needs to be activated
        // (implying that the velocity at the contact is currently strictly
        // positive).

        // Disable the normal acceleration witness. If the contact is
        // added into the force calculations, it's known that the acceleration
        // will be non-negative. Otherwise, the velocity is positive and we
        // want to identify when that will no longer be the case.
        witness->set_enabled(false);

        // Examine the normal velocity.
        const Vector2<T> v = CalcContactVelocity(context, contact_index);
        if (v[1] <= 0) {
          // Add the contact.
          AddContactToForceCalculationSet(contact_index, context, state);

          // Modes must now be redetermined.
          redetermine_modes = true;
        } else {
          GetNormalVelWitness(contact_index, *state)->set_enabled(true);
        }

        break;
      }

      case RodWitnessFunction<T>::WitnessType::kNormalForce:  {
        // Once the normal force crosses zero (it must be positive to enter
        // into the force calculations), it is time to remove the contact from
        // those used in force calculations.

        // TODO: Verify that the assumption above does not cause problems.
        // Could there be a problem with witness functions or infinite looping
        // Can a point get entered into the force calculations with a zero
        // normal force?
        DRAKE_DEMAND(contact_array_index >= 0 && !contacts.empty());

        // Mark the contact for removal.
        contacts_to_remove.push_back(contact_array_index);

        // Activate the normal acceleration witness function. Deactivate the
        // normal force witness and deactivate any other witnesses dependent
        // upon the contact being in the force set.
        witness->set_enabled(false);
        GetNormalAccelWitness(contact_index, *state)->set_enabled(true);
        GetPosSlidingWitness(contact_index, *state)->set_enabled(false);
        GetNegSlidingWitness(contact_index, *state)->set_enabled(false);
        GetStickingFrictionForceSlackWitness(contact_index, *state)->
            set_enabled(false);
        
        // Modes must now be redetermined.
        redetermine_modes = true;

        break;
      }

      case RodWitnessFunction<T>::WitnessType::kSlidingWitness:  {
        // NOTE: The sliding witnesses are purposefully not disabled here.
        // If the contact has been transitioning to proper sliding, they will
        // be used again to determine when the contact should transition to
        // non-sliding.

        // If the contact is not transitioning from not sliding to sliding,
        // it is coming to rest.
        if (contacts[contact_array_index].sliding_type !=
            multibody::constraint::SlidingModeType::kTransitioning) {
          // The contact is going to a non-sliding state.
          // Mark the contact as such.
          contacts[contact_array_index].sliding_type =
              multibody::constraint::SlidingModeType::kNotSliding;

          // Enable the sticking friction slack and disable the sliding dot
          // and sliding witnesses.
          GetStickingFrictionForceSlackWitness(contact_index, *state)
              ->set_enabled(true);
          GetPosSlidingWitness(contact_index, *state)->set_enabled(false);
          GetNegSlidingWitness(contact_index, *state)->set_enabled(false);

          // Modes must now be redetermined.
          redetermine_modes = true;
        }
        break;
      }

      case RodWitnessFunction<T>::WitnessType::kStickingFrictionForceSlack:  {
        // Disable the sliding dot / sticking friction forces slack witness
        // function.
        witness->set_enabled(false);

        // Enable the sliding velocity witnesses. 
        GetPosSlidingWitness(contact_index, *state)->set_enabled(true);
        GetNegSlidingWitness(contact_index, *state)->set_enabled(true);

        // Mark the contact as transitioning.
        contacts[contact_array_index].sliding_type =
            multibody::constraint::SlidingModeType::kTransitioning;

        // Modes must now be redetermined.
        redetermine_modes = true;

        break;
      }
    } 
  } 

  // Remove all contacts slated for removal.
  for (int i = contacts_to_remove.size() - 1; i >= 0; --i) {
    contacts[contacts_to_remove[i]] = contacts.back();
    contacts.pop_back();
  }

  // Note that if the rod were contacting, for example, a box rather than a
  // halfspace, we would need to deactivate contact points as they slid off of
  // the edge of the box.

  // Handle any impacts, then redetermine the mode for all contacts.
  if (impacting) {
    ModelImpact(context, state);
    SPDLOG_DEBUG(drake::log(), "Handling impact");

    // Get the sliding velocity tolerance.
    const T sliding_vel_tol = GetSlidingVelocityTolerance();

    // Get all points currently considered to be in contact.
    for (int i = 0; i < static_cast<int>(contact_candidates_.size()); ++i) {
      // Get the vertical velocity at the rod endpoint.
      const auto& v = CalcContactVelocity(*state, i);

      // The point is considered to be in contact if one of the following is
      // activated: the normal force witness, the normal velocity witness,
      // the normal acceleration witness.
      if (GetNormalForceWitness(i, *state)->is_enabled() ||
          GetNormalVelWitness(i, *state)->is_enabled() ||
          GetNormalAccelWitness(i, *state)->is_enabled()) {
        // The point was considered to both be in contact and be part of the
        // force calculations. Remove it from force calculations and activate
        // the normal velocity witness (and deactivate other witnesses) if the
        // velocity is strictly positive.
        if (v[1] > 0) {
          SPDLOG_DEBUG(drake::log(), "Disabling contact {} from force "
              "calculations", i);
          GetNormalVelWitness(i, *state)->set_enabled(true);
          GetNormalAccelWitness(i, *state)->set_enabled(false);
          GetStickingFrictionForceSlackWitness(i, *state)->set_enabled(false);
          GetPosSlidingWitness(i, *state)->set_enabled(false);
          GetNegSlidingWitness(i, *state)->set_enabled(false);

          // Remove the point if it is tracked.
          if (GetNormalForceWitness(i, *state)->is_enabled()) {
            contacts[i] = contacts.back();
            contacts.pop_back();

            // Disable the normal force witness.
            GetNormalForceWitness(i, *state)->set_enabled(false);

            // Need to process the newly designated element i.
            --i;
          }
        } else {
          // The contact point should potentially be accounted for in the force
          // calculations. Add it to the set if necessary.
          int contact_index = GetContactArrayIndex(*state, i);
          if (contact_index < 0) {
            AddContactToForceCalculationSet(i, context, state);
            contact_index = contacts.size() - 1;
            DRAKE_ASSERT(contact_index == GetContactArrayIndex(*state, i));
          }

          // Set the contact state to either sliding or not sliding.
          SPDLOG_DEBUG(drake::log(), "Contact velocity: {}", v[0]);
          contacts[contact_index].sliding_type = (abs(v[0]) > sliding_vel_tol) ?
              drake::multibody::constraint::SlidingModeType::kSliding :
              drake::multibody::constraint::SlidingModeType::kNotSliding;
        }
      } 
    }
  }

  // Now redetermine the acceleration level active set.
  if (impacting || redetermine_modes)
    DetermineContactModes(context, state);
}

// Redetermines the modes used for contacts at the acceleration level.
template <class T>
void Rod2D<T>::DetermineContactModes(
    const systems::Context<T>& context, systems::State<T>* state) const {
  using std::abs;

  SPDLOG_DEBUG(drake::log(), "Redetermining contact modes");

  // Get the vector of contacts.
  std::vector<multibody::constraint::PointContact>& contacts =
      get_contacts_used_in_force_calculations(state);

  // Redetermining the acceleration level active set is done by
  // solving the complementarity problem and then examining each contact.
  // If the acceleration at the contact is truly positive, then the contact
  // can be disabled.
  // 1. Populate problem data and solve the contact problem.
  const int ngv = 3;  // Number of rod generalized velocities.
  VectorX<T> cf;
  multibody::constraint::ConstraintAccelProblemData<T> problem_data(ngv);

  // Determine the residual tangential accelerations for sticking contacts,
  // and determine which contacts have no forces applied. The complementarity
  // solver *must* be used.
  VectorX<T> Lambda;
  CalcConstraintProblemData(context, *state, &problem_data);
  problem_data.use_complementarity_problem_solver = true;
  solver_.SolveConstraintProblem(problem_data, &cf, &Lambda);

  // Get some arrays that will be used repeatedly.
  const std::vector<int>& non_sliding_contacts =
      problem_data.non_sliding_contacts;
  DRAKE_ASSERT(std::is_sorted(non_sliding_contacts.begin(),
                              non_sliding_contacts.end()));

  // Determine number of contacts.
  const int num_sliding_contacts = std::accumulate(
      problem_data.sliding_contacts.begin(),
      problem_data.sliding_contacts.end(), 0);
  const int num_non_sliding_contacts = std::accumulate(
      problem_data.non_sliding_contacts.begin(),
      problem_data.non_sliding_contacts.end(), 0);
  const int num_contacts = num_sliding_contacts + num_non_sliding_contacts; 

  // Set the zero tolerance.
  const double zero_tol = 10 * std::numeric_limits<double>::epsilon();

  // Examine contact forces and residual accelerations.
  std::vector<int> contacts_to_remove;
  for (int i = 0; i < static_cast<int>(contact_candidates_.size()); ++i) {
    // Get the index in the contact array.
    int contact_array_index = GetContactArrayIndex(*state, i);
    if (contact_array_index < 0)
      continue;

    // Get whether the contact is active. 
    if (cf[contact_array_index] < 10 * std::numeric_limits<double>::epsilon()) {
      SPDLOG_DEBUG(drake::log(), "Removing contact from force calculations");

      // Mark the contact for removal.
      contacts_to_remove.push_back(contact_array_index);

      // Enable the normal acceleration witness.
      GetNormalAccelWitness(i, *state)->set_enabled(true);

      // Disable all other witnesses
      GetNormalVelWitness(i, *state)->set_enabled(false);
      GetNormalForceWitness(i, *state)->set_enabled(false);
      GetStickingFrictionForceSlackWitness(i, *state)->set_enabled(false);
      GetPosSlidingWitness(i, *state)->set_enabled(false);
      GetNegSlidingWitness(i, *state)->set_enabled(false);
    } else {
      // The contact is active. If the contact is not sliding, see whether
      // it needs to transition to sliding. Do this by examining whether the
      // tangential acceleration is clearly non-zero.
      if (contacts[contact_array_index].sliding_type ==
          multibody::constraint::SlidingModeType::kNotSliding) {
        const int non_sliding_index = std::distance(
          non_sliding_contacts.begin(),
          std::lower_bound(non_sliding_contacts.begin(),
                           non_sliding_contacts.end(),
                           contact_array_index));

        // TODO: Need a more numerically robust scheme.
        const T friction_force_slack = 
            problem_data.mu_non_sliding[non_sliding_index] *
            cf[non_sliding_index] - cf[num_contacts + non_sliding_index];
        if (abs(friction_force_slack) < zero_tol ||
            Lambda[non_sliding_index] > zero_tol) {
          SPDLOG_DEBUG(drake::log(), "Setting contact to 'transitioning'");
          contacts[contact_array_index].sliding_type =
              multibody::constraint::SlidingModeType::kTransitioning;
          GetPosSlidingWitness(i, *state)->set_enabled(true);
          GetNegSlidingWitness(i, *state)->set_enabled(true);
          GetStickingFrictionForceSlackWitness(i, *state)->set_enabled(false);
        }
      }
    }
  }
  
  // Remove all contacts slated for removal.
  for (int i = contacts_to_remove.size() - 1; i >= 0; --i) {
    contacts[contacts_to_remove[i]] = contacts.back();
    contacts.pop_back();
  }
}

// Adds a contact to those used for force calculations. 
template <class T>
void Rod2D<T>::AddContactToForceCalculationSet(
    int contact_index,
    const systems::Context<T>& context,
    systems::State<T>* state) const {

  // Get the vector of contacts.
  std::vector<multibody::constraint::PointContact>& contacts =
      get_contacts_used_in_force_calculations(state);

  // Add the contact.
  contacts.push_back(multibody::constraint::PointContact());
  DRAKE_ASSERT(contacts.size() <= 2);

  // Set the contact index.
  contacts.back().id = reinterpret_cast<void*>(contact_index);

  // See whether the contact is sliding or not.
  contacts.back().sliding_type = IsTangentVelocityZero(context, contact_index) ?
      multibody::constraint::SlidingModeType::kNotSliding :
      multibody::constraint::SlidingModeType::kSliding;

  // Activate the normal force witness.
  GetNormalForceWitness(contact_index, *state)->set_enabled(true);

  // If the contact is sliding, activate the sliding witnesses.
  if (contacts.back().sliding_type ==
      multibody::constraint::SlidingModeType::kSliding) {
    GetPosSlidingWitness(contact_index, *state)->set_enabled(true);
    GetNegSlidingWitness(contact_index, *state)->set_enabled(true);
    GetStickingFrictionForceSlackWitness(
        contact_index, *state)->set_enabled(false);
  } else {
    GetPosSlidingWitness(contact_index, *state)->set_enabled(false);
    GetNegSlidingWitness(contact_index, *state)->set_enabled(false);
    GetStickingFrictionForceSlackWitness(
        contact_index, *state)->set_enabled(true);
  }
}

// Gets the location of a point of contact in the world frame.
template <class T>
Vector2<T> Rod2D<T>::CalcContactLocationInWorldFrame(
    const systems::Context<T>& context,
    int index) const {
  return CalcContactLocationInWorldFrame(context.get_state(), index);
}

template <class T>
Vector2<T> Rod2D<T>::CalcContactLocationInWorldFrame(
    const systems::State<T>& state,
    int index) const {
  // The point of contact is x + R * u.
  const auto q = state.get_continuous_state().get_generalized_position().
      CopyToVector();
  const Vector2<T> x = q.segment(0, 2);
  const T& theta = q[2];
  const Eigen::Rotation2D<T> R(theta);
  return x + R * contact_candidates_[index];
}

// Calculates the velocity at a point of contact in the contact frame.
template <class T>
Vector2<T> Rod2D<T>::CalcContactVelocity(
    const systems::Context<T>& context,
    int index) const {
  return CalcContactVelocity(context.get_state(), index);
}

// Calculates the velocity at a point of contact in the contact frame.
template <class T>
Vector2<T> Rod2D<T>::CalcContactVelocity(
    const systems::State<T>& state,
    int index) const {
  // The point of contact is x + R * u, so its velocity is
  // xdot + Rdot * u * thetadot.
  const auto q = state.get_continuous_state().get_generalized_position().
      CopyToVector();
  const auto v = state.get_continuous_state().get_generalized_velocity().
      CopyToVector();
  const Vector2<T> xdot = v.segment(0, 2);
  const T& theta = q[2];
  const T& thetadot = v[2];
  const Matrix2<T> Rdot = GetRotationMatrixDerivative(theta, thetadot);
  return xdot + Rdot * contact_candidates_[index] * thetadot;
}

// Calculates the acceleration at a point of contact.
template <class T>
Vector2<T> Rod2D<T>::CalcContactAccel(
    const systems::Context<T>& context,
    int index) const {

  // TODO: Store rod parameters in the Context.

    // TODO(edrumwri): Speed this up (presumably) using caching. 

  // Populate problem data and solve the contact problem.
  const int ngv = 3;  // Number of rod generalized velocities.
  VectorX<T> cf;
  multibody::constraint::ConstraintAccelProblemData<T> problem_data(ngv);

  // Determine the new generalized acceleration. 
  VectorX<T> ga;
  CalcConstraintProblemData(context, context.get_state(), &problem_data);
  solver_.SolveConstraintProblem(problem_data, &cf);
  solver_.ComputeGeneralizedAcceleration(problem_data, cf, &ga);

  // The point of contact is x + R * u, so its velocity is
  // xdot + Rdot * u * thetadot, and its acceleration is
  // xddot + Rddot * u * thetadot + Rdot * u * thetaddot.
  const auto q = context.get_continuous_state().get_generalized_position().
      CopyToVector();
  const auto v = context.get_continuous_state().get_generalized_velocity().
      CopyToVector();
  const Vector2<T> xddot = ga.segment(0, 2);
  const T& theta = q[2];
  const T& thetadot = v[2];
  const T& thetaddot = ga[2];
  const Matrix2<T> Rdot = GetRotationMatrixDerivative(theta, thetadot);
  const Matrix2<T> Rddot = GetRotationMatrix2ndDerivative(theta, thetaddot);
  return xddot + Rdot * contact_candidates_[index] * thetaddot +
      Rddot * contact_candidates_[index] * thetadot;
}

// Gets the zero tolerance for tangent velocity for a point of contact.
template <class T>
bool Rod2D<T>::IsTangentVelocityZero(const systems::Context<T>& context,
                                     int contact_index) const {
  using std::abs;

  // TODO(edrumwri): Do a proper test that uses integrator tolerances and
  // distance of the c.o.m. from the contact point to determine a coherent
  // tolerance.
  const T tol = 1e-10;

  // Test tangent velocity.
  const Vector2<T> pdot = CalcContactVelocity(context, contact_index);
  return (abs(pdot[0]) < tol);
}

template <class T>
void Rod2D<T>::DoGetWitnessFunctions(const systems::Context<T>& context,
    std::vector<const systems::WitnessFunction<T>*>* witness_functions) const {
  DRAKE_DEMAND(witness_functions);
  witness_functions->clear();

  // If this is not a piecewise DAE system, return early.
  if (simulation_type_ != SimulationType::kPiecewiseDAE)
    return;

  for (int i = kContactAbstractIndex + 1; i < kNumAbstractIndices; ++i) {
    auto witnesses = context.get_abstract_state().
        get_value(i).template GetValue<std::vector<RodWitnessFunction<T>*>>();
    for (int j = 0; j < static_cast<int>(witnesses.size()); ++j)
      if (witnesses[j]->is_enabled())
        witness_functions->push_back(static_cast<systems::WitnessFunction<T>*>(witnesses[j]));
  }
}

template <class T>
void Rod2D<T>::ComputeAndCopyContactForceOut(
    const systems::Context<T>& context,
    systems::BasicVector<T>* contact_force_port_value) const {
  switch (simulation_type_) {
    case Rod2D<T>::SimulationType::kCompliant:  {
      const Vector3<T> Fc_Ro_W = CalcCompliantContactForces(context);
      contact_force_port_value->SetFromVector(Fc_Ro_W);
      break;
    }

    case Rod2D<T>::SimulationType::kPiecewiseDAE:  {
      // TODO(edrumwri): Cache this whole computation ASAP. 
      // Construct the problem data.
      const int ngc = 3;   // Number of generalized coordinates / velocities.
      multibody::constraint::ConstraintAccelProblemData<T> problem_data(ngc);
      CalcConstraintProblemData(context, context.get_state(), &problem_data);

      // Solve the constraint problem.
      VectorX<T> cf;
      solver_.SolveConstraintProblem(problem_data, &cf);

      // Compute the generalized force and set 'contact_force'.
      VectorX<T> gf;
      solver_.ComputeGeneralizedForceFromConstraintForces(
          problem_data, cf, &gf);
      contact_force_port_value->SetFromVector(gf);
      break;
    }

    case Rod2D<T>::SimulationType::kTimeStepping:  {
      // TODO(edrumwri): Cache this computation ASAP. 
      // Must solve the time stepping problem and then compute the non-impulsive
      // contact forces.
      // Get the configuration and velocity variables.
      const systems::BasicVector<T>& state = context.get_discrete_state(0);
      const auto& q = state.get_value().template segment<3>(0);
      Vector3<T> v = state.get_value().template segment<3>(3);

      // Get the external forces.
      const Vector3<T> fext = ComputeExternalForces(context);

      // Construct the problem data.
      const int ngc = 3;      // Number of generalized coords / velocities.
      multibody::constraint::ConstraintVelProblemData<T> problem_data(ngc);

      // Compute time stepping problem data.
      v += dt_ * (GetInverseInertiaMatrix() * fext);
      ComputeTimeSteppingProblemData(q, v, &problem_data);

      // Solve the impact problem and compute the generalized impulse (gj).
      VectorX<T> cf, gj;
      solver_.SolveImpactProblem(problem_data, &cf);
      solver_.ComputeGeneralizedImpulseFromConstraintImpulses(
          problem_data, cf, &gj);
      contact_force_port_value->SetFromVector(gj / dt_);
      break;
    }
  }
}

template <typename T>
void Rod2D<T>::CopyStateOut(const systems::Context<T>& context,
                            systems::BasicVector<T>* state_port_value) const {
  // Output port value is just the continuous or discrete state.
  const VectorX<T> state = (simulation_type_ == SimulationType::kTimeStepping)
                               ? context.get_discrete_state(0).CopyToVector()
                               : context.get_continuous_state().CopyToVector();
  state_port_value->SetFromVector(state);
}

template <typename T>
void Rod2D<T>::CopyPoseOut(
    const systems::Context<T>& context,
    systems::rendering::PoseVector<T>* pose_port_value) const {
  const VectorX<T> state = (simulation_type_ == SimulationType::kTimeStepping)
                               ? context.get_discrete_state(0).CopyToVector()
                               : context.get_continuous_state().CopyToVector();
  ConvertStateToPose(state, pose_port_value);
}

template <class T>
void Rod2D<T>::ComputeTimeSteppingProblemData(
    const Vector3<T>& q,
    const Vector3<T>& v,
    multibody::constraint::ConstraintVelProblemData<T>* problem_data) const {
  const T& x = q(0);
  const T& y = q(1);
  const T& theta = q(2);

  // Determine ERP and CFM.
  const double erp = get_erp();
  const double cfm = get_cfm();

  // Two contact points, corresponding to the two rod endpoints, are always
  // used, regardless of whether any part of the rod is in contact with the
  // halfspace. This practice is standard in time stepping approaches with
  // constraint stabilization. See:
  // M. Anitescu and G. Hart. A Constraint-Stabilized Time-Stepping Approach
  // for Rigid Multibody Dynamics with Joints, Contact, and Friction. Intl.
  // J. for Numerical Methods in Engr., 60(14), 2004.
  const int nc = 2;

  // Find left and right end point locations.
  const T stheta = sin(theta), ctheta = cos(theta);
  const Vector2<T> left =
      CalcRodEndpoint(x, y, -1, ctheta, stheta, half_length_);
  const Vector2<T> right =
      CalcRodEndpoint(x, y,  1, ctheta, stheta, half_length_);

  // Set up the contact normal and tangent (friction) direction Jacobian
  // matrices. These take the form:
  //     | 0 1 n1 |        | 1 0 f1 |
  // N = | 0 1 n2 |    F = | 1 0 f2 |
  // where n1, n2/f1, f2 are the moment arm induced by applying the
  // force at the given contact point along the normal/tangent direction.
  auto& N = N_;
  auto& F = F_;
  N(0, 0) = N(1, 0) = 0;
  N(0, 1) = N(1, 1) = 1;
  N(0, 2) = (left[0]  - x);
  N(1, 2) = (right[0] - x);
  F(0, 0) = F(1, 0) = 1;
  F(0, 1) = F(1, 1) = 0;
  F(0, 2) = -(left[1]  - y);
  F(1, 2) = -(right[1] - y);

  // Set the number of tangent directions.
  problem_data->r = { 1, 1 };

  // Get the total number of tangent directions.
  const int nr = std::accumulate(
      problem_data->r.begin(), problem_data->r.end(), 0);

  // Set the coefficients of friction.
  problem_data->mu.setOnes(nc) *= get_mu_coulomb();

  // Set up the operators.
  problem_data->N_mult = [&N](const VectorX<T>& vv) -> VectorX<T> {
    return N * vv;
  };
  problem_data->N_transpose_mult = [&N](const VectorX<T>& vv) -> VectorX<T> {
    return N.transpose() * vv;
  };
  problem_data->F_transpose_mult = [&F](const VectorX<T>& vv) -> VectorX<T> {
    return F.transpose() * vv;
  };
  problem_data->F_mult = [&F](const VectorX<T>& vv) -> VectorX<T> {
    return F * vv;
  };
  problem_data->solve_inertia = [this](const MatrixX<T>& mm) -> MatrixX<T> {
    return GetInverseInertiaMatrix() * mm;
  };

  // Update the generalized velocity vector with discretized external forces
  // (expressed in the world frame).
  problem_data->Mv = GetInertiaMatrix() * v;

  // Set stabilization parameters.
  problem_data->kN.resize(nc);
  problem_data->kN[0] = erp * left[1] / dt_;
  problem_data->kN[1] = erp * right[1] / dt_;
  problem_data->kF.setZero(nr);

  // Set regularization parameters.
  problem_data->gammaN.setOnes(nc) *= cfm;
  problem_data->gammaF.setOnes(nr) *= cfm;
  problem_data->gammaE.setOnes(nc) *= cfm;
}

/// Integrates the Rod 2D example forward in time using a
/// half-explicit time stepping scheme.
template <class T>
void Rod2D<T>::DoCalcDiscreteVariableUpdates(
    const systems::Context<T>& context,
    const std::vector<const systems::DiscreteUpdateEvent<T>*>&,
    systems::DiscreteValues<T>* discrete_state) const {
  using std::sin;
  using std::cos;

  // Get the configuration and velocity variables.
  const systems::BasicVector<T>& state = context.get_discrete_state(0);
  const auto& q = state.get_value().template segment<3>(0);
  Vector3<T> v = state.get_value().template segment<3>(3);

  // Construct the problem data.
  const int ngc = 3;      // Number of generalized coords / velocities.
  multibody::constraint::ConstraintVelProblemData<T> problem_data(ngc);

  // Get the external forces and update v.
  const Vector3<T> fext = ComputeExternalForces(context);
  v += dt_ * (GetInverseInertiaMatrix() * fext);

  // Compute time stepping problem data.
  ComputeTimeSteppingProblemData(q, v, &problem_data);

  // Solve the constraint problem.
  VectorX<T> cf;
  solver_.SolveImpactProblem(problem_data, &cf);

  // Compute the updated velocity.
  VectorX<T> delta_v;
  solver_.ComputeGeneralizedVelocityChange(problem_data, cf, &delta_v);

  // Compute the new velocity. Note that external forces have already been
  // incorporated into v.
  VectorX<T> vplus = v + delta_v;

  // Compute the new position using an "explicit" update.
  VectorX<T> qplus = q + vplus * dt_;

  // Set the new discrete state.
  systems::BasicVector<T>& new_state = discrete_state->get_mutable_vector(0);
  new_state.get_mutable_value().segment(0, 3) = qplus;
  new_state.get_mutable_value().segment(3, 3) = vplus;
}

// Returns the generalized inertia matrix computed about the
// center of mass of the rod and expressed in the world frame.
template <class T>
Matrix3<T> Rod2D<T>::GetInertiaMatrix() const {
  Matrix3<T> M;
  M << mass_, 0,     0,
       0,     mass_, 0,
       0,     0,     J_;
  return M;
}

// Returns the inverse of the generalized inertia matrix computed about the
// center of mass of the rod and expressed in the world frame.
template <class T>
Matrix3<T> Rod2D<T>::GetInverseInertiaMatrix() const {
  Matrix3<T> M_inv;
  M_inv << 1.0 / mass_, 0,           0,
           0,           1.0 / mass_, 0,
           0,           0,           1.0 / J_;
  return M_inv;
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

template <class T>
bool Rod2D<T>::IsImpacting(const systems::Context<T>& context) const {
  using std::sin;
  using std::cos;

  // Note: we do not consider modes here.
  // TODO(edrumwri): Handle two-point contacts.

  // Get state data necessary to compute the point of contact.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // Get the height of the lower rod endpoint.
  const T ctheta = cos(theta), stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : (stheta < 0) ? 1 : 0;
  const T cy = y + k * stheta * half_length_;

  // If rod endpoint is not touching, there is no impact.
  if (cy >= 10 * std::numeric_limits<double>::epsilon()) return false;

  // Compute the velocity at the point of contact.
  const T cydot = ydot + k * ctheta * half_length_ * thetadot;

  // Verify that the rod is not impacting.
  return (cydot < -10 * std::numeric_limits<double>::epsilon());
}

// Computes the accelerations of the rod center of mass for the rod, both
// in compliant contact and contact-free.
template <typename T>
void Rod2D<T>::CalcAccelerationsCompliantContactAndBallistic(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Obtain the structure we need to write into (ds=d/dt state).
  systems::VectorBase<T>& ds = derivatives->get_mutable_vector();

  // Get external applied force (a spatial force at Ro, in W).
  const auto Fext_Ro_W = ComputeExternalForces(context);

  // Calculate contact forces (also spatial force at Ro, in W).
  const Vector3<T> Fc_Ro_W = CalcCompliantContactForces(context);
  const Vector3<T> F_Ro_W = Fc_Ro_W + Fext_Ro_W;  // Total force.

  // Second three derivative components are acceleration due to gravity,
  // contact forces, and non-gravitational, non-contact external forces.
  ds.SetAtIndex(3, F_Ro_W[0]/mass_);
  ds.SetAtIndex(4, F_Ro_W[1]/mass_);
  ds.SetAtIndex(5, F_Ro_W[2]/J_);
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
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& xdot = state.GetAtIndex(3);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // Obtain the structure we need to write into.
  systems::VectorBase<T>& f = derivatives->get_mutable_vector();

  // First three derivative components are xdot, ydot, thetadot.
  f.SetAtIndex(0, xdot);
  f.SetAtIndex(1, ydot);
  f.SetAtIndex(2, thetadot);

  // Compute the velocity derivatives (accelerations).
  if (simulation_type_ == SimulationType::kCompliant) {
    return CalcAccelerationsCompliantContactAndBallistic(context, derivatives);
  } else {
    // Piecewise DAE approach: it is assumed that the set of active constraints
    // is valid. The witness functions are responsible for detecting when the
    // set of active constraints is no longer valid, and the unrestricted
    // update function is responsible for changing the set of active
    // constraints.

    // Construct the problem data.
    const int ngc = 3;   // Number of generalized coordinates / velocities.
    multibody::constraint::ConstraintAccelProblemData<T> problem_data(ngc);
    CalcConstraintProblemData(context, context.get_state(), &problem_data);

    // Solve the constraint problem.
    VectorX<T> cf;
    solver_.SolveConstraintProblem(problem_data, &cf);

    // Get the generalized velocity.
    const Vector3<T> gv = GetRodVelocity(context);

    // Compute and save the acceleration at the center-of-mass.
    VectorX<T> ga;
    solver_.ComputeGeneralizedAcceleration(problem_data, cf, &ga);
    derivatives->get_mutable_generalized_position().SetFromVector(gv);
    derivatives->get_mutable_generalized_velocity().SetFromVector(ga);
  }
}

/// Allocates the abstract state (for piecewise DAE systems).
template <typename T>
std::unique_ptr<systems::AbstractValues> Rod2D<T>::AllocateAbstractState()
    const {
  // TODO: Make the sliding tolerance a function of accuracy.
  const double slip_tol = 1e-4;

  if (simulation_type_ == SimulationType::kPiecewiseDAE) {
    // Piecewise DAE approach needs multiple abstract variables: one vector
    // of contact indices used in force calculations and one vector for each
    // type of witness function. 
    std::vector<std::unique_ptr<systems::AbstractValue>> abstract_data;

    abstract_data.push_back(
        std::make_unique<systems::Value<std::vector<
        multibody::constraint::PointContact>>>());

    // Create all of the witness function vectors.
    for (int i = kContactAbstractIndex + 1; i < kNumAbstractIndices; ++i) {
      abstract_data.push_back(
          std::make_unique<
          systems::Value<std::vector<RodWitnessFunction<T>*>>>());
      auto& witnesses = abstract_data.back()->template GetMutableValue<std::vector<RodWitnessFunction<T>*>>();
     
      switch (i) {
        case kSignedDistanceWitnessAbstractIndex:
          witnesses.push_back(new SignedDistanceWitness<T>(this, 0));
          witnesses.push_back(new SignedDistanceWitness<T>(this, 1));
          break;

        case kNormalAccelWitnessAbstractIndex:
          witnesses.push_back(new NormalAccelWitness<T>(this, 0));
          witnesses.push_back(new NormalAccelWitness<T>(this, 1));
          break;

        case kNormalVelWitnessAbstractIndex:
          witnesses.push_back(new NormalVelWitness<T>(this, 0));
          witnesses.push_back(new NormalVelWitness<T>(this, 1));
          break;

        case kStickingFrictionForceSlackWitnessAbstractIndex:
          witnesses.push_back(new StickingFrictionForcesSlackWitness<T>(this, 0));
          witnesses.push_back(new StickingFrictionForcesSlackWitness<T>(this, 1));
          break;

        case kNormalForceWitnessAbstractIndex:
          witnesses.push_back(new NormalForceWitness<T>(this, 0));
          witnesses.push_back(new NormalForceWitness<T>(this, 1));
          break;

        case kNegSlidingWitnessAbstractIndex:
          witnesses.push_back(new SlidingWitness<T>(this, 0, false, slip_tol));
          witnesses.push_back(new SlidingWitness<T>(this, 1, false, slip_tol));
          break;

        case kPosSlidingWitnessAbstractIndex:
          witnesses.push_back(new SlidingWitness<T>(this, 0, true, slip_tol));
          witnesses.push_back(new SlidingWitness<T>(this, 1, true, slip_tol));
          break;
      }
    }

    return std::make_unique<systems::AbstractValues>(std::move(abstract_data));
  } else {
    // Time stepping and compliant approaches need no abstract variables.
    return std::make_unique<systems::AbstractValues>();
  }
}

/// Sets the rod to a 45 degree angle with the halfspace and positions the rod
/// such that it and the halfspace are touching at exactly one point of contact.
template <typename T>
void Rod2D<T>::SetDefaultState(const systems::Context<T>&,
                               systems::State<T>* state) const {
  using std::sqrt;
  using std::sin;

  // Initial state corresponds to an inconsistent configuration for piecewise
  // DAE.
  const double half_len = get_rod_half_length();
  VectorX<T> x0(6);
  const double r22 = sqrt(2) / 2;
  x0 << 0, half_len * r22, M_PI / 4.0, -1, 0, 0;  // Initial state.
  if (simulation_type_ == SimulationType::kTimeStepping) {
    state->get_mutable_discrete_state().get_mutable_vector(0)
        .SetFromVector(x0);
  } else {
    // Continuous variables.
    state->get_mutable_continuous_state().SetFromVector(x0);

    // Set abstract variables for piecewise DAE approach.
    if (simulation_type_ == SimulationType::kPiecewiseDAE) {
      // Initialize the vector of contact points.
      auto& contacts = get_contacts_used_in_force_calculations(state);

      // Indicate that the rod is in the single contact sliding mode.
      contacts.resize(1);

      // First contact candidate is Rl in Rod Frame (see class documentation);
      // in the default rod configuration, Rl contacts the half-space and is
      // sliding.
      contacts.front().sliding_type =
          multibody::constraint::SlidingModeType::kSliding;
      contacts.front().id = reinterpret_cast<void*>(0);

      // Enable both signed distance witnesses.
      GetSignedDistanceWitness(kLeft, *state)->set_enabled(true);
      GetSignedDistanceWitness(kRight, *state)->set_enabled(true);

      // Enable the normal force witness and the sliding witnesses for the
      // left (sliding) contact.
      GetNormalForceWitness(kLeft, *state)->set_enabled(true);
      GetPosSlidingWitness(kLeft, *state)->set_enabled(true);
      GetNegSlidingWitness(kLeft, *state)->set_enabled(true);

      // Disable all other witness functions.
      GetNormalVelWitness(kLeft, *state)->set_enabled(false);
      GetNormalVelWitness(kRight, *state)->set_enabled(false);
      GetNormalAccelWitness(kLeft, *state)->set_enabled(false);
      GetNormalAccelWitness(kRight, *state)->set_enabled(false);
      GetPosSlidingWitness(kRight, *state)->set_enabled(false);
      GetNegSlidingWitness(kRight, *state)->set_enabled(false);
      GetNormalForceWitness(kRight, *state)->set_enabled(false);
      GetStickingFrictionForceSlackWitness(kLeft, *state)->
          set_enabled(false);
      GetStickingFrictionForceSlackWitness(kRight, *state)->
          set_enabled(false);
    }
  }
}

// Sets the contact candidates.
template <class T>
void Rod2D<T>::SetContactCandidates() {
  contact_candidates_.resize(2);

  // First point is Rl in Rod Frame (see class documentation).
  contact_candidates_.front() = Vector2<T>(-get_rod_half_length(), 0);

  // Second point is Rr in Rod Frame.
  contact_candidates_.back() = Vector2<T>(get_rod_half_length(), 0);   
}

// Converts a state vector to a rendering PoseVector.
template <class T>
void Rod2D<T>::ConvertStateToPose(const VectorX<T>& state,
                                  systems::rendering::PoseVector<T>* pose) {
  // Converts the configuration of the rod to a pose, accounting for both
  // the change to a y+ up coordinate system and the fact that Drake's cylinder
  // up-direction defaults to +z.
  const T theta = state[2] + M_PI_2;
  pose->set_translation(Eigen::Translation<T, 3>(state[0], 0, state[1]));
  const Vector3<T> y_axis{0, 1, 0};
  const Eigen::AngleAxis<T> rotation(theta, y_axis);
  pose->set_rotation(Eigen::Quaternion<T>(rotation));
}

}  // namespace rod2d
}  // namespace examples
}  // namespace drake
