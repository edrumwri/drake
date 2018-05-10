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
Rod2D<T>::Rod2D(SystemType system_type, double dt)
    : system_type_(system_type), dt_(dt) {
  // Verify that the simulation approach is either piecewise DAE or
  // compliant ODE.
  if (system_type == SystemType::kDiscretized) {
    if (dt <= 0.0)
      throw std::logic_error(
          "Discretization approach must be constructed using"
          " strictly positive step size.");

    // Discretization approach requires three position variables and
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

    // Piecewise DAE approaches must declare witness functions.
    if (system_type_ == SystemType::kPiecewiseDAE) {
      for (int i = kLeft; i <= kRight; ++i) {
        // Declare the signed distance witness function.
        signed_distance_witnesses_[i] = this->DeclareWitnessFunction(
            AppendEndpoint("signed distance", i),
            systems::WitnessFunctionDirection::kCrossesZero,
            [this, i](const systems::Context<T>& context) -> T {
              return this->CalcSignedDistance(context, i);
            },
            systems::UnrestrictedUpdateEvent<T>());

        // Declare the normal acceleration witness function. 
        normal_acceleration_witnesses_[i] = this->DeclareWitnessFunction(
            AppendEndpoint("normal acceleration", i),
            systems::WitnessFunctionDirection::kPositiveThenNonPositive,
            [this, i](const systems::Context<T>& context) -> T {
              return this->CalcNormalAcceleration(context, i);
            },
            systems::UnrestrictedUpdateEvent<T>());

        // Declare the normal force witness function. 
        normal_force_witnesses_[i] = this->DeclareWitnessFunction(
            AppendEndpoint("normal force", i),
            systems::WitnessFunctionDirection::kPositiveThenNonPositive,
            [this, i](const systems::Context<T>& context) -> T {
              return this->CalcNormalForce(context, i);
            },
            systems::UnrestrictedUpdateEvent<T>());

        // Declare the normal velocity witness function.
        normal_velocity_witnesses_[i] = this->DeclareWitnessFunction(
            AppendEndpoint("normal velocity", i),
            systems::WitnessFunctionDirection::kPositiveThenNonPositive,
            [this, i](const systems::Context<T>& context) -> T {
              return this->CalcNormalVelocity(context, i);
            },
            systems::UnrestrictedUpdateEvent<T>());

        // Declare the positive and negative sliding velocity witness functions.
        positive_sliding_witnesses_[i] = this->DeclareWitnessFunction(
            AppendEndpoint("positive sliding", i),
            systems::WitnessFunctionDirection::kCrossesZero,
            [this, i](const systems::Context<T>& context) -> T {
              return this->CalcSlidingVelocity(context, i, true /* along +x */);
            },
            systems::UnrestrictedUpdateEvent<T>());
        negative_sliding_witnesses_[i] = this->DeclareWitnessFunction(
            AppendEndpoint("negative sliding", i),
            systems::WitnessFunctionDirection::kCrossesZero,
            [this, i](const systems::Context<T>& context) -> T {
              return this->CalcSlidingVelocity(
                  context, i, false /* along -x */);
            },
            systems::UnrestrictedUpdateEvent<T>());

        // Declare the sticking friction force slack witness function.
        sticking_friction_force_slack_witnesses_[i] =
            this->DeclareWitnessFunction(
                AppendEndpoint("sticking friction force slack", i),
                systems::WitnessFunctionDirection::kPositiveThenNonPositive,
                [this, i](const systems::Context<T>& context) -> T {
                    return this->CalcStickingFrictionForceSlack(context, i);
                },
                systems::UnrestrictedUpdateEvent<T>());

        // Set up the mapping here from witnesses to witness function type
        // and endpoint.
        EndpointID endpoint = static_cast<EndpointID>(i);
        witness_function_info_[signed_distance_witnesses_[i].get()] =
            std::make_pair(kSignedDistance, endpoint);
        witness_function_info_[normal_acceleration_witnesses_[i].get()] =
            std::make_pair(kNormalAcceleration, endpoint);
        witness_function_info_[normal_force_witnesses_[i].get()] =
            std::make_pair(kNormalForce, endpoint);
        witness_function_info_[normal_velocity_witnesses_[i].get()] =
            std::make_pair(kNormalVelocity, endpoint);
        witness_function_info_[positive_sliding_witnesses_[i].get()] =
            std::make_pair(kPositiveSliding, endpoint);
        witness_function_info_[negative_sliding_witnesses_[i].get()] =
            std::make_pair(kNegativeSliding, endpoint);
        witness_function_info_[
            sticking_friction_force_slack_witnesses_[i].get()] =
            std::make_pair(kStickingFrictionForceSlack, endpoint);
      }
    }
  }

  this->DeclareInputPort(systems::kVectorValued, 3);
  state_output_port_ = &this->DeclareVectorOutputPort(
      systems::BasicVector<T>(6), &Rod2D::CopyStateOut);
  pose_output_port_ = &this->DeclareVectorOutputPort(&Rod2D::CopyPoseOut);

  // Create the contact candidates.
  SetContactCandidates();
}

template <class T>
std::string Rod2D<T>::AppendEndpoint(
    const std::string& witness, int endpoint) {
  DRAKE_DEMAND(endpoint == kLeft || endpoint == kRight);
  if (endpoint == kLeft)
    return witness + " (L)";
  return witness + " (R)";
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

/// Gets the abstract state corresponding to the rod endpoints used in force
/// calculations.
template <class T>
std::vector<PointContact>& Rod2D<T>::get_endpoints_used_in_force_calculations(
    systems::State<T>* state) const {
  return state->get_mutable_abstract_state()
      .get_mutable_value(kContactAbstractIndex)
      .template GetMutableValue<std::vector<PointContact>>();
}

template <class T>
const std::vector<PointContact>&
Rod2D<T>::get_endpoints_used_in_force_calculations(
    const systems::State<T>& state) const {
  return state.get_abstract_state()
      .get_value(kContactAbstractIndex).
          template GetValue<std::vector<PointContact>>();
}

template <class T>
const typename Rod2D<T>::ActiveRodWitnesses& Rod2D<T>::GetActiveWitnesses(
    int endpoint_index,
    const systems::State<T>& state) const {
  return state.get_abstract_state()
      .get_value(kContactAbstractIndex + endpoint_index + 1).
          template GetValue<typename Rod2D<T>::ActiveRodWitnesses>();
}

template <class T>
typename Rod2D<T>::ActiveRodWitnesses& Rod2D<T>::GetActiveWitnesses(
    int endpoint_index,
    systems::State<T>* state) const {
  return state->get_mutable_abstract_state()
      .get_mutable_value(kContactAbstractIndex + endpoint_index + 1).
          template GetMutableValue<typename Rod2D<T>::ActiveRodWitnesses>();
}

template <class T>
T Rod2D<T>::CalcSignedDistance(
    const systems::Context<T>& context, int contact_index) const {
  // Verify the system is modeled as a piecewise DAE.
  DRAKE_DEMAND(system_type_ == SystemType::kPiecewiseDAE);

  // Get the relevant parts of the state.
  const Vector3<T> q = context.get_continuous_state().
      get_generalized_position().CopyToVector();

  // Get the contact candidate in the world frame.
  const Vector2<T>& u = get_contact_candidate(contact_index);
  const Vector2<T> p = GetPointInWorldFrame(q, u);

  // Return the vertical location.
  return p[1];
}

template <class T>
T Rod2D<T>::CalcNormalAcceleration(
    const systems::Context<T>& context, int contact_index) const {
  // Verify the system is simulated using piecewise DAE.
  DRAKE_DEMAND(system_type_ == SystemType::kPiecewiseDAE);

  // Return the vertical acceleration at the tracked point.
  const Vector2<T> vdot = CalcContactAccel(context, contact_index);
  return vdot[1];
}

template <class T>
T Rod2D<T>::CalcNormalForce(
    const systems::Context<T>& context, int contact_index) const {
  // Verify the system is simulated using piecewise DAE.
  DRAKE_DEMAND(system_type_ == SystemType::kPiecewiseDAE);

  // TODO(edrumwri): Speed this up (presumably) using caching.

  // Get the force index of the contact.
  const int force_index = GetActiveSetArrayIndex(
      context.get_state(), contact_index);
  DRAKE_DEMAND(force_index >= 0);

  // Populate problem data and solve the contact problem.
  const int ngv = 3;  // Number of rod generalized velocities.
  VectorX<T> cf;
  multibody::constraint::ConstraintAccelProblemData<T> problem_data(ngv);
  CalcConstraintProblemData(context, context.get_state(), &problem_data);
  problem_data.use_complementarity_problem_solver = false;
  solver_.SolveConstraintProblem(problem_data, &cf);

  // Return the normal force. A negative value means that the force has
  // become tensile, which violates the compressivity constraint.
  return cf[force_index];
}

template <class T>
T Rod2D<T>::CalcNormalVelocity(
    const systems::Context<T>& context, int contact_index) const {
  // Verify the system is simulated using piecewise DAE.
  DRAKE_DEMAND(system_type_ == SystemType::kPiecewiseDAE);

  // Return the vertical velocity at the tracked point.
  const Vector2<T> v = CalcContactVelocity(context, contact_index);
  return v[1];
}

/// Determines whether the tangent velocity crosses the sliding velocity
/// threshold. This witness is used for two purposes: (1) it determines when
/// a contact has moved from non-sliding-to-sliding-transition to proper sliding
/// and (2) it determines when a contact has moved from sliding to non-sliding.
template <class T>
T Rod2D<T>::CalcSlidingVelocity(const systems::Context<T>& context,
                                int contact_index, bool positive) const {
  // Get the sliding velocity threshold.
  const double sliding_threshold = GetSlidingVelocityThreshold(context);

  // Verify the system is simulated using piecewise DAE.
  DRAKE_DEMAND(system_type_ == SystemType::kPiecewiseDAE);

  // Get the contact information.
  const int active_set_array_index = GetActiveSetArrayIndex(
      context.get_state(), contact_index);
  const auto& contact = get_endpoints_used_in_force_calculations(
      context.get_state())[active_set_array_index];

  // Verify rod is undergoing sliding contact at the specified index.
  DRAKE_DEMAND(contact.sliding_type == SlidingModeType::kSliding ||
               contact.sliding_type == SlidingModeType::kTransitioning);

  // Compute the translational velocity at the point of contact.
  const Vector2<T> pdot = CalcContactVelocity(context,
                                              contact_index);

  // Return the tangent velocity.
  if (positive) {
    return sliding_threshold - pdot[0];
  } else {
    return -pdot[0] - sliding_threshold;
  }
}

/// The witness function itself.
template <class T>
T Rod2D<T>::CalcStickingFrictionForceSlack(const systems::Context<T>& context,
                                           int contact_index) const {
using std::abs;

  // Verify the system is simulated using piecewise DAE.
  DRAKE_DEMAND(system_type_ == SystemType::kPiecewiseDAE);

  // Get the contact information.
  const int active_set_array_index = GetActiveSetArrayIndex(
      context.get_state(), contact_index);
  const auto& contact =
      get_endpoints_used_in_force_calculations(
        context.get_state())[active_set_array_index];

  // Verify rod is not undergoing sliding contact at the specified index.
  DRAKE_DEMAND(contact.sliding_type == SlidingModeType::kNotSliding);

  // TODO(edrumwri): Speed this up (presumably) using caching.

  // Populate problem data and solve the contact problem.
  const int ngv = 3;  // Number of rod generalized velocities.
  VectorX<T> cf;
  multibody::constraint::ConstraintAccelProblemData<T> problem_data(ngv);
  CalcConstraintProblemData(context, context.get_state(), &problem_data);
  solver_.SolveConstraintProblem(problem_data, &cf);

  // Determine the index of this contact in the non-sliding constraint set.
  const std::vector<int>& sliding_contacts = problem_data.sliding_contacts;
  const std::vector<int>& non_sliding_contacts =
      problem_data.non_sliding_contacts;
  DRAKE_ASSERT(std::is_sorted(non_sliding_contacts.begin(),
                              non_sliding_contacts.end()));
  const int non_sliding_index = std::distance(
      non_sliding_contacts.begin(),
      std::lower_bound(non_sliding_contacts.begin(),
                       non_sliding_contacts.end(),
                       active_set_array_index));
  const int num_sliding = sliding_contacts.size();
  const int num_non_sliding = non_sliding_contacts.size();
  const int nc = num_sliding + num_non_sliding;
  const int k = get_num_tangent_directions_per_contact();
  const int r = k / 2;

  // Get the normal force and the l1-norm of the frictional force.
  const auto fN = cf[active_set_array_index];
  const auto fF = cf.segment(nc + non_sliding_index * r, r).template
      lpNorm<1>();

  // Determine the slack.
  return problem_data.mu_non_sliding[non_sliding_index] * fN - fF;
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
  const Vector3<T> q = GetRodConfig(context);
  const Vector3<T> v = GetRodVelocity(context);

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

template <typename T>
void Rod2D<T>::CopyStateOut(const systems::Context<T>& context,
                            systems::BasicVector<T>* state_port_value) const {
  // Output port value is just the continuous or discrete state.
  const VectorX<T> state = (system_type_ == SystemType::kDiscretized)
                               ? context.get_discrete_state(0).CopyToVector()
                               : context.get_continuous_state().CopyToVector();
  state_port_value->SetFromVector(state);
}

template <typename T>
void Rod2D<T>::CopyPoseOut(
    const systems::Context<T>& context,
    systems::rendering::PoseVector<T>* pose_port_value) const {
  const VectorX<T> state = (system_type_ == SystemType::kDiscretized)
                               ? context.get_discrete_state(0).CopyToVector()
                               : context.get_continuous_state().CopyToVector();
  ConvertStateToPose(state, pose_port_value);
}

/// Integrates the Rod 2D example forward in time using a
/// half-explicit discretization scheme.
template <class T>
void Rod2D<T>::DoCalcDiscreteVariableUpdates(
    const systems::Context<T>& context,
    const std::vector<const systems::DiscreteUpdateEvent<T>*>&,
    systems::DiscreteValues<T>* discrete_state) const {
  using std::sin;
  using std::cos;

  // Determine ERP and CFM.
  const double erp = get_erp();
  const double cfm = get_cfm();

  // Get the necessary state variables.
  const systems::BasicVector<T>& state = context.get_discrete_state(0);
  const auto& q = state.get_value().template segment<3>(0);
  Vector3<T> v = state.get_value().template segment<3>(3);
  const T& x = q(0);
  const T& y = q(1);
  const T& theta = q(2);

  // Construct the problem data.
  const int ngc = 3;      // Number of generalized coords / velocities.
  multibody::constraint::ConstraintVelProblemData<T> problem_data(ngc);

  // Two contact points, corresponding to the two rod endpoints, are always
  // used, regardless of whether any part of the rod is in contact with the
  // halfspace. This practice is standard in discretization approaches with
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
  Eigen::Matrix<T, nc, ngc> N, F;
  N(0, 0) = N(1, 0) = 0;
  N(0, 1) = N(1, 1) = 1;
  N(0, 2) = (left[0]  - x);
  N(1, 2) = (right[0] - x);
  F(0, 0) = F(1, 0) = 1;
  F(0, 1) = F(1, 1) = 0;
  F(0, 2) = -(left[1]  - y);
  F(1, 2) = -(right[1] - y);

  // Set the number of tangent directions.
  problem_data.r = { 1, 1 };

  // Get the total number of tangent directions.
  const int nr = std::accumulate(
      problem_data.r.begin(), problem_data.r.end(), 0);

  // Set the coefficients of friction.
  problem_data.mu.setOnes(nc) *= get_mu_coulomb();

  // Set up the operators.
  problem_data.N_mult = [&N](const VectorX<T>& vv) -> VectorX<T> {
    return N * vv;
  };
  problem_data.N_transpose_mult = [&N](const VectorX<T>& vv) -> VectorX<T> {
    return N.transpose() * vv;
  };
  problem_data.F_transpose_mult = [&F](const VectorX<T>& vv) -> VectorX<T> {
    return F.transpose() * vv;
  };
  problem_data.F_mult = [&F](const VectorX<T>& vv) -> VectorX<T> {
    return F * vv;
  };
  problem_data.solve_inertia = [this](const MatrixX<T>& mm) -> MatrixX<T> {
    return GetInverseInertiaMatrix() * mm;
  };

  // Update the generalized velocity vector with discretized external forces
  // (expressed in the world frame).
  const Vector3<T> fext = ComputeExternalForces(context);
  v += dt_ * problem_data.solve_inertia(fext);
  problem_data.Mv = GetInertiaMatrix() * v;

  // Set stabilization parameters.
  problem_data.kN.resize(nc);
  problem_data.kN[0] = erp * left[1] / dt_;
  problem_data.kN[1] = erp * right[1] / dt_;
  problem_data.kF.setZero(nr);

  // Set regularization parameters.
  problem_data.gammaN.setOnes(nc) *= cfm;
  problem_data.gammaF.setOnes(nr) *= cfm;
  problem_data.gammaE.setOnes(nc) *= cfm;

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
  DRAKE_DEMAND(system_type_ == SystemType::kContinuous);

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

  // Don't compute any derivatives if this is the discretized system.
  if (system_type_ == SystemType::kDiscretized) {
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
  if (system_type_ == SystemType::kContinuous) {
    return CalcAccelerationsCompliantContactAndBallistic(context, derivatives);
  } else {
    // TODO(edrumwri): Implement the piecewise DAE approach.
    DRAKE_ABORT();
  }
}

/// Allocates the abstract state (for piecewise DAE systems).
template <typename T>
std::unique_ptr<systems::AbstractValues> Rod2D<T>::AllocateAbstractState()
    const {
  if (system_type_ == SystemType::kPiecewiseDAE) {
    // Piecewise DAE approach needs multiple abstract variables: one vector
    // of contact indices used in force calculations and one vector for each
    // type of witness function. 
    std::vector<std::unique_ptr<systems::AbstractValue>> abstract_data;

    abstract_data.push_back(
        std::make_unique<systems::Value<std::vector<PointContact>>>());

    // Create the left and right set of active witnesses.
    abstract_data.push_back(
        std::make_unique<systems::Value<ActiveRodWitnesses>>());
    abstract_data.push_back(
        std::make_unique<systems::Value<ActiveRodWitnesses>>());

    return std::make_unique<systems::AbstractValues>(std::move(abstract_data));
  } else {
    // Discretized and continuous approaches need no abstract variables.
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
  x0 << half_len * r22, half_len * r22, M_PI / 4.0, -1, 0, 0;  // Initial state.
  if (system_type_ == SystemType::kDiscretized) {
    state->get_mutable_discrete_state().get_mutable_vector(0)
        .SetFromVector(x0);
  } else {
    // Continuous variables.
    state->get_mutable_continuous_state().SetFromVector(x0);

    // Set abstract variables for piecewise DAE approach.
    if (system_type_ == SystemType::kPiecewiseDAE) {
      // Initialize the vector of contact points.
      auto& active_set_endpoints =
          get_endpoints_used_in_force_calculations(state);

      // Indicate that the rod is in the single contact sliding mode.
      active_set_endpoints.resize(1);

      // Get the active witness functions.
      ActiveRodWitnesses& left_witnesses = GetActiveWitnesses(kLeft, state);
      ActiveRodWitnesses& right_witnesses = GetActiveWitnesses(kRight, state);

      // First contact candidate is Rl in Rod Frame (see class documentation);
      // in the default rod configuration, Rl contacts the half-space and is
      // sliding.
      active_set_endpoints.front().sliding_type = SlidingModeType::kSliding;
      active_set_endpoints.front().id = 0;

      // Enable both signed distance witnesses.
      left_witnesses.signed_distance = true;
      right_witnesses.signed_distance = true;

      // Enable the normal force witness and the sliding witnesses for the
      // left (sliding) contact.
      left_witnesses.normal_force = true;
      left_witnesses.positive_sliding = true;
      left_witnesses.negative_sliding = true;

      // Disable all other witness functions.
      left_witnesses.normal_velocity = false;
      right_witnesses.normal_velocity = false;
      left_witnesses.normal_acceleration = false;
      right_witnesses.normal_acceleration = false;
      left_witnesses.sticking_friction_force_slack = false;
      right_witnesses.sticking_friction_force_slack = false;
      right_witnesses.positive_sliding = false;
      right_witnesses.negative_sliding = false;
      right_witnesses.normal_force = false;
    }
  }
}

// Sets the contact candidates.
template <class T>
void Rod2D<T>::SetContactCandidates() {
  // First point is Rl in Rod Frame (see class documentation).
  contact_candidates_[kLeft] = Vector2<T>(-get_rod_half_length(), 0);

  // Second point is Rr in Rod Frame.
  contact_candidates_[kRight] = Vector2<T>(get_rod_half_length(), 0);   
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
