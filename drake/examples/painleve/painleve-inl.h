#pragma once

// @file
// Template method implementations for painleve.h.
// Most users should only include that file, not this one.
// For background, see http://drake.mit.edu/cxx_inl.html.

#include <limits>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/examples/painleve/painleve.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace painleve {

template <typename T>
Painleve<T>::Painleve() {
  // Piecewise DAE approach needs six continuous variables and three
  // abstract variables (one mode, one contact point indicator, and one
  // sliding direction).
  this->DeclareContinuousState(3, 3, 0);
  this->DeclareOutputPort(systems::kVectorValued, 6);
}

template <typename T>
Painleve<T>::Painleve(T dt) : dt_(dt) {
  // Verify that this is a time stepping system.
  if (dt_ <= 0.0)
    throw std::logic_error("Time stepping system must be constructed using "
                               "positive integration step.");

  // Time stepping approach requires only three position variables and
  // three velocity variables.
  this->DeclareDiscreteState(6);
  const double offset = 0.0;
  this->DeclarePeriodicUpdate(dt, offset);
  this->DeclareOutputPort(systems::kVectorValued, 6);
}

// Utility method for determining the lower rod endpoint.
template <class T>
std::pair<T, T> Painleve<T>::CalcRodLowerEndpoint(const T& x,
                                                  const T& y,
                                                  const int k,
                                                  const T& ctheta,
                                                  const T& stheta,
                                                  const double half_rod_len) {
  const T cx = x + k * ctheta * half_rod_len;
  const T cy = y + k * stheta * half_rod_len;
  return std::make_pair(cx, cy);
}

template <typename T>
void Painleve<T>::DoCalcOutput(const systems::Context<T>& context,
                               systems::SystemOutput<T>* output) const {
  // Obtain the structure we need to write into.
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

  // Output port value is just the continuous state.
  output_vector->get_mutable_value() =
      context.get_continuous_state()->CopyToVector();
}

/// The witness function for signed distance between the rod and the half-space.
template <class T>
T Painleve<T>::CalcSignedDistance(const Painleve<T>& painleve,
                                  const systems::Context<T>& context) {
  // Verify the system is not time stepping.
  DRAKE_DEMAND(painleve.get_integration_step_size() == 0);

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T y = state.GetAtIndex(1);
  const T theta = state.GetAtIndex(2);

  // The two points of the rod are located at (x,y) + R(theta)*[0,r/2] and
  // (x,y) + R(theta)*[0,-r/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and r is designated as the rod length. Thus, the heights of
  // the rod endpoints are y + sin(theta)*r/2 and y - sin(theta)*r/2,
  // or, y + k*sin(theta)*r/2, where k = +/-1.

  // Get the lowest point on the rod. If sin(theta) = 0, meaning that the rod
  // is perfectly horizontal, we arbitrarily pick k=+1.
  const T stheta = sin(theta);
  const T k = (stheta > 0.0) ? -1.0 : 1.0;
  const T yc = y + k * stheta * painleve.get_rod_length() / 2;

  return yc;
}

/// The witness function for the signed distance between one endpoint of the
/// rod (not already touching the half-space) and the halfspace *for the case
/// when the rod is contacting the ground with a single point of contact.
/// @pre One endpoint of the rod is in contact with the ground, indicated by
///      the mode variable being set to .
template <class T>
T Painleve<T>::CalcEndpointDistance(const Painleve<T>& painleve,
                                    const systems::Context<T>& context) {
  using std::sin;

  // Verify the system is not time stepping.
  DRAKE_DEMAND(painleve.get_integration_step_size() == 0);

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T y = state.GetAtIndex(1);
  const T theta = state.GetAtIndex(2);
  const T stheta = sin(theta);

  // The two points of the rod are located at (x,y) + R(theta)*[0,r/2] and
  // (x,y) + R(theta)*[0,-r/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and r is designated as the rod length. Thus, the heights of
  // the rod endpoints are y + sin(theta)*r/2 and y - sin(theta)*r/2,
  // or, y + k*sin(theta)*r/2, where k = +/-1.

  // Get the abstract variables that determine the current system mode and
  // the endpoint in contact.
  const Mode mode = context.template get_abstract_state<Mode>(0);
  DRAKE_DEMAND(mode == Mode::kSlidingSingleContact ||
               mode == Mode::kStickingSingleContact);
  const int k = context.template get_abstract_state<int>(1);

  // Get the vertical position of the other rod endpoint.
  const int k2 = k * -1;
  return y + k2 * stheta * painleve.get_rod_length() / 2;
}

/// The witness function that determines whether the rod should separate from
/// the halfspace.
/// @pre It is assumed that the vertical velocity at the point of contact will
///      be approximately zero.
template <class T>
T Painleve<T>::CalcNormalAccelWithoutContactForces(const Painleve<T>& painleve,
                                                   const systems::Context<T>&
                                                     context) {
  DRAKE_ASSERT_VOID(painleve.CheckValidContext(context));
  using std::sin;
  using std::cos;
  using std::abs;

  // Verify the system is not time stepping.
  DRAKE_DEMAND(painleve.get_integration_step_size() == 0);

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T theta = state.GetAtIndex(2);
  const T thetadot = state.GetAtIndex(5);

  // Get 'k', which determines which endpoint of the rod is in contact.
  // The two endpoints of the rod are located at (x,y) + R(theta)*[0,l/2] and
  // (x,y) + R(theta)*[0,-l/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and l is designated as the rod endpoint. Thus, the vertical positions of
  // the rod endpoints are located at y + sin(theta)*l/2 and y - sin(theta)*l/2.

  // Get the abstract variables that determine the current system mode and
  // the endpoint in contact.
  const Mode mode = context.template get_abstract_state<Mode>(0);
  DRAKE_DEMAND(mode != Mode::kBallisticMotion);
  const int k = context.template get_abstract_state<int>(1);
  const T half_rod_length = painleve.get_rod_length() / 2;
  const T stheta = sin(theta);

  // Compute the normal acceleration at the point of contact (yc_ddot),
  // *assuming zero contact force*.
  T ycddot = painleve.get_gravitational_acceleration() -
        k * half_rod_length * stheta * thetadot * thetadot;

  return ycddot;
}

/// Evaluates the witness function for sliding direction changes.
template <class T>
T Painleve<T>::EvaluateSlidingDot(const Painleve<T>& painleve,
                                  const systems::Context<T>& context) {
  // Verify the system is not time stepping.
  DRAKE_DEMAND(painleve.get_integration_step_size() == 0);

  // Verify rod is undergoing sliding contact.
  const Mode mode = context.template get_abstract_state<Mode>(0);
  DRAKE_DEMAND(mode == Mode::kSlidingSingleContact ||
               mode == Mode::kSlidingTwoContacts);

  // Get the point of contact.
  const int k = context.template get_abstract_state<int>(1);

  // Get the sliding velocity at the beginning of the interval.
  const T xcdot_t0 = context.template get_abstract_state<T>(2);

  // Get the relevant parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T theta = state.GetAtIndex(2);
  const T xdot = state.GetAtIndex(3);
  const T thetadot = state.GetAtIndex(5);

  // The two points of the rod are located at (x,y) + R(theta)*[0,ell/2] and
  // (x,y) + R(theta)*[0,-ell/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and ell is designated as the rod endpoint. Thus, the vertical positions of
  // the rod endpoints are located at y + sin(theta)*l/2 and y - sin(theta)*l/2,
  // or, y + k*sin(theta)*l/2, where k = +/-1.

  // Compute the velocity at the point of contact
  const T stheta = sin(theta);
  const T half_rod_length = painleve.get_rod_length() / 2;
  const T xcdot = xdot - k * stheta * half_rod_length * thetadot;
  return xcdot * xcdot_t0;
}

/// Gets the number of witness functions.
template <class T>
int Painleve<T>::DetermineNumberWitnessFunctions(const systems::Context<T>&
                                                  context) const {
  // No witness functions if this is the time stepping system.
  if (dt_ > 0)
    return 0;

  // Get the abstract variable that determines the current system mode.
  Mode mode = context.template get_abstract_state<Mode>(0);

  switch (mode) {
    case Painleve::kBallisticMotion:
      // The rod is in ballistic flight, there is just one witness function:
      // the signed distance between the rod and the half-space.
      return 1;

    case Painleve::kSlidingSingleContact:
      // The rod is undergoing contact without impact and is sliding at a single
      // point of contact. Three witness functions are necessary: one for
      // checking whether the rod is to separate from the half-space, another
      // for checking whether the direction of sliding has changed, and a third
      // for checking for contact between the other rod endpoint and the ground.
      return 3;

    case Painleve::kStickingSingleContact:
      // The rod is undergoing contact without impact and is sticking at a
      // single point of contact. Two witness functions are necessary: one for
      // checking whether the rod is to separate from the half-space and a
      // second for checking for contact between the other rod endpoint
      // and the ground. We assume that a sticking contact cannot transition
      // to a sliding contact, as this particular example lacks non-frictional
      // tangential forces.
      return 2;

    case Painleve::kSlidingTwoContacts:
      // The rod is undergoing sliding contact without impact at two points of
      // contact. Three witness functions are necessary: two for checking
      // whether the rod is to separate from the half-space and one more to
      // check whether the rod is to transition from sliding to sticking.
      return 3;

    case Painleve::kStickingTwoContacts:
      // The rod is undergoing sliding contact without impact at two points of
      // contact. Two witness functions are necessary to check whether the rod
      // is to separate from the half-space. We assume that a sticking contact
      // cannot transition to a sliding contact, as this particular example
      // lacks non-frictional tangential forces.
      return 2;

    default:
      DRAKE_ABORT();
  }

  DRAKE_ABORT();
  return 0;
}

/// Integrates the Painleve Paradox example forward in time using a
/// half-explicit time stepping scheme.
template <class T>
void Painleve<T>::DoCalcDiscreteVariableUpdates(
                           const systems::Context<T>& context,
                           systems::DiscreteState<T>* discrete_state) const {
  // Verify integration step size is strictly positive.
  DRAKE_DEMAND(dt_ > 0);

  // Set ERP and CFM to make this problem "mostly rigid" and with rapid
  // stabilization.
  const double erp = 0.8;
  const double cfm = 1e-8;

  // Get the necessary parts of the state.
  const systems::BasicVector<T>& state = *context.get_discrete_state(0);
  const Vector3<T> q = state.get_value().segment(0, 3);
  Vector3<T> v = state.get_value().segment(3, 3);
  const T x = q(0);
  const T y = q(1);
  const T theta = q(2);

  // Compute the two rod vertical endpoint locations.
  const T stheta = sin(theta);
  const T ctheta = cos(theta);
  const int k1 = -1.0;
  const int k2 = 1.0;
  const T half_rod_length = rod_length_ / 2;
  const T xep1 = x + k1 * ctheta * half_rod_length;
  const T yep1 = y + k1 * stheta * half_rod_length;
  const T xep2 = x + k2 * ctheta * half_rod_length;
  const T yep2 = y + k2 * stheta * half_rod_length;

  // Three generalized coordinates / velocities.
  const int ngc = 3;

  // Number of contacts is constant.
  const int nc = 2;

  // Total number of friction directions = number of friction directions
  // per contact * number of contacts.
  const int nk = 2 * nc;

  // Set up the inverse generalized inertia matrix (center of mass frame).
  Matrix3<T> iM;
  iM << 1.0/mass_, 0,         0,
        0,         1.0/mass_, 0,
        0,         0,         1.0/J_;

  // Update the generalized velocity vector with discretized external forces.
  v(1) += dt_*get_gravitational_acceleration();

  // Set up the contact normal and tangent direction Jacobian matrices.
  MatrixX<T> N(nc, ngc), F(nc, ngc);
  N(0, 0) = N(1, 0) = 0;
  N(0, 1) = N(1, 1) = 1;
  N(0, 2) = (xep1 - x);
  N(1, 2) = (xep2 - x);
  F(0, 0) = F(1, 0) = 1;
  F(0, 1) = F(1, 1) = 0;
  F(0, 2) = -(yep1 - y);
  F(1, 2) = -(yep2 - y);

  // Set up the block diagonal matrix (commonly denoted E).
  MatrixX<T> E(nk, nc);
  E.col(0) << 1, 1, 0, 0;
  E.col(1) << 0, 0, 1, 1;

  // Set up the LCP matrix. First do the "normal contact direction" rows.
  MatrixX<T> MM(8,8);
  MM.template block<2, 2>(0, 0) = N * iM * N.transpose();
  MM.template block<2, 2>(0, 2) = N * iM * F.transpose();
  MM.template block<2, 2>(0, 4) = -MM.template block<2, 2>(0, 2);
  MM.template block<2, 2>(0, 6).setZero();

  // Now do the un-negated tangent contact direction rows (everything but last
  // block column).
  MM.template block<2, 2>(2, 0) = F * iM * N.transpose();
  MM.template block<2, 2>(2, 2) = F * iM * F.transpose();
  MM.template block<2, 2>(2, 4) = -MM.template block<2, 2>(2, 2);

  // Now do the negated tangent contact direction rows (everything but last
  // block column).
  MM.template block<2, 6>(4, 0) = -MM.template block<2, 6>(2, 0);

  // Do the last block column for the last set of rows.
  MM.template block<4, 2>(2, 6) = E;

  // Setup the last two rows, which provide the friction "cone" constraint..
  MM.template block<2, 2>(6, 0) = Matrix2<T>::Identity() * get_mu_coulomb();
  MM.template block<2, 4>(6, 2) = -E.transpose();
  MM.template block<2, 2>(6, 6).setZero();

  // Set up the LCP vector.
  VectorX<T> qq(8);
  qq.segment(0,2) = N * v;
  qq(0) += erp*yep1/dt_;
  qq(1) += erp*yep2/dt_;
  qq.segment(2,2) = F * v;
  qq.segment(4,2) = -qq.segment(2,2);
  qq.template segment(6,2).setZero();

  // Regularize the LCP matrix.
  MM += MatrixX<T>::Identity(8,8) * cfm;

  // Solve the LCP.
  VectorX<T> zz;
  bool success = lcp_.SolveLcpLemke(MM, qq, &zz);
  DRAKE_DEMAND(success);

  // Get the normal and frictional contact forces out.
  VectorX<T> fN = zz.segment(0,2);
  VectorX<T> fF_pos = zz.segment(2,2);
  VectorX<T> fF_neg = zz.segment(4,2);

  // Compute the new velocity.
  VectorX<T> vplus = v + iM * (N.transpose()*fN + F.transpose()*fF_pos -
                               F.transpose()*fF_neg);

  // Compute the new position.
  VectorX<T> qplus = q + vplus*dt_;

  // Set the new discrete state.
  systems::BasicVector<T>* new_state = discrete_state->
      get_mutable_discrete_state(0);
  new_state->get_mutable_value().segment(0, 3) = qplus;
  new_state->get_mutable_value().segment(3, 3) = vplus;
}

/// Models impact using an inelastic impact model with friction.
template <typename T>
void Painleve<T>::HandleImpact(const systems::Context<T>& context,
                               systems::ContinuousState<T>* new_state) const {
  using std::abs;

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // Get the state vector.
  systems::VectorBase<T>* new_statev = new_state->get_mutable_vector();

  // Positional aspects of state do not change.
  new_statev->SetAtIndex(0, x);
  new_statev->SetAtIndex(1, y);
  new_statev->SetAtIndex(2, theta);

  // If there is no impact, quit now.
  if (!IsImpacting(context)) {
    new_statev->SetAtIndex(3, xdot);
    new_statev->SetAtIndex(4, ydot);
    new_statev->SetAtIndex(5, thetadot);
    return;
  }

  // TODO(edrumwri): Handle two-point impact.
  DRAKE_DEMAND(abs(sin(theta)) >= std::numeric_limits<T>::epsilon());

  // The two points of the rod are located at (x,y) + R(theta)*[0,r/2] and
  // (x,y) + R(theta)*[0,-r/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and r is designated as the rod length. Thus, the heights of
  // the rod endpoints are y + sin(theta)*r/2 and y - sin(theta)*r/2,
  // or, y + k*sin(theta)*r/2, where k = +/-1.

  // Determine the point of contact (cx,cy).
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;
  const double half_rod_length = rod_length_ / 2;
  const std::pair<T, T> c = CalcRodLowerEndpoint(x, y, k, ctheta, stheta,
                                                 half_rod_length);
  const T cx = c.first;
  const T cy = c.second;

  // Compute the impulses such that the tangential velocity post-collision
  // is zero.
  const Vector2<T> f_sticking = CalcStickingImpactImpulse(context);
  T fN = f_sticking(0);
  T fF = f_sticking(1);

  // See whether it is necessary to recompute the impulses (because the impulse
  // lies outside of the friction cone). In that case, the rod will have
  // non-zero sliding velocity post-impact (i.e., it will be sliding).
  if (mu_*fN < abs(fF)) {
    const Vector2<T> f_sliding = CalcFConeImpactImpulse(context);
    fN = f_sliding(0);
    fF = f_sliding(1);
  }

  // Compute the change in linear velocity.
  const T delta_xdot = fF / mass_;
  const T delta_ydot = fN / mass_;

  // Change in thetadot is equivalent to the third component of:
  // | cx - x |    | fF |
  // | cy - y | ×  | fN | = (cx - x) * fN - (cy - y) * fF)
  // | 0      |    | 0  |
  // divided by the moment of inertia.
  const T delta_thetadot = ((cx - x) * fN - (cy - y) * fF) / J_;

  // Verify that the post-impact velocity is non-negative (to allowable floating
  // point error).
  DRAKE_DEMAND((ydot + delta_ydot) +
                   k * ctheta * half_rod_length * (thetadot + delta_thetadot) >
               -std::numeric_limits<double>::epsilon() * 10);

  // Update the velocity.
  new_statev->SetAtIndex(3, xdot + delta_xdot);
  new_statev->SetAtIndex(4, ydot + delta_ydot);
  new_statev->SetAtIndex(5, thetadot + delta_thetadot);
}

// Computes the impulses such that the vertical velocity at the contact point
// is zero and the frictional impulse lies exactly on the friction cone.
// These equations were determined by issuing the
// following commands in Mathematica:
//
// cx[t_] := x[t] + k*Cos[theta[t]]*(r/2)
// cy[t_] := y[t] + k*Sin[theta[t]]*(r/2)
// Solve[{mass*delta_xdot == fF,
//        mass*delta_ydot == fN,
//        J*delta_thetadot == (cx[t] - x)*fN - (cy - y)*fF,
//        0 == (D[y[t], t] + delta_ydot) +
//              k*(r/2) *Cos[theta[t]]*(D[theta[t], t] + delta_thetadot),
//        fF == mu*fN *-sgn_cxdot},
//       {delta_xdot, delta_ydot, delta_thetadot, fN, fF}]
// where theta is the counter-clockwise angle the rod makes with the x-axis,
// fN and fF are contact normal and frictional forces; delta_xdot,
// delta_ydot, and delta_thetadot represent the changes in velocity,
// r is the length of the rod, sgn_xdot is the sign of the tangent
// velocity (pre-impact), and (hopefully) all other variables are
// self-explanatory.
//
// The first two equations above are the formula
// for the location of the point of contact. The next two equations
// describe the relationship between the horizontal/vertical change in
// momenta at the center of mass of the rod and the frictional/normal
// contact impulses. The fifth equation yields the moment from
// the contact impulses. The sixth equation specifies that the post-impact
// velocity in the vertical direction be zero. The last equation corresponds
// to the relationship between normal and frictional impulses (dictated by the
// Coulomb friction model).
// @returns a Vector2, with the first element corresponding to the impulse in
//          the normal direction (positive y-axis) and the second element
//          corresponding to the impulse in the tangential direction (positive
//          x-axis).
template <class T>
Vector2<T> Painleve<T>::CalcFConeImpactImpulse(
    const systems::Context<T>& context) const {
  using std::abs;

  // Get the necessary parts of the state.
  const systems::VectorBase<T> &state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // The two points of the rod are located at (x,y) + R(theta)*[0,r/2] and
  // (x,y) + R(theta)*[0,-r/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and r is designated as the rod length. Thus, the heights of
  // the rod endpoints are y + sin(theta)*r/2 and y - sin(theta)*r/2,
  // or, y + k*sin(theta)*r/2, where k = +/-1.
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;
  const double half_rod_length = rod_length_ / 2;
  const T cy = y + k * stheta * half_rod_length;
  const double mu = mu_;
  const double J = J_;
  const double mass = mass_;
  const double r = rod_length_;

  // Compute the impulses.
  const T cxdot = xdot - k * stheta * half_rod_length * thetadot;
  const int sgn_cxdot = (cxdot > 0) ? 1 : -1;
  const T fN = (J * mass * (-(r * k * ctheta * thetadot) / 2 - ydot)) /
              (J + (r * k * mass * mu * (-y + cy) * ctheta * sgn_cxdot) / 2 -
              (r * k * mass * ctheta * (x - (r * k * ctheta) / 2 - x)) / 2);
  const T fF = -sgn_cxdot * mu * fN;

  // Verify normal force is non-negative.
  DRAKE_DEMAND(fN >= 0);

  return Vector2<T>(fN, fF);
}

// Computes the impulses such that the velocity at the contact point is zero.
// These equations were determined by issuing the following commands in
// Mathematica:
//
// cx[t_] := x[t] + k*Cos[theta[t]]*(r/2)
// cy[t_] := y[t] + k*Sin[theta[t]]*(r/2)
// Solve[{mass*delta_xdot == fF, mass*delta_ydot == fN,
//        J*delta_thetadot == (cx[t] - x)*fN - (cy - y)*fF,
//        0 == (D[y[t], t] + delta_ydot) +
//             k*(r/2) *Cos[theta[t]]*(D[theta[t], t] + delta_thetadot),
//        0 == (D[x[t], t] + delta_xdot) +
//             k*(r/2)*-Cos[theta[t]]*(D[theta[t], t] + delta_thetadot)},
//       {delta_xdot, delta_ydot, delta_thetadot, fN, fF}]
// which computes the change in velocity and frictional (fF) and normal (fN)
// impulses necessary to bring the system to rest at the point of contact,
// 'r' is the rod length, theta is the counter-clockwise angle measured
// with respect to the x-axis; delta_xdot, delta_ydot, and delta_thetadot
// are the changes in velocity.
//
// The first two equations above are the formula
// for the location of the point of contact. The next two equations
// describe the relationship between the horizontal/vertical change in
// momenta at the center of mass of the rod and the frictional/normal
// contact impulses. The fifth equation yields the moment from
// the contact impulses. The sixth and seventh equations specify that the
// post-impact velocity in the horizontal and vertical directions at the
// point of contact be zero.
// @returns a Vector2, with the first element corresponding to the impulse in
//          the normal direction (positive y-axis) and the second element
//          corresponding to the impulse in the tangential direction (positive
//          x-axis).
template <class T>
Vector2<T> Painleve<T>::CalcStickingImpactImpulse(
    const systems::Context<T>& context) const {
  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // The two points of the rod are located at (x,y) + R(theta)*[0,r/2] and
  // (x,y) + R(theta)*[0,-r/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and r is designated as the rod length. Thus, the heights of
  // the rod endpoints are y + sin(theta)*l/2 and y - sin(theta)*l/2,
  // or, y + k*sin(theta)*l/2, where k = +/-1.
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;
  const double half_rod_length = rod_length_ / 2;
  const T cy = y + k * stheta * half_rod_length;

  // Compute the impulses.
  const double r = rod_length_;
  const double mass = mass_;
  const double J = J_;
  const T fN = (2 * (-(r * J * k * mass * ctheta * thetadot) +
      r * k * mass * mass * y * ctheta * xdot -
      r * k * mass * mass * cy * ctheta * xdot -
      2 * J * mass * ydot + r * k * mass * mass * y * ctheta * ydot -
      r * k * mass * mass * cy * ctheta * ydot)) /
      (4 * J - 2 * r * k * mass * x * ctheta -
          2 * r * k * mass * y * ctheta + 2 * r * k * mass * cy * ctheta +
          r * r * k * k * mass * ctheta * ctheta +
          2 * r * k * mass * ctheta * x);
  const T fF = -((mass * (-2 * r * J * k * ctheta * thetadot + 4 * J * xdot -
      2 * r * k * mass * x * ctheta * xdot +
      r * r * k * k * mass * ctheta * ctheta * xdot +
      2 * r * k * mass * ctheta * x * xdot -
      2 * r * k * mass * x * ctheta * ydot +
      r * r * k * k * mass * ctheta * ctheta * ydot +
      2 * r * k * mass * ctheta * x * ydot)) /
      (4 * J - 2 * r * k * mass * x * ctheta -
          2 * r * k * mass * y * ctheta + 2 * r * k * mass * cy * ctheta +
          r * r * k * k * mass * ctheta * ctheta +
          2 * r * k * mass * ctheta * x));

  // Verify that normal impulse is non-negative.
  DRAKE_DEMAND(fN > 0.0);

  return Vector2<T>(fN, fF);
}

// Sets the velocity derivatives for the rod, given contact forces.
template <class T>
void Painleve<T>::SetVelocityDerivatives(const systems::Context<T>& context,
                                         systems::VectorBase<T>* const f,
                                         T fN, T fF, T cx, T cy) const {
  using std::abs;

  // Get necessary state variables.
  const T& x = context.get_continuous_state_vector().GetAtIndex(0);
  const T& y = context.get_continuous_state_vector().GetAtIndex(1);
  const T& theta = context.get_continuous_state_vector().GetAtIndex(2);
  const T& thetadot = context.get_continuous_state_vector().GetAtIndex(5);

  // Compute the derivatives
  const T xddot = fF / mass_;
  const T yddot = fN / mass_ + get_gravitational_acceleration();
  const T thetaddot = ((cx - x) * fN - (cy - y) * fF) / J_;

  // Set the derivatives.
  f->SetAtIndex(3, xddot);
  f->SetAtIndex(4, yddot);
  f->SetAtIndex(5, thetaddot);

  // Get constants for checking accelerations.
  const double mu = mu_;
  const double r = rod_length_;
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;

  // Verify that the vertical acceleration at the point of contact is zero
  // (i.e., cyddot = 0).
  const T cyddot =
      yddot +
          r * k * (ctheta * thetaddot - stheta * thetadot * thetadot) / 2;
  DRAKE_DEMAND(abs(cyddot) < std::numeric_limits<double>::epsilon());

  // If the force is within the friction cone, verify that the horizontal
  // acceleration at the point of contact is zero (i.e., cxddot = 0).
  if (fN*mu > abs(fF)) {
    const T cxddot =
        xddot +
            r * k * (-stheta * thetaddot - ctheta * thetadot * thetadot) / 2;

    DRAKE_DEMAND(abs(cxddot) < std::numeric_limits<double>::epsilon());
  }
}

// Computes the contact forces for the case of zero sliding velocity, assuming
// that the tangential acceleration at the point of contact will be zero
// (i.e., cxddot = 0). This function solves for these forces.
//
// Equations were determined by issuing the following command in Mathematica:
// cx[t_] := x[t] + k*Cos[theta[t]]*(r/2)
// cy[t_] := y[t] + k*Sin[theta[t]]*(r/2)
// Solve[{0 == D[D[cy[t], t], t],
//        D[D[y[t], t], t] == fN/mass + g,
//        D[D[x[t], t], t] == fF/mass,
//        J*D[D[theta[t], t], t] == (cx[t]-x[t])*fN - (cy[t]-y[t])*fF,
//        0 == D[D[cx[t], t], t]},
//       { fN, fF, D[D[y[t], t], t], D[D[x[t], t], t],
//          D[D[theta[t], t], t] } ]
// where theta is the counter-clockwise angle the rod makes with the
// x-axis, fN and fF are contact normal and frictional forces, g is the
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
Vector2<T> Painleve<T>::CalcStickingContactForces(
    const systems::Context<T>& context) const {

  // Get necessary state variables.
  const T& theta = context.get_continuous_state_vector().GetAtIndex(2);
  const T& thetadot = context.get_continuous_state_vector().GetAtIndex(5);

  // Precompute quantities that will be used repeatedly.
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;

  // Set named Mathematica constants.
  const double mass = mass_;
  const double r = rod_length_;
  const double g = get_gravitational_acceleration();
  const double J = J_;
  const T fN =
      (mass *
          (-8 * g * J - 2 * r * r * g * k * k * mass * stheta * stheta +
              4 * r * J * k * stheta * thetadot * thetadot +
              r * r * r * k * k * k * mass * ctheta * ctheta * stheta *
                  thetadot * thetadot +
              r * r * r * k * k * k * mass * stheta * stheta * stheta *
                  thetadot * thetadot)) /
          (2 * (4 * J + r * r * k * k * mass * ctheta * ctheta +
              r * r * k * k * mass * stheta * stheta));
  const T fF =
      -(2 * r * r * g * k * k * mass * mass * ctheta * stheta -
          4 * r * J * k * mass * ctheta * thetadot * thetadot -
          r * r * r * k * k * k * mass * mass * ctheta * ctheta *
              ctheta * thetadot * thetadot -
          r * r * r * k * k * k * mass * mass * ctheta * stheta *
              stheta * thetadot * thetadot) /
          (2 * (4 * J + r * r * k * k * mass * ctheta * ctheta +
              r * r * k * k * mass * stheta * stheta));

  return Vector2<T>(fN, fF);
}

// Computes the time derivatives for the case of the rod contacting the
// surface at exactly one point and with sliding velocity.
template <class T>
void Painleve<T>::CalcTimeDerivativesOneContactSliding(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::abs;

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
  const double half_rod_length = rod_length_ / 2;
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;

  // Determine the point of contact (cx, cy).
  const std::pair<T, T> c = CalcRodLowerEndpoint(x, y, k, ctheta, stheta,
                                                 half_rod_length);
  const T cx = c.first;
  const T cy = c.second;

  // Compute the horizontal velocity at the point of contact.
  const T cxdot = xdot - k * stheta * half_rod_length * thetadot;

  // Compute the normal acceleration at the point of contact (cy_ddot),
  // *assuming zero contact force*.
  T cyddot = get_gravitational_acceleration() -
      k * half_rod_length * stheta * thetadot * thetadot;

  // These equations were determined by issuing the following
  // commands in Mathematica:
  // cx[t_] := x[t] + k*Cos[theta[t]]*(r/2)
  // cy[t_] := y[t] + k*Sin[theta[t]]*(r/2)
  // Solve[{0 == D[D[cy[t], t], t],
  //        D[D[y[t], t], t] == fN/mass + g,
  //        D[D[x[t], t], t] == fF/mass,
  //        J*D[D[theta[t], t], t] == (cx[t]-x[t])*fN - (cy[t]-y[t])*fF,
  //        fF == -sgn_cxdot*mu*fN},
  // { fN, fF, D[D[y[t], t], t], D[D[x[t], t], t], D[D[theta[t], t], t]}]
  // where theta is the counter-clockwise angle the rod makes with the
  // x-axis, 'r' is the length of the rod, fN and fF are normal and
  // frictional forces, respectively, sgn_cxdot = sgn(cxdot), g is the
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
  const double r = rod_length_;
  const T fN = (2 * mass *
      (-2 * g * J + r * J * k * stheta * thetadot * thetadot)) /
      (4 * J + r * r * k * k * mass * ctheta * ctheta +
          r * r * k * k * mass * mu * ctheta * sgn_cxdot * stheta);

  // Check for inconsistent configuration.
  if (fN < 0)
    throw std::runtime_error("Inconsistent configuration detected.");

  // Now that normal force is computed, set the acceleration.
  const T fF = -sgn_cxdot * mu_ * fN;
  f->SetAtIndex(3, fF / mass_);
  f->SetAtIndex(4, fN / mass_ + get_gravitational_acceleration());
  f->SetAtIndex(5, ((cx - x) * fN - (cy - y) * fF) / J);

  // Compute the normal acceleration at the contact point (a check).
  const T yddot = f->GetAtIndex(4);
  const T thetaddot = f->GetAtIndex(5);
  cyddot =
      yddot +
          r * k * (ctheta * thetaddot - stheta * thetadot * thetadot) / 2;

  // Lines below currently unused but are occasionally helpful for
  // debugging.
  //        const T xddot = f->GetAtIndex(3);
  //        const T cxddot = xddot + r*k*(-stheta*thetaddot -
  //                                        +ctheta*thetadot*thetadot)/2;

  // Verify that the normal acceleration is zero.
  DRAKE_DEMAND(abs(cyddot) < std::numeric_limits<double>::epsilon() * 10);
}

// Computes the time derivatives for the case of the rod contacting the
// surface at exactly one point and without any sliding velocity.
template <class T>
void Painleve<T>::CalcTimeDerivativesOneContactNoSliding(
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

  // Compute contact point.
  const double half_rod_length = rod_length_ / 2;
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;
  const std::pair<T, T> c = CalcRodLowerEndpoint(x, y, k, ctheta, stheta,
                                                 half_rod_length);
  const T cx = c.first;
  const T cy = c.second;

  // Compute the contact forces, assuming sticking contact.
  Vector2<T> cf = CalcStickingContactForces(context);
  const T fN = cf(0);
  const T fF = cf(1);

  // Sanity check that normal force is non-negative.
  DRAKE_DEMAND(fN >= 0);

  // Recompute fF if it does not lie within the friction cone.
  // Constrain F such that it lies on the edge of the friction cone.
  const double mu = get_mu_coulomb();
  if (abs(fF) > mu * fN) {
    // TODO(edrumwri): Test this once inputs have been added to the system
    //                 in a future PR.

    // Set named Mathematica constants.
    const double mass = get_rod_mass();
    const double r = get_rod_length();
    const double J = get_rod_moment_of_inertia();
    const double g = get_gravitational_acceleration();

    // Pick the solution that minimizes the tangential acceleration.
    // This solution was obtained by solving for zero normal acceleration
    // with the frictional force pointing either possible direction
    // (positive x-axis and negative x-axis).
    auto calc_force = [=](int d) {
      const T N =
          (2 * mass *
              (-2 * g * J + r * J * k * stheta * thetadot * thetadot)) /
              (4 * J + r * r * k * k * mass * ctheta * ctheta +
                  r * r * k * k * mass * mu * ctheta * d * stheta);
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

    // Calculate candidate tangential accelerations.
    auto calc_tan_accel = [=](int d, const T N, const T F) {
      const T thetaddot = ((cx - x) * N - (cy - y) * F) / J;
      return F / mass +
          r * k * (-stheta * thetaddot - ctheta * thetadot * thetadot) / 2;
    };

    // Compute two tangential acceleration candidates.
    const T cxddot1 = calc_tan_accel(+1, fN1, fF1);
    const T cxddot2 = calc_tan_accel(-1, fN2, fF2);

    // Pick the one that is smaller in magnitude.
    if (abs(cxddot1) < abs(cxddot2)) {
      SetVelocityDerivatives(context, f, fN1, fF1, cx, cy);
    } else {
      SetVelocityDerivatives(context, f, fN2, fF2, cx, cy);
    }
  } else {
    // Friction force is within the friction cone.
    SetVelocityDerivatives(context, f, fN, fF, cx, cy);
  }
}

// Computes the time derivatives for the case of the rod contacting the
// surface at more than one point.
template <class T>
void Painleve<T>::CalcTimeDerivativesTwoContact(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::abs;

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& xdot = state.GetAtIndex(3);

  // Obtain the structure we need to write into.
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();
  DRAKE_ASSERT(f != nullptr);

  // Look to see whether there is sliding velocity.
  if (abs(xdot) < std::numeric_limits<double>::epsilon()) {
    // Set the time derivatives to "resting".
    f->SetAtIndex(3, T(0));
    f->SetAtIndex(4, T(0));
    f->SetAtIndex(5, T(0));
  } else {
    // This code assumes no sliding will occur with contacts at multiple
    // points unless the system is initialized to such a condition. This
    // assumption has been neither proven nor rigorously validated.
    throw std::logic_error("Sliding detected with non-point contact.");
  }
}

template <class T>
bool Painleve<T>::IsImpacting(const systems::Context<T>& context) const {
  using std::sin;
  using std::cos;

  // Get state data necessary to compute the point of contact.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // Get the height of the lower rod endpoint.
  const double half_rod_length = rod_length_ / 2;
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;
  const T cy = y + k * stheta * half_rod_length;

  // If rod endpoint is not touching, there is no impact.
  if (cy >= std::numeric_limits<double>::epsilon())
    return false;

  // Compute the velocity at the point of contact.
  const T cydot = ydot + k * ctheta * half_rod_length * thetadot;

  // Verify that the rod is not impacting.
  return (cydot < -std::numeric_limits<double>::epsilon());
}

template <typename T>
void Painleve<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  using std::sin;
  using std::cos;
  using std::abs;

  // Don't compute any derivatives if this is the time stepping system.
  if (dt_ > 0)
    return;

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // Obtain the structure we need to write into.
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();

  // The two endpoints of the rod are located at (x,y) + R(theta)*[0,r/2] and
  // (x,y) + R(theta)*[0,-r/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and r is designated as the rod length. Thus, the heights of
  // the rod endpoints are y + sin(theta)*r/2 and y - sin(theta)*r/2.
  const double half_rod_length = rod_length_ / 2;
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;

  // Determine the height of the point of contact (cx, cy).
  const T cy =  CalcRodLowerEndpoint(x, y, k, ctheta, stheta,
                                     half_rod_length).second;

  // Compute the horizontal velocity at the point of contact.
  const T cxdot = xdot - k * stheta * half_rod_length * thetadot;

  // First three derivative components are xdot, ydot, thetadot.
  f->SetAtIndex(0, xdot);
  f->SetAtIndex(1, ydot);
  f->SetAtIndex(2, thetadot);

  // Case 1 (ballistic mode): the rod is not touching the ground
  // (located at y=0).
  if (cy > std::numeric_limits<double>::epsilon()) {
    // Second three derivative components are simple: just add in gravitational
    // acceleration.
    f->SetAtIndex(3, T(0));
    f->SetAtIndex(4, get_gravitational_acceleration());
    f->SetAtIndex(5, T(0));
  } else {
    // Case 2: the rod is touching the ground (or even embedded in the ground).
    // Constraint stabilization should be used to eliminate embedding, but we
    // perform no such check in the derivative evaluation.

    // Handle the case where the rod is both parallel to the halfspace and
    // contacting the halfspace (at the entire length of the rod).
    // TODO(edrumwri): Modify this two-contact point routine to account for
    //                 contacts along the entire length of the rod, assumingly
    //                 only a single coefficient of friction).
    if (abs(sin(theta)) < std::numeric_limits<double>::epsilon()) {
      CalcTimeDerivativesTwoContact(context, derivatives);
      return;
    }

    // At this point, it is known that exactly one endpoint of the rod is
    // touching the halfspace. Compute the normal acceleration at that point of
    // contact (cy_ddot), *assuming zero contact force*.
    T cyddot = get_gravitational_acceleration() -
               k * half_rod_length * stheta * thetadot * thetadot;

    // If this derivative is negative, we must compute the normal force
    // necessary to set it to zero.
    if (cyddot < 0) {
      // Look for the case where the tangential velocity is zero.
      if (abs(cxdot) < std::numeric_limits<double>::epsilon())
        CalcTimeDerivativesOneContactNoSliding(context, derivatives);
      else
        CalcTimeDerivativesOneContactSliding(context, derivatives);
    }
  }
}

/// Sets the rod to a 45 degree angle with the halfspace and positions the rod
/// such that it and the halfspace are touching at exactly one point of contact.
template <typename T>
void Painleve<T>::SetDefaultState(const systems::Context<T>& context,
                                  systems::State<T>* state) const {
  using std::sqrt;

  // Initial state corresponds to an inconsistent configuration.
  const double half_len = get_rod_length() / 2;
  VectorX<T> x0(6);
  const double r22 = sqrt(2) / 2;
  x0 << half_len * r22, half_len * r22, M_PI / 4.0, -1, 0, 0;  // Initial state.
  if (dt_ == 0) {
    state->get_mutable_continuous_state()->SetFromVector(x0);
  } else {
    state->get_mutable_discrete_state()->get_mutable_discrete_state(0)->
        SetFromVector(x0);
  }
}

}  // namespace painleve
}  // namespace drake
