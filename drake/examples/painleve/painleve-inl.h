#pragma once

/// @file
/// Template method implementations for ball.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include <limits>

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
  // Time stepping approach requires only three position variables and
  // three velocity variables.
  this->DeclareDiscreteState(6);
  const double offset = 0.0;
  this->DeclarePeriodicUpdate(dt, offset);
  this->DeclareOutputPort(systems::kVectorValued, 6);
}

template <typename T>
void Painleve<T>::DoCalcOutput(const systems::Context<T>& context,
                               systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // Obtain the structure we need to write into.
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

  output_vector->get_mutable_value() =
      context.get_continuous_state()->CopyToVector();
}

/// The witness function for signed distance between the rod and the half-space.
template <class T>
T Painleve<T>::CalcSignedDistance(const Painleve<T>& painleve,
                                  const systems::Context<T>& context) {
  // Verify the system is not time stepping.
  DRAKE_DEMAND(dt_ == 0);

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T y = state.GetAtIndex(1);
  const T theta = state.GetAtIndex(2);

  // The two points of the rod are located at (x,y) + R(theta)*[0,ell/2] and
  // (x,y) + R(theta)*[0,-ell/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and ell is designated as the rod endpoint. Thus, the vertical positions of
  // the rod endpoints are located at y + sin(theta)*l/2 and y - sin(theta)*l/2,
  // or, y + k*sin(theta)*l/2, where k = +/-1.

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
  DRAKE_DEMAND(dt_ == 0);

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T y = state.GetAtIndex(1);
  const T theta = state.GetAtIndex(2);
  const T stheta = sin(theta);

  // The two points of the rod are located at (x,y) + R(theta)*[0,ell/2] and
  // (x,y) + R(theta)*[0,-ell/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and ell is designated as the rod endpoint. Thus, the vertical positions of
  // the rod endpoints are located at y + sin(theta)*l/2 and y - sin(theta)*l/2,
  // or, y + k*sin(theta)*l/2, where k = +/-1.

  // Get the abstract variables that determine the current system mode and
  // the endpoint in contact.
  const Mode mode = context.get_abstract_state(0).
                      template GetValueOrThrow<Mode>();
  DRAKE_DEMAND(mode == Mode::kSlidingSingleContact ||
               mode == Mode::kStickingSingleContact);
  const int k = context.get_abstract_state(1).template GetValueOrThrow<int>();

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
  DRAKE_DEMAND(dt_ == 0);

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
  const Mode mode = context.get_abstract_state(0).
      template GetValueOrThrow<Mode>();
  DRAKE_DEMAND(mode != Mode::kBallisticMotion);
  const int k = context.get_abstract_state(1).template GetValueOrThrow<int>();
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
  DRAKE_DEMAND(dt_ == 0);

  // Verify rod is undergoing sliding contact.
  const Mode mode = context.get_abstract_state(0).
      template GetValueOrThrow<Mode>();
  DRAKE_DEMAND(mode == Mode::kSlidingSingleContact ||
               mode == Mode::kSlidingTwoContacts);

  // Get the point of contact.
  const int k = context.get_abstract_state(1).template GetValueOrThrow<int>();

  // Get the sliding velocity at the beginning of the interval.
  const T xcdot_t0 = context.get_state().get_abstract_state()->
      get_abstract_state(2).template GetValueOrThrow<T>();

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
  Mode mode = context.get_abstract_state(0).template GetValueOrThrow<Mode>();

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
  MM.template block<2, 2>(6, 0) = Matrix2<T>::Identity() * get_mu_Coulomb();
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
  systems::BasicVector<T>* new_state = discrete_state->get_mutable_discrete_state(0);
  new_state->get_mutable_value().segment(0, 3) = qplus;
  new_state->get_mutable_value().segment(3, 3) = vplus;
}

/// Handles impact using an inelastic impact model with friction.
template <typename T>
void Painleve<T>::HandleImpact(const systems::Context<T>& context,
                               systems::ContinuousState<T>* new_state) const {
  using std::abs;

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T x = state.GetAtIndex(0);
  const T y = state.GetAtIndex(1);
  const T theta = state.GetAtIndex(2);
  const T xdot = state.GetAtIndex(3);
  const T ydot = state.GetAtIndex(4);
  const T thetadot = state.GetAtIndex(5);

  // Get the state vector
  systems::VectorBase<T>* new_statev = new_state->get_mutable_vector();

  // Positional aspects of state do not change.
  new_statev->SetAtIndex(0, x);
  new_statev->SetAtIndex(1, y);
  new_statev->SetAtIndex(2, theta);

  // TODO(edrumwri): Handle two-point impact.
  DRAKE_DEMAND(abs(sin(theta)) >= std::numeric_limits<T>::epsilon());

  // The two points of the rod are located at (x,y) + R(theta)*[0,ell/2] and
  // (x,y) + R(theta)*[0,-ell/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and ell is designated as the rod endpoint. Thus, the vertical positions of
  // the rod endpoints are located at y + sin(theta)*l/2 and y - sin(theta)*l/2,
  // or, y + k*sin(theta)*l/2, where k = +/-1.

  // Verify that there is an impact.
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const T k = (stheta > 0.0) ? -1.0 : 1.0;
  const T half_rod_length = rod_length_ * 0.5;

  // Compute the velocity at the point of contact
  const T xc = x + k * ctheta * rod_length_ / 2;
  const T yc = y + k * stheta * rod_length_ / 2;
  const T xcdot = xdot - k * stheta * rod_length_ / 2 * thetadot;
  const T ycdot = ydot + k * ctheta * rod_length_ / 2 * thetadot;

  // If the rod is touching, but separating, don't apply any impact forces.
  if (ycdot > -std::numeric_limits<double>::epsilon()) {
    new_statev->SetAtIndex(3, xdot);
    new_statev->SetAtIndex(4, ydot);
    new_statev->SetAtIndex(5, thetadot);
    return;
  }

  // Compute the change in velocities such that the velocity at the contact
  // point is zero. These equations were determined by issuing the following
  // commands in Mathematica:
  // xc[t_] := x[t] + k*Cos[theta[t]]*(ell/2)
  // yc[t_] := y[t] + k*Sin[theta[t]]*(ell/2)
  // Solve[{mass*delta_xdot == fF, mass*delta_ydot == fN,
  //        J*delta_thetadot == (xc[t] - x)*fN - (yc - y)*fF,
  //        0 == (D[y[t], t] + delta_ydot) +
  //             k*(ell/2) *Cos[theta[t]]*(D[theta[t], t] + delta_thetadot),
  //        0 == (D[x[t], t] + delta_xdot) +
  //             k*(ell/2)*-Cos[theta[t]]*(D[theta[t], t] + delta_thetadot)},
  //       {delta_xdot, delta_ydot, delta_thetadot, fN, fF}]
  // which computes the change in velocity and frictional (fF) and normal (fN)
  // impulses necessary to bring the system to rest at the point of contact,
  // 'ell' is the rod length, theta is the counter-clockwise angle measured
  // with respect to the x-axis; delta_xdot, delta_ydot, and delta_thetadot
  // are the changes in velocity.
  const T ell = rod_length_;
  const T mass = mass_;
  const T J = J_;
  T fN = (2 * (-(ell * J * k * mass * ctheta * thetadot) +
               ell * k * mass * mass * y * ctheta * xdot -
               ell * k * mass * mass * yc * ctheta * xdot -
               2 * J * mass * ydot + ell * k * mass * mass * y * ctheta * ydot -
               ell * k * mass * mass * yc * ctheta * ydot)) /
         (4 * J - 2 * ell * k * mass * x * ctheta -
          2 * ell * k * mass * y * ctheta + 2 * ell * k * mass * yc * ctheta +
          ell * ell * k * k * mass * ctheta * ctheta +
          2 * ell * k * mass * ctheta * x);
  T fF = -((mass * (-2 * ell * J * k * ctheta * thetadot + 4 * J * xdot -
                    2 * ell * k * mass * x * ctheta * xdot +
                    ell * ell * k * k * mass * ctheta * ctheta * xdot +
                    2 * ell * k * mass * ctheta * x * xdot -
                    2 * ell * k * mass * x * ctheta * ydot +
                    ell * ell * k * k * mass * ctheta * ctheta * ydot +
                    2 * ell * k * mass * ctheta * x * ydot)) /
           (4 * J - 2 * ell * k * mass * x * ctheta -
            2 * ell * k * mass * y * ctheta + 2 * ell * k * mass * yc * ctheta +
            ell * ell * k * k * mass * ctheta * ctheta +
            2 * ell * k * mass * ctheta * x));

  // Verify that fN is non-negative.
  DRAKE_DEMAND(fN > 0.0);

  // Compute the change in velocity.
  T delta_xdot = fF / mass;
  T delta_ydot = fN / mass;
  T delta_thetadot = ((xc - x) * fN - (yc - y) * fF) / J;

  // If F is not within the friction cone, recompute so that F is on the edge
  // of the friction cone. These equations were determined by issuing the
  // following commands in Mathematica:
  // xc[t_] := x[t] + k*Cos[theta[t]]*(ell/2)
  // yc[t_] := y[t] + k*Sin[theta[t]]*(ell/2)
  // Solve[{mass*delta_xdot == fF,
  //        mass*delta_ydot == fN,
  //        J*delta_thetadot == (xc[t] - x)*fN - (yc - y)*fF,
  //        0 == (D[y[t], t] + delta_ydot) +
  //              k*(ell/2) *Cos[theta[t]]*(D[theta[t], t] + delta_thetadot),
  //        fF == mu*fN *-sgn_xcdot},
  //       {delta_xdot, delta_ydot, delta_thetadot, fN, fF}]
  // where theta is the counter-clockwise angle the rod makes with the x-axis,
  // fN and fF are contact normal and frictional forces; delta_xdot,
  // delta_ydot, and delta_thetadot represent the changes in velocity,
  // ell is the length of the rod, sgn_xdot is the sign of the tangent
  // velocity (pre-impact), and (hopefully) all other variables are
  // self-explanatory.
  const T mu = mu_;
  if (abs(fF) > mu * fN) {
    const T sgn_xcdot = (xcdot > 0.0) ? 1.0 : -1.0;

    // Compute the normal force.
    fN = (J * mass * (-(ell * k * ctheta * thetadot) / 2. - ydot)) /
         (J + (ell * k * mass * mu * (-y + yc) * ctheta * sgn_xcdot) / 2. -
          (ell * k * mass * ctheta * (x - (ell * k * ctheta) / 2. - x)) / 2.);

    // Compute the frictional force.
    fF = -sgn_xcdot * mu * fN;

    // Verify normal force is non-negative.
    DRAKE_DEMAND(fN >= 0.0);

    // Recompute the change in velocity.
    delta_xdot = fF / mass;
    delta_ydot = fN / mass;
    delta_thetadot = ((xc - x) * fN - (yc - y) * fF) / J;
  }

  // Verify that the new velocity is reasonable.
  DRAKE_DEMAND((ydot + delta_ydot) +
                   k * ctheta * half_rod_length * (thetadot + delta_thetadot) >
               -std::numeric_limits<double>::epsilon() * 10);

  // Update the velocity.
  new_statev->SetAtIndex(3, xdot + delta_xdot);
  new_statev->SetAtIndex(4, ydot + delta_ydot);
  new_statev->SetAtIndex(5, thetadot + delta_thetadot);
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
  const T x = state.GetAtIndex(0);
  const T y = state.GetAtIndex(1);
  const T theta = state.GetAtIndex(2);
  const T xdot = state.GetAtIndex(3);
  const T ydot = state.GetAtIndex(4);
  const T thetadot = state.GetAtIndex(5);

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();
  DRAKE_ASSERT(f != nullptr);

  // The two points of the rod are located at (x,y) + R(theta)*[0,l/2] and
  // (x,y) + R(theta)*[0,-l/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and l is designated as the rod endpoint. Thus, the vertical positions of
  // the rod endpoints are located at y + sin(theta)*l/2 and y - sin(theta)*l/2.
  const T half_rod_length = rod_length_ * 0.5;
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const T k = (stheta > 0.0) ? -1.0 : 1.0;
  const T xc = x + k * ctheta * rod_length_ / 2;
  const T yc = y + k * stheta * rod_length_ / 2;

  // Compute the velocity at the point of contact
  const T xcdot = xdot - k * stheta * rod_length_ / 2 * thetadot;
  const T ycdot = ydot + k * ctheta * rod_length_ / 2 * thetadot;

  // First three derivative components are xdot, ydot, thetadot.
  f->SetAtIndex(0, xdot);
  f->SetAtIndex(1, ydot);
  f->SetAtIndex(2, thetadot);

  // Case 1: the rod is not touching the ground (located at y=0).
  if (yc > std::numeric_limits<T>::epsilon()) {
    // Second three derivative components are simple: just add in gravitational
    // acceleration.
    f->SetAtIndex(3, T(0.));
    f->SetAtIndex(4, get_gravitational_acceleration());
    f->SetAtIndex(5, T(0.));
  } else {
    // Case 2: the rod is touching the ground (or even embedded in the ground).
    // Constraint stabilization should be used to eliminate embedding, but we
    // perform no such check in the derivative evaluation.

    // Verify that the rod is not impacting and not separating.
    DRAKE_DEMAND(ycdot > -std::numeric_limits<T>::epsilon() &&
                 ycdot < std::numeric_limits<T>::epsilon());

    // Handle the two contact case specially.
    if (abs(sin(theta)) < std::numeric_limits<T>::epsilon()) {
      // Verify that the normal velocity is zero.
      DRAKE_DEMAND(abs(ydot) < std::numeric_limits<T>::epsilon() &&
                   abs(thetadot) < std::numeric_limits<T>::epsilon());

      // Look to see whether there is sliding velocity.
      if (abs(xdot) < std::numeric_limits<T>::epsilon()) {
        // Set the time derivatives to "resting".
        f->SetAtIndex(3, T(0.));
        f->SetAtIndex(4, T(0.));
        f->SetAtIndex(5, T(0.));
      } else {
        // This code assumes no sliding will occur with contacts at multiple
        // points unless the system is initialized to such a condition. This
        // assumption has been neither proven nor rigorously validated.
        throw std::logic_error("Sliding detected with non-point contact.");
      }

      return;
    }

    // Compute the normal acceleration at the point of contact (yc_ddot),
    // *assuming zero contact force*.
    T ycddot = get_gravitational_acceleration() -
               k * half_rod_length * stheta * thetadot * thetadot;

    // If this derivative is negative, we must compute the normal force
    // necessary to set it to zero.
    if (ycddot < 0.0) {
      // Look for the case where the tangential velocity is zero.
      if (abs(xcdot) < std::numeric_limits<T>::epsilon()) {
        // Solve for the case where xddot = 0. I've computed these
        // equations by issuing the following command in Mathematica:
        // xc[t_] := x[t] + k*Cos[theta[t]]*(ell/2)
        // yc[t_] := y[t] + k*Sin[theta[t]]*(ell/2)
        // Solve[{0 == D[D[yc[t], t], t],
        //        D[D[y[t], t], t] == fN/mass + g,
        //        D[D[x[t], t], t] == fF/mass,
        //        J*D[D[theta[t], t], t] == (xc[t]-x[t])*fN - (yc[t]-y[t])*fF,
        //        0 == D[D[xc[t], t], t]},
        //       { fN, fF, D[D[y[t], t], t], D[D[x[t], t], t],
        //          D[D[theta[t], t], t] } ]
        // where theta is the counter-clockwise angle the rod makes with the
        // x-axis, fN and fF are contact normal and frictional forces, g is the
        // acceleration due to gravity, and (hopefully) all other variables are
        // self-explanatory.
        const T mu = mu_;
        const T mass = mass_;
        const T ell = rod_length_;
        const T g = get_gravitational_acceleration();
        const T J = J_;
        const T fN =
            (mass *
             (-8 * g * J - 2 * ell * ell * g * k * k * mass * stheta * stheta +
              4 * ell * J * k * stheta * thetadot * thetadot +
              ell * ell * ell * k * k * k * mass * ctheta * ctheta * stheta *
                  thetadot * thetadot +
              ell * ell * ell * k * k * k * mass * stheta * stheta * stheta *
                  thetadot * thetadot)) /
            (2. * (4 * J + ell * ell * k * k * mass * ctheta * ctheta +
                   ell * ell * k * k * mass * stheta * stheta));
        const T fF =
            -(2 * ell * ell * g * k * k * mass * mass * ctheta * stheta -
              4 * ell * J * k * mass * ctheta * thetadot * thetadot -
              ell * ell * ell * k * k * k * mass * mass * ctheta * ctheta *
                  ctheta * thetadot * thetadot -
              ell * ell * ell * k * k * k * mass * mass * ctheta * stheta *
                  stheta * thetadot * thetadot) /
            (2. * (4 * J + ell * ell * k * k * mass * ctheta * ctheta +
                   ell * ell * k * k * mass * stheta * stheta));

        // Sanity check that normal force is non-negative.
        DRAKE_DEMAND(fN >= 0.0);

        // Now that normal force is computed, set the acceleration.
        f->SetAtIndex(3, fF / mass_);
        f->SetAtIndex(4, fN / mass_ + get_gravitational_acceleration());
        f->SetAtIndex(5, ((xc - x) * fN - (yc - y) * fF) / J);

        // Get yddot and xddot and compute xcddot and ycddot.
        const T xddot = f->GetAtIndex(3);
        const T yddot = f->GetAtIndex(4);
        const T thetaddot = f->GetAtIndex(5);

        // Verify that xcddot, ycddot = 0
        const T xcddot =
            xddot +
            ell * k * (-stheta * thetaddot - +ctheta * thetadot * thetadot) / 2;
        ycddot =
            yddot +
            ell * k * (ctheta * thetaddot - stheta * thetadot * thetadot) / 2;

        DRAKE_DEMAND(abs(xcddot) < std::numeric_limits<T>::epsilon());
        DRAKE_DEMAND(abs(ycddot) < std::numeric_limits<T>::epsilon());

        // Constrain F such that it lies on the edge of the friction cone.
        if (abs(fN) > mu * fN) {
          // Pick the solution that minimizes the tangential acceleration.
          const int d1 = 1;
          const T fN1 =
              (2 * mass *
               (-2 * g * J + ell * J * k * stheta * thetadot * thetadot)) /
              (4 * J + ell * ell * k * k * mass * ctheta * ctheta +
               ell * ell * k * k * mass * mu * ctheta * d1 * stheta);
          const T fF1 = -d1 * mu * fN1;
          const T d2 = -1;
          const T fN2 =
              (2 * mass *
               (-2 * g * J + ell * J * k * stheta * thetadot * thetadot)) /
              (4 * J + ell * ell * k * k * mass * ctheta * ctheta +
               ell * ell * k * k * mass * mu * ctheta * d2 * stheta);
          const T fF2 = -d2 * mu * fN2;

          // Compute two theta acceleration candidates.
          const T thetaddot1 = ((xc - x) * fN1 - (yc - y) * fF1) / J;
          const T thetaddot2 = ((xc - x) * fN2 - (yc - y) * fF2) / J;

          // Compute two tangential acceleration candidates.
          const T xcddot1 =
              fF1 / mass +
              ell * k * (-stheta * thetaddot1 - +ctheta * thetadot * thetadot) /
                  2;
          const T xcddot2 =
              fF2 / mass +
              ell * k * (-stheta * thetaddot2 - +ctheta * thetadot * thetadot) /
                  2;

          // Pick the one that is smaller in magnitude.
          if (abs(xcddot1) < abs(xcddot2)) {
            f->SetAtIndex(3, fF1 / mass_);
            f->SetAtIndex(4, fN1 / mass_ + get_gravitational_acceleration());
            f->SetAtIndex(5, ((xc - x) * fN1 - (yc - y) * fF1) / J);
          } else {
            f->SetAtIndex(3, fF2 / mass_);
            f->SetAtIndex(4, fN2 / mass_ + get_gravitational_acceleration());
            f->SetAtIndex(5, ((xc - x) * fN2 - (yc - y) * fF2) / J);
          }
        }
      } else {
        // Rod is sliding at the point of contact.
        // These equations were determined by issuing the following
        // commands in Mathematica:
        // xc[t_] := x[t] + k*Cos[theta[t]]*(ell/2)
        // yc[t_] := y[t] + k*Sin[theta[t]]*(ell/2)
        // Solve[{0 == D[D[yc[t], t], t],
        //        D[D[y[t], t], t] == fN/mass + g,
        //        D[D[x[t], t], t] == fF/mass,
        //        J*D[D[theta[t], t], t] == (xc[t]-x[t])*fN - (yc[t]-y[t])*fF,
        //        fF == -sgn_xcdot*mu*fN},
        // { fN, fF, D[D[y[t], t], t], D[D[x[t], t], t], D[D[theta[t], t], t]}]
        // where theta is the counter-clockwise angle the rod makes with the
        // x-axis, 'ell' is the length of the rod, fN and fF are normal and
        // frictional forces, respectively, sgn_xcdot = sgn(xcdot), g is the
        // acceleration due to gravity, and (hopefully) all other variables are
        // self-explanatory.
        const T J = J_;
        const T mass = mass_;
        const T mu = mu_;
        const T sgn_xcdot = (xcdot > 0.0) ? 1.0 : -1.0;
        const T g = get_gravitational_acceleration();
        const T ell = rod_length_;
        T fN = (2 * mass *
                (-2 * g * J + ell * J * k * stheta * thetadot * thetadot)) /
               (4 * J + ell * ell * k * k * mass * ctheta * ctheta +
                ell * ell * k * k * mass * mu * ctheta * sgn_xcdot * stheta);

        // Check for inconsistent configuration.
        if (fN < 0.0)
          throw std::runtime_error("Inconsistent configuration detected.");

        // Now that normal force is computed, set the acceleration.
        const T fF = -sgn_xcdot * mu_ * fN;
        f->SetAtIndex(3, fF / mass_);
        f->SetAtIndex(4, fN / mass_ + get_gravitational_acceleration());
        f->SetAtIndex(5, ((xc - x) * fN - (yc - y) * fF) / J);

        // Compute the normal acceleration at the contact point (a check).
        const T yddot = f->GetAtIndex(4);
        const T thetaddot = f->GetAtIndex(5);
        ycddot =
            yddot +
            ell * k * (ctheta * thetaddot - stheta * thetadot * thetadot) / 2;

        // Lines below currently unused but are occasionally helpful for
        // debugging.
        //        const T xddot = f->GetAtIndex(3);
        //        const T xcddot = xddot + ell*k*(-stheta*thetaddot -
        //                                        +ctheta*thetadot*thetadot)/2;

        // Verify that the normal acceleration is zero.
        DRAKE_DEMAND(abs(ycddot) < std::numeric_limits<T>::epsilon() * 10);
      }
    }
  }
}

template <typename T>
void Painleve<T>::SetDefaultState(const systems::Context<T>& context,
                                  systems::State<T>* state) const {
  using std::sqrt;
  const T half_len = get_rod_length() / 2;
  VectorX<T> x0(6);
  const T r22 = sqrt(2) / 2;
  x0 << half_len * r22, half_len * r22, M_PI / 4.0, -1, 0, 0;  // Initial state.
  if (dt_ == 0)
    state->get_mutable_continuous_state()->SetFromVector(x0);
  else
    state->get_mutable_discrete_state()->get_mutable_discrete_state(0)->
        SetFromVector(x0);
}

}  // namespace painleve
}  // namespace drake
