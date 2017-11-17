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
}

// Sets the rod's state to ballistic.
template <class T>
void Rod2D<T>::SetBallisticMode(systems::State<T>* state) const {
  // Remove all contacts from force calculations.
  std::vector<multibody::constraint::PointContact>& contacts =
      get_contacts(state);
  contacts.clear();

  // Disable all witness functions except the signed distance one.
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
std::vector<multibody::constraint::PointContact>& Rod2D<T>::get_contacts(
    systems::State<T>* state) const {
  return state->get_mutable_abstract_state()
      .get_mutable_value(kContactAbstractIndex)
      .template GetMutableValue<std::vector<multibody::constraint::PointContact>>();
}

template <class T>
const std::vector<multibody::constraint::PointContact>& Rod2D<T>::get_contacts(
    const systems::State<T>& state) const {
  return state.get_abstract_state()
      .get_value(kContactAbstractIndex).
      template GetValue<std::vector<multibody::constraint::PointContact>>();
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
*/

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

// Models any impacts for the piecewise-DAE based system.
// @param[in,out] state the pre-impact state on entry, the post-impact state
//                on return.
// @param[out] zero_tol if non-null, contains the determined zero tolerance
//             on return.
template <class T>
void Rod2D<T>::ModelImpact(systems::State<T>* state,
                           T* zero_tol) const {
  DRAKE_DEMAND(state);
/*
  // Get state variables.
  const VectorX<T> q = state->get_continuous_state()->
      get_generalized_position().CopyToVector();
  systems::VectorBase<T>* qdot = state->get_mutable_continuous_state()->
      get_mutable_generalized_velocity();

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
*/
}

template <class T>
int Rod2D<T>::GetContactArrayIndex(
    const systems::State<T>& state,
    int contact_candidate_index) const {
  // Get the vector of contacts.
  const std::vector<multibody::constraint::PointContact>& contacts =
      get_contacts(state);

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
    int contact_index, systems::State<T>* state) const {
  auto witnesses = state->get_mutable_abstract_state()
      .get_mutable_value(kSignedDistanceWitnessAbstractIndex)
      .template GetMutableValue<std::vector<RodWitnessFunction<T>*>>();
  for (int i = 0; i < static_cast<int>(witnesses.size()); ++i)
    if (witnesses[i]->get_contact_index() == contact_index)
      return witnesses[i];

  throw std::logic_error("Missing signed distance witness.");
  return nullptr;
}

template <class T>
RodWitnessFunction<T>* Rod2D<T>::GetNormalAccelWitness(
    int contact_index, systems::State<T>* state) const {
  auto witnesses = state->get_mutable_abstract_state()
      .get_mutable_value(kNormalAccelWitnessAbstractIndex)
      .template GetMutableValue<std::vector<RodWitnessFunction<T>*>>();
  for (int i = 0; i < static_cast<int>(witnesses.size()); ++i)
    if (witnesses[i]->get_contact_index() == contact_index)
      return witnesses[i];

  throw std::logic_error("Missing normal acceleration witness.");
  return nullptr;
}

template <class T>
RodWitnessFunction<T>* Rod2D<T>::GetNormalVelWitness(
    int contact_index, systems::State<T>* state) const {
  auto witnesses = state->get_mutable_abstract_state()
      .get_mutable_value(kNormalVelWitnessAbstractIndex)
      .template GetMutableValue<std::vector<RodWitnessFunction<T>*>>();
  for (int i = 0; i < static_cast<int>(witnesses.size()); ++i)
    if (witnesses[i]->get_contact_index() == contact_index)
      return witnesses[i];

  throw std::logic_error("Missing normal velocity witness.");
  return nullptr;
}

/// The velocity tolerance for sliding.
template <class T>
RodWitnessFunction<T>* Rod2D<T>::GetStickingFrictionForceSlackWitness(
    int contact_index, systems::State<T>* state) const {
  auto witnesses = state->get_mutable_abstract_state()
      .get_mutable_value(kStickingFrictionForceSlackWitnessAbstractIndex)
      .template GetMutableValue<std::vector<RodWitnessFunction<T>*>>();
  for (int i = 0; i < static_cast<int>(witnesses.size()); ++i)
    if (witnesses[i]->get_contact_index() == contact_index)
      return witnesses[i];

  throw std::logic_error("Sticking friction force slack witness.");
  return nullptr;
}

// Gets the time derivative of a 2D rotation matrix.
template <class T>
RodWitnessFunction<T>* Rod2D<T>::GetSlidingDotWitness(
    int contact_index, systems::State<T>* state) const {
  auto witnesses = state->get_mutable_abstract_state()
      .get_mutable_value(kSlidingDotWitnessAbstractIndex)
      .template GetMutableValue<std::vector<RodWitnessFunction<T>*>>();
  for (int i = 0; i < static_cast<int>(witnesses.size()); ++i)
    if (witnesses[i]->get_contact_index() == contact_index)
      return witnesses[i];

  throw std::logic_error("Sliding velocity dot witness.");
  return nullptr;
}

template <class T>
RodWitnessFunction<T>* Rod2D<T>::GetNormalForceWitness(
    int contact_index, systems::State<T>* state) const {
  auto witnesses = state->get_mutable_abstract_state()
      .get_mutable_value(kNormalForceWitnessAbstractIndex)
      .template GetMutableValue<std::vector<RodWitnessFunction<T>*>>();
  for (int i = 0; i < static_cast<int>(witnesses.size()); ++i)
    if (witnesses[i]->get_contact_index() == contact_index)
      return witnesses[i];

  throw std::logic_error("Missing normal force witness.");
  return nullptr;
}

template <class T>
RodWitnessFunction<T>* Rod2D<T>::GetPosSlidingWitness(
    int contact_index, systems::State<T>* state) const {
  auto witnesses = state->get_mutable_abstract_state()
      .get_mutable_value(kPosSlidingWitnessAbstractIndex)
      .template GetMutableValue<std::vector<RodWitnessFunction<T>*>>();
  for (int i = 0; i < static_cast<int>(witnesses.size()); ++i)
    if (witnesses[i]->get_contact_index() == contact_index)
      return witnesses[i];

  throw std::logic_error("Missing positive sliding witness.");
  return nullptr;
}

template <class T>
RodWitnessFunction<T>* Rod2D<T>::GetNegSlidingWitness(
    int contact_index, systems::State<T>* state) const {
  auto witnesses = state->get_mutable_abstract_state()
      .get_mutable_value(kNegSlidingWitnessAbstractIndex)
      .template GetMutableValue<std::vector<RodWitnessFunction<T>*>>();
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
  // Get the current configuration of the system.
  const VectorX<T> q = state->get_continuous_state().
      get_generalized_position().CopyToVector();

  // Copy the state in the context into the state.
  state->CopyFrom(context.get_state());

  // Get the vector of contacts.
  std::vector<multibody::constraint::PointContact>& contacts =
      get_contacts(state);

  // Indicate there are no impacts.
  bool impacting = false;

  // Process all events.
  for (int i = 0; i < static_cast<int>(events.size()); ++i) {
    DRAKE_DEMAND(events[i]->has_attribute());

    // Get the attribute as a RodWitnessFunction type.
    auto const_witness = dynamic_cast<const RodWitnessFunction<T>*>(
        events[i]->get_attribute());
    auto witness = const_cast<RodWitnessFunction<T>*>(const_witness);

    // Get the contact (candidate) index.
    const int contact_index = witness->get_contact_index();

    // Get the index of the contact in the contacts array.
    const int contact_array_index = GetContactArrayIndex(*state, contact_index);

    switch (witness->get_witness_function_type()) {
      case RodWitnessFunction<T>::WitnessType::kSignedDistance:  {
        // If the witness is a signed distance witness, the point should either
        // no longer be tracked or it should newly be tracked.
        // See whether the point is tracked.
        if (contact_array_index < 0) {
          // Indicate that an impact is occurring.
          impacting = true;

          // Add the contact.
          AddContactToForceCalculationSet(contact_index, state);
        } else {
          // The point is part of the force calculation set. Remove it.
          contacts[contact_array_index] = contacts.back();
          contacts.pop_back();

          // Deactivate all but signed distance function witness.
          GetNormalForceWitness(contact_index, state)->set_enabled(false);
          GetNormalAccelWitness(contact_index, state)->set_enabled(false);
          GetNormalVelWitness(contact_index, state)->set_enabled(false);
          GetSlidingDotWitness(contact_index, state)->set_enabled(false);
          GetStickingFrictionForceSlackWitness(contact_index, state)->
              set_enabled(false);
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
        AddContactToForceCalculationSet(contact_index, state);

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

// TODO: fix this.
const double normal_velocity_positive = 0;
        if (normal_velocity_positive <= 0) {
          // Add the contact.
          AddContactToForceCalculationSet(contact_index, state);
        } else {
          GetNormalVelWitness(contact_index, state)->set_enabled(true);
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
        contacts[contact_array_index] = contacts.back();
        contacts.pop_back();

        // Activate the normal acceleration witness function. Deactivate the
        // normal force witness.
        witness->set_enabled(false);
        GetNormalAccelWitness(contact_index, state)->set_enabled(true);

        break;
      }

      case RodWitnessFunction<T>::WitnessType::kSlidingWitness:  {
        // Disable the sliding velocity witnesses. 
        GetPosSlidingWitness(contact_index, state)->set_enabled(false);
        GetNegSlidingWitness(contact_index, state)->set_enabled(false);

        // Enable the sliding-dot witness.
        GetSlidingDotWitness(contact_index, state)->set_enabled(true);

        break;
      }

      case RodWitnessFunction<T>::WitnessType::kSlidingDot:
      case RodWitnessFunction<T>::WitnessType::kStickingFrictionForceSlack:  {
        // Disable the sliding dot / sticking friction forces slack witness
        // function.
        witness->set_enabled(false);

        // Enable the sliding velocity witnesses. 
        GetPosSlidingWitness(contact_index, state)->set_enabled(true);
        GetNegSlidingWitness(contact_index, state)->set_enabled(true);

        // Mark the contact as sliding.
        contacts[contact_array_index].sliding = true;

        break;
      }
    } 
  } 

  // Note that if the rod were contacting, for example, a box rather than a
  // halfspace, we would need to deactivate contact points as they slid off of
  // the edge of the box.

  // Handle any impacts, then redetermine which contacts are sliding and which
  // are not sliding.
  if (impacting) {
    T zero_tol;
    ModelImpact(state, &zero_tol);
  }
}

// Adds a contact to those used for force calculations. 
template <class T>
void Rod2D<T>::AddContactToForceCalculationSet(
    int contact_index,
    systems::State<T>* state) const {

  // Get the vector of contacts.
  std::vector<multibody::constraint::PointContact>& contacts =
      get_contacts(state);

  // Add the contact.
  contacts.push_back(multibody::constraint::PointContact());

  // Set the contact index.
  contacts.back().id = reinterpret_cast<void*>(contact_index);

  // See whether the contact is sliding or not.
  contacts.back().sliding = !IsTangentVelocityZero(*state, contacts.back());

  // Activate the normal force witness.
  GetNormalForceWitness(contact_index, state)->set_enabled(true);

  // Neither of the significant velocity sliding witnesses need be enabled.
  GetPosSlidingWitness(contact_index, state)->set_enabled(false);
  GetNegSlidingWitness(contact_index, state)->set_enabled(false);

  // If the contact is sliding, activate the sliding dot witness.
  if (contacts.back().sliding) {
    GetSlidingDotWitness(contact_index, state)->set_enabled(true);
    GetStickingFrictionForceSlackWitness(
        contact_index, state)->set_enabled(false);
  } else {
    GetSlidingDotWitness(contact_index, state)->set_enabled(false);
    GetStickingFrictionForceSlackWitness(
        contact_index, state)->set_enabled(true);
  }
}

// Calculates the velocity at a point of contact.
template <class T>
Vector2<T> Rod2D<T>::CalcContactVelocity(
    const systems::State<T>& state,
    const multibody::constraint::PointContact& c) const {

  // TODO: Store rod parameters in the Context.

  // The point of contact is x + R * u, so it's velocity is
  // dx/dt + Rdot * u * thetadot.
  const long id = reinterpret_cast<long>(c.id);
  const auto v = state.get_continuous_state().get_generalized_velocity().
      CopyToVector();
  const Vector2<T> dxdt = v.segment(0, 2);
  const T& thetadot = v[2];
  const Matrix2<T> Rdot = get_rotation_matrix_derivative(state);
  return dxdt + Rdot * contact_candidates_[id] * thetadot;
}

// Gets the zero tolerance for tangent velocity for a point of contact.
template <class T>
bool Rod2D<T>::IsTangentVelocityZero(const systems::State<T>& state,
                                     const multibody::constraint::PointContact& c) const {
  using std::abs;

  // TODO(edrumwri): Do a proper test that uses integrator tolerances and
  // distance of the c.o.m. from the contact point to determine a coherent
  // tolerance.
  const T tol = 1e-10;

  // Test tangent velocity.
  const Vector2<T> pdot = CalcContactVelocity(state, c);
  return (abs(pdot[0]) < tol);
}

template <class T>
void Rod2D<T>::DoGetWitnessFunctions(const systems::Context<T>& context,
    std::vector<const systems::WitnessFunction<T>*>* witness_functions) const {
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

/// Integrates the Rod 2D example forward in time using a
/// half-explicit time stepping scheme.
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
    CalcConstraintProblemData(context, &problem_data);

    // TODO: We need to determine when to use the LCP solver.

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
      auto witnesses = abstract_data.back()->template GetMutableValue<std::vector<RodWitnessFunction<T>*>>();
     
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

        case kSlidingDotWitnessAbstractIndex:
          witnesses.push_back(new SlidingDotWitness<T>(this, 0));
          witnesses.push_back(new SlidingDotWitness<T>(this, 1));
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
  x0 << half_len * r22, half_len * r22, M_PI / 4.0, -1, 0, 0;  // Initial state.
  if (simulation_type_ == SimulationType::kTimeStepping) {
    state->get_mutable_discrete_state().get_mutable_vector(0)
        .SetFromVector(x0);
  } else {
    // Continuous variables.
    state->get_mutable_continuous_state().SetFromVector(x0);

    // Set abstract variables for piecewise DAE approach.
    if (simulation_type_ == SimulationType::kPiecewiseDAE) {
      // TODO: Complete this.

      // Initialize the vector of contact points.
      auto& contacts = get_contacts(state);

      // Indicate that the rod is in the single contact sliding mode.
      contacts.resize(1);

      // First contact candidate is Rl in Rod Frame (see class documentation);
      // in the default rod configuration, Rl contacts the half-space and is
      // sliding.
      contacts.front().sliding = true;
      contacts.front().id = 0;
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
