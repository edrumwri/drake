#pragma once

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rigid_contact.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace examples {
namespace rod2d {

template <class T>
class Rod2D;

/// Computes the tangent velocity at a point of contact.
template <class T>
class SlidingDotWitness : public systems::WitnessFunction<T> {
 public:
  SlidingDotWitness(Rod2D<T>* rod, int contact_index) :
      rod_(rod), contact_index_(contact_index) {}

  /// This witness function indicates an unrestricted update needs to be taken.
  typename systems::DiscreteEvent<T>::ActionType get_action_type()
  const override {
    return systems::DiscreteEvent<T>::ActionType::kUnrestrictedUpdateAction;
  }

  /// This witness triggers whenever the velocity changes direction.
  typename systems::WitnessFunction<T>::TriggerType get_trigger_type()
      const override {
    return systems::WitnessFunction<T>::TriggerType::kCrossesZero;
  }

  // Select the trigger time for this witness function to bisect the two
  // time values.
  T do_get_trigger_time(const std::pair<T, T>& time_and_witness_value0,
                        const std::pair<T, T>& time_and_witness_valuef)
  const override {
    return (time_and_witness_value0.first + time_and_witness_valuef.first) / 2;
  }

  /// The witness function itself.
  T Evaluate(const systems::Context<T>& context) override {
    using std::sin;

    // Verify the system is simulated using piecewise DAE.
    DRAKE_DEMAND(rod_->get_simulation_type() ==
        Rod2D<T>::SimulationType::kPiecewiseDAE);

    // Get the contact information.
    const RigidContact& contact =
        rod_->get_contacts(context.get_state())[contact_index_];

    // Verify rod is undergoing sliding contact at the specified index.
    DRAKE_DEMAND(contact.state ==
        RigidContact::ContactState::kContactingAndSliding);

    // Get the relevant parts of the state.
    const systems::VectorBase<T>& state = context.get_continuous_state_vector();
    const T& xdot = state.GetAtIndex(3);
    const T& ydot = state.GetAtIndex(4);
    const T& thetadot = state.GetAtIndex(5);

    // Compute the velocity at the point of contact.
    const Vector2<T> v(xdot, ydot);

    // Get the time derivative of the rotation matrix.
    const Matrix2<T> Rdot = rod_->get_rotation_matrix_derivative(
        context.get_state());

    // Compute the translational velocity at the point of contact.
    const Vector2<T> pdot = v + Rdot * contact.u.segment(0,2) * thetadot;

    // Return the tangent velocity.
    return pdot[0];
  }

  // Uninformed time tolerance.
  T get_time_isolation_tolerance() const override {
    using std::sqrt;
    return sqrt(std::numeric_limits<double>::epsilon());
  }

  // Uninformed dead-band values and time tolerance.
  T get_positive_dead_band() const override {
    using std::sqrt;
    return sqrt(std::numeric_limits<double>::epsilon());
  }

  // Uninformed dead-band values and time tolerance.
  T get_negative_dead_band() const override {
    using std::sqrt;
    return sqrt(std::numeric_limits<double>::epsilon());
  }

 private:
  /// Pointer to the rod system.
  Rod2D<T>* rod_;

  /// Index of the contact point that this witness function applies to.
  int contact_index_{-1};
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

