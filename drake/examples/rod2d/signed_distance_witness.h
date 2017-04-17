#pragma once

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rigid_contact.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/discrete_event.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace examples {
namespace rod2d {

template <class T>
class Rod2D;

/// Computes the signed distance between a point of contact and the half-space.
template <class T>
class SignedDistanceWitness : public systems::WitnessFunction<T> {
 public:
  SignedDistanceWitness(Rod2D<T>* rod, int contact_index) :
      rod_(rod), contact_index_(contact_index) {}

  // Gets the contact index for this witness function.
  int get_contact_index() const { return contact_index_; }

  /// This witness function indicates an unrestricted update needs to be taken.
  typename systems::DiscreteEvent<T>::ActionType get_action_type()
      const override {
    return systems::DiscreteEvent<T>::ActionType::kUnrestrictedUpdateAction;
  }

  /// This witness triggers only when the signed distance goes from strictly
  /// positive to zero/negative.
  typename systems::WitnessFunction<T>::TriggerType get_trigger_type()
      const override {
    return systems::WitnessFunction<T>::TriggerType::kPositiveThenNegative;
  }

  // Select the trigger time for this witness function to always go with the
  // earlier time.
  T do_get_trigger_time(const std::pair<T, T>& time_and_witness_value0,
                        const std::pair<T, T>& time_and_witness_valuef)
                        const override {
    return time_and_witness_value0.first;
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

    // Get the relevant parts of the state.
    const Vector3<T> q = context.get_continuous_state()->
        get_generalized_position().CopyToVector();

    // Get the relevant point on the rod in the world frame.
    const Eigen::Rotation2D<T> R(q(2));
    const Vector2<T> x(q(0), q(1));
    const Vector2<T> p = x + R * contact.u.segment(0,2);

    // Return the vertical location.
    return p[1];
  }

  // Uninformed time tolerance.
  T get_time_isolation_tolerance() const override {
    return std::numeric_limits<double>::epsilon();
  }

  // Dead-band value here should be strictly zero to match true solution.
  T get_positive_dead_band() const override { return
        std::numeric_limits<double>::epsilon(); }

  // Uninformed dead-band values and time tolerance.
  T get_negative_dead_band() const override { return
        -std::numeric_limits<double>::epsilon(); }

 private:
  /// Pointer to the rod system.
  Rod2D<T>* rod_;

  /// Index of the contact point that this witness function applies to.
  int contact_index_{-1};
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

