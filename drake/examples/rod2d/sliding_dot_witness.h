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
  SlidingDotWitness(const Rod2D<T>* rod, int contact_index) :
      systems::WitnessFunction<T>(rod), rod_(rod),
      contact_index_(contact_index) {
    this->name_ = "SlidingDot";
  }

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

    // Compute the translational velocity at the point of contact.
    const Vector2<T> pdot = rod_->CalcContactVelocity(context.get_state(),
                                                      contact);

    // Return the tangent velocity.
    return pdot[0];
  }

  // Time tolerance- we want to ensure that the zero is isolated within the
  // dead bands (i.e., zero finding should not terminate based on this
  // tolerance).
  T get_time_isolation_tolerance() const override {
    return 0;
  }

  // Informed dead-band values (dictated by Rod2D::IsTangentVelocityZero()).
  T get_positive_dead_band() const override {
    using std::sqrt;
    return 1e-10;
  }

  // Informed dead-band values (dictated by Rod2D::IsTangentVelocityZero()).
  T get_negative_dead_band() const override {
    using std::sqrt;
    return -1e-10;
  }

 private:
  /// Pointer to the rod system.
  const Rod2D<T>* rod_;

  /// Index of the contact point that this witness function applies to.
  int contact_index_{-1};
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

