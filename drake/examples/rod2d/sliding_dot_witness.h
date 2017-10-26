#pragma once

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rigid_contact.h"

#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event.h"
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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SlidingDotWitness)

  SlidingDotWitness(const Rod2D<T>* rod, int contact_index) :
      systems::WitnessFunction<T>(*rod,
          systems::WitnessFunctionDirection::kPositiveThenNonPositive),
      rod_(rod),
      contact_index_(contact_index) {
    this->name_ = "SlidingDot";
    event_ = std::make_unique<systems::UnrestrictedUpdateEvent<T>>(
      systems::Event<T>::TriggerType::kWitness);
  }

 private:
  /// The witness function itself.
  T DoEvaluate(const systems::Context<T>& context) const override {
    using std::sin;

    // Verify the system is simulated using piecewise DAE.
    DRAKE_DEMAND(rod_->get_simulation_type() ==
        Rod2D<T>::SimulationType::kPiecewiseDAE);

    // Get the contact information.
    const auto& contact =
        rod_->get_contacts(context.get_state())[contact_index_];

    // Verify rod is undergoing sliding contact at the specified index.
    DRAKE_DEMAND(contact.sliding);

    // Compute the translational velocity at the point of contact.
    const Vector2<T> pdot = rod_->CalcContactVelocity(context.get_state(),
                                                      contact);

    // Return the tangent velocity.
    return pdot[0];
  }

  void DoAddEvent(systems::CompositeEventCollection<T>* events) const override {
    event_->add_to_composite(events);
  }

  /// Unique pointer to the event.
  std::unique_ptr<systems::UnrestrictedUpdateEvent<T>> event_;

  /// Pointer to the rod system.
  const Rod2D<T>* rod_;

  /// Index of the contact point that this witness function applies to.
  int contact_index_{-1};
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

