#pragma once

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rigid_contact.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace examples {
namespace rod2d {

template <class T>
class Rod2D;

/// Witness function for determining whether a point of contact is accelerating
/// away from the half-space.
template <class T>
class SeparatingAccelWitness : public systems::WitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SeparatingAccelWitness)

  SeparatingAccelWitness(const Rod2D<T>* rod, int contact_index) :
      systems::WitnessFunction<T>(*rod,
          systems::WitnessFunctionDirection::kPositiveThenNonPositive),
      rod_(rod),
      contact_index_(contact_index) {
    event_ = std::make_unique<systems::UnrestrictedUpdateEvent<T>>(
      systems::Event<T>::TriggerType::kWitness);
    this->name_ = "SeparatingAccel";
  }

 private:
  /// The witness function itself.
  T DoEvaluate(const systems::Context<T>& context) const override {
    using std::sin;

    // Verify the system is simulated using piecewise DAE.
    DRAKE_DEMAND(rod_->get_simulation_type() ==
        Rod2D<T>::SimulationType::kPiecewiseDAE);

    // Get the contact information.
    const RigidContact& contact =
        rod_->get_contacts(context.get_state())[contact_index_];

    // Verify rod is undergoing contact at the specified index.
    DRAKE_DEMAND(contact.state != RigidContact::ContactState::kNotContacting);

    // TODO(edrumwri): Only compute this once over the entire set
    //                 of witness functions generally.

    // Populate problem data and solve the contact problem.
    RigidContactAccelProblemData<T> problem_data;
    const auto cf = rod_->SolveContactProblem(context, &problem_data);

    // Return the normal force. A negative value means that the force has
    // become tensile, which violates the compressiveness constraint.
    return cf[contact_index_];
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

