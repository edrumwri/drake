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

/// Computes the signed distance between a point of contact and the half-space.
template <class T>
class SignedDistanceWitness : public systems::WitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SignedDistanceWitness)

  SignedDistanceWitness(const Rod2D<T>* rod, int contact_index) :
      systems::WitnessFunction<T>(*rod,
          systems::WitnessFunctionDirection::kPositiveThenNonPositive),
      rod_(rod),
      contact_index_(contact_index) {
    this->name_ = "SignedDistance";
    event_ = std::make_unique<systems::UnrestrictedUpdateEvent<T>>(
      systems::Event<T>::TriggerType::kWitness);
  }

  // Gets the contact index for this witness function.
  int get_contact_index() const { return contact_index_; }

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

