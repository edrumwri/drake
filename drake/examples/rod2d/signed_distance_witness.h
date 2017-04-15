#pragma once

#include "drake/example/rod2d/rod2d.h"
#include "drake/example/rod2d/rigid_contact.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace examples {
namespace rod2d {

/// Computes the signed distance between a point of contact and the half-space.
template <class T>
class SignedDistanceWitness : public systems::WitnessFunction<T> {
 public:
  SignedDistanceWitness(Rod2D<T>* rod, int contact_index) :
      rod_(rod), contact_index_(contact_index) {}

  /// This witness function indicates an unrestricted update needs to be taken.
  systems::ActionType<T> get_action_type() const override {
    return systems::ActionType<T>::kUnrestrictedUpdateAction;
  }

  /// This witness triggers only when the signed distance goes from strictly
  /// positive to zero/negative.
  TriggerType get_trigger_type() const override {
    return systems::WitnessFunction::TriggerType::kPositiveThenNegative;
  }

  /// The witness function itself.
  T Evaluate(const systems::Context<T>& context) override {
    using std::sin;

    // Verify the system is simulated using piecewise DAE.
    DRAKE_DEMAND(rod->get_simulation_type() ==
        Rod2D<T>::SimulationType::kPiecewiseDAE);

    // Get the contact information.
    const RigidContact& contact = rod_->get_contacts(context)[contact_index_];

    // Verify rod is not in contact at the specified contact index.
    DRAKE_DEMAND(contact.state == RigidContact::ContactState::kNotContacting);

    // Get the relevant parts of the state.
    const systems::VectorBase<T>& state = context.get_continuous_state_vector();
    const Vector2<T> q = context.get_continuous_state()->
        get_generalized_position().CopyToVector().segment(0,2);

    // Get the relevant point on the rod in the world frame.
    const Eigen::Rotation2D<T> R(q(2));
    const Vector2<T> x(q(0), q(1));
    const Vector2<T> p = v + R * contacts[i].u.segment(0,2);

    // Return the vertical location.
    return p[1];
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

