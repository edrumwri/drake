#pragma once

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rod_witness_function.h"

#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace rod2d {

/// Determines whether the tangent velocity is above the sliding velocity
/// threshold.
template <class T>
class SlidingWitness : public RodWitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SlidingWitness)

  SlidingWitness(
      const Rod2D<T>* rod,
      int contact_index,
      bool pos_direction,
      double sliding_velocity_threshold) :
      RodWitnessFunction<T>(
          rod,
          systems::WitnessFunctionDirection::kPositiveThenNonPositive,
          contact_index) {
    this->name_ = "Sliding";
  }

  typename RodWitnessFunction<T>::WitnessType
      get_witness_function_type() const override {  
    return RodWitnessFunction<T>::WitnessType::kSlidingWitness;
  }

 private:
  /// The witness function itself.
  T DoEvaluate(const systems::Context<T>& context) const override {
    using std::sin;

    // Get the rod system.
    const Rod2D<T>& rod = this->get_rod();

    // Verify the system is simulated using piecewise DAE.
    DRAKE_DEMAND(rod.get_simulation_type() ==
        Rod2D<T>::SimulationType::kPiecewiseDAE);

    // Get the contact information.
    const int contact_index = this->get_contact_index();
    const auto& contact = rod.get_contacts_used_in_force_calculations(
        context.get_state())[contact_index];

    // Verify rod is undergoing sliding contact at the specified index.
    DRAKE_DEMAND(contact.sliding);

    // Compute the translational velocity at the point of contact.
    const Vector2<T> pdot = rod.CalcContactVelocity(context,
                                                    contact_index);

    // Return the tangent velocity.
    if (positive_) {
      return velocity_threshold_ - pdot[0];
    } else {
      return -pdot[0] - velocity_threshold_;
    }
  }

  // If 'true', witness function triggers when the sliding velocity is
  // sufficiently positive. Otherwise, it triggers when the sliding velocity
  // is sufficiently negative.
  bool positive_{false};

  // The contact is only to be considered as properly sliding once this
  // threshold has been met.
  double velocity_threshold_{10 * std::numeric_limits<double>::epsilon()};
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

