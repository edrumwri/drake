#pragma once

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rod_witness_function.h"

#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace rod2d {

/// Computes the tangent velocity at a point of contact.
template <class T>
class SlidingDotWitness : public RodWitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SlidingDotWitness)

  SlidingDotWitness(const Rod2D<T>* rod, int contact_index) :
      RodWitnessFunction<T>(
          rod,
          systems::WitnessFunctionDirection::kPositiveThenNonPositive,
          contact_index) {
    this->name_ = "SlidingDot";
  }

  typename RodWitnessFunction<T>::WitnessType
      get_witness_function_type() const override {  
    return RodWitnessFunction<T>::WitnessType::kSlidingDot;
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
    return pdot[0];
  }
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

