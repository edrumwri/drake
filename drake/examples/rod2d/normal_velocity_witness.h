#pragma once

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rod_witness_function.h"

#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event.h"

namespace drake {
namespace examples {
namespace rod2d {

/// Witness function for determining whether a point of contact is moving. 
/// away from the half-space.
template <class T>
class NormalVelocityWitness : public RodWitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NormalVelocityWitness)

  NormalVelocityWitness(const Rod2D<T>* rod, int contact_index) :
      RodWitnessFunction<T>(
          rod,
          systems::WitnessFunctionDirection::kPositiveThenNonPositive,
          contact_index) {
    this->name_ = "NormalVelocity";
    solver_ = &rod->solver_;
  }

  WitnessType get_witness_function_type() const override {  
    return WitnessType::kNormalVelocity;
  }

 private:
  /// The witness function itself.
  T DoEvaluate(const systems::Context<T>& context) const override {
    using std::sin;

    // Get the rod system.
    const Rod2D<T>& rod = get_rod();

    // Verify the system is simulated using piecewise DAE.
    DRAKE_DEMAND(rod.get_simulation_type() ==
        Rod2D<T>::SimulationType::kPiecewiseDAE);

    // TODO: Get the velocity at the tracked point.
    
    DRAEK_DEMAND(false);
  }

  /// Pointer to the rod's constraint solver.
  const multibody::constraint::ConstraintSolver<T>* solver_;
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

