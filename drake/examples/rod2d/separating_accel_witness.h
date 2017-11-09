#pragma once

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rod_witness_function.h"

#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event.h"

namespace drake {
namespace examples {
namespace rod2d {

/// Witness function for determining whether a point of contact is accelerating
/// away from the half-space.
template <class T>
class SeparatingAccelWitness : public RodWitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SeparatingAccelWitness)

  SeparatingAccelWitness(const Rod2D<T>* rod, int contact_index) :
      RodWitnessFunction<T>(
          rod,
          systems::WitnessFunctionDirection::kPositiveThenNonPositive,
          contact_index) {
    this->name_ = "SeparatingAccel";
    solver_ = &rod->solver_;
  }

  bool is_separating_accel_witness() const override { return true; }

 private:
  /// The witness function itself.
  T DoEvaluate(const systems::Context<T>& context) const override {
    using std::sin;

    // Get the rod system.
    const Rod2D<T>& rod = get_rod();

    // Verify the system is simulated using piecewise DAE.
    DRAKE_DEMAND(rod.get_simulation_type() ==
        Rod2D<T>::SimulationType::kPiecewiseDAE);

    // TODO(edrumwri): Speed this up (presumably) using caching. 

    // Populate problem data and solve the contact problem.
    const int ngv = 3;  // Number of rod generalized velocities.
    VectorX<T> cf;
    multibody::constraint::ConstraintAccelProblemData<T> problem_data(ngv);
    rod.CalcConstraintProblemData(context, &problem_data);
    solver_->SolveConstraintProblem(problem_data, &cf);

    // Return the normal force. A negative value means that the force has
    // become tensile, which violates the compressivity constraint.
    return cf[get_contact_index()];
  }

  /// Pointer to the rod's constraint solver.
  const multibody::constraint::ConstraintSolver<T>* solver_;
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

