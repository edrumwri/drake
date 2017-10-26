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

template <class T>
class StickingFrictionForcesSlackWitness : public systems::WitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StickingFrictionForcesSlackWitness)

  StickingFrictionForcesSlackWitness(const Rod2D<T>* rod, int contact_index) :
      systems::WitnessFunction<T>(*rod, 
          systems::WitnessFunctionDirection::kPositiveThenNonPositive),
          rod_(rod),
          contact_index_(contact_index) {
    this->name_ = "StickingFrictionForcesSlack";
    event_ = std::make_unique<systems::UnrestrictedUpdateEvent<T>>(
      systems::Event<T>::TriggerType::kWitness);
    solver_ = &rod->solver_;
  }

 private:
  /// The witness function itself.
  T DoEvaluate(const systems::Context<T>& context) const override {
    using std::sin;
    using std::abs;

    // Verify the system is simulated using piecewise DAE.
    DRAKE_DEMAND(rod_->get_simulation_type() ==
        Rod2D<T>::SimulationType::kPiecewiseDAE);

    // Get the contact information.
    const auto& contact =
        rod_->get_contacts(context.get_state())[contact_index_];

    // Verify rod is not undergoing sliding contact at the specified index.
    DRAKE_DEMAND(!contact.sliding);

    // TODO(edrumwri): Speed this up (presumably) using caching. 

    // Populate problem data and solve the contact problem.
    const int ngv = 3;  // Number of rod generalized velocities.
    VectorX<T> cf;
    multibody::constraint::ConstraintAccelProblemData<T> problem_data(ngv);
    rod_->FormConstraintProblemData(context, &problem_data);
    solver_->SolveConstraintProblem(problem_data, &cf);

    // Determine the index of this contact in the non-sliding constraint set.
    const std::vector<int>& sliding_contacts = problem_data.sliding_contacts;
    const std::vector<int>& non_sliding_contacts =
        problem_data.non_sliding_contacts;
    DRAKE_ASSERT(std::is_sorted(non_sliding_contacts.begin(),
                                non_sliding_contacts.end()));
    const int non_sliding_index = std::distance(
        non_sliding_contacts.begin(),
        std::lower_bound(non_sliding_contacts.begin(),
                         non_sliding_contacts.end(),
                         contact_index_));
    const int num_sliding = sliding_contacts.size();
    const int num_non_sliding = non_sliding_contacts.size();
    const int nc = num_sliding + num_non_sliding;
    const int k = rod_->get_num_tangent_directions_per_contact();
    const int r = k / 2;

    // Get the normal force and the l1-norm of the frictional force.
    const auto fN = cf[contact_index_];
    const auto fF = cf.segment(nc + non_sliding_index * r, r).template
        lpNorm<1>();

    // Determine the slack.
    return problem_data.mu_non_sliding[non_sliding_index] * fN - fF;
  }

  void DoAddEvent(systems::CompositeEventCollection<T>* events) const override {
    event_->add_to_composite(events);
  }

  /// Unique pointer to the event.
  std::unique_ptr<systems::UnrestrictedUpdateEvent<T>> event_;

  /// Pointer to the rod system.
  const Rod2D<T>* rod_;

  /// Pointer to the rod's constraint solver.
  const multibody::constraint::ConstraintSolver<T>* solver_;

  /// Index of the contact point that this witness function applies to.
  int contact_index_{-1};
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

