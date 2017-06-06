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

template <class T>
class StickingFrictionForcesSlackWitness : public systems::WitnessFunction<T> {
 public:
  StickingFrictionForcesSlackWitness(const Rod2D<T>* rod, int contact_index) :
      systems::WitnessFunction<T>(rod), rod_(rod),
      contact_index_(contact_index) {
    this->name_ = "StickingFrictionForcesSlack";
  }

  /// This witness function indicates an unrestricted update needs to be taken.
  typename systems::DiscreteEvent<T>::ActionType get_action_type()
  const override {
    return systems::DiscreteEvent<T>::ActionType::kUnrestrictedUpdateAction;
  }

  /// This witness triggers only when the signed distance goes from strictly
  /// positive to zero/negative.
  typename systems::WitnessFunction<T>::TriggerType get_trigger_type()
  const override {
    return systems::WitnessFunction<T>::TriggerType::kPositiveThenNegative;
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
    using std::abs;

    // Verify the system is simulated using piecewise DAE.
    DRAKE_DEMAND(rod_->get_simulation_type() ==
        Rod2D<T>::SimulationType::kPiecewiseDAE);

    // Get the contact information.
    const RigidContact& contact =
        rod_->get_contacts(context.get_state())[contact_index_];

    // Verify rod is not undergoing sliding contact at the specified index.
    DRAKE_DEMAND(contact.state ==
        RigidContact::ContactState::kContactingWithoutSliding);

    // TODO(edrumwri): Only compute this once over the entire set
    //                 of witness functions generally.

    // Populate problem data and solve the contact problem.
    RigidContactAccelProblemData<T> problem_data;
    VectorX<T> cf = rod_->SolveContactProblem(context, &problem_data);

    // Determine the index of this contact in the non-sliding constraint set.
    const std::vector<int>& sliding_contacts = problem_data.sliding_contacts;
    const std::vector<int>& non_sliding_contacts =
        problem_data.non_sliding_contacts;
    DRAKE_ASSERT(std::is_sorted(non_sliding_contacts.begin(),
                                non_sliding_contacts.end()));
    const int non_sliding_index = std::distance(non_sliding_contacts.begin(),
                                                std::lower_bound(
                                                    non_sliding_contacts.begin(),
                                                    non_sliding_contacts.end(),
                                                    contact_index_));
    const int num_sliding = sliding_contacts.size();
    const int num_non_sliding = non_sliding_contacts.size();
    const int nc = num_sliding + num_non_sliding;
    const int k = rod_->get_num_tangent_directions_per_contact();

    // Get the normal force and the absolute value of the frictional force.
    const auto fN = cf[contact_index_];
    const auto fF = cf.segment(nc + non_sliding_index * (k/2), k / 2).template
        lpNorm<1>();

    // Determine the slack.
    return contact.mu*fN - fF;
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
    return -sqrt(std::numeric_limits<double>::epsilon());
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

