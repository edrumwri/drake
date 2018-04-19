#pragma once

#include <sstream>

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rod_witness_function.h"

#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace rod2d {

/// Determines whether the tangent velocity crosses the sliding velocity
/// threshold. This witness is used for two purposes: (1) it determines when
/// a contact has moved from non-sliding-to-sliding-transition to proper sliding
/// and (2) it determines when a contact has moved from sliding to non-sliding.
template <class T>
class SlidingWitness : public RodWitnessFunction<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SlidingWitness)

  SlidingWitness(
      const Rod2D<T>& rod,
      int contact_index,
      bool pos_direction) :
      RodWitnessFunction<T>(
          rod,
          systems::WitnessFunctionDirection::kCrossesZero,
          contact_index) {
    std::ostringstream oss;
    oss << "Sliding ";
    if (pos_direction) {
      oss << "+";
    } else {
      oss << "-";
    }
    oss << " (" << contact_index << ")";
    this->set_name(oss.str());
    positive_ = pos_direction;
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

    // Get the sliding velocity threshold. 
    const double sliding_threshold = rod.GetSlidingVelocityThreshold(context); 

    // Verify the system is simulated using piecewise DAE.
    DRAKE_DEMAND(rod.get_simulation_type() ==
        Rod2D<T>::SimulationType::kPiecewiseDAE);

    // Get the contact information.
    const int contact_index = this->get_contact_index();
    const int contact_array_index = rod.GetContactArrayIndex(
        context.get_state(), contact_index);
    const auto& contact = rod.get_contacts_used_in_force_calculations(
        context.get_state())[contact_array_index];

    // Verify rod is undergoing sliding contact at the specified index.
    DRAKE_DEMAND(contact.sliding_type ==
        multibody::constraint::SlidingModeType::kSliding || 
        contact.sliding_type ==
        multibody::constraint::SlidingModeType::kTransitioning);

    // Compute the translational velocity at the point of contact.
    const Vector2<T> pdot = rod.CalcContactVelocity(context,
                                                    contact_index);

    // Return the tangent velocity.
    if (positive_) {
      return sliding_threshold - pdot[0];
    } else {
      return -pdot[0] - sliding_threshold;
    }
  }

  // If 'true', witness function triggers when the sliding velocity is
  // sufficiently positive. Otherwise, it triggers when the sliding velocity
  // is sufficiently negative.
  bool positive_{false};
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

