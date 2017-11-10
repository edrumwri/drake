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

/// All witness functions for the 2D Rod inherit from this one. 
template <class T>
class RodWitnessFunction : public systems::WitnessFunction<T> {
 public:
  RodWitnessFunction(const Rod2D<T>* rod,
                     systems::WitnessFunctionDirection dir,
                     int contact_index) :
      systems::WitnessFunction<T>(*rod, dir),
      rod_(rod),
      contact_index_(contact_index) {
    event_ = std::make_unique<systems::UnrestrictedUpdateEvent<T>>(
      systems::Event<T>::TriggerType::kWitness);
  }

  /// Gets the rod.
  const Rod2D<T>& get_rod() const { return *rod_; } 

  /// Gets the index of the contact candidate for this witness function.
  int get_contact_index() const { return contact_index_; }

  /// The types of witness function.
  enum class WitnessType {
      /// The signed distance for a contact from the half-space.
      kSignedDistance,

      /// The acceleration along the contact normal at a point of contact. 
      kNormalAcceleration,

      /// The velocity along the contact normal at a point of contact. 
      kNormalVelocity,

      /// The slack in the stiction forces. If the slack is non-zero, stiction
      /// will be maintained. When it is less than zero, too much stiction
      /// force is being generated. 
      kStickingFrictionForceSlack,

      kSlidingDot,

      kNormalForce
  };

  /// Gets the type of witness function. 
  virtual WitnessType get_witness_function_type() const = 0; 

 private:
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

