#pragma once

#include "drake/multibody/constraint/point_contact.h"

#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/framework/abstract_values.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace multibody {

template <class T>
class RigidBodyPlant;

// A witness function for tracking the signed distance of a point that is not
// contacting *on a triangle that is otherwise contacting another* from all
// planes of contact.
template <class T>
class AdditionalContactWitnessFunction : public systems::AbstractValues,
                                         public systems::WitnessFunction<T> {
 public:
  AdditionalContactWitnessFunction(
      const RigidBodyPlant<T>* rb_plant,
      systems::WitnessFunctionDirection dir) :
      systems::WitnessFunction<T>(*rb_plant, dir),
      plant_(rb_plant) {
/*
    event_ = std::make_unique<systems::UnrestrictedUpdateEvent<T>>(
      systems::Event<T>::TriggerType::kWitness);
*/
  }

  /// Gets whether the witness function is active (default). If it is not
  /// active, it will not be used to track state changes.
  bool is_enabled() const { return enabled_; }

  /// Sets whether the witness function is active.
  void set_enabled(bool flag) { enabled_ = flag; }

  /// Gets the plant.
  const RigidBodyPlant<T>& get_plant() const { return *plant_; } 

/*
  /// Gets the index of the contact candidate for this witness function.
  int get_contact_index() const { return contact_index_; }

  /// The types of witness function.
  enum class WitnessType {
      /// The signed distance for a contact from the half-space.
      kSignedDistance,

      /// The acceleration along the contact normal at a point of contact. 
      kNormalAccel,

      /// The velocity along the contact normal at a point of contact. 
      kNormalVel,

      /// The slack in the stiction forces. If the slack is non-zero, stiction
      /// will be maintained. When it is less than zero, too much stiction
      /// force is being generated. 
      kStickingFrictionForceSlack,

      kNormalForce,

      kSlidingWitness,
  };

  /// Gets the type of witness function. 
  virtual WitnessType get_witness_function_type() const = 0; 
*/
 private:
  T DoEvaluate(const systems::Context<T>& context) const override {
  }

/*
  void DoAddEvent(systems::CompositeEventCollection<T>* events) const override {
    event_->set_attribute(
        std::make_unique<systems::Value<const RodWitnessFunction<T>*>>(this));
    event_->add_to_composite(events);
  }

  /// Unique pointer to the event.
  std::unique_ptr<systems::UnrestrictedUpdateEvent<T>> event_;
*/
  /// Pointer to the plant.
  const RigidBodyPlant<T>* plant_;

/*
  /// Index of the contact point that this witness function applies to.
  int contact_index_{-1};
*/
  /// Whether the witness function is used to track state changes.
  bool enabled_{false};
};

}  // namespace multibody 
}  // namespace drake

