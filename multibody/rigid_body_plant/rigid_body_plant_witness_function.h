#pragma once

#include "drake/systems/framework/event.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace multibody {

// A generic witness function for the RigidBodyPlant.
template <class T>
class RigidBodyPlantWitnessFunction : public systems::AbstractValues,
                                      public systems::WitnessFunction<T> {
 public:
  RigidBodyPlantWitnessFunction(
      const systems::RigidBodyPlant<T>& rb_plant,
      typename systems::WitnessFunctionDirection dir) :
      systems::WitnessFunction<T>(rb_plant, dir), plant_(rb_plant) {
            event_ = std::make_unique<systems::UnrestrictedUpdateEvent<T>>(
                systems::Event<T>::TriggerType::kWitness);
          }

  /// Gets the plant.
  const systems::RigidBodyPlant<T>& get_plant() const { return plant_; } 

  /// The types of witness function.
  enum WitnessType {
    /// The Euclidean distance between all pairs of triangles not already
    /// designated as contacting.
    kEuclideanDistance,

    /// A witness that can determine when two contacting triangles separate
    /// tangentially (and should no longer be designated as contacting).
    kTangentialSeparation,
  };

  /// Gets the type of witness function. 
  virtual WitnessType get_witness_function_type() const = 0; 

 private:
  void DoAddEvent(systems::CompositeEventCollection<T>* events) const override {
    event_->set_attribute(
        std::make_unique<systems::Value<const RigidBodyPlantWitnessFunction<T>*>>(
        this));
    event_->add_to_composite(events);
  }

  /// Unique pointer to the event.
  std::unique_ptr<systems::UnrestrictedUpdateEvent<T>> event_;

  /// Pointer to the plant.
  const systems::RigidBodyPlant<T>& plant_;
};

}  // namespace multibody 
}  // namespace drake

