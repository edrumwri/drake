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

// A witness function for the minimum Euclidean distance, for two bodies,
// between all triangles *not already determined to be intersecting*. This
// witness function is to be activated when the bodies are sufficiently close
// as to pass the broad phase collision detection check. 
template <class T>
class EuclideanDistanceWitnessFunction : public systems::AbstractValues,
                                         public systems::WitnessFunction<T> {
 public:
  EuclideanDistanceWitnessFunction(
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
      kEuclideanDistance,

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
    using std::sqrt;
    using std::min;

    // TODO: Pick a better value for this.
    const double kInitialDistance = 1.0;

    // Set the initial square distance.
    double square_distance = kInitialDistance;

    // Get all triangles currently considered to be contacting.

    // Get all triangles that are sufficiently close (according to the broad
    // phase check).

    // Examine all candidate pairs.
    for (int i = 0; i < static_cast<int>(candidate_pairs.size()); ++i) {
      // If the candidate pair is already contacting, skip it.
      if (contacting.find(candidate_pairs[i]) != contacting.end())
        continue;

      // Get the two triangles.

      // Get the square distance between the triangles.

      square_distance = min(square_distance, tri_square_dist);
    }

    return sqrt(square_distance);
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

