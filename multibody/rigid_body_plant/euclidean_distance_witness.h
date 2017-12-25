#pragma once

#include "drake/multibody/collision/element.h"
#include "drake/multibody/rigid_body_plant/trimesh.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace multibody {

// A witness function for the minimum Euclidean distance, for two bodies,
// between all triangles *not already determined to be intersecting*. This
// witness function is to be activated when the bodies are sufficiently close
// as to pass the broad phase collision detection check. 
template <class T>
class EuclideanDistanceWitnessFunction : public systems::AbstractValues,
                                         public systems::WitnessFunction<T> {
 public:
  EuclideanDistanceWitnessFunction(
      const systems::RigidBodyPlant<T>* rb_plant,
      multibody::collision::Element* elementA,
      multibody::collision::Element* elementB);

  /// Gets whether the witness function is active (default). If it is not
  /// active, it will not be used to track state changes.
  bool is_enabled() const { return enabled_; }

  /// Sets whether the witness function is active.
  void set_enabled(bool flag) { enabled_ = flag; }

  /// Gets the plant.
  const systems::RigidBodyPlant<T>& get_plant() const { return *plant_; } 

/*
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
  T DoEvaluate(const systems::Context<T>& context) const override; 

  void DoAddEvent(systems::CompositeEventCollection<T>* events) const override {
    event_->set_attribute(
        std::make_unique<systems::Value<const systems::WitnessFunction<T>*>>(
        this));
    event_->add_to_composite(events);
  }

  /// Unique pointer to the event.
  std::unique_ptr<systems::UnrestrictedUpdateEvent<T>> event_;

  /// Pointer to the plant.
  const systems::RigidBodyPlant<T>* plant_;

/*
  /// Index of the contact point that this witness function applies to.
  int contact_index_{-1};
*/
  /// Whether the witness function is used to track state changes.
  bool enabled_{false};

  // The two triangle meshes.
  const multibody::Trimesh<T>* meshA_{nullptr};
  const multibody::Trimesh<T>* meshB_{nullptr};

  // The two elements for which the distance will be computed.
  multibody::collision::Element* elementA_{nullptr};
  multibody::collision::Element* elementB_{nullptr};
};

}  // namespace multibody 
}  // namespace drake

