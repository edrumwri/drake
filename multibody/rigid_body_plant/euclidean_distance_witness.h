#pragma once

#include "drake/multibody/collision/element.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant_witness_function.h"
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
class EuclideanDistanceWitnessFunction :
    public RigidBodyPlantWitnessFunction<T>,  public systems::AbstractValues {
 public:
  EuclideanDistanceWitnessFunction(
      const systems::RigidBodyPlant <T>& rb_plant,
      multibody::collision::Element* elementA,
      multibody::collision::Element* elementB);

  /// Gets the type of witness function.
  typename RigidBodyPlantWitnessFunction<T>::WitnessType
      get_witness_function_type() const override {
    return RigidBodyPlantWitnessFunction<T>::kEuclideanDistance; 
 }

  multibody::collision::Element* get_element_A() const { return elementA_; }
  multibody::collision::Element* get_element_B() const { return elementB_; }

 private:
  T DoEvaluate(const systems::Context<T>& context) const override; 

  // The two triangle meshes.
  const multibody::Trimesh<T>* meshA_{nullptr};
  const multibody::Trimesh<T>* meshB_{nullptr};

  // The two elements for which the distance will be computed.
  multibody::collision::Element* elementA_{nullptr};
  multibody::collision::Element* elementB_{nullptr};
};

}  // namespace multibody 
}  // namespace drake

