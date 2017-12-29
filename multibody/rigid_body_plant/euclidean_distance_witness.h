#pragma once

#include "drake/common/sorted_pair.h"
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
    public RigidBodyPlantWitnessFunction<T> {
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

  EuclideanDistanceWitnessFunction(
      const EuclideanDistanceWitnessFunction<T>& e) :
    RigidBodyPlantWitnessFunction<T>(
        this->get_plant(),
        systems::WitnessFunctionDirection::kPositiveThenNonPositive) {
    operator=(e);
  }

  EuclideanDistanceWitnessFunction& operator=(
      const EuclideanDistanceWitnessFunction<T>& e) {
    elementA_ = e.elementA_;
    elementB_ = e.elementB_;
    meshA_ = e.meshA_;
    meshB_ = e.meshB_;
    return *this;
  }

  multibody::collision::Element* get_element_A() const { return elementA_; }
  multibody::collision::Element* get_element_B() const { return elementB_; }

  /// Gets the vector of closest triangles determined during the last witness
  /// function evaluation.
  const std::vector<std::pair<int, int>>& get_last_closest_tris() const {
    return closest_triangles_; };

 private:
  T DoEvaluate(const systems::Context<T>& context) const override; 

  // The two triangle meshes.
  const multibody::Trimesh<T>* meshA_{nullptr};
  const multibody::Trimesh<T>* meshB_{nullptr};

  // The two elements for which the distance will be computed.
  multibody::collision::Element* elementA_{nullptr};
  multibody::collision::Element* elementB_{nullptr};

  // The vector of closest triangles from the last witness function evaluation.
  mutable std::vector<std::pair<int, int>> closest_triangles_;
};

}  // namespace multibody 
}  // namespace drake

