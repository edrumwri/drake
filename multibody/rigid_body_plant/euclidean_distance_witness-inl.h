#pragma once

/// @file
/// Template method implementations for euclidean_distance_witness.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

namespace drake {
namespace multibody {

template <class T>
EuclideanDistanceWitnessFunction<T>::EuclideanDistanceWitnessFunction(
    const systems::RigidBodyPlant<T>* rb_plant,
    multibody::collision::Element* elementA,
    multibody::collision::Element* elementB) :
    systems::WitnessFunction<T>(*rb_plant,
        systems::WitnessFunctionDirection::kPositiveThenNonPositive),
    plant_(rb_plant),
    elementA_(elementA),
    elementB_(elementB) {
//  meshA_ = &plant_->meshes_.find(elementA_)->second;
//  meshB_ = &plant_->meshes_.find(elementB_)->second;
/*
    event_ = std::make_unique<systems::UnrestrictedUpdateEvent<T>>(
      systems::Event<T>::TriggerType::kWitness);
*/
}

template <class T>
T EuclideanDistanceWitnessFunction<T>::DoEvaluate(
    const systems::Context<T>& context) const {
  using std::sqrt;
  using std::min;

  // TODO: Pick a better value for this.
  const double kInitialDistance = 1.0;

  // Set the initial square distance.
  double square_distance = kInitialDistance;

  // Get the trimeshes for the elements.

  // TODO: Update the AABB pose only once for each element.

  // Update the broad phase structures. TODO: This should also be cached.

  // Do the broad phase between these two meshes and get the pairs of
  // triangles to check.

  // Compute the distances between pairs of triangles.

  // Subtract the distance by some epsilon.
}



}  // namespace multibody
}  // namespace drake

