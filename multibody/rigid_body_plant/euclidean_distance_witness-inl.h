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
  meshA_ = &plant_->GetMesh(elementA_);
  meshB_ = &plant_->GetMesh(elementB_);
  event_ = std::make_unique<systems::UnrestrictedUpdateEvent<T>>(
    systems::Event<T>::TriggerType::kWitness);
}

template <class T>
T EuclideanDistanceWitnessFunction<T>::DoEvaluate(
    const systems::Context<T>& context) const {
  using std::sqrt;
  using std::min;

  // Use the signed distance epsilon.
  const double signed_distance_epsilon = 1e-8;

  // Get the rigid bodies.
  const RigidBody<T>& rbA = *elementA_->get_body();
  const RigidBody<T>& rbB = *elementB_->get_body();

  // Compute the transforms for the RigidBody objects.
  const auto& tree = plant_->get_rigid_body_tree();
  const int nq = plant_->get_num_positions();
  const int nv = plant_->get_num_velocities();
  auto x = context.get_discrete_state(0).get_value();
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  auto kinematics_cache = tree.doKinematics(q, v);
  auto wTA = tree.CalcBodyPoseInWorldFrame(kinematics_cache, rbA);
  auto wTB = tree.CalcBodyPoseInWorldFrame(kinematics_cache, rbB);

  // Update the AABB pose.
  // TODO: Do this only once for each element, over all distance functions.
  plant_->get_collision_detection().UpdateAABBs(*meshA_, wTA);
  plant_->get_collision_detection().UpdateAABBs(*meshB_, wTB);

  // Update the broad phase structures.
  // TODO: This should also be cached.
  plant_->get_collision_detection().UpdateBroadPhaseStructs();

  // Do the broad phase between these two meshes and get the pairs of
  // triangles to check.
  std::vector<std::pair<int, int>> to_check;
  plant_->get_collision_detection().DoBroadPhase(*meshA_, *meshB_, &to_check);

  // Compute the distances between pairs of triangles.
  const T distance = plant_->get_collision_detection().CalcDistance(
      *meshA_, *meshB_, to_check);

  // Subtract the distance by some epsilon.
  return distance - signed_distance_epsilon;
}


}  // namespace multibody
}  // namespace drake

