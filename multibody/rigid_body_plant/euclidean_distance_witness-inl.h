#pragma once

/// @file
/// Template method implementations for euclidean_distance_witness.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

namespace drake {
namespace multibody {

template <class T>
EuclideanDistanceWitnessFunction<T>::EuclideanDistanceWitnessFunction(
    const systems::RigidBodyPlant<T>& plant,
    multibody::collision::Element* elementA,
    multibody::collision::Element* elementB) :
    RigidBodyPlantWitnessFunction<T>(plant,
        systems::WitnessFunctionDirection::kPositiveThenNonPositive),
    elementA_(elementA),
    elementB_(elementB) {
  meshA_ = &this->get_plant().GetMesh(elementA_);
  meshB_ = &this->get_plant().GetMesh(elementB_);
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
  const auto& tree = this->get_plant().get_rigid_body_tree();
  const int nq = this->get_plant().get_num_positions();
  const int nv = this->get_plant().get_num_velocities();
  auto x = context.get_discrete_state(0).get_value();
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  auto kinematics_cache = tree.doKinematics(q, v);
  auto wTA = tree.CalcBodyPoseInWorldFrame(kinematics_cache, rbA);
  auto wTB = tree.CalcBodyPoseInWorldFrame(kinematics_cache, rbB);

  // Update the AABB pose.
  // TODO: Do this only once for each element, over all distance functions.
  this->get_plant().get_collision_detection().UpdateAABBs(*meshA_, wTA);
  this->get_plant().get_collision_detection().UpdateAABBs(*meshB_, wTB);

  // Update the broad phase structures.
  // TODO: This should also be cached.
  this->get_plant().get_collision_detection().UpdateBroadPhaseStructs();

  // Do the broad phase between these two meshes and get the pairs of
  // triangles to check.
  std::vector<std::pair<int, int>> to_check;
  this->get_plant().get_collision_detection().DoBroadPhase(*meshA_, *meshB_,
                                                           &to_check);

  // Compute the distances between pairs of triangles.
  const T distance = this->get_plant().get_collision_detection().
      CalcMinDistance(*meshA_, *meshB_, to_check);

  // Subtract the distance by some epsilon.
  return distance - signed_distance_epsilon;
}


}  // namespace multibody
}  // namespace drake

