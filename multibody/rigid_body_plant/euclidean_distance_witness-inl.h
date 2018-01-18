#pragma once

/// @file
/// Template method implementations for euclidean_distance_witness.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "euclidean_distance_witness.h"
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
  this->set_name("EuclideanDistanceWitness");
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
  SPDLOG_DEBUG(drake::log(), "State (as seen by Euclidean witness): {}", x);
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

  // TODO: Enable this only when SPDLOG_DEBUG is enabled.
  SPDLOG_DEBUG(drake::log(), "Pairs to check after broad phase");
  for (int i = 0; i < to_check.size(); ++i) {
    SPDLOG_DEBUG(drake::log(), " -- {}, {}", to_check[i].first,
                 to_check[i].second);
    SPDLOG_DEBUG(drake::log(), "   from A: {}",
                 meshA_->triangle(to_check[i].first));
    SPDLOG_DEBUG(drake::log(), "   from B: {}",
                 meshB_->triangle(to_check[i].second));
  }

  // Remove pairs of triangles that are already in contact.
  auto& contacting_features = context.get_abstract_state().get_value(0).template
      GetValue<std::map<SortedPair<multibody::collision::Element*>,
      std::vector<TriTriContactData<T>>>>();
  auto tri_tri_contact_iterator = contacting_features.find(
      MakeSortedPair(elementA_, elementB_));
  if (tri_tri_contact_iterator != contacting_features.end()) {
    auto& tri_tri_contact_data = tri_tri_contact_iterator->second;

    // TODO: Solve this using a n log n method instead.
    for (int i = 0; i < tri_tri_contact_data.size(); ++i) {
      for (int j = 0; j < to_check.size();) {
        const Triangle3<T>& tA_check = meshA_->triangle(to_check[j].first);
        const Triangle3<T>& tB_check = meshB_->triangle(to_check[j].second);
        if ((tri_tri_contact_data[i].tA == &tA_check &&
            tri_tri_contact_data[i].tB == &tB_check) ||
            (tri_tri_contact_data[i].tA == &tB_check &&
                tri_tri_contact_data[i].tB == &tA_check)) {
          to_check[j] = to_check.back();
          to_check.pop_back();
        } else {
          ++j;
        }
      }
    }
  }

  // Return no contact.
  if (to_check.empty())
    return std::numeric_limits<double>::infinity();

  // Compute the distances between pairs of triangles.
  std::vector<std::pair<std::pair<int, int>, T>> distances;
  const T min_distance = this->get_plant().get_collision_detection().
      CalcDistances(*meshA_, *meshB_, to_check, &distances);

  // TODO: Use a properly determined tolerance.
  const double equiv_tol = 1000 * std::numeric_limits<double>::epsilon();

  // Store closest triangles.
  closest_triangles_.clear();
  for (int i = 0; i < distances.size(); ++i) {
    if (distances[i].second - equiv_tol < min_distance)
      closest_triangles_.push_back(distances[i].first);
  }
  DRAKE_DEMAND(!closest_triangles_.empty());

  // Subtract the distance by some epsilon. TODO: Record why we do this.
  return min_distance - signed_distance_epsilon;
}


}  // namespace multibody
}  // namespace drake

