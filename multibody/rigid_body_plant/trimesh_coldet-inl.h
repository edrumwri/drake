#pragma once

/// @file
/// Template method implementations for trimesh_coldet.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

namespace drake {
namespace multibody {

/// Updates all axis-aligned bounding boxes corresponding to `mesh`.
template <class T>
void TrimeshColdet<T>::UpdateAABBs(
    const Trimesh<T>& mesh, const Isometry3<T>& wTm) {
}

/// Sorts data structures for broad-phase collision detection.
/// @pre All AABBs have already been updated.
template <class T>
void TrimeshColdet<T>::UpdateBroadPhaseStructs() {
  for (int i = 0; i < bounds_.size(); ++i) {
    // Get the bounding structures for the i'th axis.
    auto& vec_axis_pair = bounds_[i];

    // Get the i'th axis.
    const Vector3<T>& axis = vec_axis_pair.second;

    // Get the vector.
    auto& vec = vec_axis_pair.first;

    // Update the bounds data using the corresponding AABBs.
    for (int j = 0; j < static_cast<int>(vec.size()); ++j) {
      // Get the axis-aligned bounding box.
      const AABB<T>& aabb = *vec[j].aabb;

      // Update the bound.
      const Vector3<T>& bound = (vec[j].second.end) ? aabb.upper_bounds() : 
                                                      aabb.lower_bounds();
      vec[j].first = axis.dot(bound);
    }

    // Do an insertion sort of each vector.
    for (auto it = vec.begin(); it != vec.end(); ++it) {
      // Search the upper bound for the first element greater than *it.
      auto const insertion_point = std::upper_bound(vec.begin(), it, *it);
      
      // Shift the unsorted part.
      std::rotate(insertion_point, it, it + 1);
    }
  }
}

// Does broad phase collision detection between the triangles from two meshes
// using already-updated AABBs.
// @param[out] to_check On return, contains pairs of triangles for which the
//             corresponding leaf bounding volumes are intersecting. 
// @pre UpdateBroadPhaseStructs() was called immediately prior to calling this
//      function. TODO: Update this wording.
template <class T>
void TrimeshColdet<T>::DoBroadPhase(
    const Trimesh<T>& mA, const Trimesh<T>& mB,
    std::vector<std::pair<int, int>>* to_check) const {
  DRAKE_DEMAND(to_check);
  to_check->clear();

  // Prepare to store overlaps for pairs.
  std::set<std::pair<int, int>> overlaps;

  // Iterate over each bounds vector.
  for (int i = 0; i < bounds_.size(); ++i)
    UpdateOverlaps(i, &overlaps);

  // Update to_check.
  std::copy(overlaps.begin(), overlaps.end(), std::back_inserter(*to_check)); 
}

template <class T>
void TrimeshColdet<T>::UpdateOverlaps(
    int bounds_index,
    std::set<std::pair<int, int>>* overlaps) const {
  // The set of active bounds.
  std::vector<int> active_bounds;

  // Make a new set of overlaps.
  std::set<std::pair<int, int>> new_overlaps;

  for (int j = 0; j < bounds_[bounds_index].first.size(); ++j) {
    // Eliminate from the active bounds if at the end of a bound.
    auto bound_info = bounds_[bounds_index].first[j];
    if (bound_info.end) {
      assert(active_bounds.find(bound_info.tri) != active_bounds.end());
      active_bounds.erase(bound_info.tri);
    } else {
      // Encountered a new bound. 
      for (const auto& tri : active_bounds) {
        new_overlaps.insert(std::min(tri, bound_info.tri),
                            std::max(tri, bound_info.tri));

      // Add the triangle to the active set.
      active_bounds.insert(bound_info.tri);
    }
  }

  // Intersect with the old overlaps: only ones that are in both sets are
  // retained.
  std::vector<std::pair<int, int>> intersected_overlaps;
  std::set_intersection(new_overlaps.begin(), new_overlaps.end(),
                        overlaps->begin(), overlaps->end(),
                        std::back_inserter(intersected_overlaps); 

  // Update overlaps. 
  overlaps->clear();
  overlaps->insert(intersected_overlaps.begin(), intersected_overlaps.end());
}

template <class T>
T TrimeshColdet<T>::CalcDistance(
    const Isometry3<T>& poseA, const Trimesh<T>& tA,
    const Isometry3<T>& poseB, const Trimesh<T>& tB,
    const std::set<std::pair<int, int>>& pairs_to_ignore) const {
  // TODO: Use the distance between the two AABBs as the default
  // non-intersecting distance.
  T distance = 100.0;

  // Call the broad phase algorithm to determine pairs of triangles to check.

  // Get the distance between each pair of triangles.
  for (int i = 0; i < candidate_tris.size(); ++i) {
    // See whether the triangle is in the pairs to ignore.
    if (pairs_to_ignore.find(candidate_tris[i]) != pairs_to_ignore.end())
      continue;

    // Get the two triangles.
    Vector3<T> closest_on_tA, closest_on_tB;
    const auto tA = mA.get_triangle(candidate_tris[i].first).transform(poseA);
    const auto tB = mB.get_triangle(candidate_tris[i].second).transform(poseB);

    // Get the distance between the two triangles.
    T tri_distance = tA.CalcSquareDistance(tB, &closest_on_tA, &closest_on_tB); 

    // See whether the distance is below the minimum.
    distance = std::min(tri_distance, distance);
  }

  return distance;
}

/*
/// @note aborts if `contacts` is null or not empty on entry.
/// @pre There exists some positive distance between any two triangles not
///      already designated as being intersecting.
template <class T>
void TrimeshColdet<T>::CalcIntersections(
    const Isometry3<T>& poseA, const Trimesh<T>& tA,
    const Isometry3<T>& poseB, const Trimesh<T>& tB,
    const std::set<std::pair<int, int>>& pairs_to_ignore,
    std::vector<multibody::collision::PointPair>* contacts) const {
  using std::sqrt;

  DRAKE_DEMAND(contact_data && contact_data->empty());

  // TODO: Set the intersecting threshold in a principled manner.
  const double intersecting_threshold = 1e-8;

  // Call the broad phase algorithm to determine pairs of triangles to check.

  // TODO: it seems like we also need to store contact data for (a) the IDs of
  // the triangles newly determined to be in contact, (b) contact points in the
  // body frames, (c) features of the triangles (both so we can track which
  // features are in contact, and which features aren't).

  // Get the distance between each pair of triangles.
  for (int i = 0; i < candidate_tris.size(); ++i) {
    // See whether the triangle is in the pairs to ignore.
    if (pairs_to_ignore.find(candidate_tris[i]) != pairs_to_ignore.end())
      continue;

    // Get the two triangles.
    const auto tA = mA.get_triangle(candidate_tris[i].first).transform(poseA);
    const auto tB = mB.get_triangle(candidate_tris[i].second).transform(poseB);

    // Get the distance between the two triangles.
    Vector3<T> closest_on_tA, closest_on_tB;
    T tri_distance = sqrt(tA.CalcSquareDistance(
        tB, &closest_on_tA, &closest_on_tB)); 

    // See whether the distance is sufficiently small.
    if (tri_distance < intersecting_threshold) {
      // Verify that the distance isn't *too* small.
      DRAKE_DEMAND(tri_distance > 0);

      // Create a new contact.
      contacts->push_back(multibody::collision::PointPair());  

      // Set the closest points.
      contacts->back().ptA = closest_on_tA;
      contacts->back().ptB = closest_on_tB;

      // Set the normal such that it points toward A.
      Vector3<T> normal = closest_on_tA - closest_on_tB;
      normal.normalize();
      contacts->back().normal = normal;

      // Determine the offset for the plane parallel and halfway between the
      // two planes passing through the closest points.
      const T offsetA = normal.dot(closest_on_tA);
      const T offsetB = normal.dot(closest_on_tB);
      const T offset = (offsetA + offsetB) / 2.0;

      // Project all points that are within the given tolerance of the offset
      // to the contact plane.

      //  
    }
  }

  return distance;
}
*/
