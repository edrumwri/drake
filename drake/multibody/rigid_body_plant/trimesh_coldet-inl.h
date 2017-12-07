#pragma once

/// @file
/// Template method implementations for trimesh_coldet.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

namespace drake {
namespace multibody {

template <class T>
void TrimeshColdet<T>::DoBroadPhase(
    std::vector<std::pair<int, int>>* to_check) {
  DRAKE_DEMAND(to_check);
  to_check->clear();

  // Sort the AABBs.

  // Prepare to store overlaps for pairs.
  std::set<std::pair<int, int>> overlaps;

  // Store the number of overlaps for pairs (?): this cannot work- we do not
  // want to process O(n^2) overlaps

  // Scan through the x-bounds, eliminating from active bounds if at the end of
  // a bound.
  
}

template <class T>
void TrimeshColdet<T>::DetermineOverlaps(
    int axis, std::set<std::pair<int, int>>* overlaps) {
  // Compute overlaps over the axis.

}

template <class T>
void TrimeshColdet<T>::UpdateOverlaps(std::set<std::pair<int, int>>* overlaps)
{
  // Make a new set of overlaps.
  std::set<std::pair<int, int>> new_overlaps;

  // TODO: Compute overlaps over the axis.

  // Intersect with the old overlaps: only ones that are in both sets are
  // retained.
  std::set<std::pair<int, int>> intersected_overlaps;

  // Update overlaps. 
  *overlaps = intersected_overlaps;
}

template <class T>
T TrimeshColdet<T>::CalcDistance(
    const Pose3d<T>& poseA, const Trimesh<T>& tA,
    const Pose3d<T>& poseB, const Trimesh<T>& tB,
    const std::set<std::pair<int, int>>& pairs_to_ignore) const {
  // TODO: Use the distance between the two AABBs as the default
  // non-intersecting distance.
  T distance = 100.0;

  // Call the broad phase algorithm to determine pairs to check.

  // TODO: How can this function be adapted to do triangle feature
  // intersections?

  // Get the distance between each pair of triangles.
  for (int i = 0; i < candidate_tris.size(); ++i) {
    // See whether the triangle is in the pairs to ignore.
    if (pairs_to_ignore.find(candidate_tris[i]) != pairs_to_ignore.end())
      continue;

    // Get the distance between the two triangles.
    T tri_distance = Triangle::CalcDist(
        tA.get_triangle(candidate_tris[i].first),
        tB.get_triangle(candidate_tris[i].second));

    // See whether the distance is below the minimum.
    distance = std::min(tri_distance, distance);
  }

  return distance;
}

