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
  // Update the pose in the map.
  auto poses_iter = poses_.find(&mesh);
  DRAKE_DEMAND(poses_iter != poses_.end());
  poses_iter->second = wTm;

  // Update each AABB corresponding to this mesh.
  auto trimesh_bs_iter = trimesh_aabbs_.find(&mesh);
  DRAKE_DEMAND(trimesh_bs_iter != trimesh_aabbs_.end());
  const std::vector<BoundingStructs*>& bs = trimesh_bs_iter->second; 
  for (int i = 0; i < static_cast<int>(bs.size()); ++i) {
    // Get the requisite triangle in the mesh, transform it, and reconstruct
    // the AABB around it.
    const Triangle3<T>& tri = mesh.get_triangle(bs[i]->tri);
    AABB<T>& aabb = *bs[i]->aabb; 
    aabb = AABB<T>(tri.transform(wTm));
  }
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
    const Trimesh<T>& mA,
    const Trimesh<T>& mB,
    const std::set<std::pair<int, int>>& candidate_tris) const {
  // TODO: Use the distance between the two AABBs as the default
  // non-intersecting distance.
  T distance = 100.0;

  // Get iterators to the two poses for the triangle meshes.
  const auto& poses_iter_mA = poses_.find(&mA);
  const auto& poses_iter_mB = poses_.find(&mB);

  // Verify that the meshes were found.
  if (poses_iter_mA == poses_.end() || poses_iter_mB == poses_.end())
    throw std::logic_error("Mesh was not found in the pose map.");

  // Get the poses.
  const Isometry3<T>& poseA = poses_iter_mA->second;
  const Isometry3<T>& poseB = poses_iter_mB->second;

  // Get the distance between each pair of triangles.
  for (int i = 0; i < candidate_tris.size(); ++i) {
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

// Projects a point to 2D given a normal.
template <class T>
Vector2<T> TrimeshColdet<T>::ProjectTo2d(
    const Vector3<T>& point, const Vector3<T>& normal) {
  // Compute the orthonormal basis.
  Matrix3<T> R = math::ComputeBasisFromAxis(0, normal);
  Vector3<T> v1 = R.col(1);
  Vector3<T> v2 = R.col(2); 

  // Construct a 2 x 3 projection matrix from the two vectors in the basis.
  Eigen::Matrix<T, 2, 3> P;
  P.row(0) = v1;
  P.row(1) = v2;

  return P * point;
}

// Projects an edge to 2D given a normal.
template <class T>
std::pair<Vector2<T>, Vector2<T>> TrimeshColdet<T>::ProjectTo2d(
    const std::pair<Vector3<T>, Vector3<T>>& edge, const Vector3<T>& normal) {
  // Compute the orthonormal basis.
  Matrix3<T> R = math::ComputeBasisFromAxis(0, normal);
  Vector3<T> v1 = R.col(1);
  Vector3<T> v2 = R.col(2);

  // Construct a 2 x 3 projection matrix from the two vectors in the basis.
  Eigen::Matrix<T, 2, 3> P;
  P.row(0) = v1;
  P.row(1) = v2;

  return std::make_pair(P * edge.first, P * edge.second);
}

// Projects a point from 2D back to 3D given a normal and offset.
template <class T>
Vector3<T> TrimeshColdet<T>::ReverseProject(
    const Vector2<T>& p,
    const Vector3<T>& normal,
    T offset) {
  // Compute the orthonormal basis.
  Matrix3<T> R = math::ComputeBasisFromAxis(0, normal);
  Vector3<T> v1 = R.col(1);
  Vector3<T> v2 = R.col(2); 

  // Construct the reverse projection matrix.
  Matrix3<T> rP;
  rP.col(0) = v1;
  rP.col(1) = v2;
  rP.col(2) = normal;

  return rP * Vector3(p[0], p[1], offset);
}

/// @note aborts if `contacts` is null or not empty on entry.
/// @pre There exists some positive distance between any two triangles not
///      already designated as being intersecting.
template <class T>
void TrimeshColdet<T>::CalcIntersections(
    const Trimesh<T>& mA,
    const Trimesh<T>& mB,
    const std::set<std::pair<int, int>>& candidate_tris,
    std::vector<multibody::TriTriContactData>* contacts) const {
  using std::sqrt;

  DRAKE_DEMAND(contact_data && contact_data->empty());

  // TODO: Set the intersecting threshold in a principled manner.
  const double intersecting_threshold = 1e-8;

  // Get iterators to the two poses for the triangle meshes.
  const auto& poses_iter_mA = poses_.find(&mA);
  const auto& poses_iter_mB = poses_.find(&mB);

  // Verify that the meshes were found.
  if (poses_iter_mA == poses_.end() || poses_iter_mB == poses_.end())
    throw std::logic_error("Mesh was not found in the pose map.");

  // Get the poses.
  const Isometry3<T>& poseA = poses_iter_mA->second;
  const Isometry3<T>& poseB = poses_iter_mB->second;

  // Create two triangles, including vertices.
  Vector3<T> Aa, Ab, Ac, Ba, Bb, Bc;
  Triangle3<T> tA(&Aa, &Ab, &Ac);
  Triangle3<T> tB(&Ba, &Bb, &Bc);

  // TODO: we also need to store contact data for (a) the IDs of the triangles
  // newly determined to be in contact and (b) features of the triangles (both
  // so we can track which features are in contact, and which features aren't).

  // Get the distance between each pair of triangles.
  for (int i = 0; i < candidate_tris.size(); ++i) {
    // See whether the triangle is in the pairs to ignore.
    if (pairs_to_ignore.find(candidate_tris[i]) != pairs_to_ignore.end())
      continue;

    // Get the two triangles.
    mA.get_triangle(candidate_tris[i].first).transform(poseA, &tA);
    mB.get_triangle(candidate_tris[i].second).transform(poseB, &tB);

    // Get the distance between the two triangles.
    Vector3<T> closest_on_tA, closest_on_tB;
    T tri_distance = sqrt(tA.CalcSquareDistance(
        tB, &closest_on_tA, &closest_on_tB)); 

    // See whether the distance is sufficiently small.
    if (tri_distance < intersecting_threshold) {
      // Verify that the distance isn't *too* small.
      DRAKE_DEMAND(tri_distance > 0);

      // Compute the surface normal such that it points toward A.
      Vector3<T> normal = closest_on_tA - closest_on_tB;
      normal.normalize();

      // Determine the offset for the plane parallel and halfway between the
      // two planes passing through the closest points.
      const T offsetA = normal.dot(closest_on_tA);
      const T offsetB = normal.dot(closest_on_tB);
      const T offset = (offsetA + offsetB) / 2.0;

      // Project all points from a triangle to the plane. Stores a point
      // if it is within tolerance.
      auto project_and_store = [&normal, offset, tolerance](
          const Triangle3<T>& t, std::vector<int>* p) {
        const T d_a = t.a().dot(normal) - offset;
        if (abs(d_a) < tolerance)
          p->push_back(0);

        const T d_b = t.b().dot(normal) - offset;
        if (abs(d_b) < tolerance)
          p->push_back(1);

        const T d_c = t.c().dot(normal) - offset;
        if (abs(d_c) < tolerance)
          p->push_back(2);
      };

      // Project all points from both triangles to the plane, storing indices
      // of points that lie within the tolerance away.
      std::vector<int> pA, pB;
      project_and_store(tA, &pA);
      project_and_store(tB, &pA);

      // Degenerate cases that we can reject immediately: nothing/anything,
      // vertex/vertex and vertex edge.
      if (pA.size() == 0 || pB.size() == 0)
        continue;
      if (pA.size() == 1) {
        if (pB.size() <= 2)
          continue;
      } else {
        if (pA.size() == 2 && pB.size() == 1)
          continue;
      }

      // Note: We treat the case of no intersection after the projection as a
      // singular configuration (i.e., active only instantaneously) and keep
      // looping.

      // Remaining cases are edge/edge, vertex/face, edge/face, and face/face.
      switch (pA.size()) {
        case 1: {
          // Record the contact data.
          contacts->push_back(TriTriContactData());
          contacts->back().tA = &mA.get_triangle(candidate_tris[i].first); 
          contacts->back().tB = &mB.get_triangle(candidate_second[i].first);
          contacts->back().feature_A_id = static_cast<void*>(pA[0]);
          contacts->back().typeA = FeatureType::kVertex;
          contacts->back().typeB = FeatureType::kFace;
          break;
        }

        case 2: {
          if (pB.size() == 2) {
            // Edge/edge case. Intersect the projected edges.
            auto eA_2d = ProjectTo2d(
                std::make_pair(tA.get_vertex(pA.front()),
                               tA.get_vertex(pA.back())), normal);
            auto eB_2d = ProjectTo2d(
                std::make_pair(tB.get_vertex(pB.front()),
                               tB.get_vertex(pB.back())), normal);
            
            // Record the contact data.
            contacts->push_back(TriTriContactData());  
            contacts->back().tA = &mA.get_triangle(candidate_tris[i].first); 
            contacts->back().tB = &mB.get_triangle(candidate_second[i].first);
            contacts->back().feature_A_id = GetEdgeIndex(
                mA.get_triangle(candidate_tris[i].first),
                pA.front(), pA.back());
            contacts->back().feature_B_id = GetEdgeIndex(
                mB.get_triangle(candidate_tris[i].second),
                pB.front(), pB.back());
            contacts->back().typeA = FeatureType::kEdge;
            contacts->back().typeB = FeatureType::kEdge;
          } else {
            DRAKE_DEMAND(pB.size() == 3);
            // Edge/face case. Intersect the edge with the projected triangle.

            // Record the contact data.
            contacts->push_back(TriTriContactData());  
            contacts->back().tA = &mA.get_triangle(candidate_tris[i].first); 
            contacts->back().tB = &mB.get_triangle(candidate_second[i].first);
            contacts->back().feature_A_id = GetEdgeIndex(
                mA.get_triangle(candidate_tris[i].first),
                pA.front(), pA.back());
            contacts->back().typeA = FeatureType::kEdge;
            contacts->back().typeB = FeatureType::kFace;
          }
          break;
        }

        case 3: {
          if (pB.size() == 1) {
            // Record the contact data.
            contacts->push_back(TriTriContactData());  
            contacts->back().tA = &mA.get_triangle(candidate_tris[i].first); 
            contacts->back().tB = &mB.get_triangle(candidate_second[i].first);
            contacts->back().feature_B_id = static_cast<void*>(
                mB.get_triangle(candidate_tris[i].second).get_vertex(pB[0]);
            contacts->back().typeA = FeatureType::kFace;
            contacts->back().typeB = FeatureType::kVertex;
          } else {
            // Edge / face. Intersect the edge with the projected triangle.
            if (pB.size() == 2) {
              // Record the contact data.
              contacts->push_back(TriTriContactData());  
              contacts->back().tA = &mA.get_triangle(candidate_tris[i].first); 
              contacts->back().tB = &mB.get_triangle(candidate_second[i].first);
              contacts->back().feature_B_id = GetEdgeIndex(
                  mB.get_triangle(candidate_tris[i].second),
                  pB.front(), pB.back());
              contacts->back().typeA = FeatureType::kFace;
              contacts->back().typeB = FeatureType::kEdge;
            } else {
              // Record the contact data.
              contacts->push_back(TriTriContactData());  
              contacts->back().tA = &mA.get_triangle(candidate_tris[i].first); 
              contacts->back().tB = &mB.get_triangle(candidate_second[i].first);
              contacts->back().typeA = FeatureType::kFace;
              contacts->back().typeB = FeatureType::kFace;
            }
          }
          break;
        }

        default:
          DRAKE_ABORT();  // Should never get here.
      }
    }
  }

  return distance;
}

