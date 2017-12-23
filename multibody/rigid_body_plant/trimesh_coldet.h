#pragma once

#include <map>
#include <set>

#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_plant/aabb.h"
#include "drake/multibody/rigid_body_plant/trimesh.h"
#include "drake/multibody/rigid_body_plant/tri_tri_contact_data.h"

namespace drake {
namespace multibody {

template <class T>
class TrimeshColdet {
 public:
  T CalcDistance(const Trimesh<T>& mA,
                 const Trimesh<T>& mB,
                 const std::vector<std::pair<int, int>>& pairs_to_check) const;
  void CalcIntersections(
                 const Trimesh<T>& mA,
                 const Trimesh<T>& mB,
                 const std::vector<std::pair<int, int>>& pairs_to_check,
                 std::vector<TriTriContactData<T>>* contacts) const;
  void UpdateAABBs(const Trimesh<T>& mesh, const Isometry3<T>& wTm);
  void UpdateBroadPhaseStructs();
  void DoBroadPhase(const Trimesh<T>& mA, const Trimesh<T>& mB,
                    std::vector<std::pair<int, int>>* to_check) const;

 private:
  // Structure for doing broad phase collision detection.
  struct BoundsStruct {
    bool end;                      // Bounds is for start (false) or end (true).
    int tri{-1};                   // The index of the bounded triangle.
    AABB<T>* aabb;                 // Pointer to the AABB. 
    bool operator<(const BoundsStruct& bs) const { return (!end && bs.end); }
  };

  static void* GetEdgeIndex(int v0, int v1);
  static Vector3<T> ReverseProject(
      const Vector2<T>& point, const Vector3<T>& normal, T offset);
  static Vector2<T> ProjectTo2d(
      const Vector3<T>& point, const Vector3<T>& normal);
  static std::pair<Vector2<T>, Vector2<T>> ProjectTo2d(
      const std::pair<Vector3<T>, Vector3<T>>& edge,
      const Vector3<T>& normal);
  void UpdateOverlaps(
      int bounds_index,
      std::set<std::pair<int, int>>* overlaps) const; 

  // The poses for each triangle mesh; these correspond to the transformation
  // from the trimesh frame to the world frame.
  std::map<const Trimesh<T>*, Isometry3<T>> poses_;

  // The bounding structs for each triangle mesh.
  std::map<const Trimesh<T>*, std::vector<BoundsStruct*>> trimesh_bs_;

  // Sweep and prune data structure. Mutable because it is used only for
  // speeding computations- it does not affect the correctness. The first
  // element of each vector is a pair of (scalar, BoundStruct) that tells HOW
  // to do the sweep and prune. The second element is a vector that gives the
  // axis that should be sorted along.
  std::vector<std::pair<std::vector<std::pair<T, BoundsStruct>>,
                        Vector3<T>>> bounds_;
};

}  // namespace multibody
}  // namespace drake

