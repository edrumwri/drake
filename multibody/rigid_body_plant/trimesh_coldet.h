#pragma once

#include <set>

#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_plant/aabb.h"
#include "drake/multibody/rigid_body_plant/trimesh.h"

namespace drake {
namespace multibody {

template <class T>
class TrimeshColdet {
 public:
  T CalcDistance(const Isometry3<T>& poseA, const Trimesh<T>& mA,
                 const Isometry3<T>& poseB, const Trimesh<T>& mB,
                 const std::set<std::pair<int, int>>& pairs_to_ignore) const;

 private:
  // Structure for doing broad phase collision detection.
  struct BoundsStruct {
    bool end;                      // Bounds is for start (false) or end (true).
    int tri{-1};                   // The index of the bounded triangle.
    AABB<T>* aabb;                 // Pointer to the AABB. 
    bool operator<(const BoundsStruct& bs) const { return (!end && bs.end); }
  };

  void DoBroadPhase(const Trimesh<T>& mA, const Trimesh<T>& mB,
                    std::vector<std::pair<int, int>>* to_check) const;
  void UpdateAABBs(const Trimesh<T>& mesh, const Isometry3<T>& wTm);
  void UpdateBroadPhaseStructs();
  void UpdateOverlaps(
      int bounds_index,
      std::set<std::pair<int, int>>* overlaps) const; 

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

