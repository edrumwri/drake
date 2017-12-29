#pragma once

#include <map>
#include <set>

#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/multibody/rigid_body_plant/aabb.h"
#include "drake/multibody/rigid_body_plant/trimesh.h"
#include "drake/multibody/rigid_body_plant/tri_tri_contact_data.h"

namespace drake {
namespace multibody {

template <class T>
class TrimeshColdet {
 public:
  T CalcMinDistance(const Trimesh<T>& mA,
                    const Trimesh<T>& mB,
                    const std::vector<std::pair<int, int>>& pairs_to_check)
                    const;
  T CalcDistances(const Trimesh<T>& mA,
                  const Trimesh<T>& mB,
                  const std::vector<std::pair<int, int>>& pairs_to_check,
                  std::vector<std::pair<std::pair<int, int>, T>>* distances)
                  const;
  void CalcIntersections(
                 const Trimesh<T>& mA,
                 const Trimesh<T>& mB,
                 const Isometry3<T>& poseA,
                 const Isometry3<T>& poseB,
                 const std::vector<std::pair<int, int>>& triangle_indices,
                 std::vector<TriTriContactData<T>>* contacts) const;
  void UpdateAABBs(const Trimesh<T>& mesh, const Isometry3<T>& wTm);
  void UpdateBroadPhaseStructs();
  void DoBroadPhase(const Trimesh<T>& mA, const Trimesh<T>& mB,
                    std::vector<std::pair<int, int>>* to_check) const;
  void AddMesh(const Trimesh<T>* mesh);

  // TODO: Remove this method once UpdateAABBs() tested
  void SetPose(const Trimesh<T>* trimesh, const Isometry3<T>& wTm) {
    poses_[trimesh] = wTm;
  } 

 private:
  // Structure for doing broad phase collision detection.
  struct BoundsStruct {
    bool end;                      // Bounds is for start (false) or end (true).
    int tri{-1};                   // The index of the bounded triangle.
    AABB<T>* aabb;                 // Pointer to the AABB. 
    const Trimesh<T>* mesh;        // Pointer to the mesh.
    bool operator<(const BoundsStruct& bs) const { return (!end && bs.end); }
  };

  // Structure for holding all broad phase data for a single trimesh.
  struct BoundsStructsVectorPlusAxis {
    std::vector<std::pair<T, BoundsStruct>> bound_struct_vector;
    Vector3<T> axis;
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
      const std::vector<BoundsStructsVectorPlusAxis>& bs_vec_plus_axis_A,
      const std::vector<BoundsStructsVectorPlusAxis>& bs_vec_plus_axis_B,
      const Trimesh<T>* mesh_A,
      const Trimesh<T>* mesh_B,
      std::set<std::pair<int, int>>* overlaps) const;

  // The poses for each triangle mesh; these correspond to the transformation
  // from the trimesh frame to the world frame.
  std::map<const Trimesh<T>*, Isometry3<T>> poses_;

  // Sweep and prune data structure. Mutable because it is used only for
  // speeding computations- it does not affect the correctness. The first
  // element of each vector is a pair of (scalar, BoundStruct) that tells HOW
  // to do the sweep and prune. The second element is a vector that gives the
  // axis that should be sorted along.
  std::map<const Trimesh<T>*, std::vector<BoundsStructsVectorPlusAxis>> bounds_;

  // Vector of all AABBs created.
  std::vector<std::unique_ptr<AABB<T>>> aabbs_;
};

}  // namespace multibody
}  // namespace drake

