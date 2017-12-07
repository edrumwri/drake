#pragma once

namespace drake {
namespace multibody {

template <class T>
class TrimeshCollisionDetection {
 public:
  T CalcSignedDistance(const Pose3d<T>& poseA, const Trimesh<T>& tA, const Pose3d<T>& poseB, const Trimesh<T>& tB) const;

 private:
  // Structure for doing broad phase collision detection.
  struct BoundsStruct {
    bool end;                   // bounds is for start or end
    CollisionGeometryPtr geom;  // the geometry
    AABB<T>* aabb;                 // Pointer to the AABB. 
    bool operator<(const BoundsStruct<T>& bs) const { return (!end && bs.end); }
  };

  void DoBroadPhase(std::vector<std::pair<int, int>>* to_check);

  // AABB bounds vectors.
  std::vector<std::pair<T, BoundsStruct<T>>> x_bounds_, y_bounds_, z_bounds_;

  // Sweep and prune data structure. Mutable because it is used only for
  // speeding computations- it does not affect the correctness.

  // Axis-aligned bounding boxes for each triangle.
};

}  // multibody
}  // drake

