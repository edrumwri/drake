#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_plant/triangle.h"

namespace drake {
namespace multibody {

enum class FeatureType {
  kVertex,
  kEdge,
  kFace
};

template <class T>
struct TriTriContactData {
  void* idA;          // The identifier for geometry A.
  void* idB;          // The identifier for geometry B.
  FeatureType typeA;  // The feature in contact on geometry A.
  FeatureType typeB;  // The feature in contact on geometry B.
  void* feature_A_id; // The identifier of the feature on geometry A.
  void* feature_B_id; // The identifier of the feature on geometry B.
  Triangle3<T>* tA;   // The triangle on geometry A.
  Triangle3<T>* tB;   // The triangle on geometry B.

  /// Gets the surface normal (pointing toward A).
  Vector3<T> GetSurfaceNormalExpressedInWorld(
      const Isometry3<T>& poseA, const Isometry3<T>& poseB) const {
    switch (typeA) {
      case FeatureType::kVertex: 
        DRAKE_DEMAND(typeB == FeatureType::kFace);
        return poseB.linear() * tB->CalcNormal();
        break;

      case FeatureType::kEdge:
        DRAKE_DEMAND(typeB == FeatureType::kEdge ||
                     typeB == FeatureType::kFace);
        if (typeB == FeatureType::kEdge) {
          // TODO: Compute the cross product between the two edges.
        } else {
          return poseB.linear() * tB->CalcNormal();
        }
        break;

      case FeatureType::kFace:
        if (typeB == FeatureType::kVertex || typeB == FeatureType::kEdge) {
          return -poseA.linear() * tA->CalcNormal();
        } else {
          // Average the two normals.
          const Vector3<T> nA = -poseA.linear() * tA->CalcNormal(); 
          const Vector3<T> nB = poseB.linear() * tB->CalcNormal();
          return ((nA * 0.5) + (nB * 0.5)).normalized();
        break;
      }
    }
  }

  /// Gets the contact points.
  void DetermineContactPoints(const Vector3<T>& normal, T offset,
      const Isometry3<T>& poseA, const Isometry3<T>& poseB,
      std::vector<Vector3<T>>* points) const {
    DRAKE_DEMAND(points);
    DRAKE_DEMAND(points->is_empty());

    // Points calculated from intersections.
    Vector2<T> isects[6];

    switch (typeA) {
      case FeatureType::kVertex:  { 
        DRAKE_DEMAND(typeB == FeatureType::kFace);

        // Get the triangle and the vector representing the vertex.

        // Project the vertex and the triangle to the contact plane. 
        const Vector2<T> v_2d = ProjectTo2d(v);
        const Triangle2<T> t_2d = t.ProjectTo2d();

        // Only process points that lie within the projected triangle.
        if (t_2d.PointInside(v_2d)) {
          points->push_back(Unproject(v_2d, offset));
          return;
        }

      case FeatureType::kEdge:
        if (typeB == FeatureType::kEdge) {
          // Project each edge to 2D.

          // Intersect the two line segments together.
          Triangle2<T>::SegSegIntersectType intersect_code = 
            Triangle2<T>::IntersectSegs(std::make_pair(vA1_2d, vA2_2d),
                                        std::make_pair(vB1_2d, vB2_2d),
                                        &isects[0], &isects[1]);

          switch (intersect_code) {
            case Triangle2<T>::kSegSegVertex:
            case Triangle2<T>::kSegSegIntersect:
              // Un-project the first point.
              points->push_back(Unproject(isects[0], offset));
              break;

            case Triangle2<T>::kSegSegEdge: 
              // Un-project the two points.
              points->push_back(Unproject(isects[0], offset));
              points->push_back(Unproject(isects[1], offset));
              break;

            case Triangle2<T>::kSegSegNoIntersect:
              // Do nothing.
              break;
          }
        } else {
          DRAKE_DEMAND(typeB == FeatureType::kFace);

          // Project the edge and the face to the contact plane.
          const Triangle2<T> tB_2d = tB.ProjectTo2d();

          // Intersect the triangle with the line segment.
          Triangle2<T>::SegTriIntersectType intersect_type = tB_2d.Intersect(
              std::make_pair(vA1_2d, vA2_2d), tol, &isects[0], &isects[1]); 

          switch (intersect_type) {
            case Triangle2<T>::kSegSegVertex:
            case Triangle2<T>::kSegSegEdge:
              // Un-project the first point.
              points->push_back(Unproject(isects[0], offset));
              break;

            case Triangle2<T>::kSegTriPlanarIntersect:
            case Triangle2<T>::kSegTriEdgeOverlap:
            case Triangle2<T>::kSegTriInside:
              // Un-project the two points.
              points->push_back(Unproject(isects[0], offset));
              points->push_back(Unproject(isects[1], offset));
              break;

            case Triangle2<T>::kSegTriNoIntersect:
              // Do nothing.
              break;
          }
        }
        return;

      case FeatureType::kFace:
        if (typeB == FeatureType::kVertex) {
          // Get the triangle and the vector representing the vertex.

          // Project the vertex and the triangle to the contact plane. 
          const Vector2<T> v_2d = ProjectTo2d(v);
          const Triangle2<T> t_2d = t.ProjectTo2d();

          // Only process points that lie within the projected triangle.
          if (t_2d.PointInside(v_2d)) {
            points->push_back(Unproject(v_2d, offset));
            return;
          }
        } else {
          if (typeB == FeatureType::kEdge) {
            // Project the edge and the face to the contact plane.
            const Triangle2<T> tA_2d = tA.ProjectTo2d();

            // Intersect the triangle with the line segment.
            Triangle2<T>::SegTriIntersectType intersect_type = tA_2d.Intersect(
                std::make_pair(vB1_2d, vB2_2d), tol, &isects[0], &isects[1]); 

            switch (intersect_type) {
              case Triangle2<T>::kSegSegVertex:
              case Triangle2<T>::kSegSegEdge:
                // Un-project the first point.
                points->push_back(Unproject(isects[0], offset));
                break;

              case Triangle2<T>::kSegTriPlanarIntersect:
              case Triangle2<T>::kSegTriEdgeOverlap:
              case Triangle2<T>::kSegTriInside:
                // Un-project the two points.
                points->push_back(Unproject(isects[0], offset));
                points->push_back(Unproject(isects[1], offset));
                break;

              case Triangle2<T>::kSegTriNoIntersect:
                // Do nothing.
                break;
            }

            return;
          } else {
              // It is a face-face intersection. Project both triangles to 2D.
              const Triangle2<T> tA_2d = tA.ProjectTo2d();
              const Triangle2<T> tB_2d = tB.ProjectTo2d();

              // Intersect the two triangles and then unproject the vertices of 
              // the construction.
              int num_intersections = tA_2d.Intersect(tB_2d, &isects[0]);
              for (int i = 0; i < num_intersections; ++i)
                points->push_back(Unproject(isects[i], offset));
              return;
            }
          }
      }
    }
  }
};

}  // multibody
}  // drake
