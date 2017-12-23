#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/math/project_3d_to_2d.h"
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
  void* idA;                // The identifier for geometry A.
  void* idB;                // The identifier for geometry B.
  FeatureType typeA;        // The feature in contact on geometry A.
  FeatureType typeB;        // The feature in contact on geometry B.
  void* feature_A_id;       // The identifier of the feature on geometry A.
  void* feature_B_id;       // The identifier of the feature on geometry B.
  const Triangle3<T>* tA;   // The triangle on geometry A.
  const Triangle3<T>* tB;   // The triangle on geometry B.

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
          // Compute the cross product between the two edges.
          auto edgeA = tA->get_edge(reinterpret_cast<long>(feature_A_id)); 
          auto edgeB = tB->get_edge(reinterpret_cast<long>(feature_B_id));
          const Vector3<T> vA = (edgeA.second - edgeA.first).normalized();
          const Vector3<T> vB = (edgeB.second - edgeB.first).normalized();
          return vA.cross(vB).normalized(); 
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
    DRAKE_DEMAND(points->empty());

    // TODO: Compute tolerance in an informed manner.
    const T tol = std::numeric_limits<double>::epsilon();

    // Points calculated from intersections.
    Vector2<T> isects[6];

    // Get the projection matrix.
    const auto P = math::Determine3dTo2dProjectionMatrix(normal);

    switch (typeA) {
      case FeatureType::kVertex:  { 
        DRAKE_DEMAND(typeB == FeatureType::kFace);

        // Get the triangle and the vector representing the vertex.
        const Vector3<T>& v = tA->get_vertex(
            reinterpret_cast<long>(feature_A_id)); 
        const Triangle3<T>& t = *tB; 

        // Project the vertex and the triangle to the contact plane. 
        const Vector2<T> v_2d = P * v;
        const Triangle2<T> t_2d = t.ProjectTo2d(P);

        // Only process points that lie within the projected triangle.
        if (t_2d.PointInside(v_2d))
          points->push_back(Unproject(normal, P, offset, v_2d));
        return;
      }

      case FeatureType::kEdge:
        if (typeB == FeatureType::kEdge) {
          // Get the two edges.
          auto edgeA = tA->get_edge(reinterpret_cast<long>(feature_A_id)); 
          auto edgeB = tB->get_edge(reinterpret_cast<long>(feature_B_id)); 

          // Project each edge to 2D.
          auto eA_2d = std::make_pair(P * edgeA.first, P * edgeA.second);
          auto eB_2d = std::make_pair(P * edgeB.first, P * edgeB.second);

          // Intersect the two line segments together.
          typename Triangle2<T>::SegSegIntersectType intersect_code = 
            Triangle2<T>::IntersectSegs(eA_2d, eB_2d,
                                        &isects[0], &isects[1]);

          switch (intersect_code) {
            case Triangle2<T>::kSegSegVertex:
            case Triangle2<T>::kSegSegIntersect:
              // Un-project the first point.
              points->push_back(Unproject(normal, P, offset, isects[0]));
              break;

            case Triangle2<T>::kSegSegEdge: 
              // Un-project the two points.
              points->push_back(Unproject(normal, P, offset, isects[0]));
              points->push_back(Unproject(normal, P, offset, isects[1]));
              break;

            case Triangle2<T>::kSegSegNoIntersect:
              // Do nothing.
              break;
          }
        } else {
          DRAKE_DEMAND(typeB == FeatureType::kFace);

          // Project the edge and the face to the contact plane.
          auto edge = tA->get_edge(reinterpret_cast<long>(feature_A_id));
          auto e_2d = std::make_pair(P * edge.first, P * edge.second);
          const Triangle2<T> tB_2d = tB->ProjectTo2d(P);

          // Intersect the triangle with the line segment.
          typename Triangle2<T>::SegTriIntersectType intersect_type =
              tB_2d.Intersect(e_2d, tol, &isects[0], &isects[1]); 

          switch (intersect_type) {
            case Triangle2<T>::kSegSegVertex:
            case Triangle2<T>::kSegSegEdge:
              // Un-project the first point.
              points->push_back(Unproject(normal, P, offset, isects[0]));
              break;

            case Triangle2<T>::kSegTriPlanarIntersect:
            case Triangle2<T>::kSegTriEdgeOverlap:
            case Triangle2<T>::kSegTriInside:
              // Un-project the two points.
              points->push_back(Unproject(normal, P, offset, isects[0]));
              points->push_back(Unproject(normal, P, offset, isects[1]));
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
          const Triangle3<T>& t = *tA; 
          const Vector3<T>& v = tB->get_vertex(
              reinterpret_cast<long>(feature_B_id)); 

          // Project the vertex and the triangle to the contact plane. 
          const Vector2<T> v_2d = P * v;
          const Triangle2<T> t_2d = t.ProjectTo2d(P);

          // Only process points that lie within the projected triangle.
          if (t_2d.PointInside(v_2d)) {
            points->push_back(Unproject(normal, P, offset, v_2d));
            return;
          }
        } else {
          if (typeB == FeatureType::kEdge) {
            // Project the edge and the face to the contact plane.
            const Triangle2<T> tA_2d = tA->ProjectTo2d(P);
            auto edge = tB->get_edge(reinterpret_cast<long>(feature_B_id)); 
            auto e_2d = std::make_pair(P * edge.first, P * edge.second);

            // Intersect the triangle with the line segment.
            typename Triangle2<T>::SegTriIntersectType intersect_type =
                tA_2d.Intersect(e_2d, tol, &isects[0], &isects[1]); 

            switch (intersect_type) {
              case Triangle2<T>::kSegSegVertex:
              case Triangle2<T>::kSegSegEdge:
                // Un-project the first point.
                points->push_back(Unproject(normal, P, offset, isects[0]));
                break;

              case Triangle2<T>::kSegTriPlanarIntersect:
              case Triangle2<T>::kSegTriEdgeOverlap:
              case Triangle2<T>::kSegTriInside:
                // Un-project the two points.
                points->push_back(Unproject(normal, P, offset, isects[0]));
                points->push_back(Unproject(normal, P, offset, isects[1]));
                break;

              case Triangle2<T>::kSegTriNoIntersect:
                // Do nothing.
                break;
            }

            return;
          } else {
              // It is a face-face intersection. Project both triangles to 2D.
              const Triangle2<T> tA_2d = tA->ProjectTo2d(P);
              const Triangle2<T> tB_2d = tB->ProjectTo2d(P);

              // Intersect the two triangles and then unproject the vertices of 
              // the construction.
              int num_intersections = tA_2d.Intersect(tB_2d, &isects[0]);
              for (int i = 0; i < num_intersections; ++i)
                points->push_back(Unproject(normal, P, offset, isects[i]));
              return;
            }
          }
        }
    }

 private:
  static Vector3<T> Unproject(
      const Vector3<T>& normal,
      const Eigen::Matrix<T, 2, 3>& P, T offset, const Vector2<T>& p) {
    return P.transpose() * p + normal*offset;
  }
};

}  // multibody
}  // drake
