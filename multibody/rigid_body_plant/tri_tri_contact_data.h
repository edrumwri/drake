#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/common/eigen_types.h"
#include "drake/math/project_3d_to_2d.h"
#include "drake/multibody/collision/element.h"
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
  const collision::Element* idA;  // The identifier for geometry A.
  const collision::Element* idB;  // The identifier for geometry B.
  FeatureType typeA;              // The feature in contact on geometry A.
  FeatureType typeB;              // The feature in contact on geometry B.
  const Triangle3<T>* tA;         // The triangle on geometry A.
  const Triangle3<T>* tB;         // The triangle on geometry B.

  // Feature identifiers. For a face, the feature ID will be null. Otherwise,
  // the ID will hold the vertex ID or edge ID (an integer type, stored as a
  // long to keep the compiler happy), as appropriate.
  void* feature_A_id{nullptr};
  void* feature_B_id{nullptr};

  /// Gets the vertices involved in contact from A, expressed in A's frame.
  int get_A_vertices(Vector3<T>* array) const {
    return 0;
  }

  /// Gets the vertices involved in contact from B, expressed in B's frame.
  int get_B_vertices(Vector3<T>* array) const {
    return 0;
  }

  /// Determines whether the type of contact is degenerate.
  bool is_degenerate() const {
    if (typeA == FeatureType::kVertex && typeB == FeatureType::kVertex)
      return true;
    if (typeA == FeatureType::kVertex && typeB == FeatureType::kEdge)
      return true;
    if (typeA == FeatureType::kEdge && typeB == FeatureType::kVertex)
      return true;

    return false;
  }

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
          Vector3<T> nB = poseB.linear() * tB->CalcNormal();
          if (nA.dot(nB) < 0)
            nB = -nB;
          return ((nA * 0.5) + (nB * 0.5)).normalized();
        break;
      }
    }
  }

  /// Gets the contact points and returns the signed distance.
  T DetermineContactPoints(const Vector3<T>& normal,
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
        if (typeB == FeatureType::kVertex || typeB == FeatureType::kEdge)
          return 0;

        // Get the triangle and the vector representing the vertex.
        const Vector3<T> v = poseA * tA->get_vertex(
            reinterpret_cast<long>(feature_A_id));
        const Vector3<T> v1 = poseB * tB->a();
        const Vector3<T> v2 = poseB * tB->b();
        const Vector3<T> v3 = poseB * tB->c();
        Triangle3<T> t(&v1, &v2, &v3);

        // Determine the contact plane.
        const T offA = normal.dot(v);
        const T offB = normal.dot(t.a());
        T offset = 0.5 * (offA + offB);
        SPDLOG_DEBUG(drake::log(), "Contact plane: normal({}) offset: {}",
                     normal.transpose(), offset);

        // Project the vertex and the triangle to the contact plane. 
        const Vector2<T> v_2d = P * v;
        const Triangle2<T> t_2d = t.ProjectTo2d(P);

        // Only process points that lie within the projected triangle.
        if (t_2d.PointInside(v_2d)) {
          points->push_back(Unproject(normal, P, offset, v_2d));
          SPDLOG_DEBUG(drake::log(), "Vertex/face contact: {}",
                       points->back().transpose());
        } else {
          SPDLOG_DEBUG(drake::log(), "Vertex/face contact: point outside");
        }

        return offA - offB;
      }

      case FeatureType::kEdge:
        if (typeB == FeatureType::kEdge) {
          // Get the two edges.
          auto edgeA = tA->get_edge(reinterpret_cast<long>(feature_A_id)); 
          auto edgeB = tB->get_edge(reinterpret_cast<long>(feature_B_id)); 

          // Transform the edges into the world.
          edgeA.first = poseA * edgeA.first;
          edgeA.second = poseA * edgeA.second;
          edgeB.first = poseB * edgeB.first;
          edgeB.second = poseB * edgeB.second;

          // Determine the contact plane.
          const T offA = normal.dot(edgeA.first);
          const T offB = normal.dot(edgeB.first);
          T offset = 0.5 * (offA + offB);
          SPDLOG_DEBUG(drake::log(), "Contact plane: normal({}) offset: {}",
                       normal.transpose(), offset);

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
              SPDLOG_DEBUG(drake::log(), "Edge/edge contact: point {}",
                           points->back().transpose());
              break;

            case Triangle2<T>::kSegSegEdge: 
              // Un-project the two points.
              points->push_back(Unproject(normal, P, offset, isects[0]));
              SPDLOG_DEBUG(drake::log(), "Edge/edge contact: point {}",
                           points->back().transpose());
              points->push_back(Unproject(normal, P, offset, isects[1]));
              SPDLOG_DEBUG(drake::log(), "Edge/edge contact: point {}",
                           points->back().transpose());
              break;

            case Triangle2<T>::kSegSegNoIntersect:
              // Do nothing.
              SPDLOG_DEBUG(drake::log(), "Edge/edge contact: point outside");
              break;
          }

          return offA - offB;
        } else {
          if (typeB == FeatureType::kVertex)
            return 0;

          // Project the edge and the face to the contact plane.
          auto edge = tA->get_edge(reinterpret_cast<long>(feature_A_id));
          edge.first = poseA * edge.first;
          edge.second = poseA * edge.second;
          auto e_2d = std::make_pair(P * edge.first, P * edge.second);
          const Vector3<T> v1 = poseB * tB->a();
          const Vector3<T> v2 = poseB * tB->b();
          const Vector3<T> v3 = poseB * tB->c();
          Triangle3<T> t(&v1, &v2, &v3); 
          const Triangle2<T> tB_2d = t.ProjectTo2d(P);

          // Determine the contact plane.
          const T offA = normal.dot(edge.first);
          const T offB = normal.dot(t.a());
          T offset = 0.5 * (offA + offB);
          SPDLOG_DEBUG(drake::log(), "Contact plane: normal({}) offset: {}",
                       normal.transpose(), offset);

          // Intersect the triangle with the line segment.
          typename Triangle2<T>::SegTriIntersectType intersect_type =
              tB_2d.Intersect(e_2d, tol, &isects[0], &isects[1]); 

          switch (intersect_type) {
            case Triangle2<T>::kSegSegVertex:
            case Triangle2<T>::kSegSegEdge:
              // Un-project the first point.
              points->push_back(Unproject(normal, P, offset, isects[0]));
              SPDLOG_DEBUG(drake::log(), "Edge/face contact: point {}",
                           points->back().transpose());
              break;

            case Triangle2<T>::kSegTriPlanarIntersect:
            case Triangle2<T>::kSegTriEdgeOverlap:
            case Triangle2<T>::kSegTriInside:
              // Un-project the two points.
              points->push_back(Unproject(normal, P, offset, isects[0]));
              SPDLOG_DEBUG(drake::log(), "Edge/face contact: point {}",
                           points->back().transpose());
              points->push_back(Unproject(normal, P, offset, isects[1]));
              SPDLOG_DEBUG(drake::log(), "Edge/face contact: point {}",
                           points->back().transpose());
              break;

            case Triangle2<T>::kSegTriNoIntersect:
              SPDLOG_DEBUG(drake::log(), "Edge/face contact: point outside");
              // Do nothing.
              break;
          }

          return offA - offB;
        }

      case FeatureType::kFace:
        if (typeB == FeatureType::kVertex) {
          // Get the triangle and the vector representing the vertex.
          const Vector3<T> v1 = poseA * tA->a();
          const Vector3<T> v2 = poseA * tA->b();
          const Vector3<T> v3 = poseA * tA->c();
          Triangle3<T> t(&v1, &v2, &v3); 
          const Vector3<T> v = poseB * tB->get_vertex(
              reinterpret_cast<long>(feature_B_id)); 

          // Determine the contact plane.
          const T offA = normal.dot(t.a());
          const T offB = normal.dot(v);
          T offset = 0.5 * (offA + offB);
          SPDLOG_DEBUG(drake::log(), "Contact plane: normal({}) offset: {}",
                       normal.transpose(), offset);

          // Project the vertex and the triangle to the contact plane. 
          const Vector2<T> v_2d = P * v;
          const Triangle2<T> t_2d = t.ProjectTo2d(P);

          // Only process points that lie within the projected triangle.
          if (t_2d.PointInside(v_2d)) {
            points->push_back(Unproject(normal, P, offset, v_2d));
            SPDLOG_DEBUG(drake::log(), "Vertex/face contact: point {}",
                         points->back().transpose());
          } else {
            SPDLOG_DEBUG(drake::log(), "Vertex/face contact: point outside");
          }

          return offA - offB;
        } else {
          if (typeB == FeatureType::kEdge) {
            // Project the edge and the face to the contact plane.
            const Vector3<T> v1 = poseA * tA->a();
            const Vector3<T> v2 = poseA * tA->b();
            const Vector3<T> v3 = poseA * tA->c();
            const Triangle3<T> t(&v1, &v2, &v3); 
            const Triangle2<T> tA_2d = t.ProjectTo2d(P);
            auto edge = tB->get_edge(reinterpret_cast<long>(feature_B_id));
            edge.first = poseB * edge.first;
            edge.second = poseB * edge.second;
            auto e_2d = std::make_pair(P * edge.first, P * edge.second);

            // Determine the contact plane.
            const T offA = normal.dot(t.a());
            const T offB = normal.dot(edge.first);
            T offset = 0.5 * (offA + offB);
            SPDLOG_DEBUG(drake::log(), "Contact plane: normal({}) offset: {}",
                         normal.transpose(), offset);

            // Intersect the triangle with the line segment.
            typename Triangle2<T>::SegTriIntersectType intersect_type =
                tA_2d.Intersect(e_2d, tol, &isects[0], &isects[1]); 

            switch (intersect_type) {
              case Triangle2<T>::kSegSegVertex:
              case Triangle2<T>::kSegSegEdge:
                // Un-project the first point.
                points->push_back(Unproject(normal, P, offset, isects[0]));
                SPDLOG_DEBUG(drake::log(), "Edge/face contact: point {}",
                             points->back().transpose());
                break;

              case Triangle2<T>::kSegTriPlanarIntersect:
              case Triangle2<T>::kSegTriEdgeOverlap:
              case Triangle2<T>::kSegTriInside:
                // Un-project the two points.
                points->push_back(Unproject(normal, P, offset, isects[0]));
                SPDLOG_DEBUG(drake::log(), "Edge/face contact: point {}",
                             points->back().transpose());
                points->push_back(Unproject(normal, P, offset, isects[1]));
                SPDLOG_DEBUG(drake::log(), "Edge/face contact: point {}",
                             points->back().transpose());
                break;

              case Triangle2<T>::kSegTriNoIntersect:
                SPDLOG_DEBUG(drake::log(), "Edge/face contact: point outside");
                // Do nothing.
                break;
            }

            return offA - offB;
          } else {
              // It is a face-face intersection. Project both triangles to 2D.
              const Vector3<T> vA1 = poseA * tA->a();
              const Vector3<T> vA2 = poseA * tA->b();
              const Vector3<T> vA3 = poseA * tA->c();
              const Vector3<T> vB1 = poseB * tB->a();
              const Vector3<T> vB2 = poseB * tB->b();
              const Vector3<T> vB3 = poseB * tB->c();
              const Triangle3<T> tA_3d(&vA1, &vA2, &vA3); 
              const Triangle3<T> tB_3d(&vB1, &vB2, &vB3); 
              const Triangle2<T> tA_2d = tA_3d.ProjectTo2d(P);
              const Triangle2<T> tB_2d = tB_3d.ProjectTo2d(P);

              // Determine the contact plane.
              const T offA = normal.dot(tA_3d.a());
              const T offB = normal.dot(tB_3d.a());
              T offset = 0.5 * (offA + offB);
              SPDLOG_DEBUG(drake::log(), "Contact plane: normal({}) offset: {}",
                           normal.transpose(), offset);

              // Intersect the two triangles and then unproject the vertices of 
              // the construction.
              SPDLOG_DEBUG(drake::log(), "Intersecting triangle: {}", tA_3d);
              SPDLOG_DEBUG(drake::log(), "and triangle: {}", tB_3d);
              int num_intersections = tA_2d.Intersect(tB_2d, &isects[0]);
              for (int i = 0; i < num_intersections; ++i) {
                points->push_back(Unproject(normal, P, offset, isects[i]));
                SPDLOG_DEBUG(drake::log(), "Face/face contact: point {}",
                             points->back().transpose());
              }
              if (num_intersections == 0) {
                SPDLOG_DEBUG(drake::log(),
                             "Face/face contact: no intersection");
              }

              return offA - offB;
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
