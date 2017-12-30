#pragma once

#include "drake/common/sorted_pair.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant_witness_function.h"
#include "drake/multibody/rigid_body_plant/trimesh.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace multibody {

// A witness function for two triangles separating tangentially.
template <class T>
class TangentialSeparationWitnessFunction :
    public RigidBodyPlantWitnessFunction<T> {
 public:
  TangentialSeparationWitnessFunction(
      const systems::RigidBodyPlant <T>& rb_plant,
      multibody::collision::Element* elementA,
      multibody::collision::Element* elementB,
      int triA,
      int triB);

  /// Gets the type of witness function.
  typename RigidBodyPlantWitnessFunction<T>::WitnessType
      get_witness_function_type() const override {
    return RigidBodyPlantWitnessFunction<T>::kTangentialSeparation;
 }

  TangentialSeparationWitnessFunction(
      const TangentialSeparationWitnessFunction<T>& e) :
    RigidBodyPlantWitnessFunction<T>(
        this->get_plant(),
        systems::WitnessFunctionDirection::kPositiveThenNonPositive) {
    operator=(e);
  }

  TangentialSeparationWitnessFunction& operator=(
      const TangentialSeparationWitnessFunction<T>& e) {
    elementA_ = e.elementA_;
    elementB_ = e.elementB_;
    meshA_ = e.meshA_;
    meshB_ = e.meshB_;
    triA_ = e.triA_;
    triB_ = e.triB_;
    return *this;
  }

  multibody::collision::Element* get_element_A() const { return elementA_; }
  multibody::collision::Element* get_element_B() const { return elementB_; }

 private:
  T DoEvaluate(const systems::Context<T>& context) const override {
    // Compute the poses for the rigid bodies.

    // Get the contact normal.

    // TODO: Compute tolerance in an informed manner.
    const T tol = std::numeric_limits<double>::epsilon();

    // Get the projection matrix.
    const auto P = math::Determine3dTo2dProjectionMatrix(normal);

    switch (typeA) {
      case FeatureType::kVertex:  {
        DRAKE_DEMAND(typeB != FeatureType::kVertex &&
            typeB != FeatureType::kEdge);

        // Get the triangle and the vector representing the vertex.
        const Vector3<T> v = poseA * tA->get_vertex(
            reinterpret_cast<long>(feature_A_id));
        const Vector3<T> v1 = poseB * tB->a();
        const Vector3<T> v2 = poseB * tB->b();
        const Vector3<T> v3 = poseB * tB->c();
        Triangle3<T> t(&v1, &v2, &v3);

        // Project the vertex and the triangle to the contact plane.
        const Vector2<T> v_2d = P * v;
        const Triangle2<T> t_2d = t.ProjectTo2d(P);

        // Return the signed distance.
        return t_2d.CalcSignedDistance(v_2d);
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

          // Project each edge to 2D.
          auto eA_2d = std::make_pair(P * edgeA.first, P * edgeA.second);
          auto eB_2d = std::make_pair(P * edgeB.first, P * edgeB.second);

          return Triangle2<T>::CalcSignedDistance(eA_2d, eB_2d);
        } else {
          DRAKE_DEMAND(typeB != FeatureType::kVertex);

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

          const T signed_dist = tB_2d.CalcSignedDistance(e_2d);
          SPDLOG_DEBUG(drake::log(), "Edge/face signed distance: {}",
                       signed_dist);
          break;
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

          // Project the vertex and the triangle to the contact plane.
          const Vector2<T> v_2d = P * v;
          const Triangle2<T> t_2d = t.ProjectTo2d(P);

          // Return the signed distance.
          return t_2d.CalcSignedDistance(v_2d);
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

            const T signed_dist = tB_2d.CalcSignedDistance(e_2d);
            SPDLOG_DEBUG(drake::log(), "Edge/face signed distance: {}",
                         signed_dist);
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

            // Compute the signed distance.
            const T signed_dist = tA_2d.CalcSignedDistance(tB_2d);
            SPDLOG_DEBUG(drake::log(), "Face/face signed distance: {}",
                         signed_dist);
          }
        }
    }
  }

  // Computes the signed distance between two edges.
  static T CalcSignedDistance(const std::pair<Vector2<T>, Vector2<T>>& e1,
                              const std::pair<Vector2<T>, Vector2<T>>& e2) {
    // If the two edges do not intersect, compute the distance between them.

    // Otherwise- edges intersect- use the separating axis theorem to determine
    // the minimum distance necessary for separation.
  }

  // The triangle index from element A.
  int triA_{-1};
  int triB_{-1};

  // The two triangle meshes.
  const multibody::Trimesh<T>* meshA_{nullptr};
  const multibody::Trimesh<T>* meshB_{nullptr};

  // The two elements for which the distance will be computed.
  multibody::collision::Element* elementA_{nullptr};
  multibody::collision::Element* elementB_{nullptr};
};

}  // namespace multibody 
}  // namespace drake

