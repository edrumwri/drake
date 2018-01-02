#pragma once

#include "drake/common/sorted_pair.h"
#include "drake/math/project_3d_to_2d.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant_witness_function.h"
#include "drake/multibody/rigid_body_plant/tri_tri_contact_data.h"
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
      const systems::RigidBodyPlant<T>& rb_plant,
      const TriTriContactData<T>& contact_data) :
      RigidBodyPlantWitnessFunction<T>(rb_plant,
          systems::WitnessFunctionDirection::kNegativeThenNonNegative),
      contact_data_(contact_data) {
    this->set_name("TangentialSeparationWitness");
  }

  /// Gets the type of witness function.
  typename RigidBodyPlantWitnessFunction<T>::WitnessType
      get_witness_function_type() const override {
    return RigidBodyPlantWitnessFunction<T>::kTangentialSeparation;
 }

  TangentialSeparationWitnessFunction(
      const TangentialSeparationWitnessFunction<T>& e) :
    RigidBodyPlantWitnessFunction<T>(
        e.get_plant(),
        systems::WitnessFunctionDirection::kNegativeThenNonNegative) {
    operator=(e);
  }

  TangentialSeparationWitnessFunction& operator=(
      const TangentialSeparationWitnessFunction<T>& e) {
    this->set_name(e.get_name());
    contact_data_ = e.contact_data_;
    return *this;
  }

  bool operator==(const TangentialSeparationWitnessFunction<T>& w) const {
    return (contact_data_ == w.contact_data_ &&
            &this->get_plant() == &w.get_plant());
  }

  /// Gets the contact data that this witness function uses.
  const TriTriContactData<T>& get_contact_data() const { return contact_data_; }

 private:
  T DoEvaluate(const systems::Context<T>& context) const override {
    // Get the two rigid bodies.
    const auto& rbA = *contact_data_.idA->get_body();
    const auto& rbB = *contact_data_.idB->get_body();

    // Compute the transforms for the RigidBody objects.
    const auto& tree = this->get_plant().get_rigid_body_tree();
    const int nq = this->get_plant().get_num_positions();
    const int nv = this->get_plant().get_num_velocities();
    auto x = context.get_discrete_state(0).get_value();
    VectorX<T> q = x.topRows(nq);
    VectorX<T> v = x.bottomRows(nv);
    auto kinematics_cache = tree.doKinematics(q, v);
    auto wTA = tree.CalcBodyPoseInWorldFrame(kinematics_cache, rbA);
    auto wTB = tree.CalcBodyPoseInWorldFrame(kinematics_cache, rbB);

    return CalcSignedDistance(contact_data_, wTA, wTB);
  }

  T CalcSignedDistance(const TriTriContactData<T>& tri_tri_data,
                       const Isometry3<T>& wTA, const Isometry3<T>& wTB) const {
    // Get the contact normal.
    auto normal = tri_tri_data.GetSurfaceNormalExpressedInWorld(
        wTA, wTB);

    // Get the projection matrix.
    const auto P = math::Determine3dTo2dProjectionMatrix(normal);

    // Get the feature types and feature info.
    auto typeA = tri_tri_data.typeA;
    auto typeB = tri_tri_data.typeB;
    auto feature_A_id = tri_tri_data.feature_A_id;
    auto feature_B_id = tri_tri_data.feature_B_id;

    // Get the two untransformed triangles.
    const auto tA = tri_tri_data.tA;
    const auto tB = tri_tri_data.tB;

    switch (typeA) {
      case FeatureType::kVertex:  {
        DRAKE_DEMAND(typeB != FeatureType::kVertex &&
            typeB != FeatureType::kEdge);

        // Get the triangle and the vector representing the vertex.
        const Vector3<T> v = wTA * tA->get_vertex(
            reinterpret_cast<long>(feature_A_id));
        const Vector3<T> v1 = wTB * tB->a();
        const Vector3<T> v2 = wTB * tB->b();
        const Vector3<T> v3 = wTB * tB->c();
        Triangle3<T> t(&v1, &v2, &v3);

        // Project the vertex and the triangle to the contact plane.
        const Vector2<T> v_2d = P * v;
        const Triangle2<T> t_2d = t.ProjectTo2d(P);

        // Return the signed distance.
        const T signed_dist = t_2d.CalcSignedDistance(v_2d);
        SPDLOG_DEBUG(drake::log(), "Vertex/face signed distance: {}",
                     signed_dist);
        return signed_dist;
      }

      case FeatureType::kEdge:
        if (typeB == FeatureType::kEdge) {
          // Get the two edges.
          auto edgeA = tA->get_edge(reinterpret_cast<long>(feature_A_id));
          auto edgeB = tB->get_edge(reinterpret_cast<long>(feature_B_id));

          // Transform the edges into the world.
          edgeA.first = wTA * edgeA.first;
          edgeA.second = wTA * edgeA.second;
          edgeB.first = wTB * edgeB.first;
          edgeB.second = wTB * edgeB.second;

          // Project each edge to 2D.
          auto eA_2d = std::make_pair(P * edgeA.first, P * edgeA.second);
          auto eB_2d = std::make_pair(P * edgeB.first, P * edgeB.second);

          const T signed_dist =
              Triangle2<T>::CalcSignedDistance(eA_2d, eB_2d);
          SPDLOG_DEBUG(drake::log(), "Edge/edge signed distance: {}",
                       signed_dist);
          return signed_dist;
        } else {
          DRAKE_DEMAND(typeB != FeatureType::kVertex);

          // Project the edge and the face to the contact plane.
          auto edge = tA->get_edge(reinterpret_cast<long>(feature_A_id));
          edge.first = wTA * edge.first;
          edge.second = wTA * edge.second;
          auto e_2d = std::make_pair(P * edge.first, P * edge.second);
          const Vector3<T> v1 = wTB * tB->a();
          const Vector3<T> v2 = wTB * tB->b();
          const Vector3<T> v3 = wTB * tB->c();
          Triangle3<T> t(&v1, &v2, &v3);
          const Triangle2<T> tB_2d = t.ProjectTo2d(P);

          const T signed_dist = tB_2d.CalcSignedDistance(e_2d);
          SPDLOG_DEBUG(drake::log(), "Edge/face signed distance: {}",
                       signed_dist);
          return signed_dist;
        }

      case FeatureType::kFace:
        if (typeB == FeatureType::kVertex) {
          // Get the triangle and the vector representing the vertex.
          const Vector3<T> v1 = wTA * tA->a();
          const Vector3<T> v2 = wTA * tA->b();
          const Vector3<T> v3 = wTA * tA->c();
          Triangle3<T> t(&v1, &v2, &v3);
          const Vector3<T> v = wTB * tB->get_vertex(
              reinterpret_cast<long>(feature_B_id));

          // Project the vertex and the triangle to the contact plane.
          const Vector2<T> v_2d = P * v;
          const Triangle2<T> t_2d = t.ProjectTo2d(P);

          // Return the signed distance.
          const T signed_dist = t_2d.CalcSignedDistance(v_2d);
          SPDLOG_DEBUG(drake::log(), "Vertex/face signed distance: {}",
                       signed_dist);
          return signed_dist;
        } else {
          if (typeB == FeatureType::kEdge) {
            // Project the edge and the face to the contact plane.
            const Vector3<T> v1 = wTA * tA->a();
            const Vector3<T> v2 = wTA * tA->b();
            const Vector3<T> v3 = wTA * tA->c();
            const Triangle3<T> t(&v1, &v2, &v3);
            const Triangle2<T> tA_2d = t.ProjectTo2d(P);
            auto edge = tB->get_edge(reinterpret_cast<long>(feature_B_id));
            edge.first = wTB * edge.first;
            edge.second = wTB * edge.second;
            auto e_2d = std::make_pair(P * edge.first, P * edge.second);

            const T signed_dist = tA_2d.CalcSignedDistance(e_2d);
            SPDLOG_DEBUG(drake::log(), "Edge/face signed distance: {}",
                         signed_dist);
            return signed_dist;
          } else {
            // It is a face-face intersection. Project both triangles to 2D.
            const Vector3<T> vA1 = wTA * tA->a();
            const Vector3<T> vA2 = wTA * tA->b();
            const Vector3<T> vA3 = wTA * tA->c();
            const Vector3<T> vB1 = wTB * tB->a();
            const Vector3<T> vB2 = wTB * tB->b();
            const Vector3<T> vB3 = wTB * tB->c();
            const Triangle3<T> tA_3d(&vA1, &vA2, &vA3);
            const Triangle3<T> tB_3d(&vB1, &vB2, &vB3);
            const Triangle2<T> tA_2d = tA_3d.ProjectTo2d(P);
            const Triangle2<T> tB_2d = tB_3d.ProjectTo2d(P);

            // Compute the signed distance.
            const T signed_dist = tA_2d.CalcSignedDistance(tB_2d);
            SPDLOG_DEBUG(drake::log(), "Face/face signed distance: {}",
                         signed_dist);
            return signed_dist;
          }
        }

      default:
        DRAKE_ABORT();
    }

    // Should never get here.
    DRAKE_ABORT();
  }

  // The contact data being tracked.
  TriTriContactData<T> contact_data_;
};

}  // namespace multibody 
}  // namespace drake

