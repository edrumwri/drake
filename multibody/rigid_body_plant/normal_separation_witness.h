#pragma once

#include "drake/common/sorted_pair.h"
#include "drake/math/orthonormal_basis.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant_witness_function.h"
#include "drake/multibody/rigid_body_plant/tri_tri_contact_data.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace multibody {

// A witness function for determining when a pair of closest features should
// no longer be tracked.
template <class T>
class NormalSeparationWitnessFunction :
    public RigidBodyPlantWitnessFunction<T> {
 public:
  NormalSeparationWitnessFunction(
      const systems::RigidBodyPlant <T>& rb_plant,
      const TriTriContactData<T>& contact_data) :
      RigidBodyPlantWitnessFunction<T>(
          rb_plant,
          systems::WitnessFunctionDirection::kNegativeThenNonNegative),
      contact_data_(contact_data) {
    this->set_name("NormalSeparationWitness");
  }

  /// Gets the type of witness function.
  typename RigidBodyPlantWitnessFunction<T>::WitnessType
      get_witness_function_type() const override {
    return RigidBodyPlantWitnessFunction<T>::kNormalSeparation;
 }

  NormalSeparationWitnessFunction(
      const NormalSeparationWitnessFunction<T>& e) :
    RigidBodyPlantWitnessFunction<T>(
        e.get_plant(),
        systems::WitnessFunctionDirection::kNegativeThenNonNegative) {
    operator=(e);
  }

  NormalSeparationWitnessFunction& operator=(
      const NormalSeparationWitnessFunction<T>& e) {
    this->set_name(e.get_name());
    contact_data_ = e.contact_data_;
    return *this;
  }

  /// Gets the contact data that this witness function uses.
  const TriTriContactData<T>& get_contact_data() const { return contact_data_; }

  bool operator==(const NormalSeparationWitnessFunction<T>& w) const {
    return (contact_data_ == w.contact_data_ &&
        &this->get_plant() == &w.get_plant());
  }

 private:
  T DoEvaluate(const systems::Context<T>& context) const override {
    // Get the vertices corresponding to the features in contact.
    const int max_vertices = 3;
    Vector3<T> verticesA[max_vertices], verticesB[max_vertices];
    const int num_A_vertices = contact_data_.GetVerticesFromA(&verticesA[0]);
    const int num_B_vertices = contact_data_.GetVerticesFromB(&verticesB[0]);

    // Get the two rigid body poses.
    const systems::RigidBodyPlant<T>& plant = this->get_plant();
    const auto& tree = plant.get_rigid_body_tree();
    const int nq = plant.get_num_positions();
    const int nv = plant.get_num_velocities();
    auto x = context.get_discrete_state(0).get_value();
    VectorX<T> q = x.topRows(nq);
    VectorX<T> v = x.bottomRows(nv);
    auto kinematics_cache = tree.doKinematics(q, v);
    const RigidBody<T>& rbA = *contact_data_.idA->get_body();
    const RigidBody<T>& rbB = *contact_data_.idB->get_body();
    auto wTA = tree.CalcBodyPoseInWorldFrame(kinematics_cache, rbA);
    auto wTB = tree.CalcBodyPoseInWorldFrame(kinematics_cache, rbB);

    // Determine the contact normal.
    const Vector3<T> normal = contact_data_.GetSurfaceNormalExpressedInWorld(
        wTA, wTB);

    // Compute the distances along the contact normal.
    T projA[max_vertices], projB[max_vertices];
    for (int i = 0; i < num_A_vertices; ++i)
      projA[i] = (wTA * verticesA[i]).dot(normal);
    for (int i = 0; i < num_B_vertices; ++i)
      projB[i] = (wTB * verticesB[i]).dot(normal);

    // The contact plane offset is determined using the closest points from
    // each. Since normal points toward A, we expect the ordering of the points
    // to be points from B then points from A. This would indicate that the
    // offset is taken using the maximum projection of B and the minimum
    // projection of A.
    std::sort(projA, projA + num_A_vertices);
    std::sort(projB, projB + num_B_vertices);
    return 0.5 * (projB[num_B_vertices-1] + projA[0]);
  }

  // The contact data used to evaluate the witness function.
  TriTriContactData<T> contact_data_;
};

}  // namespace multibody 
}  // namespace drake

