#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/common/text_logging.h"
#include "drake/math/orthonormal_basis.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/rigid_body_plant/compliant_contact_model.h"
#include "drake/multibody/rigid_body_plant/compliant_material.h"
#include "drake/multibody/rigid_body_plant/tri_tri_contact_data.h"
#include "drake/solvers/mathematical_program.h"

using std::make_unique;
using std::move;
using std::runtime_error;
using std::string;
using std::unique_ptr;
using std::vector;

using drake::multibody::collision::Element;
using drake::multibody::collision::ElementId;
using drake::multibody::EuclideanDistanceWitnessFunction;
using drake::multibody::RigidBodyPlantWitnessFunction;
using drake::multibody::TangentialSeparationWitnessFunction;
using drake::multibody::Triangle3;
using drake::multibody::TriTriContactData;
using drake::multibody::Trimesh;
using drake::sorted_pair;

namespace drake {
namespace systems {
namespace {
// Constant used to indicate that a model instance doesn't have an
// input/output port associated with it.
const int kInvalidPortIdentifier = -1;
}  // namespace

template <typename T>
RigidBodyPlant<T>::RigidBodyPlant(std::unique_ptr<const RigidBodyTree<T>> tree,
                                  double timestep)
    : tree_(move(tree)), timestep_(timestep), compliant_contact_model_(
    std::make_unique<CompliantContactModel<T>>()) {
  DRAKE_DEMAND(tree_ != nullptr);
  state_output_port_index_ =
      this->DeclareVectorOutputPort(BasicVector<T>(get_num_states()),
                                    &RigidBodyPlant::CopyStateToOutput)
          .get_index();
  ExportModelInstanceCentricPorts();

  if (is_state_discrete()) {
    discrete_update_event_ = std::make_unique<systems::DiscreteUpdateEvent<T>>(
        systems::Event<T>::TriggerType::kUnknown);

    // Allocate temporary for storing discrete state in witness function
    // isolation.
    discrete_update_temporary_ = this->AllocateDiscreteVariables();

    // Build custom meshes.
    for (int i = 0; i < static_cast<int>(tree_->bodies.size()); ++i) {
      RigidBody<T>& body = *tree_->bodies[i];
      auto collision_element_iterator = body.collision_elements_begin();
      while (collision_element_iterator != body.collision_elements_end()) {
        // Get the geometry and make sure that it is a triangle mesh.
        const auto& geometry = (*collision_element_iterator)->getGeometry();
        if (!geometry.hasFaces()) {
          std::cerr << "Warning! Geometry does not have faces!" << std::endl;
          collision_element_iterator++;
          continue;
        }

        // Get the vertices.
        Eigen::Matrix3Xd points;
        geometry.getPoints(points);
        std::vector<Vector3<T>> vertices(points.cols());
        for (int i = 0; i < static_cast<int>(points.cols()); ++i)
          vertices[i] = points.col(i);

        // Get the face indices.
        DrakeShapes::TrianglesVector face_indices;
        geometry.getFaces(&face_indices);

        // Create a triangle mesh.
        meshes_[*collision_element_iterator] =
            multibody::Trimesh<T>(vertices, face_indices);

        // Point the collision detector to the mesh.
        collision_detection_.AddMesh(&meshes_[*collision_element_iterator]);

        // Advance the iterator.
        collision_element_iterator++;
      }
    }
  }

  // Declares an abstract valued output port for kinematics results.
  kinematics_output_port_index_ =
      this->DeclareAbstractOutputPort(
              KinematicsResults<T>(tree_.get()),
              &RigidBodyPlant::CalcKinematicsResultsOutput)
          .get_index();

  // Declares an abstract valued output port for contact information.
  contact_output_port_index_ = DeclareContactResultsOutputPort();

  // Schedule time stepping update.
  if (timestep > 0.0)
    this->DeclarePeriodicDiscreteUpdate(timestep);
}

template <class T>
OutputPortIndex RigidBodyPlant<T>::DeclareContactResultsOutputPort() {
  return this->DeclareAbstractOutputPort(
      ContactResults<T>(),
      &RigidBodyPlant::CalcContactResultsOutput).get_index();
}

template <class T>
Eigen::VectorBlock<const VectorX<T>> RigidBodyPlant<T>::get_state_vector(
    const Context<T>& context) const {
  if (is_state_discrete()) {
    return dynamic_cast<const BasicVector<T>&>(
        context.get_discrete_state_vector()).get_value();
  } else {
    return dynamic_cast<const BasicVector<T>&>(
        context.get_continuous_state_vector()).get_value();
  }
}

template <typename T>
void RigidBodyPlant<T>::ExportModelInstanceCentricPorts() {
  const int num_instances = tree_->get_num_model_instances();
  const std::pair<int, int> default_entry =
      std::pair<int, int>(kInvalidPortIdentifier, 0);
  input_map_.resize(num_instances, kInvalidPortIdentifier);
  actuator_map_.resize(num_instances, default_entry);
  output_map_.resize(num_instances, kInvalidPortIdentifier);
  position_map_.resize(num_instances, default_entry);
  velocity_map_.resize(num_instances, default_entry);

  if (num_instances != 0) {
    // Figure out which actuators belong to which model instance, and
    // create the appropriate maps and input ports.  We demand that
    // the tree be constructed such that all actuator, positions, and
    // velocity indices are contiguous for each model instance.
    for (int actuator_index = 0; actuator_index < tree_->get_num_actuators();
         ++actuator_index) {
      const RigidBody<double>* body = tree_->actuators[actuator_index].body_;
      const int instance_id = body->get_model_instance_id();
      if (actuator_map_[instance_id].first == kInvalidPortIdentifier) {
        actuator_map_[instance_id] = std::pair<int, int>(actuator_index, 1);
      } else {
        std::pair<int, int> map_entry = actuator_map_[instance_id];
        DRAKE_DEMAND(actuator_index == map_entry.first + map_entry.second);
        ++map_entry.second;
        actuator_map_[instance_id] = map_entry;
      }
    }

    for (int i = 0; i < num_instances; ++i) {
      if (get_num_actuators(i) == 0) {
        continue;
      }
      input_map_[i] =
          this->DeclareInputPort(kVectorValued, actuator_map_[i].second)
              .get_index();
    }

    // Now create the appropriate maps for the position and velocity
    // components.
    for (const auto& body : tree_->bodies) {
      if (!body->has_parent_body()) {
        continue;
      }
      const int instance_id = body->get_model_instance_id();
      const int position_start_index = body->get_position_start_index();
      const int num_positions = body->getJoint().get_num_positions();
      if (num_positions) {
        if (position_map_[instance_id].first == kInvalidPortIdentifier) {
          position_map_[instance_id] =
              std::pair<int, int>(position_start_index, num_positions);
        } else {
          std::pair<int, int> map_entry = position_map_[instance_id];
          DRAKE_DEMAND(position_start_index ==
                       map_entry.first + map_entry.second);
          map_entry.second += num_positions;
          position_map_[instance_id] = map_entry;
        }
      }

      const int velocity_start_index = body->get_velocity_start_index();
      const int num_velocities = body->getJoint().get_num_velocities();
      if (num_velocities) {
        if (velocity_map_[instance_id].first == kInvalidPortIdentifier) {
          velocity_map_[instance_id] =
              std::pair<int, int>(velocity_start_index, num_velocities);
        } else {
          std::pair<int, int> map_entry = velocity_map_[instance_id];
          DRAKE_DEMAND(velocity_start_index ==
                       map_entry.first + map_entry.second);
          map_entry.second += num_velocities;
          velocity_map_[instance_id] = map_entry;
        }
      }
    }

    for (int i = 0; i < num_instances; ++i) {
      if (get_num_states(i) == 0) {
        continue;
      }
      output_map_[i] =
          this->DeclareVectorOutputPort(BasicVector<T>(get_num_states(i)),
          [this, i](const Context<T>& context, BasicVector<T>* output) {
            this->CalcInstanceOutput(i, context, output);
          }).get_index();
    }
  }
}

template <typename T>
RigidBodyPlant<T>::~RigidBodyPlant() {}

template <typename T>
void RigidBodyPlant<T>::set_contact_model_parameters(
    const CompliantContactModelParameters& parameters) {
  compliant_contact_model_->set_model_parameters(parameters);
}

template <typename T>
void RigidBodyPlant<T>::set_default_compliant_material(
    const CompliantMaterial& material) {
  compliant_contact_model_->set_default_material(material);
}

template <typename T>
optional<bool> RigidBodyPlant<T>::DoHasDirectFeedthrough(int, int) const {
  return false;
}

template <typename T>
const RigidBodyTree<T>& RigidBodyPlant<T>::get_rigid_body_tree() const {
  return *tree_.get();
}

template <typename T>
int RigidBodyPlant<T>::get_num_bodies() const {
  return tree_->get_num_bodies();
}

template <typename T>
int RigidBodyPlant<T>::get_num_positions() const {
  return tree_->get_num_positions();
}

template <typename T>
int RigidBodyPlant<T>::get_num_positions(int model_instance_id) const {
  DRAKE_ASSERT(get_num_model_instances() > model_instance_id);
  return position_map_[model_instance_id].second;
}

template <typename T>
int RigidBodyPlant<T>::get_num_velocities() const {
  return tree_->get_num_velocities();
}

template <typename T>
int RigidBodyPlant<T>::get_num_velocities(int model_instance_id) const {
  DRAKE_ASSERT(get_num_model_instances() > model_instance_id);
  return velocity_map_[model_instance_id].second;
}

template <typename T>
int RigidBodyPlant<T>::get_num_states() const {
  return get_num_positions() + get_num_velocities();
}

template <typename T>
int RigidBodyPlant<T>::get_num_states(int model_instance_id) const {
  return get_num_positions(model_instance_id) +
         get_num_velocities(model_instance_id);
}

template <typename T>
int RigidBodyPlant<T>::get_num_actuators() const {
  return tree_->get_num_actuators();
}

template <typename T>
int RigidBodyPlant<T>::get_num_actuators(int model_instance_id) const {
  DRAKE_ASSERT(get_num_model_instances() > model_instance_id);
  return actuator_map_[model_instance_id].second;
}

template <typename T>
int RigidBodyPlant<T>::get_num_model_instances() const {
  return tree_->get_num_model_instances();
}

template <typename T>
int RigidBodyPlant<T>::get_input_size() const {
  return get_num_actuators();
}

template <typename T>
int RigidBodyPlant<T>::get_output_size() const {
  return get_num_states();
}

template <typename T>
void RigidBodyPlant<T>::set_position(Context<T>* context, int position_index,
                                     T position) const {
  DRAKE_ASSERT(context != nullptr);
  if (is_state_discrete()) {
    context->get_mutable_discrete_state(0).SetAtIndex(position_index,
                                                       position);
  } else {
    context->get_mutable_continuous_state()
        .get_mutable_generalized_position()
        .SetAtIndex(position_index, position);
  }
}

template <typename T>
void RigidBodyPlant<T>::set_velocity(Context<T>* context, int velocity_index,
                                     T velocity) const {
  DRAKE_ASSERT(context != nullptr);
  if (is_state_discrete()) {
    context->get_mutable_discrete_state(0).SetAtIndex(
        get_num_positions() + velocity_index, velocity);
  } else {
    context->get_mutable_continuous_state()
        .get_mutable_generalized_velocity()
        .SetAtIndex(velocity_index, velocity);
  }
}

template <typename T>
void RigidBodyPlant<T>::set_state_vector(
    Context<T>* context, const Eigen::Ref<const VectorX<T>> x) const {
  DRAKE_ASSERT(context != nullptr);
  set_state_vector(&context->get_mutable_state(), x);
}

template <typename T>
void RigidBodyPlant<T>::set_state_vector(
    State<T>* state, const Eigen::Ref<const VectorX<T>> x) const {
  DRAKE_ASSERT(state != nullptr);
  DRAKE_ASSERT(x.size() == get_num_states());
  if (is_state_discrete()) {
    auto& xd = state->get_mutable_discrete_state();
    xd.get_mutable_vector(0).SetFromVector(x);
  } else {
    state->get_mutable_continuous_state().SetFromVector(x);
  }
}

template <typename T>
const OutputPort<T>&
RigidBodyPlant<T>::model_instance_state_output_port(
    int model_instance_id) const {
  if (model_instance_id >= static_cast<int>(output_map_.size())) {
    throw std::runtime_error(
        "RigidBodyPlant::model_state_output_port(): "
        "ERROR: Model instance with ID " +
        std::to_string(model_instance_id) + " does not exist! Maximum ID is " +
        std::to_string(output_map_.size() - 1) + ".");
  }
  if (output_map_.at(model_instance_id) == kInvalidPortIdentifier) {
    throw std::runtime_error(
        "RigidBodyPlant::model_state_output_port(): "
        "ERROR: Model instance with ID " +
        std::to_string(model_instance_id) +
        " does not have any state output port!");
  }
  return System<T>::get_output_port(output_map_.at(model_instance_id));
}

template <typename T>
std::unique_ptr<ContinuousState<T>> RigidBodyPlant<T>::AllocateContinuousState()
    const {
  if (is_state_discrete()) {
    // Return an empty continuous state if the plant state is discrete.
    return std::make_unique<ContinuousState<T>>();
  }

  // TODO(amcastro-tri): add z state to track energy conservation.
  return make_unique<ContinuousState<T>>(
      make_unique<BasicVector<T>>(get_num_states()),
      get_num_positions() /* num_q */, get_num_velocities() /* num_v */,
      0 /* num_z */);
}

template <typename T>
std::unique_ptr<DiscreteValues<T>> RigidBodyPlant<T>::AllocateDiscreteState()
    const {
  if (!is_state_discrete()) {
    // State of the plant is continuous- return an empty discrete state.
    return std::make_unique<DiscreteValues<T>>();
  }
  return make_unique<DiscreteValues<T>>(
      make_unique<BasicVector<T>>(get_num_states()));
}

template <typename T>
bool RigidBodyPlant<T>::model_instance_has_actuators(
    int model_instance_id) const {
  DRAKE_ASSERT(static_cast<int>(input_map_.size()) ==
               get_num_model_instances());
  if (model_instance_id >= get_num_model_instances()) {
    throw std::runtime_error(
        "RigidBodyPlant::model_instance_has_actuators(): ERROR: provided "
        "model_instance_id of " +
        std::to_string(model_instance_id) +
        " does not exist. Maximum model_instance_id is " +
        std::to_string(get_num_model_instances()));
  }
  return input_map_.at(model_instance_id) != kInvalidPortIdentifier;
}

template <typename T>
const InputPortDescriptor<T>&
RigidBodyPlant<T>::model_instance_actuator_command_input_port(
    int model_instance_id) const {
  if (input_map_.at(model_instance_id) == kInvalidPortIdentifier) {
    throw std::runtime_error(
        "RigidBodyPlant::"
        "model_instance_actuator_command_input_port(): ERROR model instance "
        "with ID " +
        std::to_string(model_instance_id) +
        " does not have "
        "an actuator command input ports because it does not have any "
        "actuators.");
  }
  return System<T>::get_input_port(input_map_.at(model_instance_id));
}

// Updates the state output port.
template <typename T>
void RigidBodyPlant<T>::CopyStateToOutput(const Context<T>& context,
                       BasicVector<T>* state_output_vector) const {
  // TODO(amcastro-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  const VectorX<T> state_vector =
      (is_state_discrete()) ? context.get_discrete_state(0).CopyToVector()
                            : context.get_continuous_state().CopyToVector();

  state_output_vector->get_mutable_value() = state_vector;
}

// Updates one model-instance-centric state output port.
template <typename T>
void RigidBodyPlant<T>::CalcInstanceOutput(
    int instance_id, const Context<T>& context,
    BasicVector<T>* instance_output) const {
  // TODO(sherm1) Should reference state rather than copy it here.
  const VectorX<T> state_vector =
      (is_state_discrete()) ? context.get_discrete_state(0).CopyToVector()
                            : context.get_continuous_state().CopyToVector();

  auto values = instance_output->get_mutable_value();
  const auto& instance_positions = position_map_[instance_id];
  values.head(instance_positions.second) =
      state_vector.segment(instance_positions.first, instance_positions.second);

  const auto& instance_velocities = velocity_map_[instance_id];
  values.tail(instance_velocities.second) =
      state_vector.segment(instance_velocities.first + get_num_positions(),
                           instance_velocities.second);
}

// Updates the kinematics results output port.
template <typename T>
void RigidBodyPlant<T>::CalcKinematicsResultsOutput(
    const Context<T>& context, KinematicsResults<T>* kinematics_results) const {
  kinematics_results->UpdateFromContext(context);
}

// Computes the stiffness, damping, and friction coefficient for a contact.
template <typename T>
void RigidBodyPlant<T>::CalcContactStiffnessDampingMuAndNumHalfConeEdges(
      const drake::multibody::collision::PointPair& contact,
      double* stiffness,
      double* damping,
      double* mu,
      int* num_half_cone_edges) const {
  DRAKE_DEMAND(stiffness);
  DRAKE_DEMAND(damping);
  DRAKE_DEMAND(mu);
  DRAKE_DEMAND(num_half_cone_edges);

  // Get the compliant material parameters.
  CompliantMaterial material;
  compliant_contact_model_->CalcContactParameters(
      *contact.elementA, *contact.elementB, &material);

  // Get the stiffness. Young's modulus is force per area, while stiffness is
  // force per length. That means that we must multiply by a characteristic
  // radius. See @ref hunt_crossley (in contact_model_doxygen.h) for a lengthy
  // discussion on converting Young's Modulus to a stiffness.
  // The "length" will be incorporated using the contact depth.
  // TODO(edrumwri): Make characteristic radius user settable.
  const double characteristic_radius = 1e-2;  // 1 cm.
  *stiffness = material.youngs_modulus() * characteristic_radius;

  // Get the damping value (b) from the compliant model dissipation (α).
  // Equation (16) from [Hunt 1975] yields b = 3/2 * α * k * x. We can assume
  // that the stiffness and dissipation are predicated upon small deformation
  // (x). Put another way, we determine the damping coefficient for a harmonic
  // oscillator from linearizing the dissipation factor about the characteristic
  // deformation; the system will behave like a harmonic oscillator oscillating
  // about x = characteristic_deformation (in meters).
  // TODO(edrumwri): Make characteristic deformation user settable.
  const double characteristic_deformation = 1e-4;  // 1 mm
  *damping = material.dissipation() * 1.5 * (*stiffness) *
      characteristic_deformation;

  // Get the coefficient of friction.
  *mu = material.static_friction();

  // TODO(edrumwri): The number of half-cone edges should be able to be set on
  // a per-geometry pair basis. For now, just set the value to pyramidal
  // friction.
  *num_half_cone_edges = 2;

  // Verify the friction directions are set correctly.
  DRAKE_DEMAND(*num_half_cone_edges >= 2);
}

// Gets A's translational velocity relative to B's translational velocity at a
// point common to the two rigid bodies.
// @param p_W The point of contact (defined in the world frame).
// @returns the relative velocity at p_W expressed in the world frame.
template <class T>
Vector3<T> RigidBodyPlant<T>::CalcRelTranslationalVelocity(
    const KinematicsCache<T>& kinematics_cache, int body_a_index,
    int body_b_index, const Vector3<T>& p_W) const {
  const auto& tree = this->get_rigid_body_tree();

  // TODO(edrumwri): Convert this method to avoid Jacobian computation using
  // RigidBodyTree::CalcBodySpatialVelocityInWorldFrame().

  // The contact point in A's frame.
  const auto X_AW = kinematics_cache.get_element(body_a_index)
      .transform_to_world.inverse(Eigen::Isometry);
  const Vector3<T> p_A = X_AW * p_W;

  // The contact point in B's frame.
  const auto X_BW = kinematics_cache.get_element(body_b_index)
      .transform_to_world.inverse(Eigen::Isometry);
  const Vector3<T> p_B = X_BW * p_W;

  // Get the Jacobian matrices.
  const auto JA =
      tree.transformPointsJacobian(kinematics_cache, p_A, body_a_index, 0,
      false);
  const auto JB =
      tree.transformPointsJacobian(kinematics_cache, p_B, body_b_index, 0,
      false);

  // Compute the relative velocity in the world frame.
  return (JA - JB) * kinematics_cache.getV();
}

// Updates a generalized force from a force of f (expressed in the world frame)
// applied at point p_W (defined in the global frame).
template <class T>
void RigidBodyPlant<T>::UpdateGeneralizedForce(
    const KinematicsCache<T>& kinematics_cache, int body_a_index,
    int body_b_index, const Vector3<T>& p_W, const Vector3<T>& f,
    VectorX<T>* gf) const {
  const auto& tree = this->get_rigid_body_tree();

  // TODO(edrumwri): Convert this method to avoid Jacobian computation using
  // RigidBodyTree::dynamicsBiasTerm().

  // The contact point in A's frame.
  const auto X_AW = kinematics_cache.get_element(body_a_index)
      .transform_to_world.inverse(Eigen::Isometry);
  const Vector3<T> p_A = X_AW * p_W;

  // The contact point in B's frame.
  const auto X_BW = kinematics_cache.get_element(body_b_index)
      .transform_to_world.inverse(Eigen::Isometry);
  const Vector3<T> p_B = X_BW * p_W;

  // Get the Jacobian matrices.
  const auto JA =
      tree.transformPointsJacobian(kinematics_cache, p_A, body_a_index, 0,
      false);
  const auto JB =
      tree.transformPointsJacobian(kinematics_cache, p_B, body_b_index, 0,
      false);

  // Compute the Jacobian transpose times the force, and use it to update gf.
  (*gf) += (JA - JB).transpose() * f;
}

// Evaluates the relative velocities between two bodies projected along the
// contact normals.
template <class T>
VectorX<T> RigidBodyPlant<T>::ContactNormalJacobianMult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const VectorX<T>& q, const VectorX<T>& v) const {
  const auto& tree = this->get_rigid_body_tree();
  auto kinematics_cache = tree.doKinematics(q, v);

  // Create a result vector.
  VectorX<T> result(contacts.size());

  // Loop through all contacts.
  for (int i = 0; static_cast<size_t>(i) < contacts.size(); ++i) {
    // Get the two body indices.
    const int body_a_index = contacts[i].elementA->get_body()->get_body_index();
    const int body_b_index = contacts[i].elementB->get_body()->get_body_index();

    // The reported point on A's surface (As) in the world frame (W).
    const Vector3<T> p_WAs =
        kinematics_cache.get_element(body_a_index).transform_to_world *
        contacts[i].ptA;

    // The reported point on B's surface (Bs) in the world frame (W).
    const Vector3<T> p_WBs =
        kinematics_cache.get_element(body_b_index).transform_to_world *
        contacts[i].ptB;

    // Get the point of contact in the world frame.
    const Vector3<T> p_W = (p_WAs + p_WBs) * 0.5;

    // The *relative* velocity of the contact point in A relative to that in
    // B.
    const auto v_W = CalcRelTranslationalVelocity(kinematics_cache,
       body_a_index, body_b_index, p_W);

    // Get the projected normal velocity.
    result[i] = v_W.dot(contacts[i].normal);
  }

  return result;
}

// Applies forces along the contact normals at the contact points and gets the
// effect out on the generalized forces.
template <class T>
VectorX<T> RigidBodyPlant<T>::TransposedContactNormalJacobianMult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const KinematicsCache<T>& kinematics_cache,
    const VectorX<T>& f) const {
  // Create a result vector.
  VectorX<T> result = VectorX<T>::Zero(kinematics_cache.getV().size());

  // Loop through all contacts.
  for (int i = 0; static_cast<size_t>(i) < contacts.size(); ++i) {
    // Get the two body indices.
    const int body_a_index = contacts[i].elementA->get_body()->get_body_index();
    const int body_b_index = contacts[i].elementB->get_body()->get_body_index();

    // The reported point on A's surface (As) in the world frame (W).
    const Vector3<T> p_WAs =
        kinematics_cache.get_element(body_a_index).transform_to_world *
        contacts[i].ptA;

    // The reported point on B's surface (Bs) in the world frame (W).
    const Vector3<T> p_WBs =
        kinematics_cache.get_element(body_b_index).transform_to_world *
        contacts[i].ptB;

    // Get the point of contact in the world frame.
    const Vector3<T> p_W = (p_WAs + p_WBs) * 0.5;

    // Get the contribution to the generalized force from a force of the
    // specified normal applied at this point.
    UpdateGeneralizedForce(kinematics_cache, body_a_index, body_b_index, p_W,
                           contacts[i].normal * f[i], &result);
  }

  return result;
}

// Evaluates the relative velocities between two bodies projected along the
// contact tangent directions.
template <class T>
VectorX<T> RigidBodyPlant<T>::ContactTangentJacobianMult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const VectorX<T>& q, const VectorX<T>& v,
    const std::vector<int>& half_num_cone_edges) const {
  using std::cos;
  using std::sin;
  std::vector<Vector3<T>> basis_vecs;
  const auto& tree = this->get_rigid_body_tree();
  auto kinematics_cache = tree.doKinematics(q, v);

  // Get the total (half) number of edges in the friction cones of all contacts.
  const int total_edges = std::accumulate(
      half_num_cone_edges.begin(), half_num_cone_edges.end(), 0);

  // Create a result vector.
  VectorX<T> result(total_edges);

  // Loop through all contacts.
  for (int i = 0, result_index = 0; static_cast<size_t>(i) < contacts.size();
      ++i) {
    // Get the two body indices.
    const int body_a_index = contacts[i].elementA->get_body()->get_body_index();
    const int body_b_index = contacts[i].elementB->get_body()->get_body_index();

    // The reported point on A's surface (As) in the world frame (W).
    const Vector3<T> p_WAs =
        kinematics_cache.get_element(body_a_index).transform_to_world *
        contacts[i].ptA;

    // The reported point on B's surface (Bs) in the world frame (W).
    const Vector3<T> p_WBs =
        kinematics_cache.get_element(body_b_index).transform_to_world *
        contacts[i].ptB;

    // Get the point of contact in the world frame.
    const Vector3<T> p_W = (p_WAs + p_WBs) * 0.5;

    // The *relative* velocity of the contact point in A relative to that in
    // B.
    const auto v_W = CalcRelTranslationalVelocity(kinematics_cache,
        body_a_index, body_b_index, p_W);

    // Compute an orthonormal basis.
    const int kXAxisIndex = 0, kYAxisIndex = 1, kZAxisIndex = 2;
    auto R_WC = math::ComputeBasisFromAxis(kXAxisIndex, contacts[i].normal);
    const Vector3<T> tan1_dir = R_WC.col(kYAxisIndex);
    const Vector3<T> tan2_dir = R_WC.col(kZAxisIndex);

    // Set spanning tangent directions.
    basis_vecs.resize(half_num_cone_edges[i]);
    if (half_num_cone_edges[i] == 2) {
      // Special case: pyramid friction.
      basis_vecs.front() = tan1_dir;
      basis_vecs.back() = tan2_dir;
    } else {
      for (int j = 0; j < half_num_cone_edges[i]; ++j) {
        double theta = M_PI * j /
            (static_cast<double>(half_num_cone_edges[i]) - 1);
        basis_vecs[j] = tan1_dir * cos(theta) + tan2_dir * sin(theta);
      }
    }

    // Loop over the spanning tangent directions.
    for (int j = 0; j < static_cast<int>(basis_vecs.size()); ++j) {
      // Get the projected tangent velocity.
      result[result_index++] = v_W.dot(basis_vecs[j]);
    }
  }

  return result;
}

// Applies a force at the contact spanning directions at all contacts and gets
// the effect out on the generalized forces.
template <class T>
VectorX<T> RigidBodyPlant<T>::TransposedContactTangentJacobianMult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const KinematicsCache<T>& kinematics_cache,
    const VectorX<T>& f,
    const std::vector<int>& half_num_cone_edges) const {
  std::vector<Vector3<T>> basis_vecs;

  // Create a result vector.
  VectorX<T> result = VectorX<T>::Zero(kinematics_cache.getV().size());

  // Loop through all contacts.
  for (int i = 0, result_index = 0; static_cast<size_t>(i) < contacts.size();
      ++i) {
    // Get the two body indices.
    const int body_a_index = contacts[i].elementA->get_body()->get_body_index();
    const int body_b_index = contacts[i].elementB->get_body()->get_body_index();

    // The reported point on A's surface (As) in the world frame (W).
    const Vector3<T> p_WAs =
        kinematics_cache.get_element(body_a_index).transform_to_world *
        contacts[i].ptA;

    // The reported point on B's surface (Bs) in the world frame (W).
    const Vector3<T> p_WBs =
        kinematics_cache.get_element(body_b_index).transform_to_world *
        contacts[i].ptB;

    // Get the point of contact in the world frame.
    const Vector3<T> p_W = (p_WAs + p_WBs) * 0.5;

    // Compute an orthonormal basis.
    const int kXAxisIndex = 0, kYAxisIndex = 1, kZAxisIndex = 2;
    auto R_WC = math::ComputeBasisFromAxis(kXAxisIndex, contacts[i].normal);
    const Vector3<T> tan1_dir = R_WC.col(kYAxisIndex);
    const Vector3<T> tan2_dir = R_WC.col(kZAxisIndex);

    // Set spanning tangent directions.
    basis_vecs.resize(half_num_cone_edges[i]);
    if (half_num_cone_edges[i] == 2) {
      // Special case: pyramid friction.
      basis_vecs.front() = tan1_dir;
      basis_vecs.back() = tan2_dir;
    } else {
      for (int j = 0; j < half_num_cone_edges[i]; ++j) {
        double theta = M_PI * j /
            (static_cast<double>(half_num_cone_edges[i]) - 1);
        basis_vecs[j] = tan1_dir * cos(theta) + tan2_dir * sin(theta);
      }
    }

    // Get the contribution to the generalized force from a force of the
    // specified normal applied at this point.
    for (int j = 0; j < static_cast<int>(basis_vecs.size()); ++j) {
      UpdateGeneralizedForce(kinematics_cache, body_a_index, body_b_index, p_W,
                             basis_vecs[j] * f[result_index++], &result);
    }
  }

  return result;
}

/*
 * TODO(hongkai.dai): This only works for templates on double, it does not
 * work for autodiff yet, I will add the code to compute the gradient of vdot
 * w.r.t. q and v. See issue
 * https://github.com/RobotLocomotion/drake/issues/4267.
 */
template <typename T>
void RigidBodyPlant<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  static_assert(std::is_same<double, T>::value,
                "Only support templating on double for now");

  // No derivatives to compute if state is discrete.
  if (is_state_discrete()) return;

  VectorX<T> u = EvaluateActuatorInputs(context);

  // TODO(amcastro-tri): provide nicer accessor to an Eigen representation for
  // LeafSystems.
  auto x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();

  const int nq = get_num_positions();
  const int nv = get_num_velocities();
  const int num_actuators = get_num_actuators();
  // TODO(amcastro-tri): we would like to compile here with `auto` instead of
  // `VectorX<T>`. However it seems we get some sort of block from a block
  // which
  // is not instantiated in drakeRBM.
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  // TODO(amcastro-tri): place kinematics cache in the context so it can be
  // reused.
  auto kinsol = tree_->doKinematics(q, v);

  // TODO(amcastro-tri): preallocate the optimization problem and constraints,
  // and simply update them then solve on each function eval.
  // How to place something like this in the context?
  drake::solvers::MathematicalProgram prog;
  drake::solvers::VectorXDecisionVariable vdot =
      prog.NewContinuousVariables(nv, "vdot");

  auto H = tree_->massMatrix(kinsol);
  Eigen::MatrixXd H_and_neg_JT = H;

  // There are no external wrenches, but it is a required argument in
  // dynamicsBiasTerm.
  // TODO(amcastro-tri): external_wrenches should be made an optional
  // parameter
  // of dynamicsBiasTerm().
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;
  // right_hand_side is the right hand side of the system's equations:
  // H*vdot -J^T*f = right_hand_side.
  VectorX<T> right_hand_side =
      -tree_->dynamicsBiasTerm(kinsol, no_external_wrenches);
  if (num_actuators > 0) right_hand_side += tree_->B * u;

  // Applies joint limit forces.
  // TODO(amcastro-tri): Maybe move to
  // RBT::ComputeGeneralizedJointLimitForces(C)?
  {
    for (auto const& b : tree_->bodies) {
      if (!b->has_parent_body()) continue;
      auto const& joint = b->getJoint();
      // Joint limit forces are only implemented for single-axis joints.
      if (joint.get_num_positions() == 1 && joint.get_num_velocities() == 1) {
        const T limit_force =
            JointLimitForce(joint, q(b->get_position_start_index()),
                            v(b->get_velocity_start_index()));
        right_hand_side(b->get_velocity_start_index()) += limit_force;
      }
    }
  }

  right_hand_side += compliant_contact_model_->ComputeContactForce(
      *tree_.get(), kinsol);

  solvers::VectorXDecisionVariable position_force{};

  if (tree_->getNumPositionConstraints()) {
    size_t nc = tree_->getNumPositionConstraints();
    // 1/time constant of position constraint satisfaction.
    const T alpha = 5.0;

    position_force =
        prog.NewContinuousVariables(nc, "position constraint force");

    auto phi = tree_->positionConstraints(kinsol);
    auto J = tree_->positionConstraintsJacobian(kinsol, false);
    auto Jdotv = tree_->positionConstraintsJacDotTimesV(kinsol);

    // Critically damped stabilization term.
    // phiddot = -2 * alpha * phidot - alpha^2 * phi.
    prog.AddLinearEqualityConstraint(
        J, -(Jdotv + 2 * alpha * J * v + alpha * alpha * phi), vdot);
    H_and_neg_JT.conservativeResize(Eigen::NoChange,
                                    H_and_neg_JT.cols() + J.rows());
    H_and_neg_JT.rightCols(J.rows()) = -J.transpose();
  }

  // Adds [H,-J^T] * [vdot;f] = -C.
  prog.AddLinearEqualityConstraint(H_and_neg_JT, right_hand_side,
                                   {vdot, position_force});

  prog.Solve();

  VectorX<T> xdot(get_num_states());
  const auto& vdot_value = prog.GetSolution(vdot);
  xdot << tree_->transformVelocityToQDot(kinsol, v), vdot_value;
  derivatives->SetFromVector(xdot);
}

template <typename T>
void RigidBodyPlant<T>::DoCalcNextUpdateTime(
    const Context<T>& context,
    CompositeEventCollection<T>* events,
    T* time) const {
  // Get the witnesses active for the current context.
  std::vector<const WitnessFunction<T>*> witness_functions;
  this->GetWitnessFunctions(context, &witness_functions);

  // Clone the context.
  if (!context_clone_) {
    context_clone_ = context.Clone();
  } else {
    context_clone_->set_last_discrete_update_time(
        context.get_last_discrete_update_time());
    context_clone_->set_time(context.get_time());
    context_clone_->get_mutable_state().CopyFrom(context.get_state());
  }

  // Evaluate the witness functions.
  VectorX<T> w0(witness_functions.size());
  for (size_t i = 0; i < witness_functions.size(); ++i) {
    w0[i] = this->EvaluateWitness(*context_clone_, *witness_functions[i]);
    SPDLOG_DEBUG(drake::log(), "{} at t = {}: {}",
                 witness_functions[i]->get_name(), context.get_time(), w0[i]);
  }
/*
  // Verify that no Euclidean distance witnesses are initially non-positive.
  for (int i = 0; i < witness_functions.size(); ++i) {
    auto rb_witness = static_cast<const RigidBodyPlantWitnessFunction<T>*>(
        witness_functions[i]);
    DRAKE_DEMAND(rb_witness->get_witness_function_type() !=
        RigidBodyPlantWitnessFunction<T>::kEuclideanDistance || w0[i] > 0);
  }
*/
  // Attempt to do time stepping using the standard step size and the cloned
  // context.
  const T& t0 = context.get_time();
  auto x0 = get_state_vector(context);
  T t_des = t0 + (timestep_ - std::numeric_limits<double>::epsilon());
  StepForward(t0, x0, t_des, context_clone_.get());

  // Evaluate the witness functions again.
  VectorX<T> wf(witness_functions.size());
  for (size_t i = 0; i < witness_functions.size(); ++i) {
    wf[i] = this->EvaluateWitness(*context_clone_, *witness_functions[i]);
    SPDLOG_DEBUG(drake::log(), "{} at t = {}: {}",
                 witness_functions[i]->get_name(), t_des, wf[i]);
  }

  SPDLOG_DEBUG(drake::log(), "Witness evaluations at t={}: {}", t0,
               w0.transpose());
  SPDLOG_DEBUG(drake::log(), "Witness evaluations at t={}: {}", t_des,
               wf.transpose());

  // See whether any "witnesses" were triggered.
  std::vector<const WitnessFunction<T>*> triggered_witnesses;
  for (size_t i = 0; i < witness_functions.size(); ++i) {
    if (witness_functions[i]->should_trigger(w0[i], wf[i]))
      triggered_witnesses.push_back(witness_functions[i]);
  }

  // If no witnesses were triggered, return the standard discrete update time.
  if (triggered_witnesses.empty()) {
    LeafSystem<T>::DoCalcNextUpdateTime(context, events, time);
    SPDLOG_DEBUG(drake::log(), "CalcNextUpdateTime() returns {}", *time);
    return;
  }

  // One or more witnesses triggered: isolate the triggering time.
  *time = IsolateWitnessTriggers(context, witness_functions, w0, t0, x0, t_des,
    &triggered_witnesses);
  SPDLOG_DEBUG(drake::log(), "CalcNextUpdateTime() returns {}", *time);

  // If no witnesses were triggered, set up an event for a discrete update
  // event.
  if (triggered_witnesses.empty()) {
    discrete_update_event_->add_to_composite(events);
  } else {
    // Create an event for all triggered witnesses.
    for (const WitnessFunction <T>* fn : triggered_witnesses)
      this->AddTriggeredWitnessFunctionToCompositeEventCollection(*fn, events);
  }
}

// Gets all elements from a rigid body.
template <typename T>
std::vector<Element*> RigidBodyPlant<T>::GetElements() const {
  std::vector<Element*> elements;
  auto bodies = tree_->FindModelInstanceBodies(0);
  for (size_t i = 0; i < bodies.size(); ++i) {
    // Iterate over all elements for body i.
    for (auto elm_iter = bodies[i]->collision_elements_begin();
          elm_iter != bodies[i]->collision_elements_end(); ++elm_iter) {
      const auto& geometry = (*elm_iter)->getGeometry();
      if (geometry.hasFaces())
        elements.push_back(*elm_iter);
    }
  }

  // Add the world element.
  for (auto elm_iter = tree_->world().collision_elements_begin();
       elm_iter != tree_->world().collision_elements_end(); ++elm_iter) {
     elements.push_back(*elm_iter);
  }

  return elements;
}

// Gets the witness functions active for the plant.
template <typename T>
void RigidBodyPlant<T>::DoGetWitnessFunctions(const Context<T>& context,
  std::vector<const WitnessFunction<T>*>* witness_functions) const {
  // Do we need witness functions to track each pair of contacting triangles?
  // Option 1: one witness to track each pair of intersecting triangles.
  // Note: we need something like this anyway b/c we have to look for contact
  //   type changes (include separation).

  // TODO: Consider only pairs of bodies that pass a broad phase check first.

  // Get Euclidean witness functions.
  const auto& euclidean_distance_witnesses = context.get_abstract_state().
      get_value(kEuclideanDistanceWitnessVector).
      template GetValue<EuclideanDistanceWitnessArray>();

  // Get the elements with triangle meshes.
  const auto elms = GetElements();

  // Loop through all pairs of elements adding witness functions.
  for (size_t i = 0; i < elms.size(); ++i) {
    for (size_t j = i+1; j < elms.size(); ++j) {
      if (euclidean_distance_witnesses[i][j])
        witness_functions->push_back(euclidean_distance_witnesses[i][j].get());
    }
  }

  // Get tangential separation witnesses.
  const auto& tangential_separation_witnesses = context.get_abstract_state().
      get_value(kTangentialSeparationWitnessVector).
      template GetValue<TangentialSeparationWitnessArray>();

  // Add the witness functions.
  for (int i = 0; i < tangential_separation_witnesses.size(); ++i)
    witness_functions->push_back(&tangential_separation_witnesses[i]);
}

// Steps time stepping systems forward in time from t0.
template <typename T>
void RigidBodyPlant<T>::StepForward(
    const T& t0, const VectorX<T>& x0, const T& t_des,
    Context<T>* context_clone) const {
  DRAKE_DEMAND(is_state_discrete());
  context_clone->set_time(t_des);
  context_clone->set_last_discrete_update_time(t0);
  context_clone->get_mutable_discrete_state(0).SetFromVector(x0);
  discrete_update_temporary_->CopyFrom(
      context_clone->get_discrete_state());
  DoCalcDiscreteVariableUpdates(*context_clone, {},
      discrete_update_temporary_.get());
  context_clone->get_mutable_discrete_state().CopyFrom(
      *discrete_update_temporary_);
}

template <typename T>
T RigidBodyPlant<T>::IsolateWitnessTriggers(
    const Context<T>& context,
    const std::vector<const systems::WitnessFunction<T>*>& witnesses,
    const VectorX<T>& w0, const T& t0, const VectorX<T>& x0,
    const T& tf,
    std::vector<const WitnessFunction<T>*>* triggered_witnesses) const {
  // Verify that the vector of triggered witnesses is non-null.
  DRAKE_DEMAND(triggered_witnesses);

  // Clone the context.
  context_clone_->set_last_discrete_update_time(
      context.get_last_discrete_update_time());
  context_clone_->set_time(context.get_time());
  context_clone_->get_mutable_state().CopyFrom(context.get_state());

  // Get the witness isolation interval length.
  const optional<T> witness_iso_len = get_witness_time_isolation();

  // Check whether witness functions *are* to be isolated. If not, the witnesses
  // that were triggered on entry will be the set that is returned.
  if (!witness_iso_len)
    return tf;

  // Loop until the isolation window is sufficiently small.
  SPDLOG_DEBUG(drake::log(),
      "Isolating witness functions using isolation window of {} over [{}, {}]",
      witness_iso_len.value(), t0, tf);
  VectorX<T> wc(witnesses.size());
  T a = t0;
  T b = tf;
  do {
    // Compute the midpoint and evaluate the witness functions at it.
    T c = (a + b) / 2;
    SPDLOG_DEBUG(drake::log(), "Integrating forward to time {}", c);
    StepForward(t0, x0, c, context_clone_.get());

    // See whether any witness functions trigger.
    bool trigger = false;
    for (size_t i = 0; i < witnesses.size(); ++i) {
      wc[i] = this->EvaluateWitness(*context_clone_, *witnesses[i]);
      SPDLOG_DEBUG(drake::log(), "{} evaluations at t = {}, {}: {}, {}",
          witnesses[i]->get_name(), a, c, w0[i], wc[i]);
      if (witnesses[i]->should_trigger(w0[i], wc[i]))
        trigger = true;
    }

    // If no witness function triggered, we can continue stepping forward.
    if (!trigger) {
      SPDLOG_DEBUG(drake::log(), "No witness functions triggered up to {}", c);
      triggered_witnesses->clear();
      return c;
    } else {
      b = c;
    }
  } while (b - a > witness_iso_len.value());

  // Determine the set of triggered witnesses.
  triggered_witnesses->clear();
  for (size_t i = 0; i < witnesses.size(); ++i) {
    if (witnesses[i]->should_trigger(w0[i], wc[i]))
      triggered_witnesses->push_back(witnesses[i]);
  }

  return b;
}

// Determines closest (contacting) features after the signed distance witness
// function triggered.
template <class T>
void RigidBodyPlant<T>::DoCalcUnrestrictedUpdate(const Context<T>& context,
  const std::vector<const UnrestrictedUpdateEvent<T>*>& events,
  State<T>* state) const {
  SPDLOG_DEBUG(drake::log(), "RigidBodyPlant::DoCalcUnrestrictedUpdate()"
      " entered at time t={}", context.get_time());

  // Get the triangle/triangle feature data from the abstract state.
  auto& contacting_features = state->get_mutable_abstract_state().
      get_mutable_value(kContactFeatureMap).template GetMutableValue<std::map<sorted_pair<
      Element*>, std::vector<TriTriContactData<T>>>>();

  // Get the separating witness function vector from the abstract state.
  auto& tangential_separation_witness_vector = state->
      get_mutable_abstract_state().
      get_mutable_value(kTangentialSeparationWitnessVector).
      template GetMutableValue<
      std::vector<TangentialSeparationWitnessFunction<T>>>();

  // Build a kinematics cache.
  const auto& tree = this->get_rigid_body_tree();
  const int nq = this->get_num_positions();
  const int nv = this->get_num_velocities();
  auto x = context.get_discrete_state(0).get_value();
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  auto kinematics_cache = tree.doKinematics(q, v);

  // Indices for tangential separation witnesses to be removed.
  std::vector<int> tangential_separation_removal_indices;

  // Loop through all elements whose witness functions have triggered.
  std::map<Element*, Isometry3<T>> poses;
  std::vector<std::pair<Element*, Element*>> elements_to_check;
  for (size_t i = 0; i < events.size(); ++i) {
    // Get the witness function.
    DRAKE_DEMAND(events[i]->has_attribute());
    auto const_witness = events[i]->get_attribute()->template GetValue<const
        RigidBodyPlantWitnessFunction<T>*>();
    auto witness = const_cast<RigidBodyPlantWitnessFunction<T>*>(const_witness);

    switch (witness->get_witness_function_type()) {
      case RigidBodyPlantWitnessFunction<T>::kEuclideanDistance:  {
        // Get the two elements and rigid bodies.
        auto euclidean_distance_witness =
            static_cast<EuclideanDistanceWitnessFunction<T>*>(witness);
        Element* elmA = euclidean_distance_witness->get_element_A();
        Element* elmB = euclidean_distance_witness->get_element_B();
        elements_to_check.push_back(std::make_pair(elmA, elmB));
        const RigidBody<T>& rbA = *elmA->get_body();
        const RigidBody<T>& rbB = *elmB->get_body();

        // Compute the poses.
        auto wTA = tree.CalcBodyPoseInWorldFrame(kinematics_cache, rbA);
        auto wTB = tree.CalcBodyPoseInWorldFrame(kinematics_cache, rbB);
        const Trimesh<T>& mA = meshes_.find(elmA)->second;
        const Trimesh<T>& mB = meshes_.find(elmB)->second;

        // Determine the closest triangles from this witness function.
        EuclideanDistanceWitnessFunction<T>* euclidean_dist_witness =
            static_cast<EuclideanDistanceWitnessFunction<T>*>(witness);
        const std::vector<std::pair<int, int>>&
            closest_pairs = euclidean_dist_witness->get_last_closest_tris();

        // TODO: Only enable this when SPDLOG_DEBUG activated.
        for (const auto& closest_pair : closest_pairs) {
          const auto& tA = mA.triangle(closest_pair.first);
          const auto& tB = mB.triangle(closest_pair.second);
          SPDLOG_DEBUG(drake::log(), "Added triangle pair: {}, {}",
              closest_pair.first, closest_pair.second);
          SPDLOG_DEBUG(drake::log(), " First tri: {}, {}, {}",
              tA.a().transpose(), tA.b().transpose(), tA.c().transpose());
          SPDLOG_DEBUG(drake::log(), " Second tri: {}, {}, {}",
              tB.a().transpose(), tB.b().transpose(), tB.c().transpose());
          }

        // Compute the intersections.
        auto& contacting_features_vector = contacting_features[
            make_sorted_pair(elmA, elmB)];
        const int old_size = contacting_features_vector.size();
        collision_detection_.CalcIntersections(
            mA, mB, wTA, wTB, closest_pairs, &contacting_features_vector);

        // Update contacting features with elements.
        for (int j = 0; j < closest_pairs.size(); ++j) {
          // Get the appropriate triangle/triangle contact data.
          auto& tri_tri_data = contacting_features_vector[j + old_size];

          // Set the elements.
          tri_tri_data.idA = elmA;
          tri_tri_data.idB = elmB;

          // If the contact type is degenerate, keep looping.
          if (tri_tri_data.is_degenerate())
            continue;

          // Create a tangential separation witness for the non-degenerate pair.
          tangential_separation_witness_vector.push_back(
              TangentialSeparationWitnessFunction<T>(*this, elmA, elmB,
              closest_pairs[j].first, closest_pairs[j].second));

          SPDLOG_DEBUG(drake::log(), "Added tangential separation witness for "
              "pair {}, {}", closest_pairs[j].first, closest_pairs[j].second);
        }

        break;
      }

      case RigidBodyPlantWitnessFunction<T>::kTangentialSeparation:  {
        // Get the index of the tangential separation witness (for later
        // removal).
        auto tangential_witness_function =
            static_cast<TangentialSeparationWitnessFunction<T>*>(witness);
        bool added_one = false;
        for (int i = 0; i < tangential_separation_witness_vector.size(); ++i) {
          if (*tangential_witness_function ==
              tangential_separation_witness_vector[i]) {
            tangential_separation_removal_indices.push_back(i);
            added_one = true;
            break;
          }
        }
        DRAKE_DEMAND(added_one);

        // Get the contact data vector corresponding to the elements.
        auto elementA = tangential_witness_function->get_element_A();
        auto elementB = tangential_witness_function->get_element_B();
        auto tri_tri_vector_iter = contacting_features.find(make_sorted_pair(
            elementA, elementB));
        DRAKE_DEMAND(tri_tri_vector_iter != contacting_features.end());
        auto& tri_tri_data_vector = tri_tri_vector_iter->second;

        // Remove the corresponding triangles from the contact features vector.
        const Triangle3<T>& tA = tangential_witness_function->get_triangle_A();
        const Triangle3<T>& tB = tangential_witness_function->get_triangle_B();
        auto sorted_tris = make_sorted_pair(&tA, &tB);
        for (int i = 0; i < tri_tri_data_vector.size(); ++i) {
          auto candidate_pair = make_sorted_pair(
              tri_tri_data_vector[i].tA, tri_tri_data_vector[i].tB);
          if (candidate_pair == sorted_tris) {
            tri_tri_data_vector[i] = tri_tri_data_vector.back();
            tri_tri_data_vector.pop_back();
            break;
          }
        }

        break;
      }
    }
  }

  // Remove tangential separation witnesses.
  std::sort(tangential_separation_removal_indices.begin(),
            tangential_separation_removal_indices.end());
  for (auto i = tangential_separation_removal_indices.rbegin();
       i != tangential_separation_removal_indices.rend(); ++i) {
    SPDLOG_DEBUG(drake::log(), "Removed tangential separation witness for "
        "pair {}, {}",
        tangential_separation_witness_vector[*i].get_triangle_A_index(),
        tangential_separation_witness_vector[*i].get_triangle_B_index());
    tangential_separation_witness_vector[*i] =
        tangential_separation_witness_vector.back();
    tangential_separation_witness_vector.pop_back();
  }
}

// Gets points of contact using contacting features.
template <class T>
void RigidBodyPlant<T>::DetermineContacts(const Context<T>& context,
  std::vector<drake::multibody::collision::PointPair>* contacts) const {
  DRAKE_DEMAND(contacts);
  DRAKE_DEMAND(contacts->empty());

  // Get contact features from the context.
  auto& contacting_features = context.get_abstract_state().
      get_value(kContactFeatureMap).template GetValue<
      std::map<sorted_pair<Element*>, std::vector<TriTriContactData<T>>>>();

  // Build a kinematics cache.
  const auto& tree = this->get_rigid_body_tree();
  const int nq = this->get_num_positions();
  const int nv = this->get_num_velocities();
  auto x = context.get_discrete_state(0).get_value();
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  auto kinematics_cache = tree.doKinematics(q, v);

  // Loop over each pair of elements.
  std::vector<Vector3<T>> points;
  for (const auto& contacting_features_iter : contacting_features) {
    const std::vector<TriTriContactData<T>>& contact_data =
        contacting_features_iter.second;

    // Compute the point(s) of contact, normal, and signed distance for each
    // feature pair.
    for (int i = 0; i < static_cast<int>(contact_data.size()); ++i) {
      // Get poses for the meshes.
      const RigidBody<T>& rbA = *contact_data[i].idA->get_body();
      const RigidBody<T>& rbB = *contact_data[i].idB->get_body();
      auto wTA = tree.CalcBodyPoseInWorldFrame(kinematics_cache, rbA);
      auto wTB = tree.CalcBodyPoseInWorldFrame(kinematics_cache, rbB);

      // If the contact type is degenerate, keep looping.
      if (contact_data[i].is_degenerate())
        continue;

      // Determine the contact plane.
      auto normal = contact_data[i].GetSurfaceNormalExpressedInWorld(
          wTA, wTB);

      // Determine the contact points.
      points.clear();
      T signed_distance = contact_data[i].DetermineContactPoints(
          normal, wTA, wTB, &points);

      // Create the contact(s).
      for (int j = 0; j < static_cast<int>(points.size()); ++j) {
        contacts->push_back(multibody::collision::PointPair());
        contacts->back().elementA = contact_data[i].idA;
        contacts->back().elementB = contact_data[i].idB;
        contacts->back().ptA = wTA.inverse() * points[j];
        contacts->back().ptB = wTB.inverse() * points[j];
        contacts->back().normal = normal;
        contacts->back().distance = signed_distance;
      }
    }
  }
}

template <typename T>
void RigidBodyPlant<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>&,
    drake::systems::DiscreteValues<T>* updates) const {
  using std::abs;

  static_assert(std::is_same<double, T>::value,
                "Only support templating on double for now");

  // If plant state is continuous, no discrete state to update.
  if (!is_state_discrete()) return;

  // Get the time step.
  double dt = context.get_time() - context.get_last_discrete_update_time();

  // Check for zero dt.
  if (dt == 0.0)
    dt = timestep_;

  SPDLOG_DEBUG(drake::log(), "Variable update stepping forward by {}", dt);
  VectorX<T> u = this->EvaluateActuatorInputs(context);

  const int nq = this->get_num_positions();
  const int nv = this->get_num_velocities();
  const int num_actuators = this->get_num_actuators();

  // Initialize the velocity problem data.
  drake::multibody::constraint::ConstraintVelProblemData<T> data(nv);

  // Get the rigid body tree.
  const auto& tree = this->get_rigid_body_tree();

  // Get the system state.
  auto x = context.get_discrete_state(0).get_value();
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  auto kinematics_cache = tree.doKinematics(q, v);

  // Get the generalized inertia matrix and set up the inertia solve function.
  auto H = tree.massMatrix(kinematics_cache);

  // Compute the LDLT factorizations, which will be used by the solver.
  Eigen::LDLT<MatrixX<T>> ldlt(H);
  DRAKE_DEMAND(ldlt.info() == Eigen::Success);

  // Set the inertia matrix solver.
  data.solve_inertia = [&ldlt](const MatrixX<T>& m) {
    return ldlt.solve(m);
  };

  // There are no external wrenches, but it is a required argument in
  // dynamicsBiasTerm().
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;

  // right_hand_side is the right hand side of the system's equations:
  //   right_hand_side = B*u - C(q,v)
  VectorX<T> right_hand_side =
      -tree.dynamicsBiasTerm(kinematics_cache, no_external_wrenches);
  if (num_actuators > 0) right_hand_side += tree.B * u;

  // Get the set of contacts from the abstract state.
  std::vector<drake::multibody::collision::PointPair> contacts;
  DetermineContacts(context, &contacts);

  // Set the stabilization term for contact normal direction (kN). Also,
  // determine the friction coefficients and (half) the number of friction cone
  // edges.
  data.gammaN.resize(contacts.size());
  data.kN.resize(contacts.size());
  data.mu.resize(contacts.size());
  data.r.resize(contacts.size());
  for (int i = 0; i < static_cast<int>(contacts.size()); ++i) {
    double stiffness, damping, mu;
    int half_friction_cone_edges;
    CalcContactStiffnessDampingMuAndNumHalfConeEdges(
        contacts[i], &stiffness, &damping, &mu, &half_friction_cone_edges);
    data.mu[i] = mu;
    data.r[i] = half_friction_cone_edges;

    // Set cfm and erp parameters for contacts.
    const double denom = dt * stiffness + damping;
    const double cfm = 1.0 / denom;
    const double erp = (dt * stiffness) / denom;
    data.gammaN[i] = cfm;
    SPDLOG_DEBUG(drake::log(), "Contact distance: {}", contacts[i].distance);
    data.kN[i] = erp * contacts[i].distance / dt;
  }


  // Set the joint range of motion limits.
  std::vector<JointLimit> limits;
  for (auto const& b : tree.bodies) {
    if (!b->has_parent_body()) continue;
    auto const& joint = b->getJoint();

    // Joint limit forces are only implemented for single-axis joints.
    if (joint.get_num_positions() == 1 && joint.get_num_velocities() == 1) {
      const T qmin = joint.getJointLimitMin()(0);
      const T qmax = joint.getJointLimitMax()(0);
      DRAKE_DEMAND(qmin < qmax);

      // Get the current joint position and velocity.
      const T& qjoint = q(b->get_position_start_index());
      const T& vjoint = v(b->get_velocity_start_index());

      // See whether the joint is currently violated or the *current* joint
      // velocity might lead to a limit violation. The latter is a heuristic to
      // incorporate the joint limit into the time stepping calculations before
      // it is violated.
      if (qjoint < qmin || qjoint + vjoint * dt < qmin) {
        // Institute a lower limit.
        limits.push_back(JointLimit());
        limits.back().v_index = b->get_velocity_start_index();
        limits.back().signed_distance = (qjoint - qmin);
        SPDLOG_DEBUG(drake::log(), "body name: {} ", b->get_name());
        SPDLOG_DEBUG(drake::log(), "joint name: {} ", joint.get_name());
        SPDLOG_DEBUG(drake::log(), "joint signed distance: {} ",
            limits.back().signed_distance);
        limits.back().lower_limit = true;
      }
      if (qjoint > qmax || qjoint + vjoint * dt > qmax) {
        // Institute an upper limit.
        limits.push_back(JointLimit());
        limits.back().v_index = b->get_velocity_start_index();
        limits.back().signed_distance = (qmax - qjoint);
        SPDLOG_DEBUG(drake::log(), "body name: {} ", b->get_name());
        SPDLOG_DEBUG(drake::log(), "joint name: {} ", joint.get_name());
        SPDLOG_DEBUG(drake::log(), "joint signed distance: {} ",
            limits.back().signed_distance);
        limits.back().lower_limit = false;
      }
    }
  }

  // Set up the N multiplication operator (projected velocity along the contact
  // normals) and the N' multiplication operator (effect of contact normal
  // forces on generalized forces).
  data.N_mult = [this, &contacts, &q](const VectorX<T>& w) -> VectorX<T> {
    return ContactNormalJacobianMult(contacts, q, w);
  };
  data.N_transpose_mult = [this, &contacts, &kinematics_cache]
      (const VectorX<T>& f) -> VectorX<T> {
    return TransposedContactNormalJacobianMult(contacts, kinematics_cache, f);
  };

  // Set up the F multiplication operator (projected velocity along the contact
  // tangent directions) and the F' multiplication operator (effect of contact
  // frictional forces on generalized forces).
  data.F_mult = [this, &contacts, &q, &data](const VectorX<T>& w) ->
      VectorX<T> {
    return ContactTangentJacobianMult(contacts, q, w, data.r);
  };
  data.F_transpose_mult = [this, &contacts, &kinematics_cache, &data]
      (const VectorX<T>& f) -> VectorX<T> {
    return TransposedContactTangentJacobianMult(contacts,
        kinematics_cache, f, data.r);
  };

  // Set the range-of-motion (L) Jacobian multiplication operator and the
  // transpose_mult() operation.
  data.L_mult = [this, &limits](const VectorX<T>& w) -> VectorX<T> {
    VectorX<T> result(limits.size());
    for (int i = 0; static_cast<size_t>(i) < limits.size(); ++i) {
      const int index = limits[i].v_index;
      result[i] = (limits[i].lower_limit) ? w[index] : -w[index];
    }
    return result;
  };
  data.L_transpose_mult = [this, &v, &limits](const VectorX<T>& lambda) {
    VectorX<T> result = VectorX<T>::Zero(v.size());
    for (int i = 0; static_cast<size_t>(i) < limits.size(); ++i) {
      const int index = limits[i].v_index;
      result[index] = (limits[i].lower_limit) ? lambda[i] : -lambda[i];
    }
    return result;
  };

  // Output the Jacobians.
  #ifdef SPDLOG_DEBUG_ON
  MatrixX<T> N(contacts.size(), v.size()), L(limits.size(), v.size()),
      F(contacts.size() * 2, v.size());
  for (int i = 0; i < v.size(); ++i) {
    VectorX<T> unit = VectorX<T>::Unit(v.size(), i);
    N.col(i) = data.N_mult(unit);
    F.col(i) = data.F_mult(unit);
    L.col(i) = data.L_mult(unit);
  }
  SPDLOG_DEBUG(drake::log(), "N: {}", N);
  SPDLOG_DEBUG(drake::log(), "F: {}", F);
  SPDLOG_DEBUG(drake::log(), "L: {}", L);
  #endif

  // Set the regularization and stabilization terms for contact tangent
  // directions (kF).
  const int total_friction_cone_edges = std::accumulate(
      data.r.begin(), data.r.end(), 0);
  data.kF.setZero(total_friction_cone_edges);
  data.gammaF.setZero(total_friction_cone_edges);
  data.gammaE.setZero(contacts.size());

  // Set the regularization and stabilization terms for joint limit
  // constraints (kL).
  // TODO(edrumwri): Make cfm and erp individually settable.
  const double default_limit_cfm = 1e-8;
  const double default_limit_erp = 0.5;
  data.kL.resize(limits.size());
  for (int i = 0; i < static_cast<int>(limits.size()); ++i)
    data.kL[i] = default_limit_erp * limits[i].signed_distance / dt;
  data.gammaL.setOnes(limits.size()) *= default_limit_cfm;

  // Set Jacobians for bilateral constraint terms.
  // TODO(edrumwri): Make erp individually settable.
  const double default_bilateral_erp = 0.5;
  data.kG = default_bilateral_erp *
      tree.positionConstraints(kinematics_cache) / dt;
  const auto G = tree.positionConstraintsJacobian(kinematics_cache, false);
  data.G_mult = [this, &G](const VectorX<T>& w) -> VectorX<T> {
    return G * w;
  };
  data.G_transpose_mult = [this, &G](const VectorX<T>& lambda) {
    return G.transpose() * lambda;
  };

  // Integrate the forces into the momentum.
  data.Mv = H * v + right_hand_side * dt;

  // Solve the rigid impact problem.
  VectorX<T> new_velocity, contact_force;
  constraint_solver_.SolveImpactProblem(data, &contact_force);
  constraint_solver_.ComputeGeneralizedVelocityChange(data, contact_force,
      &new_velocity);
  SPDLOG_DEBUG(drake::log(), "Actuator forces: {} ", u.transpose());
  SPDLOG_DEBUG(drake::log(), "Transformed actuator forces: {} ",
      (tree.B * u).transpose());
  SPDLOG_DEBUG(drake::log(), "force: {}", right_hand_side.transpose());
  SPDLOG_DEBUG(drake::log(), "old velocity: {}", v.transpose());
  SPDLOG_DEBUG(drake::log(), "integrated forward velocity: {}",
      data.solve_inertia(data.Mv).transpose());
  SPDLOG_DEBUG(drake::log(), "change in velocity: {}",
      new_velocity.transpose());
  new_velocity += data.solve_inertia(data.Mv);
  SPDLOG_DEBUG(drake::log(), "new velocity: {}", new_velocity.transpose());
  SPDLOG_DEBUG(drake::log(), "new configuration: {}",
      (q + dt * tree.transformVelocityToQDot(kinematics_cache, new_velocity)).
      transpose());
  SPDLOG_DEBUG(drake::log(), "N * new velocity: {} ", data.N_mult(new_velocity).
      transpose());
  SPDLOG_DEBUG(drake::log(), "F * new velocity: {} ", data.F_mult(new_velocity).
      transpose());
  SPDLOG_DEBUG(drake::log(), "L * new velocity: {} ", data.L_mult(new_velocity).
      transpose());
  SPDLOG_DEBUG(drake::log(), "G * new velocity: {} ", data.G_mult(new_velocity).
      transpose());
  SPDLOG_DEBUG(drake::log(), "G * v: {} ", data.G_mult(v).transpose());
  SPDLOG_DEBUG(drake::log(), "g(): {}",
      tree.positionConstraints(kinematics_cache).transpose());

  // qn = q + dt*qdot.
  VectorX<T> xn(this->get_num_states());
  xn << q + dt * tree.transformVelocityToQDot(kinematics_cache, new_velocity),
      new_velocity;
  updates->get_mutable_vector(0).SetFromVector(xn);
}

template <typename T>
int RigidBodyPlant<T>::FindInstancePositionIndexFromWorldIndex(
    int model_instance_id, int world_position_index) {
  DRAKE_ASSERT(get_num_model_instances() > model_instance_id);
  const auto& instance_positions = position_map_[model_instance_id];
  if (world_position_index >=
      (instance_positions.first + instance_positions.second)) {
    throw runtime_error("Unable to find position index in model instance.");
  }
  return world_position_index - instance_positions.first;
}

/*
// Gets modified poses for each rigid body based on the active contact
// constraints.
template <typename T>
void RigidBodyPlant<T>::ComputePoses(
    const Context<T>& context,
    std::map<vector<RigidBody*, Isometry3<T>>>* poses) const {
  DRAKE_DEMAND(poses);
  DRAKE_DEMAND(!poses->empty());

  // TODO: Go through the context, computing the pose of each rigid body.

  // TODO: Use contact planes and distance between closest features to determine
  // the signed distance. 
} 
*/

template <typename T>
void RigidBodyPlant<T>::DoMapQDotToVelocity(
    const Context<T>& context, const Eigen::Ref<const VectorX<T>>& qdot,
    VectorBase<T>* generalized_velocity) const {
  // Discrete state does not use this method, since there are no continuous
  // qdot variables. Verify that, then return silently in this case.
  if (is_state_discrete()) {
    DRAKE_DEMAND(qdot.size() == 0);
    return;
  }

  // TODO(amcastro-tri): provide nicer accessor to an Eigen representation for
  // LeafSystems.
  auto x = get_state_vector(context);
  const int nq = get_num_positions();
  const int nv = get_num_velocities();
  const int nstates = get_num_states();

  DRAKE_ASSERT(qdot.size() == nq);
  DRAKE_ASSERT(generalized_velocity->size() == nv);
  DRAKE_ASSERT(x.size() == nstates);

  // TODO(amcastro-tri): we would like to compile here with `auto` instead of
  // `VectorX<T>`. However it seems we get some sort of block from a block
  // that is not instantiated in drakeRBM.
  VectorX<T> q = x.topRows(nq);

  // TODO(amcastro-tri): place kinematics cache in the context so it can be
  // reused.
  auto kinsol = tree_->doKinematics(q);

  // TODO(amcastro-tri): Remove .eval() below once RigidBodyTree is fully
  // templatized.
  generalized_velocity->SetFromVector(
      tree_->transformQDotToVelocity(kinsol, qdot));
}

template <typename T>
void RigidBodyPlant<T>::DoMapVelocityToQDot(
    const Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& generalized_velocity,
    VectorBase<T>* configuration_dot) const {
  // Discrete state does not use this method, since there are no continuous
  // generalized velocity variables. Verify that, then return silently in this
  // case.
  if (is_state_discrete()) {
    DRAKE_DEMAND(generalized_velocity.size() == 0);
    return;
  }

  auto x = get_state_vector(context);
  const int nq = get_num_positions();
  const int nv = get_num_velocities();
  const int nstates = get_num_states();

  DRAKE_ASSERT(configuration_dot->size() == nq);
  DRAKE_ASSERT(generalized_velocity.size() == nv);
  DRAKE_ASSERT(x.size() == nstates);

  // TODO(amcastro-tri): we would like to compile here with `auto` instead of
  // `VectorX<T>`. However it seems we get some sort of block from a block
  // that is not instantiated in drakeRBM.
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = generalized_velocity;

  // TODO(amcastro-tri): place kinematics cache in the context so it can be
  // reused.
  auto kinsol = tree_->doKinematics(q, v);

  configuration_dot->SetFromVector(tree_->transformVelocityToQDot(kinsol, v));
}

template <typename T>
T RigidBodyPlant<T>::JointLimitForce(const DrakeJoint& joint, const T& position,
                                     const T& velocity) {
  const T qmin = joint.getJointLimitMin()(0);
  const T qmax = joint.getJointLimitMax()(0);
  DRAKE_DEMAND(qmin < qmax);
  const T joint_stiffness = joint.get_joint_limit_stiffness()(0);
  DRAKE_DEMAND(joint_stiffness >= 0);
  const T joint_dissipation = joint.get_joint_limit_dissipation()(0);
  DRAKE_DEMAND(joint_dissipation >= 0);
  if (position > qmax) {
    const T violation = position - qmax;
    const T limit_force =
        (-joint_stiffness * violation * (1 + joint_dissipation * velocity));
    using std::min;  // Needed for ADL.
    return min(limit_force, 0.);
  } else if (position < qmin) {
    const T violation = position - qmin;
    const T limit_force =
        (-joint_stiffness * violation * (1 - joint_dissipation * velocity));
    using std::max;  // Needed for ADL.
    return max(limit_force, 0.);
  }
  return 0;
}

// Calculates the value of the contact results output port.
template <typename T>
void RigidBodyPlant<T>::CalcContactResultsOutput(
    const Context<T>& context, ContactResults<T>* contacts) const {
  DRAKE_ASSERT(contacts != nullptr);
  contacts->Clear();

  // This code should do nothing if the state is discrete because the compliant
  // contact model will not be used to compute contact forces.
  if (is_state_discrete())
    return;

  // TODO(SeanCurtis-TRI): This is horribly redundant code that only exists
  // because the data is not properly accessible in the cache.  This is
  // boilerplate drawn from EvalDerivatives.  See that code for further
  // comments
  auto x = get_state_vector(context);
  const int nq = get_num_positions();
  const int nv = get_num_velocities();
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  auto kinsol = tree_->doKinematics(q, v);

  compliant_contact_model_->ComputeContactForce(*tree_.get(), kinsol, contacts);
}

template <typename T>
VectorX<T> RigidBodyPlant<T>::EvaluateActuatorInputs(
    const Context<T>& context) const {
  VectorX<T> u;  // The plant-centric input vector of actuation values.
  u.resize(get_num_actuators());
  u.fill(0.);

  if (get_num_actuators() > 0) {
    for (int instance_id = 0; instance_id < get_num_model_instances();
         ++instance_id) {
      if (input_map_[instance_id] == kInvalidPortIdentifier) {
        continue;
      }
      if (this->EvalVectorInput(context, input_map_[instance_id]) == nullptr) {
        throw runtime_error(
            "RigidBodyPlant::EvaluateActuatorInputs(): ERROR: "
                "Actuator command input port for model instance " +
                std::to_string(instance_id) + " is not connected. All " +
                std::to_string(get_num_model_instances()) +
                " actuator command input ports must be connected.");
      }
    }

    for (int instance_id = 0; instance_id < get_num_model_instances();
         ++instance_id) {
      if (input_map_[instance_id] == kInvalidPortIdentifier) {
        continue;
      }
      const BasicVector<T> *instance_input =
          this->EvalVectorInput(context, input_map_[instance_id]);
      if (instance_input == nullptr) {
        continue;
      }

      const auto &instance_actuators = actuator_map_[instance_id];
      DRAKE_ASSERT(instance_actuators.first != kInvalidPortIdentifier);
      u.segment(instance_actuators.first, instance_actuators.second) =
          instance_input->get_value();
    }
  }
  return u;
}

/// Allocates the abstract state (containing contact data).
template <typename T>
std::unique_ptr<AbstractValues> RigidBodyPlant<T>::AllocateAbstractState()
    const {
  // Only allocate state if this is a time stepping system.
  if (is_state_discrete()) {
    // Do not set any bodies as being in contact by default.
    std::vector<std::unique_ptr<AbstractValue>> abstract_data;

    // NOTE: The ordering of abstract values here reflects the ordering of
    // AbstractStateIndices enumerations. If this ordering is changed, that
    // ordering must be changed.

    // Create a mapping of element pairs to contact data.
    abstract_data.push_back(std::make_unique<Value<
      std::map<sorted_pair<Element*>, std::vector<TriTriContactData<T>>>>>());

    // Create a vector of Euclidean distance witnesses.
    abstract_data.push_back(
        std::make_unique<Value<EuclideanDistanceWitnessArray>>());

    // Create the actual Euclidean distance witnesses.
    auto& euclidean_distance_witnesses = abstract_data.back()->
        GetMutableValue<EuclideanDistanceWitnessArray>();

    // Create a signed distance witness for each pair of elements.
    auto elm = GetElements();
    for (size_t i = 0; i < elm.size(); ++i) {
      euclidean_distance_witnesses.push_back(
          std::vector<std::shared_ptr<
              multibody::EuclideanDistanceWitnessFunction<T>>>(
              elm.size()));
      for (size_t j = i+1; j < elm.size(); ++j) {
        if (collision_filtered_.find(std::make_pair(elm[i], elm[j])) ==
            collision_filtered_.end()) {
          euclidean_distance_witnesses[i][j] =
              std::make_shared<multibody::EuclideanDistanceWitnessFunction<T>>(
                  *this, elm[i], elm[j]);
        }
      }
    }

    // Create a vector of tangential separation witnesses.
    abstract_data.push_back(
        std::make_unique<Value<TangentialSeparationWitnessArray>>());

    return std::make_unique<AbstractValues>(std::move(abstract_data));
  } else {
    return std::make_unique<AbstractValues>();
  }
}

// Explicitly instantiates on the most common scalar types.
template class RigidBodyPlant<double>;

}  // namespace systems
}  // namespace drake
