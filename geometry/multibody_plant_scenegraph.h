/** @file
 Provides functions to create a Diagram from a MultibodyPlant and a SceneGraph,
 allowing this diagram to be simulated (in time) and geometric queries to be
 posed, e.g., for planning applications.
*/

#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace geometry {

/// @cond
// Helper macro to throw an exception within methods that should not be called
// post-finalize.
#define DRAKE_MBPSG_THROW_IF_FINALIZED() ThrowIfFinalized(__func__)

// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_MBPSG_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)
/// @endcond


/** Extends a Diagram with the required components to interface with
 DrakeVisualizer. This must be called _during_ Diagram building and
 uses the given `builder` to add relevant subsystems and connections.

 This is a convenience method to simplify some common boilerplate for adding
 visualization capability to a Diagram. What it does is:
 - adds an initialization event that sends the required load message to set up
   the visualizer with the relevant geometry,
 - adds systems PoseBundleToDrawMessage and LcmPublisherSystem to
   the Diagram and connects the draw message output to the publisher input,
 - connects the `scene_graph` pose bundle output to the PoseBundleToDrawMessage
   system, and
 - sets the publishing rate to 1/60 of a second (simulated time).

 @note The initialization event occurs when Simulator::Initialize() is called
 (explicitly or implicitly at the start of a simulation). If you aren't going
 to be using a Simulator, use DispatchLoadMessage() to send the message
 yourself.

 @param builder      The diagram builder being used to construct the Diagram.
 @param scene_graph  The System in `builder` containing the geometry to be
                     visualized.
 @param lcm          An optional lcm interface through which lcm messages will
                     be dispatched. Will be allocated internally if none is
                     supplied.

 @pre This method has not been previously called while building the
      builder's current Diagram.
 @pre The given `scene_graph` must be contained within the supplied
      DiagramBuilder.

 @returns the LcmPublisherSystem (in case callers, e.g., need to change the
 default publishing rate).

 @see geometry::DispatchLoadMessage()
 @ingroup visualization
 */
template <class T>
class MultibodyPlantSceneGraph : public systems::Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlantSceneGraph)
  explicit MultibodyPlantSceneGraph(
      double time_step);

  const multibody::multibody_plant::MultibodyPlant<T>& multibody_plant() const {
    return *multibody_plant_;
  }

  multibody::multibody_plant::MultibodyPlant<T>& mutable_multibody_plant() {
    return *multibody_plant_;
  }

  const SceneGraph<T>& scene_graph() const { return *scene_graph_; }

  SceneGraph<T>& mutable_scene_graph() { return *scene_graph_; }

  /// Gets the exported port of the same name from MultibodyPlant.
  /// @see MultibodyPlant::get_actuation_input_port().
  /// @pre Finalize() was already called on `this` plant.
  const systems::InputPort<T>& get_actuation_input_port() const {
    DRAKE_MBPSG_THROW_IF_NOT_FINALIZED();
    return this->get_input_port(single_actuated_port_);
  }

  /// Gets the exported port of the same name from MultibodyPlant.
  /// @see MultibodyPlant::get_actuation_input_port(ModelInstanceIndex).
  /// @pre Finalize() was already called on `this` plant.
  const systems::InputPort<T>& get_actuation_input_port(
      multibody::ModelInstanceIndex model_instance) const {
    CheckModelInstanceIsValid(model_instance);
    return this->get_input_port(
        instance_continuous_state_output_ports_.at(model_instance));
  }

  /// Gets the exported port of the same name from MultibodyPlant.
  /// @see MultibodyPlant::get_continuous_state_output_port().
  /// @pre Finalize() was already called on `this` plant.
  const systems::OutputPort<T>& get_continuous_state_output_port() const {
    DRAKE_MBPSG_THROW_IF_NOT_FINALIZED();
    return this->get_output_port(continuous_state_output_port_);
  }

  /// Gets the exported port of the same name from MultibodyPlant.
  /// @see MultibodyPlant::get_continuous_state_output_port(ModelInstanceIndex).
  /// @pre Finalize() was already called on `this` plant.
  const systems::OutputPort<T>& get_continuous_state_output_port(
      multibody::ModelInstanceIndex model_instance) const {
    DRAKE_MBPSG_THROW_IF_NOT_FINALIZED();
    CheckModelInstanceIsValid(model_instance);
    return this->get_output_port(
        instance_continuous_state_output_ports_.at(model_instance));
  }

  /// Gets the exported port of the same name from MultibodyPlant.
  /// @see MultibodyPlant::get_generalized_contact_forces_output_port().
  /// @pre Finalize() was already called on `this` plant.
  const systems::OutputPort<T>& get_generalized_contact_forces_output_port(
      multibody::ModelInstanceIndex model_instance) const {
    DRAKE_MBPSG_THROW_IF_NOT_FINALIZED();
    CheckModelInstanceIsValid(model_instance);
    return this->get_output_port(
        instance_generalized_contact_forces_output_ports_.at(model_instance));
  }

  /// Gets the exported port of the same name from MultibodyPlant.
  /// @see MultibodyPlant::get_contact_results_output_port().
  /// @pre Finalize() was already called on `this` plant.
  const systems::OutputPort<T>& get_contact_results_output_port() const {
    DRAKE_MBPSG_THROW_IF_NOT_FINALIZED();
    return this->get_output_port(contact_results_port_);
  }

  /// Users *must* call Finalize() after making any additions to the
  /// multibody plant and before using this class in the Systems framework.
  /// This should be called exactly once.
  ///
  /// @see multibody::multibody_plant::MultibodyPlant<T>::Finalize()
  void Finalize();

  /// Determines whether this system has been finalized (via a call to
  /// Finalize()).
  bool is_finalized() const {
    return finalized_;
  }

 private:
  void CheckModelInstanceIsValid(
      multibody::ModelInstanceIndex model_instance) const {
    DRAKE_THROW_UNLESS(model_instance.is_valid());
    DRAKE_THROW_UNLESS(model_instance <
        multibody_plant_->num_model_instances());
    DRAKE_THROW_UNLESS(multibody_plant_->tree().num_states(model_instance) > 0);
  }

  // Helper method for throwing an exception within public methods that should
  // not be called post-finalize. The invoking method should pass its name so
  // that the error message can include that detail.
  void ThrowIfFinalized(const char* source_method) const;

  // Helper method for throwing an exception within public methods that should
  // not be called pre-finalize. The invoking method should pass its name so
  // that the error message can include that detail.
  void ThrowIfNotFinalized(const char* source_method) const;

  multibody::multibody_plant::MultibodyPlant<T>* multibody_plant_{nullptr};
  geometry::SceneGraph<T>* scene_graph_{nullptr};
  std::unique_ptr<systems::DiagramBuilder<T>> builder_;

  // Index for the output port of ContactResults.
  systems::OutputPortIndex contact_results_port_;

  // A vector containing the index for the generalized contact forces port for
  // each model instance. This vector is indexed by ModelInstanceIndex. An
  // invalid value indicates that the model instance has no generalized
  // velocities and thus no generalized forces.
  std::vector<systems::OutputPortIndex>
      instance_generalized_contact_forces_output_ports_;

  // A vector containing actuation ports for each model instance indexed by
  // ModelInstanceIndex.  An invalid value indicates that the model instance has
  // no actuated DOFs.
  std::vector<systems::InputPortIndex> instance_actuation_ports_;

  // If only one model instance has actuated DOFs, remember it here.  If
  // multiple instances have actuated DOFs, this index will not be valid.
  systems::InputPortIndex single_actuated_port_;

  // Port for output of all continuous state from MultibodyPlant.
  systems::OutputPortIndex continuous_state_output_port_;

  // Port for PoseBundle outputs from SceneGraph.
  systems::OutputPortIndex pose_bundle_output_port_;

  // Output port for queries from SceneGraph.
  systems::OutputPortIndex query_output_port_;

  // A vector containing state output ports for each model instance indexed by
  // ModelInstanceIndex. An invalid value indicates that the model instance has
  // no state.
  std::vector<systems::OutputPortIndex> instance_continuous_state_output_ports_;

  // Whether this system has been finalized.
  bool finalized_{false};
};

template <class T>
MultibodyPlantSceneGraph<T>::MultibodyPlantSceneGraph(double time_step) {
  builder_ = std::make_unique<systems::DiagramBuilder<T>>();

  scene_graph_ = builder_->template AddSystem<SceneGraph<T>>();
  scene_graph_->set_name("scene_graph");

  multibody_plant_ =
      builder_->template AddSystem<
          multibody::multibody_plant::MultibodyPlant<T>>(time_step);
}

template <class T>
void MultibodyPlantSceneGraph<T>::Finalize() {
  // MultibodyPlant must be finalized first.
  multibody_plant_->Finalize(scene_graph_);

  // Exports the pose bundle output and query output ports from Scene Graph.
  pose_bundle_output_port_ = builder_->ExportOutput(
      scene_graph_->get_pose_bundle_output_port());
  query_output_port_ = builder_->ExportOutput(
      scene_graph_->get_query_output_port());

  // Export the contact results port for MultibodyPlant.
  contact_results_port_ = builder_->ExportOutput(
      multibody_plant_->get_contact_results_output_port());

  // Exports the model-instance based ports for MultibodyPlant.
  continuous_state_output_port_ = builder_->ExportOutput(
      multibody_plant_->get_continuous_state_output_port());
  int num_actuated_instances = 0;
  for (multibody::ModelInstanceIndex i(0);
       i < multibody_plant_->num_model_instances(); ++i) {
    if (multibody_plant_->num_actuated_dofs(i) > 0) {
      instance_actuation_ports_.push_back(builder_->ExportInput(
          multibody_plant_->get_actuation_input_port(i)));
      ++num_actuated_instances;
    }
    instance_generalized_contact_forces_output_ports_.push_back(
        builder_->ExportOutput(
            multibody_plant_->get_generalized_contact_forces_output_port(i)));
    instance_continuous_state_output_ports_.push_back(builder_->
        ExportOutput(multibody_plant_->get_continuous_state_output_port(i)));
  }

  // Export the model instance index, if possible.
  if (num_actuated_instances > 0) {
    single_actuated_port_ = builder_->ExportInput(
        multibody_plant_->get_actuation_input_port());
  }

  // Create the necessary connections.
  builder_->Connect(multibody_plant_->get_geometry_poses_output_port(),
                    scene_graph_->get_source_pose_port(
                    multibody_plant_->get_source_id().value()));

  builder_->Connect(scene_graph_->get_query_output_port(),
                    multibody_plant_->get_geometry_query_input_port());
}

}  // namespace geometry
}  // namespace drake
