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

  /// Users *must* call Finalize() after making any additions to the
  /// multibody plant and before using this class in the Systems framework.
  /// This should be called exactly once.
  ///
  /// @see multibody::multibody_plant::MultibodyPlant<T>::Finalize()
  void Finalize();

 private:
  multibody::multibody_plant::MultibodyPlant<T>* multibody_plant_{nullptr};
  geometry::SceneGraph<T>* scene_graph_{nullptr};
  std::unique_ptr<systems::DiagramBuilder<T>> builder_;
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

  // Export all outputs from MultibodyPlant.
  for (int i = 0; i < multibody_plant_->get_num_output_ports(); ++i)
    builder_->ExportOutput(multibody_plant_->get_output_port(i));

  // Export all outputs from SceneGraph.
  for (int i = 0; i < scene_graph_->get_num_output_ports(); ++i)
    builder_->ExportOutput(scene_graph_->get_output_port(i));

  // Exports the pose bundle output and query output ports from Scene Graph.
  builder_->ExportOutput(scene_graph_->get_pose_bundle_output_port());
  builder_->ExportOutput(scene_graph_->get_query_output_port());

  // Exports the actuation input ports for multi-body plant.

  // Exports the continuous state output ports.

  // Exports the generalized contact forces output ports.

  // Exports the contact results output port.

  // Create the necessary connections.
  builder_->Connect(multibody_plant_->get_geometry_poses_output_port(),
                    scene_graph_->get_source_pose_port(
                    multibody_plant_->get_source_id().value()));

  builder_->Connect(scene_graph_->get_query_output_port(),
                    multibody_plant_->get_geometry_query_input_port());
}

}  // namespace geometry
}  // namespace drake
