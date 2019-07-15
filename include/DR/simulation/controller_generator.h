#pragma once

#include <memory>
#include <tuple>
#include <utility>

#include <drake/geometry/geometry_visualization.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/plant/contact_results_to_lcm.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>

#include <DR/common/exception.h>
#include <DR/simulation/config.h>
#include <DR/simulation/model_generator.h>

namespace DR {

/**
 Defines static functions that connect the ports of a diagram together with
 controllers, planners, and other diagram elements according to the
 specification defined in a UnloadingTaskConfig struct.
 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
         Options for T are the following: (☑ = supported)
         ☑ double
         ☐ drake::AutoDiffXd
         ☐ drake::symbolic::Expression
 */
template <typename T>
class ControllerGenerator {
 public:
  typedef std::tuple<std::unique_ptr<drake::systems::Diagram<T>>, std::unique_ptr<drake::systems::Context<T>>>
      DiagramWithContext;

  /**
   Generates the control diagram for the world.
   @param config the UnloadingTaskConfig describing features of the simulation
   and each robot instance's controllers.
   @param mbp a const ref to the MultibodyPlant that contains the robot and
          world's simulated model.
   @param scene_graph a raw pointer to the SceneGraph connected to the
          MultibodyPlant.
   @param builder a raw pointer to the world sim and control diagram builder.
   @return a tuple containing unique pointers to the Diagram and its context.
   */
  DiagramWithContext CreateDiagramWithContext(const UnloadingTaskConfig& config,
                                              const drake::multibody::MultibodyPlant<T>& mbp,
                                              const drake::geometry::SceneGraph<T>& scene_graph,
                                              drake::systems::DiagramBuilder<T>* builder) {
    config.ValidateConfig();

    // Sanity check on the availability of the optional source id before using
    // it.
    DR_DEMAND(mbp.geometry_source_is_registered());

    ConnectToDrakeVisualizer(mbp, scene_graph, builder);

    return BuildDiagramWithContext(builder);
  }

 private:
  /**
  Connects the necessary output ports of the scenegraph and multibody plant to
  LCM to transmit visualization to the Drake Visualizer process.
  @param mbp a const ref to the MultibodyPlant that contains the robot and
         world's simulated model.
  @param scene_graph a raw pointer to the SceneGraph connected to the
         MultibodyPlant.
  @param builder a raw pointer to the world sim and control diagram builder.
  */
  void ConnectToDrakeVisualizer(const drake::multibody::MultibodyPlant<T>& mbp,
                                const drake::geometry::SceneGraph<T>& scene_graph,
                                drake::systems::DiagramBuilder<T>* builder) {}

  /**
   Performs the final build step for creating the world control diagram.
   @param builder a raw pointer to the world sim and control diagram builder.
   @return a tuple containing unique pointers to the Diagram and its context.
   */
  DiagramWithContext BuildDiagramWithContext(drake::systems::DiagramBuilder<T>* builder) {
    // then build diagram
    std::unique_ptr<drake::systems::Diagram<T>> diagram = builder->Build();
    // Create a context for this system:
    std::unique_ptr<drake::systems::Context<T>> diagram_context = diagram->CreateDefaultContext();
    diagram->SetDefaultContext(diagram_context.get());

    return DiagramWithContext(std::move(diagram), std::move(diagram_context));
  }
};

template <>
void ControllerGenerator<double>::ConnectToDrakeVisualizer(const drake::multibody::MultibodyPlant<double>& mbp,
                                                           const drake::geometry::SceneGraph<double>& scene_graph,
                                                           drake::systems::DiagramBuilder<double>* builder) {
  drake::multibody::ConnectContactResultsToDrakeVisualizer(builder, mbp);
  drake::geometry::ConnectDrakeVisualizer(builder, scene_graph);
}

}  // namespace DR
