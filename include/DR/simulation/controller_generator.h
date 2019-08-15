#pragma once

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

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
#include <DR/tools/indexing.h>

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
  /**
   Generates the control diagram for the world.
   @param config the UnloadingTaskConfig describing features of the simulation
          and each robot instance's controllers.
   @param mbp a const ref to the universal plant.
   @param scene_graph a raw pointer to the SceneGraph connected to `mbp`.
   @param builder a raw pointer to the universal diagram builder.
   @return a tuple containing unique pointers to the built Diagram and its Context.
   */
  std::tuple<std::unique_ptr<drake::systems::Diagram<T>>, std::unique_ptr<drake::systems::Context<T>>>
  CreateDiagramWithContext(const UnloadingTaskConfig& config, const drake::multibody::MultibodyPlant<T>& mbp,
                           const drake::geometry::SceneGraph<T>& scene_graph,
                           drake::systems::DiagramBuilder<T>* builder) {
    config.ValidateConfig();
    for (const auto& robot_config : config.robot_instance_configs()) {
      ConstructAndConnectRobotControllers(robot_config, mbp, builder);
    }
    // Sanity check on the availability of geometry source before using it for visualization.
    DR_DEMAND(mbp.geometry_source_is_registered());
    ConnectToDrakeVisualizer(mbp, scene_graph, builder);
    return BuildDiagramWithContext(builder);
  }

 private:
  /*
   Creates PID error-feedback and inverse dynamics feed-forward controllers for a robot model then connects them.
   @param config the RobotInstanceConfig describing the robot controlled by the newly created inverse
          dynamics controller.
   @param mbp a const ref to the universal plant.
   @param builder a raw pointer to the universal diagram builder.
   @return drake::systems::controllers::InverseDynamicsController<T>* a raw pointer to the newly created inverse
           dynamics controller owned by builder.
   */
  drake::systems::controllers::InverseDynamicsController<T>* ConstructAndConnectRobotInverseDynamicsController(
      const RobotInstanceConfig& config, const drake::multibody::MultibodyPlant<T>& mbp,
      drake::systems::DiagramBuilder<T>* builder) {
    drake::multibody::ModelInstanceIndex model_instance = mbp.GetModelInstanceByName(config.name());

    RobotInstanceConfig local_robot_config = config;
    local_robot_config.set_name("idyn_controller_" + config.name());

    std::unique_ptr<typename drake::multibody::MultibodyPlant<T>> controller_model =
        ModelGenerator<T>().CreateStandaloneRobotPlant(local_robot_config);

    const auto& joints = config.joint_instance_configs();
    std::map<std::string, T> name_kp_map;
    for (const auto& joint : joints) {
      name_kp_map[joint.name()] = joint.kp();
    }

    drake::VectorX<T> kp = CreatePositionIndexAlignedVector(*controller_model.get(), name_kp_map);

    // TODO(samzapo): Select better ki and kd gains.
    // `kd` is a critical damping gain for unit generalized inertia.
    drake::VectorX<T> kd = 2.0 * kp.array().sqrt();
    drake::VectorX<T> ki = drake::VectorX<T>::Zero(kp.size());

    auto* controller = builder->template AddSystem<drake::systems::controllers::InverseDynamicsController>(
        *controller_model, kp, ki, kd, true);

    // Store controller model where anyone with the controller pointer can access it.
    controller_models_[static_cast<drake::systems::System<T>*>(controller)] = std::move(controller_model);

    // Wire-up plant to controller.
    builder->Connect(mbp.get_state_output_port(model_instance), controller->get_input_port_estimated_state());
    builder->Connect(controller->get_output_port_control(), mbp.get_actuation_input_port(model_instance));

    return controller;
  }

  /*
   Connects a constant zero input source to the actuation_input_port of a target `model_instance` in `mbp`
   @param model_instance the model instance index of the passive model in `mbp`
   @param mbp a const reference to the universal plant
   @param builder a raw pointer to the universal diagram builder.
   */
  void PassivateModelInstance(drake::multibody::ModelInstanceIndex model_instance,
                              const drake::multibody::MultibodyPlant<T>& mbp,
                              drake::systems::DiagramBuilder<T>* builder) {
    // Set all effort inputs into the robot actuators to zero.
    auto* constant_zero_effort_source = builder->template AddSystem<drake::systems::ConstantVectorSource>(
        drake::VectorX<T>::Constant(mbp.num_actuated_dofs(model_instance), 0.0));
    constant_zero_effort_source->set_name(mbp.GetModelInstanceName(model_instance) + "_constant_zero_effort_source");
    builder->Connect(constant_zero_effort_source->get_output_port(), mbp.get_actuation_input_port(model_instance));
  }

  /*
   Creates all controllers for a robot model then connects them.
   @param config the RobotInstanceConfig describing the robot controlled by the newly created controllers.
   @param mbp a const ref to the universal plant.
   @param builder a raw pointer to the universal diagram builder.
   */
  void ConstructAndConnectRobotControllers(const RobotInstanceConfig& config,
                                           const drake::multibody::MultibodyPlant<T>& mbp,
                                           drake::systems::DiagramBuilder<T>* builder) {
    if (config.control_scheme() == RobotInstanceConfig::kPassiveControlScheme) {
      PassivateModelInstance(mbp.GetModelInstanceByName(config.name()), mbp, builder);
    } else {
      // The robot is controlled, add InvDyn and PID controllers.
      auto* idyn_controller = ConstructAndConnectRobotInverseDynamicsController(config, mbp, builder);
      if (config.control_scheme() == RobotInstanceConfig::kStationaryControlScheme) {
        const auto* controller_model =
            controller_models_.find(static_cast<drake::systems::System<T>*>(idyn_controller))->second.get();

        // Controller holds position at initial position with zero velocity.
        const auto& joints = config.joint_instance_configs();
        std::map<std::string, T> name_q_map;
        for (const auto& joint : joints) {
          name_q_map[joint.name()] = joint.position();
        }
        drake::VectorX<T> init_q = CreatePositionIndexAlignedVector(*controller_model, name_q_map);

        // NOTE: the correctness of this code is predicated on the generalized positions being ordered before the
        // generalized velocities.
        drake::VectorX<T> init_q_v =
            drake::VectorX<T>::Constant(controller_model->num_positions() + controller_model->num_velocities(), 0.0);
        init_q_v.head(controller_model->num_positions()) = init_q;

        auto* constant_source = builder->template AddSystem<drake::systems::ConstantVectorSource>(init_q_v);
        constant_source->set_name(config.name() + "_desired_init_q_v");
        builder->Connect(constant_source->get_output_port(), idyn_controller->get_input_port_desired_state());

        // Controller maintains zero acceleration.
        auto* constant_zero_acceleration_source = builder->template AddSystem<drake::systems::ConstantVectorSource>(
            drake::VectorX<T>::Constant(controller_model->num_actuators(), 0.0));
        constant_zero_acceleration_source->set_name(config.name() + "_constant_zero_acceleration_source");
        builder->Connect(constant_zero_acceleration_source->get_output_port(),
                         idyn_controller->get_input_port_desired_acceleration());
      } else if (config.control_scheme() == RobotInstanceConfig::kPrimitiveControlScheme) {
        // TODO(samzapo): Implement pick and place control architecture.
        throw std::runtime_error("Control scheme kPrimitiveControlScheme is not currently supported.");
      } else {
        throw std::runtime_error("Unknown or unsupported robot control scheme.");
      }
    }
  }

  /*
  Connects the necessary output ports of the scenegraph and multibody plant to
  LCM to transmit visualization to the Drake Visualizer process.
  @param mbp a const ref to the MultibodyPlant that contains the robot and
         world's simulated model.
  @param scene_graph a raw pointer to the SceneGraph connected to the
         MultibodyPlant.
  @param builder a raw pointer to the universal diagram builder.
  */
  void ConnectToDrakeVisualizer(const drake::multibody::MultibodyPlant<T>& mbp,
                                const drake::geometry::SceneGraph<T>& scene_graph,
                                drake::systems::DiagramBuilder<T>* builder) {}

  /*
   Performs the final build step for creating the world control diagram.
   @param builder a raw pointer to the universal diagram builder.
   @return a tuple containing unique pointers to the Diagram and its context.
   */
  std::tuple<std::unique_ptr<drake::systems::Diagram<T>>, std::unique_ptr<drake::systems::Context<T>>>
  BuildDiagramWithContext(drake::systems::DiagramBuilder<T>* builder) {
    // then build diagram
    std::unique_ptr<drake::systems::Diagram<T>> diagram = builder->Build();
    // Create a context for this system:
    std::unique_ptr<drake::systems::Context<T>> diagram_context = diagram->CreateDefaultContext();
    diagram->SetDefaultContext(diagram_context.get());

    return std::tuple<std::unique_ptr<drake::systems::Diagram<T>>, std::unique_ptr<drake::systems::Context<T>>>(
        std::move(diagram), std::move(diagram_context));
  }

 private:
  // TODO(samzapo): Find a better place to store plants used by model-based controllers.  The model-based controllers
  //                should own these plants, we can't guarentee that ControllerGenerator will survive longer than the
  //                controllers that use these plants.
  std::map<drake::systems::System<T>*, std::unique_ptr<drake::multibody::MultibodyPlant<T>>> controller_models_;
};  // namespace DR

template <>
void ControllerGenerator<double>::ConnectToDrakeVisualizer(const drake::multibody::MultibodyPlant<double>& mbp,
                                                           const drake::geometry::SceneGraph<double>& scene_graph,
                                                           drake::systems::DiagramBuilder<double>* builder) {
  if (mbp.is_discrete()) {
    drake::multibody::ConnectContactResultsToDrakeVisualizer(builder, mbp);
  }
  drake::geometry::ConnectDrakeVisualizer(builder, scene_graph);
}

}  // namespace DR
