#pragma once
#include <memory>
#include <utility>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/implicit_euler_integrator.h>
#include <drake/systems/analysis/runge_kutta2_integrator.h>
#include <drake/systems/analysis/runge_kutta3_integrator.h>
#include <drake/systems/analysis/semi_explicit_euler_integrator.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>

#include <DR/simulation/config.h>

namespace DR {
/**
 Generates a drake Simulator for a drake Diagram.
 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
         Options for T are the following: (☑ = supported)
         ☑ double
         ☐ drake::AutoDiffXd
         ☐ drake::symbolic::Expression
 */
template <typename T>
class SimulationGenerator {
 public:
  /**
  Sets parameters for and then returns an initialized drake Simulator for the
  input diagram.
  @param config the SimulatorInstanceConfig describing parameters of the
         simulation.
  @param diagram a const reference to the Diagram of the world system.
  @param diagram_context a unique pointer to the Context of the world system
         diagram.
  @param mbp a raw pointer to the MultibodyPlant that represents the world
         model.
  @return a unique pointer to the world simulator that should be used to advance
          the world simulation.
  */
  std::unique_ptr<drake::systems::Simulator<double>> BuildSimulator(
      const SimulatorInstanceConfig& config,
      const drake::systems::Diagram<T>& diagram,
      std::unique_ptr<drake::systems::Context<T>> diagram_context,
      drake::multibody::MultibodyPlant<T>* mbp) {
    config.ValidateConfig();

    // Set how much penetration (in meters) we are willing to accept.
    mbp->set_penetration_allowance(config.target_accuracy());

    std::unique_ptr<drake::systems::Simulator<double>> simulator =
        std::make_unique<drake::systems::Simulator<double>>(
            diagram, std::move(diagram_context));

    // Setup the integration scheme for this simulator.
    SetupIntegrator(config, diagram, *mbp, simulator.get());

    simulator->set_target_realtime_rate(config.target_realtime_rate());
    simulator->Initialize();
    return std::move(simulator);
  }

 private:
  /**
   Sets parameters for the integration scheme of the input Simulator.
   @param config the SimulatorInstanceConfig describing parameters of the
          simulation.
   @param diagram a const reference to the Diagram of the world system.
   @param mbp a const reference to the MultibodyPlant that represents the world
          model.
   @param simulator a raw pointer to the world Simulator.
   */
  void SetupIntegrator(const SimulatorInstanceConfig& config,
                       const drake::systems::Diagram<T>& diagram,
                       const drake::multibody::MultibodyPlant<T>& mbp,
                       drake::systems::Simulator<double>* simulator) {
    drake::systems::IntegratorBase<double>* integrator{nullptr};

    // Hint the integrator's time step based on the contact time scale.
    // A fraction of this time scale is used which is chosen so that the fixed
    // time step integrators are stable.
    const double max_time_step = std::min(
        mbp.get_contact_penalty_method_time_scale() / 30, config.step_size());

    const SimulatorInstanceConfig::IntegrationScheme& integration_scheme =
        config.integration_scheme();
    if (integration_scheme ==
        SimulatorInstanceConfig::kSemiExplicitEulerIntegrationScheme) {
      integrator = simulator->reset_integrator<
          drake::systems::SemiExplicitEulerIntegrator<double>>(
          diagram, max_time_step, &simulator->get_mutable_context());
    } else if (integration_scheme ==
               SimulatorInstanceConfig::kRK2IntegrationScheme) {
      integrator =
          simulator
              ->reset_integrator<drake::systems::RungeKutta2Integrator<double>>(
                  diagram, max_time_step, &simulator->get_mutable_context());
    } else if (integration_scheme ==
               SimulatorInstanceConfig::kRK3IntegrationScheme) {
      integrator =
          simulator
              ->reset_integrator<drake::systems::RungeKutta3Integrator<double>>(
                  diagram, &simulator->get_mutable_context());
    } else if (integration_scheme ==
               SimulatorInstanceConfig::kImplicitEulerIntegrationScheme) {
      integrator = simulator->reset_integrator<
          drake::systems::ImplicitEulerIntegrator<double>>(
          diagram, &simulator->get_mutable_context());
    } else {
      throw std::runtime_error("Integration scheme not supported.");
    }

    integrator->set_maximum_step_size(max_time_step);

    // Error control is only supported for variable time step integrators.
    if (!integrator->get_fixed_step_mode())
      integrator->set_target_accuracy(config.target_accuracy());
  }
};
}  // namespace DR
