#pragma once

#include <DR/simulation/simulation_facade.h>

#include <memory>
#include <unordered_set>
#include <utility>

#include <DR/simulation/config.h>

namespace DR {

/**
 The `BodySimulationDriver` derives a class from `SimulationFacade` that supports generating a simulation of at least
 one instance of a dynamic single body configured from a `SingleBodyInstanceConfig` as well as any number of static
 single bodies.

  Usage example:
  ```
  std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> bodies;

  // ... populate `bodies` with at least one dynamic single body ...

  // Create an instance of `BodySimulationDriver` passing-in the configuration `bodies`.
  std::unique_ptr<BodySimulationDriver<T>> driver = std::make_unique<BodySimulationDriver<T>>(std::move(bodies));

  // Create a simulator instance of the collection of configured bodies.
  SimulatorInstanceIndex simulator_instance_index = driver->CreateSimulatedSystem());

  // Advance the simulation.
  const double simulation_time = 0.1;
  driver->simulator(simulator_instance_index).AdvanceTo(simulation_time);
  ```

 @tparam T a drake default scalar type.
 */
template <typename T>
class BodySimulationDriver final : public SimulationFacade<T> {
 public:
  BodySimulationDriver() {}
  virtual ~BodySimulationDriver() {}

  explicit BodySimulationDriver(std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> body_instance_configs)
      : body_instance_configs_(std::move(body_instance_configs)) {}

  virtual void DoPopulateModelAndSceneGraph() {
    for (const auto& body_config : body_instance_configs_) {
      AddBodyToMBP(*body_config.get(), &this->universal_plant());
    }
  }

  virtual void DoConstructAndConnectDiagram() {}

  // Create a context from the diagram and set state according to the configuration.
  virtual void DoSetState(drake::systems::Context<T>* context) {
    drake::systems::Context<T>& mbp_context =
        this->diagram().GetMutableSubsystemContext(this->universal_plant(), context);

    // StateSetter should address all states.  Abstract states are not currently addressed because there are none. This
    // check will trigger an error if abstract states are added but not addressed here.
    // TODO(samzapo): Set initial abstract states.
    DR_DEMAND(mbp_context.num_abstract_states() == 0);

    // NOTE: StateSetter does not address check num_discrete_state_groups.  Discrete states specifically left out
    // because MBP can run in two modes: continuous mode, where the equations of motions are integrated using an ODE
    // solver, and discrete mode, where the equations of motion are integrated using a time-stepping type scheme. In the
    // latter case, the kinematic state will be discrete variables.

    // Sets the position and velocity of the dynamic bodies in the world.
    for (const auto& body : body_instance_configs_) {
      // Set the pose and spatial velocity of this floating body in world frame.
      SetBodyPoseAndVelocity(this->universal_plant().GetBodyByName(body->name()), body->pose(),
                             body->spatial_velocity(), this->universal_plant(), &mbp_context);
    }
  }

 private:
  const std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> body_instance_configs_;
};
}  // namespace DR
