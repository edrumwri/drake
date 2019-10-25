#pragma once

#include <algorithm>
#include <map>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include <drake/geometry/geometry_visualization.h>
#include <drake/multibody/plant/contact_results_to_lcm.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/implicit_euler_integrator.h>
#include <drake/systems/analysis/runge_kutta2_integrator.h>
#include <drake/systems/analysis/runge_kutta3_integrator.h>
#include <drake/systems/analysis/semi_explicit_euler_integrator.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>

#include <DR/simulation/controller_generator.h>
#include <DR/simulation/model_generator.h>
#include <DR/simulation/state_setter.h>
#include <DR/tools/optional.h>

namespace DR {

/// Type used to identify simulator instances by index within a SimulationFacade.
using SimulatorInstanceIndex = drake::TypeSafeIndex<class SimulationInstanceTag>;

struct SimulatorParameters {
  double target_accuracy;
};

const SimulatorParameters kDefaultSimulatorParameters{
    .target_accuracy = 0.001,  // 1 mm target accuracy in linear measurements.
};

/**
 The SimulationFacade is an abstract class that serves as a front-facing interface masking more complex underlying or
structural code from DR/simulation: ModelGenerator, ControllerGenerator, StateSetter, and SimulationGenerator.

Use derived versions of this Class to setup and run simulations.
 ```
   // Derive a class from SimulationFacade
   class SimulationFacadeDriver final : public SimulationFacade<double> {
    virtual void DoPopulateModelAndScenegraph() {...}
    virtual void DoConstructAndConnectDiagram() {...}
    virtual void DoSetState(.) {...}
   };

  // Instantiate derived class.
  SimulationFacadeDriver driver(...);

  // Create a new simulator for the modeled system.
  SimulatorInstanceIndex simulator_instance_index = driver.CreateSimulatedSystem();

  // .. advance simulator until it reaches a desired time (seconds).
  double desired_time = 1.0;
  driver.get_simulation(simulator_instance_index).AdvanceTo(desired_time);
```
 @tparam T drake default scalar type
 */
template <typename T>
class SimulationFacade {
 public:
  // No copy-construction, or copy-assignment.
  SimulationFacade(const SimulationFacade&) = delete;
  void operator=(const SimulationFacade&) = delete;

  // No move-construction, or move-assignment.
  SimulationFacade(SimulationFacade&&) = delete;
  void operator=(SimulationFacade&&) = delete;

  // Default destructor.
  virtual ~SimulationFacade() = default;
  SimulationFacade() = default;

  // Generate a Drake simulation at an initial state that is ready to run.
  SimulatorInstanceIndex CreateSimulatedSystem(bool connect_to_drake_visualizer = true) {
    // Create and finalize the universal_plant.
    CreateModelAndSceneGraph();

    // Build the diagram and create its context.
    ConstructAndConnectDiagram(connect_to_drake_visualizer);

    // Set state values from config into the context & make a simulator.
    return CreateSimulationAtInitialState();
  }

  // Accessors for Drake classes.
  const drake::multibody::MultibodyPlant<T>& universal_plant() const {
    DR_DEMAND(universal_plant_is_ready());
    return *universal_plant_;
  }
  const drake::geometry::SceneGraph<T>& scene_graph() const {
    DR_DEMAND(universal_plant_is_ready());
    return *scene_graph_;
  }
  const drake::systems::Diagram<T>& diagram() const {
    DR_DEMAND(diagram_is_ready());
    return *diagram_.get();
  }

  // Accessors for to Drake classes.
  drake::systems::DiagramBuilder<T>& builder() {
    DR_DEMAND(builder_.get() != nullptr);
    return *builder_.get();
  }
  drake::multibody::MultibodyPlant<T>& universal_plant() {
    DR_DEMAND(universal_plant_is_ready());
    return *universal_plant_;
  }
  drake::geometry::SceneGraph<T>& scene_graph() {
    DR_DEMAND(universal_plant_is_ready());
    return *scene_graph_;
  }
  drake::systems::Diagram<T>& diagram() {
    DR_DEMAND(diagram_is_ready());
    return *diagram_.get();
  }

  /// Clear all owned simulator instances and reset instance index counter.
  void ClearSimulators() { simulators_.clear(); }

  /// Accessor for simulators owned by class.
  drake::systems::Simulator<double>& simulator(SimulatorInstanceIndex simulator_instance) {
    DR_DEMAND(universal_plant_is_ready());
    DR_DEMAND(diagram_is_ready());
    return *simulators_.at(simulator_instance).get();
  }

  /// Accessor for simulators owned by class.
  const drake::systems::Simulator<double>& simulator(SimulatorInstanceIndex simulator_instance) const {
    DR_DEMAND(universal_plant_is_ready());
    DR_DEMAND(diagram_is_ready());
    return *simulators_.at(simulator_instance).get();
  }

 protected:
  virtual void DoPopulateModelAndSceneGraph() = 0;
  virtual void DoConstructAndConnectDiagram() = 0;
  virtual void DoSetState(drake::systems::Context<T>* context) = 0;

 private:
  // Signals whether CreateModelAndSceneGraph() has been run.
  bool universal_plant_is_ready() const { return (universal_plant_ != nullptr); }

  // Signals whether ConstructAndConnectDiagram() has been run.
  bool diagram_is_ready() const { return (diagram_.get() != nullptr); }

  /*
   Creates and finalizes a multibody plant according to a set of ConfigBase derived classes.
   Work done in this function includes:
   1) adding robot model (RobotInstanceConfig), and other bodies (SingleBodyInstanceConfig) to the universal plant.
   2) setting all body poses that will remain fixed (e.g., fixed frames, welds).
   NOTE: universal_plant_ is finalized after this function is called.
   */
  void CreateModelAndSceneGraph() {
    builder_ = std::make_unique<drake::systems::DiagramBuilder<T>>();

    // TODO(samzapo) remove hardcoded step size when performance allows.
    drake::multibody::AddMultibodyPlantSceneGraphResult<T> result = drake::multibody::AddMultibodyPlantSceneGraph(
        builder_.get(), std::make_unique<drake::multibody::MultibodyPlant<T>>(0.001));

    universal_plant_ = &(result.plant);
    scene_graph_ = &(result.scene_graph);

    DR_DEMAND(universal_plant_ != nullptr);
    DR_DEMAND(scene_graph_ != nullptr);

    this->DoPopulateModelAndSceneGraph();

    // Now the universal_plant is complete.
    universal_plant_->Finalize();

    // Check that this function completed properly.
    DR_DEMAND(universal_plant_is_ready());
  }

  /*
  Build the simulation & control diagram.
  NOTE: All diagram connections must be connected by the builder before this step.
  @pre universal_plant_ must exist.
  @pre universal_plant_ must be finalized.
 */
  void ConstructAndConnectDiagram(bool connect_to_drake_visualizer) {
    DR_DEMAND(universal_plant_is_ready());
    DR_DEMAND(universal_plant().is_finalized());

    if (connect_to_drake_visualizer) {
      if (universal_plant().is_discrete()) {
        // Visualize Contact forces.
        drake::multibody::ConnectContactResultsToDrakeVisualizer(&builder(), universal_plant());
      }
      // Visualize body visual geometries.
      drake::geometry::ConnectDrakeVisualizer(&builder(), scene_graph());
    }

    this->DoConstructAndConnectDiagram();

    // Perform the final build step for creating a diagram.
    diagram_ = builder().Build();

    // Check that this function completed properly.
    DR_DEMAND(diagram_is_ready());
  }

  /*
  Set all free floating body poses and robot joint angles.
  NOTE: this function sets all states, not just kinematic ones.
  @param simulator_params a collection of parameters for setting up a simulator.
  @return SimulatorInstanceIndex.
  @pre universal_plant_ must exist.
  @pre universal_plant_ must be finalized.
  @pre diagram_ must be built.
   */
  SimulatorInstanceIndex CreateSimulationAtInitialState() {
    DR_DEMAND(universal_plant_is_ready());
    DR_DEMAND(universal_plant().is_finalized());
    DR_DEMAND(diagram_is_ready());

    // Create a context for this system:
    std::unique_ptr<drake::systems::Context<T>> context = diagram().CreateDefaultContext();
    diagram().SetDefaultContext(context.get());

    this->DoSetState(context.get());

    // Immediately pass context to simulator (now owns context).
    std::unique_ptr<drake::systems::Simulator<double>> simulator = CreateAndConfigureSimulator(std::move(context));

    const SimulatorInstanceIndex index(simulators_.size());
    simulators_.push_back(std::move(simulator));
    return index;
  }

  /**
  Create a simulator and pass the diagram contect to it.
  Sets parameters for and then returns an initialized drake Simulator of the input universal_plant and diagram.
  @param context a unique pointer to the Context of diagram.  The returned simulator owns this object after.
  @return a unique pointer to the simulator that should be used to advance
          the state of the mbp in time.
  @pre a unique context of the diagram must be provided and its initial conditions should be set.
  NOTE: context_ is owned by the simulator after this step.
   */
  std::unique_ptr<drake::systems::Simulator<double>> CreateAndConfigureSimulator(
      std::unique_ptr<drake::systems::Context<T>> context,
      const SimulatorParameters& simulator_params = kDefaultSimulatorParameters) {
    // Set how much penetration (in meters) we are willing to accept.
    // TODO(samzapo): This is a bad place for this assignment. The universal_plant should be immutable.
    universal_plant_->set_penetration_allowance(simulator_params.target_accuracy);

    auto simulator = std::make_unique<drake::systems::Simulator<double>>(diagram(), std::move(context));

    // Hint the integrator's time step based on the contact time scale.
    // A fraction of this time scale is used which is chosen so that the fixed
    // time step integrators are stable.
    // From Drake:
    // https://github.com/RobotLocomotion/drake/blob/8490593661fe3c60167358241534771bdd5c3347/examples/simple_gripper/simple_gripper.cc#L231-L238
    // If the user specifies a time step, we use that, otherwise estimate a maximum time step based on the compliance
    // of the contact model. The maximum time step is estimated to resolve this time scale with at least 30 time
    // steps. Usually this is a good starting point for fixed step size integrators to be stable.
    simulator->get_mutable_integrator().set_maximum_step_size(
        universal_plant().get_contact_penalty_method_time_scale() / 30);

    // Error control is only supported for variable time step integrators.
    if (!simulator->get_integrator().get_fixed_step_mode())
      simulator->get_mutable_integrator().set_target_accuracy(simulator_params.target_accuracy);

    simulator->Initialize();
    return simulator;
  }

  std::unique_ptr<drake::systems::DiagramBuilder<T>> builder_;

  drake::multibody::MultibodyPlant<T>* universal_plant_;
  drake::geometry::SceneGraph<T>* scene_graph_;

  std::unique_ptr<drake::systems::Diagram<T>> diagram_;

  std::vector<std::unique_ptr<drake::systems::Simulator<double>>> simulators_;
};
}  // namespace DR
