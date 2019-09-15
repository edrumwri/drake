/**
 * Robot Demo Entrypoint.
 * Uses the ModelGenerator, ControllerGenerator, PositionAndVelocityGenerator,
 * and SimulatorGenerator Classes to create a MultibodyPlant, the "universal plant", for
 * a generic picking scenario, currently:
 * A Box (floating, manipuland body) is placed on top of a Cylinder (fixed,
 * environment body).
 * The Environment is set to be a Trailer.
 * Two robot instances have been placed in the world ("chopstick_left" and
 * "chopstick_right").
 *
 * Currently, these robots are set to hold their initial positions.  This behavior can be
 * changed by setting each robot's control scheme.
 * Example:
 * ```
 * left.set_control_scheme(RobotInstanceConfig::kPrimitiveControlScheme);
 * right.set_control_scheme(RobotInstanceConfig::kPrimitiveControlScheme);
 * ```
 *
 * Run this code for 1 minute of virtual time with:
 * ```
 * ./DR_main --model_directory="/absolute/path/to/DR/models/" --max_simulation_time=60.0
 * ```
 */
#include <gflags/gflags.h>

#include <drake/common/text_logging_gflags.h>

#include <DR/drivers/chopstick/chopstick_config.h>
#include <DR/simulation/controller_generator.h>
#include <DR/simulation/model_generator.h>
#include <DR/simulation/simulation_generator.h>
#include <DR/simulation/state_setter.h>

// No default model directory (absolute path).
DEFINE_string(model_directory, "", "Absolute path to model directory.");

// Default time is 1 second.
DEFINE_double(max_simulation_time, 1.0, "Number of seconds (virtual time) to simulate.");

namespace DR {
namespace {
using T = double;

void DoSimulationBuildAndRun(const UnloadingTaskConfig& config) {
  ModelGenerator<T> model_gen;
  ControllerGenerator<T> controller_gen;
  StateSetter<T> state_gen;
  SimulationGenerator<T> sim_gen;

  drake::systems::DiagramBuilder<T> builder;

  drake::multibody::MultibodyPlant<T>* model;
  drake::geometry::SceneGraph<T>* scenegraph;
  std::unique_ptr<drake::systems::Diagram<T>> diagram;
  std::unique_ptr<drake::systems::Context<T>> context;

  drake::log()->critical(" ======= Create Universal Plant ======= ");
  std::tie(model, scenegraph) = model_gen.CreateSceneAndRobot(config, &builder);

  drake::log()->critical(" ======= Connect & Build Diagram ======= ");
  std::tie(diagram, context) = controller_gen.CreateDiagramWithContext(config, *model, *scenegraph, &builder);

  drake::log()->critical(" ======= Set System State ======= ");
  state_gen.SetState(config, *model, *diagram.get(), context.get());

  drake::log()->critical(" ======= Make Simulation ======= ");
  std::unique_ptr<drake::systems::Simulator<double>> simulator =
      sim_gen.BuildSimulator(config.simulator_instance_config(), *diagram.get(), std::move(context), model);

  drake::log()->critical(" ======= Simulate ======= ");
  // Test that we can successfully simulate the full test duration.
  try {
    simulator->AdvanceTo(config.simulator_instance_config().simulation_time());
  } catch (const std::exception& e) {
    drake::log()->critical("Simulation failed due to uncaught exception: {}", e.what());
  } catch (...) {
    drake::log()->critical("Simulation failed due to unknown exception.");
  }
  drake::log()->critical("Done simulating. time={} ", simulator->get_context().get_time());
}

int do_main() {
  // Gflags option 'model_directory' is required. Do not accept missing or default values.
  if (gflags::GetCommandLineFlagInfoOrDie("model_directory").is_default) {
    throw std::runtime_error("Set required param '--model_directory' to absolue path of 'DR/models/'.");
  }

  UnloadingTaskConfig config;

  SimulatorInstanceConfig simulator;
  // The simulation will run for 1 hour of virtual time.
  simulator.set_simulation_time(FLAGS_max_simulation_time);
  simulator.set_integration_scheme(SimulatorInstanceConfig::kImplicitEulerIntegrationScheme);
  config.set_simulator_instance_config(simulator);

  std::vector<BodyInstanceConfig> manipulands;
  // Floating manipuland Bodies.
  {
    BodyInstanceConfig manipuland;
    manipuland.set_mass(10.0);
    manipuland.set_is_floating(true);
    // Set pose of manipuland in world.
    manipuland.set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(), drake::Vector3<double>(1.0, 0.0, 1.0)));
    manipuland.SetCoulombFriction(0.8, 0.6);
    // Setup Box manipuland attributes.
    manipuland.set_name("manip_box_1");
    manipuland.SetBoxGeometry(0.2, 0.3, 0.1);
    manipulands.push_back(manipuland);
  }
  config.set_manipuland_instance_configs(manipulands);

  // Pair of chopstick robots.
  std::vector<RobotInstanceConfig> robots = CreateChopstickRobotsConfig();
  for (auto& robot : robots) {
    robot.set_control_scheme(RobotInstanceConfig::kStationaryControlScheme);
  }
  config.set_robot_instance_configs(robots);

  // Setup Environment options.
  EnvironmentInstanceConfig environment;
  {
    // Static Environment Bodies.
    std::vector<BodyInstanceConfig> env_bodies;
    {
      BodyInstanceConfig env_body;
      env_body.set_is_floating(false);
      // Setup config shared by both environments.
      // Set location of model in world.
      env_body.set_pose(drake::math::RigidTransform<double>(
          drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(), drake::Vector3<double>(1.0, 0.0, 0.5)));
      env_body.SetCoulombFriction(0.8, 0.6);

      // Setup Cylinder manipuland attributes.
      env_body.set_name("env_cylinder_1");
      env_body.SetCylinderGeometry(0.05, 0.3);
      env_bodies.push_back(env_body);
    }
    environment.set_body_instance_configs(env_bodies);

    // Use trailer environment.
    environment.set_trailer_environment();
    environment.SetCoulombFriction(0.8, 0.6);
  }
  config.set_environment_instance_config(environment);

  DoSimulationBuildAndRun(config);

  return 0;
}
}  // namespace
}  // namespace DR

int main(int argc, char* argv[]) {
  std::cout << "Running Program: " << std::endl;
  std::copy(argv, argv + argc, std::ostream_iterator<const char*>(std::cout, " "));
  std::cout << std::endl;
  gflags::SetUsageMessage(
      "Main function driving the 'Chopstick' robot in simulation "
      "using Drake's drake::multibody::MultibodyPlant, with "
      "geometry::SceneGraph visualization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return DR::do_main();
}
