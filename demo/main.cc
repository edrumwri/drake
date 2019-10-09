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
#include <iostream>

#include <gflags/gflags.h>

#include <drake/common/text_logging_gflags.h>

#include <DR/drivers/chopstick/chopstick_config.h>
#include <DR/simulation/config.h>

// Duration virtual time before program exits.
DEFINE_double(max_simulation_time, 0.1, "Number of seconds (virtual time) to simulate.");

namespace DR {
namespace {
using T = double;

int do_main() { return 0; }
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
