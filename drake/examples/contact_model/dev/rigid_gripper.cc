/*! @file
A simple example of a rigid gripper attempting to hold a block. The gripper
has rigid geometry: two fingers at a fixed distance from each other. They
are positioned in a configuration *slightly* narrower than the box placed
between them.

This is a test to evaluate/debug the contact model.  This configuration
simplifies the test by defining a known penetration and eliminating all
controller-dependent variation.

This is an even simpler example of what is shown in schung_wsg_lift_test.
This eliminates the PID controller and ground contact.  At the end of the
simulation, the box should have slipped a hardly appreciable amount.
*/

#include <algorithm>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/dev/time_stepping_rigid_body_plant.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

DEFINE_double(timestep, 1e-3, "Sets the simulation time step.");
DEFINE_double(mu, 0.9, "The coefficient of friction (static and dynamic)");
DEFINE_double(stiffness, 10000, "The contact material stiffness");
DEFINE_double(damping, 100, "The contact material damping");
DEFINE_double(sim_duration, 5, "Amount of time to simulate");
DEFINE_bool(playback, true,
            "If true, simulation begins looping playback when complete");

namespace drake {
namespace examples {

using drake::systems::ContactResultsToLcmSystem;
using drake::systems::lcm::LcmPublisherSystem;

std::unique_ptr<RigidBodyTreed> BuildTestTree() {
  std::unique_ptr<RigidBodyTreed> tree = std::make_unique<RigidBodyTreed>();

  // Add the gripper.  Offset it slightly back and up so that we can
  // locate the box at the origin.
  auto gripper_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "gripper_pose_frame",
      &tree->world(), Eigen::Vector3d(0, -0.065, 0.05),
      Eigen::Vector3d::Zero());
  parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow("drake/examples/contact_model/rigid_gripper.urdf"),
      multibody::joints::kFixed, gripper_frame, tree.get());

  // Add a box to grip.  Position it such that if there *were* a plane at z = 0,
  // the box would be sitting on it (this maintains parity with the
  // schunk_wsg_lift_test scenario).
  auto box_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "box_offset",
      &tree->world(), Eigen::Vector3d(0, 0, 0.075), Eigen::Vector3d::Zero());
  parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow("drake/multibody/models/box_small.urdf"),
      multibody::joints::kQuaternion, box_frame, tree.get());

  return tree;
}

int main() {
  systems::DiagramBuilder<double> builder;

  auto* plant = builder.AddSystem<systems::TimeSteppingRigidBodyPlant<double>>(
    BuildTestTree(), FLAGS_timestep);
  plant->set_name("plant");

  // Command-line specified contact parameters.
  std::cout << "Contact properties:\n";
  std::cout << "\tStiffness:                " << FLAGS_stiffness << "\n";
  std::cout << "\tDamping:                  " << FLAGS_damping << "\n";
  std::cout << "\tFriction:                 " << FLAGS_mu << "\n";
  plant->set_default_stiffness(FLAGS_stiffness);
  plant->set_default_damping(FLAGS_damping);
  plant->set_default_friction_coefficient(FLAGS_mu);

  // Creates and adds LCM publisher for visualization.  The test doesn't
  // require `drake_visualizer` but it is convenient to have when debugging.
  drake::lcm::DrakeLcm lcm;
  const auto viz_publisher =
      builder.template AddSystem<systems::DrakeVisualizer>(
          plant->get_rigid_body_tree(), &lcm, true);
  builder.Connect(plant->state_output_port(),
                  viz_publisher->get_input_port(0));

  // Set up the model and simulator and set their starting state.
  const std::unique_ptr<systems::Diagram<double>> model = builder.Build();
  systems::Simulator<double> simulator(*model);

  simulator.Initialize();

  // Print a time stamp update every tenth of a second.  This helps communicate
  // progress in the event that the integrator crawls to a very small timestep.
  simulator.StepTo(FLAGS_sim_duration);

  while (FLAGS_playback) viz_publisher->ReplayCachedSimulation();
  return 0;
}

}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::main();
}
