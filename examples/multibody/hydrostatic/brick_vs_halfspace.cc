#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/plant/contact_surfaces_to_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace drake {
namespace examples {
namespace multibody {
namespace hydrostatic {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

/*
DEFINE_double(vx0, 1.0,
              "The initial x-velocity of the cylinder, m/s.");

DEFINE_double(wx0, 0.1,
              "The initial x-angular velocity of the cylinder, rad/s.");
*/
DEFINE_double(friction_coefficient, 0.3,
              "The friction coefficient of both the cylinder and the ground.");

DEFINE_double(time_step, 0.0,
              "If zero, the plant is modeled as a continuous system. "
              "If positive, the period (in seconds) of the discrete updates "
              "for the plant modeled as a discrete system."
              "This parameter must be non-negative.");

using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using geometry::SceneGraph;
using lcm::DrakeLcm;

// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::ConnectContactSurfacesToDrakeVisualizer;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialVelocity;
using drake::multibody::Parser;
using drake::multibody::UniformGravityFieldElement;

int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Make and add the cart_pole model.
  const std::string full_name = FindResourceOrThrow(
      "drake/examples/multibody/hydrostatic/box.sdf");
  MultibodyPlant<double>& plant =
      *builder.AddSystem<MultibodyPlant>(FLAGS_time_step);
  Parser(&plant, &scene_graph).AddModelFromFile(full_name);

  // Weld the ground frame.
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("ground"));

  // Add gravity to the model.
  plant.AddForceElement<UniformGravityFieldElement>();

  plant.Finalize();

  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  DrakeLcm lcm;
  geometry::ConnectDrakeVisualizer(&builder, scene_graph, &lcm);

  // This is the source of poses for the visualizer.
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));

  // Publish contact surface for visualization.
  ConnectContactSurfacesToDrakeVisualizer(&builder, plant, &lcm);

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set initial pose for the box.
  auto& state_vector = plant_context.get_mutable_continuous_state_vector();
  using std::sqrt;
  state_vector[6] = 1.0;
  state_vector[0] = 2/sqrt(2.0);
  state_vector[1] = 0;
  state_vector[2] = 2/sqrt(2.0);
  state_vector[3] = 0;

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(
      *diagram, 1e-4, &simulator.get_mutable_context());
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace hydrostatic
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A demo for a brick contact halfspace and model using "
      "hydrostatic contact.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::multibody::hydrostatic::do_main();
}
