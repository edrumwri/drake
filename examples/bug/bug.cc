#include <memory>
#include <string>

#include <gflags/gflags.h>
#include "fmt/ostream.h"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/sine.h"

namespace drake {
namespace examples {
namespace bug {
namespace {

using Eigen::Vector3d;
using geometry::SceneGraph;
using geometry::Sphere;
using lcm::DrakeLcm;
using math::RigidTransformd;
using multibody::ConnectContactResultsToDrakeVisualizer;
using multibody::MultibodyPlant;
using multibody::Parser;
using systems::Context;
using systems::ImplicitEulerIntegrator;
using systems::RungeKutta2Integrator;
using systems::RungeKutta3Integrator;
using systems::SemiExplicitEulerIntegrator;

// TODO(amcastro-tri): Consider moving this large set of parameters to a
// configuration file (e.g. YAML).
DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation. [s].");

// Integration parameters:
DEFINE_string(integration_scheme, "implicit_euler",
              "Integration scheme to be used. Available options are: "
              "'semi_explicit_euler','runge_kutta2','runge_kutta3',"
              "'implicit_euler'");
DEFINE_double(max_time_step, 1.0e-3,
              "Maximum time step used for the integrators. [s]. "
              "If negative, a value based on parameter penetration_allowance "
              "is used.");
DEFINE_double(accuracy, 1.0e-2, "Sets the simulation accuracy for variable step"
              "size integrators with error control.");

int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  DRAKE_DEMAND(FLAGS_max_time_step > 0);

  MultibodyPlant<double>& plant =
      *builder.AddSystem<MultibodyPlant>();
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  Parser parser(&plant);
  std::string full_name =
      FindResourceOrThrow("drake/examples/bug/bug.sdf");
  parser.AddModelFromFile(full_name);

  // Now the model is complete.
  plant.Finalize();

  // If the user specifies a time step, we use that, otherwise estimate a
  // maximum time step based on the compliance of the contact model.
  // The maximum time step is estimated to resolve this time scale with at
  // least 30 time steps. Usually this is a good starting point for fixed step
  // size integrators to be stable.
  const double max_time_step =
      FLAGS_max_time_step > 0 ? FLAGS_max_time_step :
      plant.get_contact_penalty_method_time_scale() / 30;

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  DrakeLcm lcm;
  geometry::ConnectDrakeVisualizer(&builder, scene_graph, &lcm);
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  auto actuation_port = builder.ExportInput(plant.get_actuation_input_port());

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram_context->FixInputPort(actuation_port, VectorX<double>::Zero(5));
  Context<double>& mbp_context = diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  VectorX<double> q_and_v(plant.num_positions() + plant.num_velocities());
  for (int i = 0; i < q_and_v.size(); ++i) q_and_v[i] = i + 1;
  plant.SetPositionsAndVelocities(&mbp_context, q_and_v);

  // Set up simulator.
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  systems::IntegratorBase<double>* integrator{nullptr};

  if (FLAGS_integration_scheme == "implicit_euler") {
    integrator =
        &simulator.reset_integrator<ImplicitEulerIntegrator<double>>();
  } else if (FLAGS_integration_scheme == "runge_kutta2") {
    integrator = &simulator.reset_integrator<RungeKutta2Integrator<double>>(
        max_time_step);
  } else if (FLAGS_integration_scheme == "runge_kutta3") {
    integrator =
        &simulator.reset_integrator<RungeKutta3Integrator<double>>();
  } else if (FLAGS_integration_scheme == "semi_explicit_euler") {
    integrator =
        &simulator.reset_integrator<SemiExplicitEulerIntegrator<double>>(
            max_time_step);
  } else {
    throw std::runtime_error(
        "Integration scheme '" + FLAGS_integration_scheme +
            "' not supported for this example.");
  }
  integrator->set_maximum_step_size(max_time_step);
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(FLAGS_accuracy);

  // The error controlled integrators might need to take very small time steps
  // to compute a solution to the desired accuracy. Therefore, to visualize
  // these very short transients, we publish every time step.
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);

  fmt::print("Stats for integrator {}:\n", FLAGS_integration_scheme);
  fmt::print("Number of time steps taken = {:d}\n",
             integrator->get_num_steps_taken());
  if (!integrator->get_fixed_step_mode()) {
    fmt::print("Initial time step taken = {:10.6g} s\n",
               integrator->get_actual_initial_step_size_taken());
    fmt::print("Largest time step taken = {:10.6g} s\n",
               integrator->get_largest_step_size_taken());
    fmt::print("Smallest adapted step size = {:10.6g} s\n",
               integrator->get_smallest_adapted_step_size_taken());
    fmt::print("Number of steps shrunk due to error control = {:d}\n",
               integrator->get_num_step_shrinkages_from_error_control());
  }

  return 0;
}

}  // namespace
}  // namespace bug 
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::bug::do_main();
}
