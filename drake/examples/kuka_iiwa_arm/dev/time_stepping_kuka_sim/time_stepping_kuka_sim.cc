/// @file
///
/// This demo sets up a position controlled and gravity compensated KUKA iiwa
/// robot within a simulation to follow an arbitrarily designed plan. The
/// generated plan takes the arm from the zero configuration to reach to a
/// position in space and then repeat this reaching task with a different joint
/// configuration constraint.

#include <iostream>
#include <memory>
#include <string>

#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/dev/time_stepping_rigid_body_plant.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

int DoMain() {
  double step_size = 1e-3;

  auto tree_ptr = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                          "iiwa14_polytope_collision.urdf"),
      multibody::joints::kFixed, nullptr /* weld to frame */, tree_ptr.get());
  multibody::AddFlatTerrainToWorld(tree_ptr.get(), 100., 10.);

  VectorX<double> x0 = tree_ptr->getZeroConfiguration();
  VectorX<double> x1 = x0;
  x1[1] += M_PI / 2.;
  const int kDim = tree_ptr->get_num_positions();

  PiecewisePolynomialTrajectory traj(PiecewisePolynomial<double>::Cubic(
      {0, 3}, {x0, x1}, VectorX<double>::Zero(kDim),
      VectorX<double>::Zero(kDim)));

  drake::lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;
  // Adds a plant
  auto plant = builder.AddSystem<systems::TimeSteppingRigidBodyPlant<double>>(
      std::move(tree_ptr), step_size);
  const auto& tree = plant->get_rigid_body_tree();

  // Adds a visualizer
  auto viz = builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);

  // Adds a iiwa controller
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);

  auto controller =
      builder
          .AddSystem<systems::controllers::InverseDynamicsController<double>>(
              tree.Clone(), iiwa_kp, iiwa_ki, iiwa_kd,
              false /* no feedforward acceleration */);

  auto traj_src = builder.AddSystem<systems::TrajectorySource<double>>(
      traj, 1 /* outputs q + v */);

  builder.Connect(plant->state_output_port(),
                  controller->get_input_port_estimated_state());
  builder.Connect(traj_src->get_output_port(),
                  controller->get_input_port_desired_state());
  builder.Connect(controller->get_output_port_control(),
                  plant->actuator_command_input_port());
  builder.Connect(plant->state_output_port(), viz->get_input_port(0));

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);

  simulator.StepTo(5);

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::kuka_iiwa_arm::DoMain();
}
