#include <iostream>
#include <chrono>

#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/systems/framework/diagram.h"
#include "drake/examples/iiwa_soccer/source_switcher.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/framework/input_port_descriptor.h"
#include "drake/systems/primitives/adder.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/common/find_resource.h"
#include "drake/examples/iiwa_soccer/impedance_controller.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/inverse_dynamics.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/examples/iiwa_soccer/spline_trajectory.h"
#include "drake/examples/iiwa_soccer/trajectory_evaluator.h"
#include "drake/examples/iiwa_soccer/soccer_common.h"
#include "drake/examples/iiwa_soccer/fsm_system.h"
#include "drake/examples/iiwa_soccer/target_shifter.h"
#include "drake/examples/iiwa_soccer/nonconstant_vector_source.h"
#include "drake/examples/iiwa_soccer/nonconstant_abstract_source.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/multibody/constraint_wrappers.h"
#include "drake/multibody/kinematics_cache_helper.h"
#include "drake/solvers/binding.h"
#include "drake/common/temp_directory.h"
#include "drake/solvers/snopt_solver.h"



namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::Diagram;
using systems::DiagramBuilder;
using systems::PassThrough;
using systems::System;
using systems::InputPortDescriptor;
using systems::Adder;
using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;
using systems::controllers::InverseDynamicsController;
using systems::controllers::InverseDynamics;
using manipulation::util::SimDiagramBuilder;
using systems::RigidBodyPlant;
using systems::Simulator;
using systems::ConstantVectorSource;
using systems::plants::KinematicsCacheHelper;
using systems::plants::SingleTimeKinematicConstraintWrapper;
using solvers::Binding;

typedef trajectories::PiecewisePolynomial<double> PiecewisePolynomialType;

const char* armModelPath = "drake/examples/iiwa_soccer/models/iiwa14_spheres_collision.urdf";
const char* ballModelPath = "drake/examples/iiwa_soccer/models/soccer_ball.urdf";

void DoTrajTest() {
  /******** Setup the world. ********/
  drake::lcm::DrakeLcm lcm;

  DiagramBuilder<double> builder;

  auto tree = std::make_unique<RigidBodyTree<double>>();

  // Import arm to tree.
  auto kuka_id_table = AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(armModelPath),
                                                           multibody::joints::kFixed,
                                                           tree.get());

//  // Import ball to tree.
//  auto ball_id_table = AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(ballModelPath),
//                                                           multibody::joints::kQuaternion,
//                                                           tree.get());

  // Make ground part of tree.
  multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);

  //auto ball_id = ball_id_table.at("soccer_ball");
  auto kuka_id = kuka_id_table.at("iiwa14");

  RigidBodyPlant<double>* plant_ptr = builder.AddSystem<RigidBodyPlant<double>>(std::move(tree));


//  // Map collision id numbers to bodies.
//  auto collision_to_body = std::map<unsigned long, std::string>();
//  for (int i = 0; i < plant_ptr->get_num_bodies(); i++ ){
//
//    auto col_ids = plant_ptr->get_rigid_body_tree().get_body(i).get_collision_element_ids();
//
//    for (unsigned long j = 0; j < col_ids.size(); j++) {
//      collision_to_body.insert(std::make_pair(col_ids.back(), plant_ptr->get_rigid_body_tree().get_body(i).get_name()));
//      col_ids.pop_back();
//    }
//  }

  // Creates and adds LCM publisher for visualization.
  auto visualizer = builder.AddSystem<systems::DrakeVisualizer>(plant_ptr->get_rigid_body_tree(), &lcm);
  visualizer->set_publish_period(0.005);

  auto context = plant_ptr->CreateDefaultContext();

  const int kNumTimeSamples = 21;
  const double kMinimumTimeStep = 0.2;
  const double kMaximumTimeStep = 0.5;
  systems::trajectory_optimization::DirectCollocation dircol(
      plant_ptr, *context, kNumTimeSamples, kMinimumTimeStep,
      kMaximumTimeStep);

  auto u = dircol.input();
  const double kTorqueLimit = 100;
  dircol.AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
  dircol.AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);


  const RigidBodyTree<double>* tree_ = &plant_ptr->get_rigid_body_tree();
  VectorXd zero_conf = tree_->getZeroConfiguration();
  zero_conf.conservativeResize(14);
  Eigen::VectorXd x0 = zero_conf;
  Eigen::VectorXd xf(14);
  xf.setZero();
  xf[0] = 0.5;
  xf[1] = -0.1;




//   TRYING WORLD CONSTRAINT
  auto tree_for_constraints = tree_->Clone();
  WorldPositionConstraint wpc(tree_for_constraints.get(), tree_for_constraints->FindBody("iiwa_link_7", "iiwa14")->get_body_index(),
                          Vector3d(0,0,0.9), Vector3d(-0.1,-0.1,0.8), Vector3d(0.1,0.1,1), Vector2d(NAN, NAN));

  KinematicsCacheHelper<double> cache_helper(*tree_for_constraints);
  auto kine_constraint_wrapper = std::make_shared<SingleTimeKinematicConstraintWrapper>(&wpc, &cache_helper);
//
//  //Binding<SingleTimeKinematicConstraintWrapper> con_bind(kine_constraint_wrapper, DECISIONVARS);
for (int i = 15; i < 20; i++){

    dircol.AddConstraint(kine_constraint_wrapper, dircol.state(i).topRows(7));
}

//  dircol.AddConstraintToAllKnotPoints(kine_constraint_wrapper);



  const std::string print_file = "/tmp/snopt.out";
  std::cout << print_file << std::endl;
  dircol.SetSolverOption(solvers::SnoptSolver::id(), "Print file", print_file);
  dircol.SetSolverOption(solvers::SnoptSolver::id(), "Major optimality tolerance", 1e-4);



  dircol.AddLinearConstraint(dircol.initial_state() == x0);
 // dircol.AddLinearConstraint(dircol.final_state() == xf);

  dircol.AddEqualTimeIntervalsConstraints();

  const double R = 10;  // Cost on input "effort".
  dircol.AddRunningCost((R * u.transpose()) * u);

  const double timespan_init = 4;
  auto traj_init_x =
      PiecewisePolynomialType::FirstOrderHold({0, timespan_init}, {x0, xf});
  dircol.SetInitialTrajectory(PiecewisePolynomialType(), traj_init_x);


  // Do optim and time it.
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  solvers::SolutionResult result = dircol.Solve();
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::seconds>( t2 - t1 ).count();
  std::cout << "Trajectory optimization time: \n";
  std::cout << duration;
  std::cout << " seconds.\n";

  const trajectories::PiecewisePolynomial<double> pp_xtraj =
      dircol.ReconstructStateTrajectory();

  dircol.
  auto source_x = builder.AddSystem<systems::TrajectorySource>(pp_xtraj);
  const trajectories::PiecewisePolynomial<double> pp_torque =
    dircol.ReconstructInputTrajectory();
  auto source_tau = builder.AddSystem<systems::TrajectorySource>(pp_torque);

  visualizer->set_publish_period(1.0 / 60.0);

  builder.Connect(source_tau->get_output_port(), plant_ptr->get_input_port(0));
  builder.Connect(source_x->get_output_port(), visualizer->get_input_port(0));
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_at_initialization(false);
  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1.0);
  Context<double>& sim_context = simulator.get_mutable_context();
  simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(*diagram, 0.002, &sim_context);

  simulator.Initialize();
  simulator.StepTo(pp_torque.end_time());
}

}
}
}


int main(int argc, char* argv[]) {
//  gflags::ParseCommandLineFlags(&argc, &argv, true);
//  drake::logging::HandleSpdlogGflags();
  drake::examples::iiwa_soccer::DoTrajTest();
  return 0;
}