#include <gflags/gflags.h>
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
#include "drake/common/text_logging_gflags.h"

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



DEFINE_double(direction_multiply_target, 0, "");
DEFINE_double(direction_multiply_waypoint, 10, "");
DEFINE_double(waypoint_time, 1, "");
DEFINE_double(target_time, 4, "");
DEFINE_double(target_offset_height, 0.09, "");



switches control_switches;


const char* armModelPath = "drake/examples/iiwa_soccer/models/iiwa14_spheres_collision.urdf";
const char* ballModelPath = "drake/examples/iiwa_soccer/models/soccer_ball.urdf";

RigidBodyTree<double> tree_for_control;
drake::lcm::DrakeLcm lcm;

std::unique_ptr<Diagram<double>> make_junction_box() {

  DiagramBuilder<double> builder;

  // Dummy origin
  auto zeros3 = Eigen::Vector3d();
  zeros3 << 0, 0, 0;
  auto zeros_src3 = builder.AddSystem<ConstantVectorSource>(zeros3);


  /**** State comes into diagram. ****/
  auto arm_state_in = builder.AddSystem<PassThrough<double>>(14);
  builder.ExportInput(arm_state_in->get_input_port());
  auto ball_state_in = builder.AddSystem<PassThrough<double>>(13);
  builder.ExportInput(ball_state_in->get_input_port());

  // Ball position extractor.
  Eigen::MatrixXd ball_filt_mat(3,13); // Cuts out all the states which are not the ball's. This seems... elaborate.
  ball_filt_mat.setZero(); // Never again forget this line. Non-deterministic segfaults
  ball_filt_mat.col(0)[0] = 1;
  ball_filt_mat.col(1)[1] = 1;
  ball_filt_mat.col(2)[2] = 1;
  auto ball_position = builder.AddSystem<systems::MatrixGain>(ball_filt_mat);
  builder.Connect(ball_state_in->get_output_port(), ball_position->get_input_port());


  /**** Switch who sends torque to motors. ****/
  SourceSwitcher<double>* SWITCH_torque = builder.AddSystem<SourceSwitcher<double>>(7);
  auto& SWITCH_torque_adder_imped_w_grav = SWITCH_torque->add_selectable_output("control_adder");
  auto& SWITCH_torque_imped = SWITCH_torque->add_selectable_output("imped_ctrl_alone");
  auto& SWITCH_torque_grav = SWITCH_torque->add_selectable_output("gravity_comp_alone");
  //auto& SWITCH_torque_adder_invdyn_w_grav = SWITCH_torque->add_selectable_output("inv_dyn_w_grav_comp");

  auto torque_adder_imped = builder.AddSystem<Adder<double>>(2, 7); // num inputs, size.
  auto torque_adder_invdyn = builder.AddSystem<Adder<double>>(2, 7); // num inputs, size.

  SWITCH_torque->switch_output("control_adder");


  // Instance of the tree with ONLY the arm. Makes picking relevant info easier.

  AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(armModelPath),
                                      multibody::joints::kFixed,
                                      &tree_for_control);

  // Gains in cartesian-land.
  Vector3<double> k_p;
  k_p << 50, 50, 50;
  Vector3<double> k_d;
  k_d << 15, 15, 15;

  // Joint gains.
  VectorX<double> iiwa_kp(7);
  VectorX<double> iiwa_kd(7);
  VectorX<double> iiwa_ki(7);
  iiwa_kp << 40,40,40,40,40,40,40;
  iiwa_kd << 40,40,40,40,40,40,40;
  iiwa_ki << 40,40,40,40,40,40,40;


  // Gravity compensation controller.
  auto grav_comp = builder.AddSystem<InverseDynamics<double>>(tree_for_control, true);

  // Target tracking controller.
  auto imped_controller = builder.AddSystem<ImpedanceController>(tree_for_control,
                                                                 *tree_for_control.findFrame("iiwa_link_7").get(),
                                                                 k_p,
                                                                 k_d,
                                                                 lcm);
  // Inverse dynamics controller for full joint trajectory tracking.
//  auto inv_dyn_controller = builder.AddSystem<systems::controllers::InverseDynamicsController<double>>(
//      tree_for_control.Clone(), iiwa_kp, iiwa_ki, iiwa_kd,
//      false /* no feedforward acceleration */);

  // Grav comp torque to switch.
  builder.Connect(grav_comp->get_output_port_torque(), SWITCH_torque_grav);

  // Imped controller torque to switch.
  builder.Connect(imped_controller->get_output_port_control(), SWITCH_torque_imped);

  // Torques to adder, adder to switch.
  builder.Connect(grav_comp->get_output_port_torque(), torque_adder_imped->get_input_port(0));
  builder.Connect(imped_controller->get_output_port_control(), torque_adder_imped->get_input_port(1));
  builder.Connect(torque_adder_imped->get_output_port(), SWITCH_torque_adder_imped_w_grav);

  builder.Connect(arm_state_in->get_output_port(), grav_comp->get_input_port_estimated_state());
  builder.Connect(arm_state_in->get_output_port(), imped_controller->get_input_port_estimated_state());
//  builder.Connect(arm_state_in->get_output_port(), inv_dyn_controller->get_input_port_estimated_state());

  builder.Connect(grav_comp->get_output_port_torque(), torque_adder_invdyn->get_input_port(0));
//  builder.Connect(inv_dyn_controller->get_output_port_control(), torque_adder_invdyn->get_input_port(1));
//  builder.Connect(torque_adder_invdyn->get_output_port(), SWITCH_torque_adder_invdyn_w_grav);

  // Torque output port to motors.
  builder.ExportOutput(SWITCH_torque->get_output_port(0));
  // Hand position. TODO stop making this part of imped controller.
  builder.ExportOutput(imped_controller->get_output_port_hand_pos());




  /**** Target offset switching. ****/
// TODO: make offsets more general.
  SourceSwitcher<double>* SWITCH_offset = builder.AddSystem<SourceSwitcher>(3);
  auto& SWITCH_offset_zero = SWITCH_offset->add_selectable_output("zero_offset");
  auto& SWITCH_offset_ball = SWITCH_offset->add_selectable_output("ball_offset");

  // No offset.
  TargetShifter* zero_offset = builder.AddSystem<TargetShifter>();
  builder.Connect(zeros_src3->get_output_port(), zero_offset->get_input_port(0));
  builder.Connect(ball_position->get_output_port(), zero_offset->get_input_port(1));
  builder.Connect(zero_offset->get_output_port(0), SWITCH_offset_zero);


  // Offset from ball.
  NonconstantVectorSource* ball_offset_src = builder.AddSystem<NonconstantVectorSource>(3);

  TargetShifter* ball_offset_shifter = builder.AddSystem<TargetShifter>();
  builder.Connect(ball_offset_src->get_output_port(0), ball_offset_shifter->get_input_port(0));
  builder.Connect(ball_position->get_output_port(), ball_offset_shifter->get_input_port(1));
  builder.Connect(ball_offset_shifter->get_output_port(0), SWITCH_offset_ball);

  Adder<double>* ball_offset_adder = builder.AddSystem<Adder<double>>(2,3);
  builder.Connect(ball_offset_shifter->get_output_port(0), ball_offset_adder->get_input_port(0));
  builder.Connect(ball_position->get_output_port(), ball_offset_adder->get_input_port(1));


  /**** Trajectory generation. ****/
  // Spline trajectory
  double reach_duration = 5;
  auto spline_maker = builder.AddSystem<SplineTrajectories>(0, reach_duration, lcm);
  auto spline_evaluator = builder.AddSystem<TrajectoryEvaluator>(3);
  spline_maker->debug_draw = false;

  // Waypoint position to spline maker.

  NonconstantVectorSource* waypoint_offset_src = builder.AddSystem<NonconstantVectorSource>(3);
  TargetShifter* waypoint_offset = builder.AddSystem<TargetShifter>();
  Adder<double>* waypoint_offset_adder = builder.AddSystem<Adder<double>>(2,3);
  builder.Connect(waypoint_offset_src->get_output_port(0), waypoint_offset->get_input_port(0));
  builder.Connect(ball_position->get_output_port(), waypoint_offset->get_input_port(1));
  builder.Connect(waypoint_offset->get_output_port(0), waypoint_offset_adder->get_input_port(0));
  builder.Connect(ball_position->get_output_port(), waypoint_offset_adder->get_input_port(1));

  builder.Connect(waypoint_offset_adder->get_output_port(), spline_maker->get_input_port(0));

  // Ball position to spline maker.
  builder.Connect(ball_offset_adder->get_output_port(), spline_maker->get_input_port(1));

  // Spline maker to spline evaluator.
  builder.Connect(spline_maker->get_output_port(0), spline_evaluator->get_input_port(0));

//  auto ik_traj_receiver = builder.AddSystem<NonconstantAbstractSource<PiecewisePolynomial<double>>>();
//  auto ik_traj_evaluator = builder.AddSystem<TrajectoryEvaluator>(14);
//
//  // Connect ik trajectory evaluator to the inverse dynamics controller.
//  builder.Connect(ik_traj_receiver->get_output_port(0), ik_traj_evaluator->get_input_port(0));
//  builder.Connect(ik_traj_evaluator->get_output_port(0), inv_dyn_controller->get_input_port_desired_state());






  /**** Switch who sends setpoints to the impedance controller. ****/
  SourceSwitcher<double>* SWITCH_setpt = builder.AddSystem<SourceSwitcher>(3);
  auto& SWITCH_setpt_fixed = SWITCH_setpt->add_selectable_output("fixed_input");
  auto& SWITCH_setpt_follow = SWITCH_setpt->add_selectable_output("follow_obj");
  auto& SWITCH_setpt_spline = SWITCH_setpt->add_selectable_output("spline");

  SWITCH_setpt->switch_output("fixed_input");

  // Dummy target
  auto offset = Eigen::Vector3d();
  offset << 1, 1, 1;
  auto const_src = builder.AddSystem<ConstantVectorSource>(offset);

  builder.Connect(const_src->get_output_port(), SWITCH_setpt_fixed);
  builder.Connect(ball_position->get_output_port(), SWITCH_setpt_follow);
  builder.Connect(spline_evaluator->get_output_port(0), SWITCH_setpt_spline);

  Adder<double>* offset_adder = builder.AddSystem<Adder<double>>(2, 3);
  builder.Connect(SWITCH_setpt->get_output_port(0), offset_adder->get_input_port(0));
  builder.Connect(SWITCH_offset->get_output_port(0), offset_adder->get_input_port(1)); // Add the selected offset to the impedance controller target.
  builder.Connect(offset_adder->get_output_port(), imped_controller->get_input_port_cartesian_target());


  control_switches = {SWITCH_torque, SWITCH_setpt, SWITCH_offset,
                      ball_offset_src, waypoint_offset_src, spline_maker, &tree_for_control};


  // Don't want this for all.
  waypoint_offset->use_cartesian = true;
  ball_offset_shifter->use_cartesian = true;


  return builder.Build();
}


void DoMain() {
  direction_multiply_target = FLAGS_direction_multiply_target;
  direction_multiply_waypoint = FLAGS_direction_multiply_waypoint;
  waypoint_time = FLAGS_waypoint_time;
  target_time = FLAGS_target_time;
  target_offset_height = FLAGS_target_offset_height;


  /******** Setup the world. ********/
  SimDiagramBuilder<double> builder; // Wraps normal DiagramBuilder. Has additional utility stuff for controlled systems.

  auto tree = std::make_unique<RigidBodyTree<double>>();

  // Import arm to tree.
  auto kuka_id_table = AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(armModelPath),
                                                           multibody::joints::kFixed,
                                                           tree.get());

  // Import ball to tree.
  auto ball_id_table = AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(ballModelPath),
                                                           multibody::joints::kQuaternion,
                                                           tree.get());

  // Make ground part of tree.
  multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);

  auto ball_id = ball_id_table.at("soccer_ball");
  auto kuka_id = kuka_id_table.at("iiwa14");

  RigidBodyPlant<double>* plant_ptr = builder.AddPlant(std::move(tree));


  // Map collision id numbers to bodies.
  auto collision_to_body = std::map<unsigned long, std::string>();
  for (int i = 0; i < plant_ptr->get_num_bodies(); i++ ){

    auto col_ids = plant_ptr->get_rigid_body_tree().get_body(i).get_collision_element_ids();

    for (unsigned long j = 0; j < col_ids.size(); j++) {
      collision_to_body.insert(std::make_pair(col_ids.back(), plant_ptr->get_rigid_body_tree().get_body(i).get_name()));
      col_ids.pop_back();
    }
  }

  // Creates and adds LCM publisher for visualization.
  auto visualizer = builder.AddVisualizer(&lcm);//<systems::DrakeVisualizer>(plant_ptr->get_rigid_body_tree(), &lcm);
  visualizer->set_publish_period(0.005);

  DiagramBuilder<double>* base_builder = builder.get_mutable_builder();

  auto switch_box_diagram = make_junction_box();

  //base_builder->BuildInto(switch_box_diagram.get());

  auto switch_box = base_builder->AddSystem(std::move(switch_box_diagram));

  // Why +n? I don't know.
  base_builder->Connect(plant_ptr->get_output_port(kuka_id + 3), switch_box->get_input_port(0));
  base_builder->Connect(plant_ptr->get_output_port(ball_id + 3), switch_box->get_input_port(1));
  base_builder->Connect(switch_box->get_output_port(0), plant_ptr->get_input_port(0));


  auto fsm = base_builder->AddSystem<FSM_System>(control_switches, collision_to_body);
  base_builder->Connect(plant_ptr->get_output_port(kuka_id + 3), fsm->get_input_port(0));
  base_builder->Connect(plant_ptr->get_output_port(ball_id + 3), fsm->get_input_port(1));
  base_builder->Connect(switch_box->get_output_port(1), fsm->get_input_port(2)); // ee position.
  base_builder->Connect(plant_ptr->contact_results_output_port(), fsm->get_input_port(3));


  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  simulator.set_publish_at_initialization(false);
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(1.0);

  Context<double>& context = simulator.get_mutable_context();
  plant_ptr->set_position(&context, 7, 0.3); // Moving the bowling ball away from the origin.
  plant_ptr->set_position(&context, 8, 0.3);
  plant_ptr->set_position(&context, 9, 0.2);



  simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(*diagram, 0.002, &context);
  simulator.Initialize();
  simulator.StepTo(INT64_MAX);
}


}  // namespace iiwa_soccer
}  // namespace examples
}  // namespace drake}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  drake::examples::iiwa_soccer::DoMain();
  return 0;
}