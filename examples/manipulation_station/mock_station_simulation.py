import argparse
from pydrake.all import (DrakeLcm, DiagramBuilder, Simulator)


'''
Runs a simulation of the manipulation station plant as a stand-alone
simulation which mocks the network inputs and outputs of the real robot
station.  This is a useful test in the transition from a single-process
simulation to operating on the real robot hardware.
'''

# Converts a string to a Boolean value.
def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def main():
    # Parse argument flags.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--duration", type=float, default=float('inf'),
        help="Simulation duration.")
    parser.add_argument(
        "--target_realtime_rate", type=float, default=1.0,
        help="Playback speed.  See documentation for " + \
             "Simulator::set_target_realtime_rate() for details.")
    parser.add_argument(
        "setup", default="clutter_clearing",
        help="Manipulation station type to simulate. " +
             "Can be {default, clutter_clearing}.")
    args = parser.parse_args()

  builder = DiagramBuilder()

  # Create the "manipulation station".
  station = builder.AddSystem<ManipulationStation>();
  if (FLAGS_setup == "default") {
    station.SetupDefaultStation();
  } else if (FLAGS_setup == "clutter_clearing") {
    station.SetupClutterClearingStation();
    station.AddManipulandFromFile(
        "drake/manipulation/models/ycb/sdf/003_cracker_box.sdf",
        math::RigidTransform<double>(math::RollPitchYaw<double>(-1.57, 0, 3),
                               Eigen::Vector3d(-0.3, -0.55, 0.36)));
  } else {
    throw std::domain_error(
        "Unrecognized station type. Options are {default, clutter_clearing}.");
  }
  # TODO(russt): Load sdf objects specified at the command line.  Requires
  # #9747.
  station.Finalize();

  # Connect components to Drake Visualizer.
  geometry::ConnectDrakeVisualizer(builder, station.get_scene_graph(),
                                   station.GetOutputPort("pose_bundle"));

  lcm = DrakeLcm();
  lcm.StartReceiveThread();

  iiwa_command_subscriber = builder.AddSystem(
      kuka_iiwa_arm::MakeIiwaCommandLcmSubscriberSystem(
          kuka_iiwa_arm::kIiwaArmNumJoints, "IIWA_COMMAND", lcm));
  iiwa_command = builder.AddSystem<kuka_iiwa_arm::IiwaCommandReceiver>();
  builder.Connect(iiwa_command_subscriber.get_output_port(),
                  iiwa_command.GetInputPort("command_message"));

  # Pull the positions out of the state.
  builder.Connect(iiwa_command.get_commanded_position_output_port(),
                  station.GetInputPort("iiwa_position"));
  builder.Connect(iiwa_command.get_commanded_torque_output_port(),
                  station.GetInputPort("iiwa_feedforward_torque"));

  iiwa_status = builder.AddSystem<kuka_iiwa_arm::IiwaStatusSender>();
  builder.Connect(station.GetOutputPort("iiwa_position_commanded"),
                  iiwa_status.get_position_commanded_input_port());
  builder.Connect(station.GetOutputPort("iiwa_position_measured"),
                  iiwa_status.get_position_measured_input_port());
  builder.Connect(station.GetOutputPort("iiwa_velocity_estimated"),
                  iiwa_status.get_velocity_estimated_input_port());
  builder.Connect(station.GetOutputPort("iiwa_torque_commanded"),
                  iiwa_status.get_torque_commanded_input_port());
  builder.Connect(station.GetOutputPort("iiwa_torque_measured"),
                  iiwa_status.get_torque_measured_input_port());
  builder.Connect(station.GetOutputPort("iiwa_torque_external"),
                  iiwa_status.get_torque_external_input_port());
  iiwa_status_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm, 0.005 /* publish period */));
  builder.Connect(iiwa_status.get_output_port(),
                  iiwa_status_publisher.get_input_port());

  # Receive the WSG commands.
  wsg_command_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::MakeFixedSize(
          drake::lcmt_schunk_wsg_command{}, "SCHUNK_WSG_COMMAND", lcm));
  wsg_command =
      builder.AddSystem<manipulation::schunk_wsg::SchunkWsgCommandReceiver>();
  builder.Connect(wsg_command_subscriber.get_output_port(),
                  wsg_command.GetInputPort("command_message"));
  builder.Connect(wsg_command.get_position_output_port(),
                  station.GetInputPort("wsg_position"));
  builder.Connect(wsg_command.get_force_limit_output_port(),
                  station.GetInputPort("wsg_force_limit"));

  # Publish the WSG status.
  wsg_status =
      builder.AddSystem<manipulation::schunk_wsg::SchunkWsgStatusSender>();
  builder.Connect(station.GetOutputPort("wsg_state_measured"),
                  wsg_status.get_state_input_port());
  builder.Connect(station.GetOutputPort("wsg_force_measured"),
                  wsg_status.get_force_input_port());
  wsg_status_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_schunk_wsg_status>(
          "SCHUNK_WSG_STATUS", lcm, 0.05 /* publish period */));
  builder.Connect(wsg_status.get_output_port(0),
                  wsg_status_publisher.get_input_port());

  # TODO(russt): Publish the camera outputs.

  diagram = builder.Build();

  simulator = Simulator(diagram);
  context = simulator.get_mutable_context();
  station_context =
      diagram.GetMutableSubsystemContext(station, context);

  # Get the initial Iiwa pose and initialize the iiwa_command to match.
  VectorXd q0 = station.GetIiwaPosition(station_context);
  iiwa_command.set_initial_position(
      diagram.GetMutableSubsystemContext(iiwa_command, context), q0);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.StepTo(FLAGS_duration);

  return 0;
}

if __name__ == "__main__"
    main()
