import argparse
from pydrake.all import (Diagram, DiagramBuilder, Simulator)

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
        "--duration", type=float, default=4.0,
        help="Simulation duration.")
    parser.add_argument(
        "--target_realtime_rate", type=float, default=1.0,
        help="Playback speed.  See documentation for " + \
             "Simulator::set_target_realtime_rate() for details.")
    parser.add_argument(
        "test", type=str2bool, default=False,
        help="Disable random initial conditions in test mode.")
    parser.add_argument(
        "setup", default="clutter_clearing",
        help="Manipulation Station setup option.")
    args = parser.parse_args()

    # Create the manipulation station.

    # Connect relevant ports to the visualizer.

    # Construct camera images to LCM messages systems.

    # Construct the simulator.
    simulator = Simulator(diagram)
    station_context = diagram.GetMutableSubsystemContext(
        station, simulator.get_mutable_context());

    # Position command should hold the arm at the initial state.
    Eigen::VectorXd q0 = station->GetIiwaPosition(station_context);
    station_context.FixInputPort(
      station->GetInputPort("iiwa_position").get_index(), q0);

     # Zero feed-forward torque.
    station_context.FixInputPort(
        station->GetInputPort("iiwa_feedforward_torque").get_index(),
        VectorXd::Zero(station->num_iiwa_joints()));

    # Nominal WSG position is open.
    station_context.FixInputPort(
        station->GetInputPort("wsg_position").get_index(), Vector1d(0.1));
    # Force limit at 40N.
    station_context.FixInputPort(
        station->GetInputPort("wsg_force_limit").get_index(), Vector1d(40.0));

    if not args.test:
        std::random_device rd;
        RandomGenerator generator{rd()};
        diagram.SetRandomContext(&simulator.get_mutable_context(), &generator);

    simulator.set_target_realtime_rate(args.target_realtime_rate);
    simulator.StepTo(args.duration);

    # Check that the arm is (very roughly) in the commanded position.
  VectorXd q = station->GetIiwaPosition(station_context);
  if (!is_approx_equal_abstol(q, q0, 1.e-3)) {
    std::cout << "q is not sufficiently close to q0.\n";
    std::cout << "q - q0  = " << (q - q0).transpose() << "\n";
    return -1



if __name__ == "__main__"
    main()
