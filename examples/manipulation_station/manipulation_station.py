from pydrake.all import (LeafSystem)

class ManipulationStation(LeafSystem):
    def __init__(self):

    def GetManipulatorPosition(self):

    def GetManipulatorVelocity(self):

    def SetManipulatorPosition(self, q):

    def SetManipulatorVelocity(self, v):

    def GetGripperPosition(self):

    def GetGripperVelocity(self):

    def SetGripperPosition(self, q):

    def SetGripperVelocity(self, v):

    def get_gripper_force_limit_input_port(self):

    def get_manipulator_added_torque_input_port(self):

    def get_manipulator_commanded_position_input_port(self):

    def get_gripper_commanded_position_input_port(self):

    def get_manipulator_position_output_port(self):

    def get_manipulator_velocity_output_port(self):

    def get_gripper_position_output_port(self):

    def get_gripper_velocity_output_port(self):

    def get_manipulator_desired_position_input_port(self):

    def get_gripper_desired_position_input_port(self):


def SetupDefaultStation(self):
    # Add the table and 80/20 workcell frame.

    # Add the cupboard.

    # Add the object.

    # TODO: Add the robot models.

    # Add default cameras.

# TODO: Should the below method stlil exist, or should it be a non-class method.
def SetDefaultState(self):

