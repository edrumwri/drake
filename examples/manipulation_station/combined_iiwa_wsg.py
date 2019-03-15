from pydrake.all import ()

'''
Builds models and controllers for the combined Iiwa manipulator and Schunk
WSG gripper.
'''

# Returns a spatial inertia matrix that models the entire gripper as a single
# rigid body.
def MakeCompositeGripperInertia():

# Returns a MultibodyPlant.
def MakeIiwaModelForControl():

# Gets an initial configuration for the robot and the gripper.
def get_initial_configuration(scenario_type):



# TODO: Should the below method stlil exist, or should it be a non-class method.
def SetDefaultState(self):

