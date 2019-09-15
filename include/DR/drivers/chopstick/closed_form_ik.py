# Script for computing the closed form solution to inverse kinematics for a robot with exactly as many DoF as the
# dimension of the task space.

# Run using:
# user:~$ python2.7 closed_form_ik.py

# Solutions for both left and right chopsticks will be output to the console.

from sympy import *
import math

# Constructs a transformation from a revolute joint.
def X_revolute(axis, symbol):
    # Ensure that the axis is normalized.
    value = axis[0]**2 + axis[1]**2 + axis[2]**2 - 1
    assert(abs(value) < 1e-8)

    # From https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
    c = cos(symbol)
    s = sin(symbol)
    t = 1 - c
    x = axis[0]
    y = axis[1]
    z = axis[2]
    return Matrix(([t*x*x + c, t*x*y - z*s, t*x*z + y*s, 0], [t*x*y + z*s, t*y*y + c, t*y*z - x*s, 0],
           [t*x*z - y*s, t*y*z + x*s, t*z*z + c, 0], [0, 0, 0, 1]))

# Constructs a transformation from a prismatic joint.
def X_prismatic(axis, symbol):
    return Matrix(([1, 0, 0, axis[0] * symbol], [0, 1, 0, axis[1] * symbol], [0, 0, 1, axis[2] * symbol], [0, 0, 0, 1]))

# Constructs an offset transformation.
def X_offset(offset):
    return Matrix(([1, 0, 0, offset[0]], [0, 1, 0, offset[1]], [0, 0, 1, offset[2]], [0, 0, 0, 1]))

# Constructs the left chopsticks model.
def construct_left_chopstick():
    x, y, z, pitch, yaw, roll, p_FG1, p_FG2, p_FG3 = symbols('x y z pitch yaw roll p_FG1 p_FG2 p_FG3')

    # First frame: link zero to base.
    X_b0 = X_offset(Matrix(([0, 0, 0])))

    # Second frame: link one to link zero (transform induced by joint 1).
    X_01 = X_prismatic(Matrix(([0, -1, 0])), y)

    # Third frame: link two to link one (transform induced by joint 2).
    X_12 = X_prismatic(Matrix(([1, 0, 0])), x)

    # Fourth frame: link three to link two (transform induced by joint 3).
    X_23 = X_revolute(Matrix(([0, 0, 1])), yaw)

    # Fifth frame: link four to link three (transform induced by joint 4).
    X_34 = X_prismatic(Matrix(([0, 0, 1])), z)

    # Sixth frame: link five to link four (transform induced by joint 5).
    X_45 = X_revolute(Matrix(([0, -1, 0])), pitch)

    # Sixth frame (prime): link 5 prime to link five. Note that this is a dummy joint introduced to
    # permit solving the nonlinear system of equations.
    X_55prime = X_revolute(Matrix(([1, 0, 0])), roll)

    # Seventh frame: end effector to link five prime (this is just an offset from the joint).
    X_5primee = X_offset(Matrix(([p_FG1, p_FG2 - 0.09375, p_FG3])))

    # Compute end-effector frame to base frame.
    X_be = X_b0 * X_01 * X_12 * X_23 * X_34 * X_45 * X_55prime * X_5primee
    return [X_be, [x, y, z, pitch, yaw, roll], X_be.subs([
        (x, 0), (y, 0), (z, 0), (pitch, 0), (roll, 0), (yaw, 0), (p_FG1, 0), (p_FG2, 0), (p_FG3, 0)])]

# Constructs the right chopsticks model.
def construct_right_chopstick():
    x, y, z, pitch, yaw, roll, p_FG1, p_FG2, p_FG3 = symbols('x y z pitch yaw roll p_FG1 p_FG2 p_FG3')

    # First frame: link zero to base.
    X_b0 = X_offset(Matrix(([0, 0, 0])))

    # Second frame: link one to link zero (transform induced by joint 1).
    X_01 = X_prismatic(Matrix(([0, 1, 0])), y)

    # Third frame: link two to link one (transform induced by joint 2).
    X_12 = X_prismatic(Matrix(([1, 0, 0])), x)

    # Fourth frame: link three to link two (transform induced by joint 3).
    X_23 = X_revolute(Matrix(([0, 0, 1])), yaw)

    # Fifth frame: link four to link three (transform induced by joint 4).
    X_34 = X_prismatic(Matrix(([0, 0, 1])), z)

    # Sixth frame: link five to link four (transform induced by joint 5).
    X_45 = X_revolute(Matrix(([0, 1, 0])), pitch)

    # Sixth frame (prime): link 5 prime to link five. Note that this is a dummy joint introduced to
    # permit solving the nonlinear system of equations.
    X_55prime = X_revolute(Matrix(([1, 0, 0])), roll)

    # Seventh frame: end effector to link five prime (this is just an offset from the joint).
    X_5primee = X_offset(Matrix(([p_FG1, p_FG2 + 0.09375, p_FG3])))

    # Compute end-effector frame to base frame.
    X_be = X_b0 * X_01 * X_12 * X_23 * X_34 * X_45 * X_55prime * X_5primee
    return [X_be, [x, y, z, pitch, yaw, roll], X_be.subs([
            (x, 0), (y, 0), (z, 0), (pitch, 0), (roll, 0), (yaw, 0), (p_FG1, 0), (p_FG2, 0), (p_FG3, 0)])]

def OutputForwardKinematics():
    # Outputs the pose from forward kinematics using the given values for q.
    [X_be, vars, unused] = construct_left_chopstick()

    print('left chopstick forward kinematics: ')
    print(str(X_be.subs([(vars[0], 0), (vars[1], 0), (vars[2], 0), (vars[3], 0), (vars[4], 0), (vars[5], 0)])))

    [X_be, vars, unused] = construct_right_chopstick()

    print('left chopstick forward kinematics: ')
    print(str(X_be.subs([(vars[0], 0), (vars[1], 0), (vars[2], 0), (vars[3], 0), (vars[4], 0), (vars[5], 0)])))


def OutputInverseKinematicsSolutions():
    init_printing()

    # Define the symbols for the 4x4 homogeneous transformation.
    r11, r12, r13 = symbols('r11 r12 r13')
    r21, r22, r23 = symbols('r21 r22 r23')
    r31, r32, r33 = symbols('r31 r32 r33')
    x1, x2, x3 = symbols('x1 x2 x3')
    X_target = Matrix(([r11, r12, r13, x1], [r21, r22, r23, x2], [r31, r32, r33, x3], [0, 0, 0, 1]))

    # Construct the symbolic form of the chopstick kinematics.
    [X_be, vars, X_be_zero_config] = construct_left_chopstick()
    sol = solve(X_be - X_target, vars)

    for i in range(len(sol)):
        # Don't output solutions that do not yield zero pitch, roll, or yaw at the identity
        # configuration.
        good_solution = True
        for j in range(len(vars)):
            [p_FG1, p_FG2, p_FG3] = symbols(['p_FG1', 'p_FG2', 'p_FG3'])
            if (str(vars[j]) == 'roll' or str(vars[j]) == 'pitch' or str(vars[j]) == 'yaw') and abs(sol[i][j].subs([
                    (r11, X_be_zero_config[0]), (r12, X_be_zero_config[4]), (r13, X_be_zero_config[8]),
                    (r21, X_be_zero_config[1]), (r22, X_be_zero_config[5]), (r23, X_be_zero_config[9]),
                    (r31, X_be_zero_config[2]), (r32, X_be_zero_config[6]), (r33, X_be_zero_config[10]),
                    (x1, X_be_zero_config[12]), (x2, X_be_zero_config[13]), (x3, X_be_zero_config[14]),
                    (p_FG1, 0), (p_FG2, 0), (p_FG3, 0)]).evalf()) > 1e-8:
                good_solution = False
        if good_solution == False:
            continue

        print 'Solution (left):'
        for j in range(len(vars)):
            print ccode(vars[j]) + ' = ' + ccode(sol[i][j]) + ';'

    [X_be, vars, X_be_zero_config] = construct_right_chopstick()
    sol = solve(X_be - X_target, vars)
    for i in range(len(sol)):
        # Don't output solutions that do not yield zero pitch, roll, or yaw at the identity
        # configuration.
        good_solution = True
        for j in range(len(vars)):
            [p_FG1, p_FG2, p_FG3] = symbols(['p_FG1', 'p_FG2', 'p_FG3'])
            if (str(vars[j]) == 'roll' or str(vars[j]) == 'pitch' or str(vars[j]) == 'yaw') and abs(sol[i][j].subs([
                    (r11, X_be_zero_config[0]), (r12, X_be_zero_config[4]), (r13, X_be_zero_config[8]),
                    (r21, X_be_zero_config[1]), (r22, X_be_zero_config[5]), (r23, X_be_zero_config[9]),
                    (r31, X_be_zero_config[2]), (r32, X_be_zero_config[6]), (r33, X_be_zero_config[10]),
                    (x1, X_be_zero_config[12]), (x2, X_be_zero_config[13]), (x3, X_be_zero_config[14]), (p_FG1, 0),
                    (p_FG2, 0), (p_FG3, 0)]).evalf()) > 1e-8:
                good_solution = False
        if good_solution == False:
            continue

        print 'Solution (right):'
        for j in range(len(vars)):
            print ccode(vars[j]) + ' = ' + ccode(sol[i][j]) + ';'

def main():
    # NOTE: Uncomment the next line to output the forward kinematics.
    #    OutputForwardKinematics()
    OutputInverseKinematicsSolutions()

main()
