# TODO: turn this system into an actual discrete system (estimated time required: 30m)
import math
import scipy.optimize
import numpy as np
import logging
from manipulation_plan import ManipulationPlan
from embedded_box_soccer_sim import EmbeddedSim

from pydrake.all import (LeafSystem, ComputeBasisFromAxis, PortDataType,
BasicVector, MultibodyForces, CreateArrowOutputCalcCallback,
CreateArrowOutputAllocCallback, ArrowVisualization)
from pydrake.solvers import mathematicalprogram

class BoxController(LeafSystem):
  def __init__(self, sim_dt, robot_type, all_plant, robot_plant, mbw, robot_instance, ball_instance, fully_actuated=False, controller_type='BlackBoxDynamics'):
    LeafSystem.__init__(self)

    # Saves whether the entire system will be actuated.
    self.fully_actuated = fully_actuated

    # Save the robot type.
    self.set_name('box_controller')
    self.robot_type = robot_type

    # Construct the plan.
    self.plan = ManipulationPlan()

    # Save the robot and ball instances.
    self.robot_instance = robot_instance
    self.ball_instance = ball_instance

    # Get the plants.
    self.robot_plant = robot_plant
    self.robot_and_ball_plant = all_plant
    self.mbw = mbw

    # Initialize the embedded sim.
    self.embedded_sim = EmbeddedSim(sim_dt)

    # Set the controller type.
    self.controller_type = controller_type

    # Set the output size.
    self.command_output_size = self.robot_and_ball_plant.num_velocities()

    # Create contexts.
    self.mbw_context = mbw.CreateDefaultContext()
    self.robot_context = robot_plant.CreateDefaultContext()
    self.robot_and_ball_context = self.mbw.GetMutableSubsystemContext(
      self.robot_and_ball_plant, self.mbw_context)

      # Gains in Cartesian-land.

    # Set PID gains.
    self.cartesian_kp = np.ones([3, 1]) * 60
    self.cartesian_kd = np.ones([3, 1]) * 30

    # Joint gains for the robot.
    self.num_robot_actuators()
    self.robot_gv_kp = np.ones([self.nv_robot()]) * 10
    self.robot_gv_ki = np.ones([self.nv_robot()]) * 0.1
    self.robot_gv_kd = np.ones([self.nv_robot()]) * 1.0

    # Set the control frequency.
    self.control_freq = 1000.0  # 1000 Hz.

    # Declare states and ports.
    self._DeclareContinuousState(self.nq_robot())   # For integral control state.
    self.input_port_index_estimated_robot_q = self._DeclareInputPort(
        PortDataType.kVectorValued, self.nq_robot()).get_index()
    self.input_port_index_estimated_robot_qd = self._DeclareInputPort(
        PortDataType.kVectorValued, self.nv_robot()).get_index()
    self.input_port_index_estimated_ball_q = self._DeclareInputPort(
        PortDataType.kVectorValued, self.nq_ball()).get_index()
    self.input_port_index_estimated_ball_v = self._DeclareInputPort(
        PortDataType.kVectorValued, self.nv_ball()).get_index()
    self._DeclareVectorOutputPort("command_output",
        BasicVector(self.command_output_size),
        self.DoControlCalc) # Output 0.

    # Get the geometry query input port.
    self.geometry_query_input_port = self.robot_and_ball_plant.get_geometry_query_input_port()

    # Set up ball c.o.m. acceleration visualization.
    self.ball_acceleration_visualization_output_port = self._DeclareAbstractOutputPort(
        "arrow_output",
        CreateArrowOutputAllocCallback(),
        CreateArrowOutputCalcCallback(self.OutputBallAccelerationAsGenericArrow))

    # Actuator limits.
    self.actuator_limit = float('inf')

    # TODO: delete the line below.
    logging.warning('box_controller.py is violating the const System assumption')
    self.ball_accel_from_controller = np.array([0, 0, 0])

  # Gets the value of the integral term in the state.
  def get_integral_value(self, context):
    return context.get_continuous_state_vector().CopyToVector()

  # Sets the value of the integral term in the state.
  def set_integral_value(self, context, qint):
    assert len(qint) == self.nq_robot()
    context.get_mutable_continuous_state_vector().SetFromVector(qint)

  # Gets the ball body from the robot and ball plant.
  def get_ball_from_robot_and_ball_plant(self):
    return self.robot_and_ball_plant.GetBodyByName("ball")

  # Gets the foot links from the robot and ball plant.
  def get_foot_links_from_robot_and_ball_plant(self):
    return [ self.robot_and_ball_plant.GetBodyByName("box") ]

  # Gets the foot links from the robot tree.
  def get_foot_links_from_robot_plant(self):
    return [ self.robot_plant.GetBodyByName("box") ]

  # Gets the world body from the robot and ball tree.
  def get_ground_from_robot_and_ball_plant(self):
    return self.robot_and_ball_plant.GetBodyByName("ground_body")

  def get_input_port_estimated_robot_q(self):
    return self.get_input_port(self.input_port_index_estimated_robot_q)

  def get_input_port_estimated_robot_v(self):
    return self.get_input_port(self.input_port_index_estimated_robot_qd)

  def get_input_port_estimated_ball_q(self):
    return self.get_input_port(self.input_port_index_estimated_ball_q)

  def get_input_port_estimated_ball_v(self):
    return self.get_input_port(self.input_port_index_estimated_ball_v)

  def get_output_port_control(self):
    return self.get_output_port(0)

  # Gets the number of robot actuators.
  def num_robot_actuators(self):
      if self.robot_plant.num_actuators() == 0:
          return self.robot_plant.num_velocities()
      else:
          return self.robot_plant.num_actuators()

  # Gets the number of robot degrees of freedom.
  def nq_robot(self):
    return self.robot_plant.num_positions()

  # Gets the number of robot velocity variables.
  def nv_robot(self):
    return self.robot_plant.num_velocities()

  # Gets the number of ball degrees of freedom.
  def nq_ball(self):
    return self.robot_and_ball_plant.num_positions() - self.nq_robot()

  # Gets the number of ball velocity variables.
  def nv_ball(self):
    return self.robot_and_ball_plant.num_velocities() - self.nv_robot()

  # Gets the robot and ball configuration.
  def get_q_all(self, context):
    qrobot = self.get_q_robot(context)
    qball = self.get_q_ball(context)
    all_q = np.zeros([len(qrobot) + len(qball)])
    self.robot_and_ball_plant.SetPositionsInArray(self.robot_instance, qrobot, all_q)
    self.robot_and_ball_plant.SetPositionsInArray(self.ball_instance, qball, all_q)
    return all_q

  # Gets the robot and ball velocities.
  def get_v_all(self, context):
    vrobot = self.get_v_robot(context)
    vball = self.get_v_ball(context)
    all_v = np.zeros([len(vrobot) + len(vball)])
    self.robot_and_ball_plant.SetVelocitiesInArray(self.robot_instance, vrobot, all_v)
    self.robot_and_ball_plant.SetVelocitiesInArray(self.ball_instance, vball, all_v)
    return all_v

  # Gets the robot configuration.
  def get_q_robot(self, context):
    return self.EvalVectorInput(context, self.get_input_port_estimated_robot_q().get_index()).CopyToVector()

  # Gets the ball configuration.
  def get_q_ball(self, context):
    return self.EvalVectorInput(context, self.get_input_port_estimated_ball_q().get_index()).CopyToVector()

  # Gets the robot velocity.
  def get_v_robot(self, context):
    return self.EvalVectorInput(context, self.get_input_port_estimated_robot_v().get_index()).CopyToVector()

  # Gets the ball velocity
  def get_v_ball(self, context):
    return self.EvalVectorInput(context, self.get_input_port_estimated_ball_v().get_index()).CopyToVector()

  ### "Private" methods below.

  # Debugging function for visualizing the desired ball acceleration using
  # white arrows.
  def OutputBallAccelerationAsGenericArrow(self, controller_context):
    # Get the desired ball acceleration.
    vdot_ball_des = self.plan.GetBallQVAndVdot(controller_context.get_time())[-self.nv_ball():]
    vdot_ball_des = np.reshape(vdot_ball_des, [self.nv_ball(), 1])

    # Get the translational ball acceleration.
    xdd_ball_des = np.reshape(vdot_ball_des[3:6], [-1])

    # Evaluate the ball center-of-mass.
    all_plant = self.robot_and_ball_plant
    ball_body = self.get_ball_from_robot_and_ball_plant()
    X_WB = all_plant.EvalBodyPoseInWorld(self.robot_and_ball_context, ball_body)
    com = X_WB.translation()

    # Populate the arrow visualization data structure.
    arrow_viz = ArrowVisualization()
    arrow_viz.origin_W = com
    arrow_viz.target_W = com + xdd_ball_des
    arrow_viz.color_rgb = np.array([1, 1, 1])  # White.

    # TODO: Delete this.
    # Construct a second one for the computed ball acceleration.
    arrow_viz_2 = ArrowVisualization()
    arrow_viz_2.origin_W = com
    arrow_viz_2.target_W = com + self.ball_accel_from_controller
    arrow_viz_2.color_rgb = np.array([1, 0, 1])
    return [ arrow_viz, arrow_viz_2 ]

    # A list must be returned.
    return [ arrow_viz ]

  # Makes a sorted pair.
  def MakeSortedPair(self, a, b):
    if b > a:
      return (b, a)
    else:
      return (a, b)

  # Loads all plans into the controller.
  def LoadPlans(self, path):
    from pydrake.common import FindResourceOrThrow

    # Set the Drake prefix.
    prefix = 'drake/examples/iiwa_soccer/'

    # Add a '/' to the end of the path if necessary.
    if path[-1] != '/':
      path += '/'

    # Read in the plans for the robot.
    if self.robot_type == 'iiwa':
      self.plan.ReadIiwaRobotQVAndVdot(
        FindResourceOrThrow(prefix + 'plan/joint_timings_fit.mat'),
        FindResourceOrThrow(prefix + 'plan/joint_angle_fit.mat'),
        FindResourceOrThrow(prefix + 'plan/joint_vel_fit.mat'),
        FindResourceOrThrow(prefix + 'plan/joint_accel_fit.mat'))

      # Read in the plans for the point of contact.
      self.plan.ReadContactPoint(
        FindResourceOrThrow(prefix + 'plan/contact_pt_timings.mat'),
        FindResourceOrThrow(prefix + 'plan/contact_pt_positions.mat'),
        FindResourceOrThrow(prefix + 'plan/contact_pt_velocities.mat'))

      # Read in the plans for the ball kinematics.
      self.plan.ReadBallQVAndVdot(
        FindResourceOrThrow(prefix + 'plan/ball_timings.mat'),
        FindResourceOrThrow(prefix + 'plan/ball_com_positions.mat'),
        FindResourceOrThrow(prefix + 'plan/ball_quats.mat'),
        FindResourceOrThrow(prefix + 'plan/ball_com_velocities.mat'),
        FindResourceOrThrow(prefix + 'plan/ball_omegas.mat'),
        FindResourceOrThrow(prefix + 'plan/ball_com_accelerations.mat'),
        FindResourceOrThrow(prefix + 'plan/ball_alphas.mat'),
        FindResourceOrThrow(prefix + 'plan/contact_status.mat'))

    if self.robot_type == 'box':
      self.plan.ReadBoxRobotQVAndVdot(
        FindResourceOrThrow(prefix + path + 'timings.mat'),
        FindResourceOrThrow(prefix + path + 'box_positions.mat'),
        FindResourceOrThrow(prefix + path + 'box_quats.mat'),
        FindResourceOrThrow(prefix + path + 'box_linear_vel.mat'),
        FindResourceOrThrow(prefix + path + 'box_angular_vel.mat'),
        FindResourceOrThrow(prefix + path + 'box_linear_accel.mat'),
        FindResourceOrThrow(prefix + path + 'box_angular_accel.mat'))

      # Read in the plans for the point of contact.
      self.plan.ReadContactPoint(
        FindResourceOrThrow(prefix + path + 'timings.mat'),
        FindResourceOrThrow(prefix + path + 'contact_pt_positions.mat'),
        FindResourceOrThrow(prefix + path + 'contact_pt_velocities.mat'))

      # Read in the plans for the ball kinematics.
      self.plan.ReadBallQVAndVdot(
        FindResourceOrThrow(prefix + path + 'timings.mat'),
        FindResourceOrThrow(prefix + path + 'ball_com_positions.mat'),
        FindResourceOrThrow(prefix + path + 'ball_quats.mat'),
        FindResourceOrThrow(prefix + path + 'ball_com_velocities.mat'),
        FindResourceOrThrow(prefix + path + 'ball_omegas.mat'),
        FindResourceOrThrow(prefix + path + 'ball_com_accelerations.mat'),
        FindResourceOrThrow(prefix + path + 'ball_alphas.mat'),
        FindResourceOrThrow(prefix + path + 'contact_status.mat'))

  # Constructs the Jacobian matrices.
  def ConstructJacobians(self, contacts, q, v):

    # Get the tree.
    all_plant = self.robot_and_ball_plant

    # Get the numbers of contacts and generalized velocities.
    nc = len(contacts)

    # Set the number of generalized velocities.
    nv = all_plant.num_velocities()

    # Size the matrices.
    N = np.empty([nc, nv])
    S = np.empty([nc, nv])
    T = np.empty([nc, nv])
    Ndot_v = np.empty([nc, 1])
    Sdot_v = np.empty([nc, 1])
    Tdot_v = np.empty([nc, 1])

    # Evaluate scene graph's output port, getting a SceneGraph reference.
    self.UpdateRobotAndBallConfigurationForGeometricQueries(q)
    query_object = all_plant.EvalAbstractInput(
      self.robot_and_ball_context, self.geometry_query_input_port.get_index()).get_value()
    inspector = query_object.inspector()

    # Get the two body indices.
    for i in range(nc):
      point_pair = contacts[i]

      # Get the surface normal in the world frame.
      n_BA_W = point_pair.nhat_BA_W

      # Get the two bodies.
      geometry_A_id = point_pair.id_A
      geometry_B_id = point_pair.id_B
      frame_A_id = inspector.GetFrameId(geometry_A_id)
      frame_B_id = inspector.GetFrameId(geometry_B_id)
      body_A = all_plant.GetBodyFromFrameId(frame_A_id)
      body_B = all_plant.GetBodyFromFrameId(frame_B_id)

      # The reported point on A's surface (A) in the world frame (W).
      pr_WA = point_pair.p_WCa

      # The reported point on B's surface (B) in the world frame (W).
      pr_WB = point_pair.p_WCb

      # Get the point of contact in the world frame.
      pc_W = (pr_WA + pr_WB) * 0.5

      # Get the geometric Jacobian for the velocity of the contact point
      # as moving with Body A.
      J_WAc = all_plant.CalcPointsGeometricJacobianExpressedInWorld(
          self.robot_and_ball_context, body_A.body_frame(), pr_WA)

      # Get the geometric Jacobian for the velocity of the contact point
      # as moving with Body B.
      J_WBc = all_plant.CalcPointsGeometricJacobianExpressedInWorld(
          self.robot_and_ball_context, body_B.body_frame(), pr_WB)

      # Compute the linear components of the Jacobian.
      J = J_WAc - J_WBc

      # Compute an orthonormal basis using the contact normal.
      kXAxisIndex = 0
      kYAxisIndex = 1
      kZAxisIndex = 2
      R_WC = ComputeBasisFromAxis(kXAxisIndex, n_BA_W)
      t1_BA_W = R_WC[:,kYAxisIndex]
      t2_BA_W = R_WC[:,kZAxisIndex]

      # Set N, S, and T.
      N[i,:] = n_BA_W.T.dot(J)
      S[i,:] = t1_BA_W.T.dot(J)
      T[i,:] = t2_BA_W.T.dot(J)

      # TODO: Set Ndot_v, Sdot_v, Tdot_v properly.
      Ndot_v *= 0 # = n_BA_W.T * Jdot_v
      Sdot_v *= 0 # = t1_BA_W.T * Jdot_v
      Tdot_v *= 0 # = t2_BA_W.T * Jdot_v

    return [ N, S, T, Ndot_v, Sdot_v, Tdot_v ]

  # Computes the control torques when contact is not desired.
  def ComputeActuationForContactNotDesired(self, context):
    # Get the desired robot acceleration.
    q_robot_des = self.plan.GetRobotQVAndVdot(
        context.get_time())[0:nv_robot()-1]
    qdot_robot_des = self.plan.GetRobotQVAndVdot(
        context.get_time())[nv_robot(), 2*nv_robot()-1]
    qddot_robot_des = self.plan.GetRobotQVAndVdot(
        context.get_time())[-nv_robot():]

    # Get the robot current generalized position and velocity.
    q_robot = get_q_robot(context)
    qd_robot = get_v_robot(context)

    # Set qddot_robot_des using error feedback.
    qddot = qddot_robot_des + np.diag(self.robot_gv_kp).dot(q_robot_des - q_robot) + np.diag(self.robot_gv_ki).diag(get_integral_value(context)) + np.diag(self.robot_gv_kd).dot(qdot_robot_des - qd_robot)

    # Set the state in the robot context to q_robot and qd_robot.
    x = self.robot_plant.tree().get_mutable_multibody_state_vector(
      robot_context)
    assert len(x) == len(q_robot) + len(qd_robot)
    x[0:len(q_robot)-1] = q_robot
    x[-len(qd_robot):] = qd_robot

    # Get the generalized inertia matrix.
    M = self.robot_plant.tree().CalcMassMatrixViaInverseDynamics(robot_context)

    # Compute the contribution from force elements.
    robot_tree = self.robot_plant.tree()
    link_wrenches = MultibodyForces(robot_tree)

    # Compute the external forces.
    fext = -robot_tree.CalcInverseDynamics(
        robot_context, np.zeros([nv_robot(), 1]), link_wrenches)

    # Compute inverse dynamics.
    u = M * qddot - fext
    return u

  # Updates the robot and ball configuration in the relevant context so that
  # geometric queries can be performed in configuration q.
  def UpdateRobotAndBallConfigurationForGeometricQueries(self, q):

    # Set the state.
    self.robot_and_ball_plant.SetPositions(self.robot_and_ball_context, q)

  # Gets the signed distance between the ball and the ground.
  def GetSignedDistanceFromBallToGround(self, context):
    all_plant = self.robot_and_ball_plant

    # Get the ball body and foot bodies.
    ball_body = self.get_ball_from_robot_and_ball_plant()
    world_body = self.get_ground_from_robot_and_ball_plant()

    # Make sorted pair to check.
    ball_world_pair = self.MakeSortedPair(ball_body, world_body)

    # Get the current configuration of the robot and the foot.
    q = self.get_q_all(context)

    # Update the context to use configuration q1 in the query. This will modify
    # the mbw context, used immediately below.
    self.UpdateRobotAndBallConfigurationForGeometricQueries(q)

    # Evaluate scene graph's output port, getting a SceneGraph reference.
    query_object = self.robot_and_ball_plant.EvalAbstractInput(
      self.robot_and_ball_context, self.geometry_query_input_port.get_index()).get_value()
    inspector = query_object.inspector()

    # Get the closest points on the robot foot and the ball corresponding to q1
    # and v0.
    closest_points = query_object.ComputeSignedDistancePairwiseClosestPoints()
    assert len(closest_points) > 0

    dist = 1e20
    for i in range(len(closest_points)):
      geometry_A_id = closest_points[i].id_A
      geometry_B_id = closest_points[i].id_B
      frame_A_id = inspector.GetFrameId(geometry_A_id)
      frame_B_id = inspector.GetFrameId(geometry_B_id)
      body_A = all_plant.GetBodyFromFrameId(frame_A_id)
      body_B = all_plant.GetBodyFromFrameId(frame_B_id)
      body_A_B_pair = self.MakeSortedPair(body_A, body_B)
      if body_A_B_pair != ball_world_pair:
        continue
      dist = min(dist, closest_points[i].distance)

    return dist

  # Gets the signed distance between the ball and the foot.
  def GetSignedDistanceFromRobotToBall(self, context):
    all_plant = self.robot_and_ball_plant

    # Get the ball body and foot bodies.
    ball_body = self.get_ball_from_robot_and_ball_plant()
    foot_bodies = self.get_foot_links_from_robot_and_ball_plant()

    # Make sorted pairs to check.
    ball_foot_pairs = [0] * len(foot_bodies)
    for i in range(len(foot_bodies)):
      ball_foot_pairs[i] = self.MakeSortedPair(ball_body, foot_bodies[i])

    # Get the current configuration of the robot and the foot.
    q = self.get_q_all(context)

    # Update the context to use configuration q1 in the query. This will modify
    # the mbw context, used immediately below.
    self.UpdateRobotAndBallConfigurationForGeometricQueries(q)

    # Evaluate scene graph's output port, getting a SceneGraph reference.
    query_object = self.robot_and_ball_plant.EvalAbstractInput(
      self.robot_and_ball_context, self.geometry_query_input_port.get_index()).get_value()
    inspector = query_object.inspector()

    # Get the closest points on the robot foot and the ball corresponding to q1
    # and v0.
    closest_points = query_object.ComputeSignedDistancePairwiseClosestPoints()
    assert len(closest_points) > 0

    dist = 1e20
    for i in range(len(closest_points)):
      geometry_A_id = closest_points[i].id_A
      geometry_B_id = closest_points[i].id_B
      frame_A_id = inspector.GetFrameId(geometry_A_id)
      frame_B_id = inspector.GetFrameId(geometry_B_id)
      body_A = all_plant.GetBodyFromFrameId(frame_A_id)
      body_B = all_plant.GetBodyFromFrameId(frame_B_id)
      body_A_B_pair = self.MakeSortedPair(body_A, body_B)
      if body_A_B_pair not in ball_foot_pairs:
        continue
      dist = min(dist, closest_points[i].distance)

    return dist

  # Computes the control torques when contact is desired and the robot and the
  # ball are *not* in contact.
  def ComputeActuationForContactDesiredButNoContact(self, controller_context):
    # Get the relevant trees.
    all_plant = self.robot_and_ball_plant
    robot_plant = self.robot_plant

    # Get the generalized positions and velocities for the robot and the ball.
    q0 = self.get_q_all(controller_context)
    v0 = self.get_v_all(controller_context)

    # Set the state in the "all plant" context.
    all_plant.SetPositions(self.robot_and_ball_context, q0)
    all_plant.SetVelocities(self.robot_and_ball_context, v0)

    # Set the joint velocities for the robot to zero.
    self.robot_and_ball_plant.SetVelocities(self.robot_and_ball_context, self.robot_instance, np.zeros([self.nv_robot()]))

    # Transform the velocities to time derivatives of generalized
    # coordinates.
    qdot0 = self.robot_and_ball_plant.MapVelocityToQDot(self.robot_and_ball_context, v0)
    dt = 1.0/self.control_freq

    # Get the estimated position of the ball and the robot at the next time
    # step using a first order approximation to position and the current
    # velocities.
    q1 = q0 + dt * qdot0

    # Update the context to use configuration q1 in the query. This will modify
    # the mbw context, used immediately below.
    self.UpdateRobotAndBallConfigurationForGeometricQueries(q1)

    # Evaluate scene graph's output port, getting a SceneGraph reference.
    query_object = self.robot_and_ball_plant.EvalAbstractInput(
        self.robot_and_ball_context, self.geometry_query_input_port.get_index()).get_value()
    inspector = query_object.inspector()

    # Get the robot and the ball bodies.
    ball_body = self.get_ball_from_robot_and_ball_plant()
    foot_bodies = self.get_foot_links_from_robot_and_ball_plant()
    foots_and_ball = [0] * len(foot_bodies)
    for i in range(len(foot_bodies)):
      foots_and_ball[i] = self.MakeSortedPair(ball_body, foot_bodies[i])

    # Get the closest points on the robot foot and the ball corresponding to q1
    # and v0.
    closest_points = query_object.ComputeSignedDistancePairwiseClosestPoints()
    assert len(closest_points) > 0
    found_index = -1
    for i in range(len(closest_points)):
      # Get the two bodies in contact.
      point_pair = closest_points[i]
      geometry_A_id = point_pair.id_A
      geometry_B_id = point_pair.id_B
      frame_A_id = inspector.GetFrameId(geometry_A_id)
      frame_B_id = inspector.GetFrameId(geometry_B_id)
      body_A = all_plant.GetBodyFromFrameId(frame_A_id)
      body_B = all_plant.GetBodyFromFrameId(frame_B_id)
      bodies = self.MakeSortedPair(body_A, body_B)

      # If the two bodies correspond to the foot and the ball, mark the
      # found index and stop looping.
      if bodies in foots_and_ball:
        found_index = i
        break

    # Get the signed distance data structure.
    assert found_index >= 0
    closest = closest_points[found_index]

    # Make A be the body belonging to the robot.
    geometry_A_id = closest.id_A
    geometry_B_id = closest.id_B
    frame_A_id = inspector.GetFrameId(geometry_A_id)
    frame_B_id = inspector.GetFrameId(geometry_B_id)
    body_A = all_plant.GetBodyFromFrameId(frame_A_id)
    body_B = all_plant.GetBodyFromFrameId(frame_B_id)
    if body_B != ball_body:
      # Swap A and B.
      body_A, body_B = body_B, body_A
      closest.id_A, closest.id_B = closest.id_B, closest.id_A
      closest.p_ACa, closest.p_BCb = closest.p_BCb, closest.p_ACa

    # Get the closest points on the bodies. They'll be in their respective body
    # frames.
    closest_Aa = closest.p_ACa
    closest_Bb = closest.p_BCb

    # Transform the points in the body frames corresponding to q1 to the
    # world frame.
    X_wa = all_plant.EvalBodyPoseInWorld(self.robot_and_ball_context, body_A)
    X_wb = all_plant.EvalBodyPoseInWorld(self.robot_and_ball_context, body_B)
    closest_Aw = X_wa.multiply(closest_Aa)
    closest_Bw = X_wb.multiply(closest_Bb)

    # Get the vector from the closest point on the foot to the closest point
    # on the ball in the body frames.
    linear_v_des = (closest_Bw - closest_Aw) / dt

    # Get the robot current generalized position and velocity.
    q_robot = self.get_q_robot(controller_context)
    v_robot = self.get_v_robot(controller_context)

    # Set the state in the robot context to q_robot and qd_robot.
    x = robot_plant.GetMutablePositionsAndVelocities(self.robot_context)
    assert len(x) == len(q_robot) + len(v_robot)
    x[0:len(q_robot)] = q_robot
    x[-len(v_robot):] = v_robot

    # Get the geometric Jacobian for the velocity of the closest point on the
    # robot as moving with the robot Body A.
    foot_bodies_in_robot_plant = self.get_foot_links_from_robot_plant()
    for body in foot_bodies_in_robot_plant:
      if body.name() == body_A.name():
        foot_body_to_use = body
    J_WAc = self.robot_plant.CalcPointsGeometricJacobianExpressedInWorld(
        self.robot_context, foot_body_to_use.body_frame(), closest_Aw)
    q_robot_des = q_robot

    # Use resolved-motion rate control to determine the robot velocity that
    # would be necessary to realize the desired end-effector velocity.
    v_robot_des, residuals, rank, singular_values = np.linalg.lstsq(J_WAc, linear_v_des)
    dvdot = np.reshape(v_robot_des - v_robot, (-1, 1))

    # Set vdot_robot_des using purely error feedback.
    if self.nq_robot() != self.nv_robot():
      # Since the coordinates and velocities are different, we convert the
      # difference in configuration to a difference in velocity coordinates.
      # TODO: Explain why this is allowable.
      self.robot_plant.SetPositions(self.robot_context, q_robot)
      dv = np.reshape(self.robot_plant.MapQDotToVelocity(self.robot_context, q_robot_des - q_robot), (-1, 1))
      vdot = np.diag(np.reshape(self.robot_gv_kp, (-1))).dot(dv) + np.diag(np.reshape(self.robot_gv_kd, (-1))).dot(dvdot)
    else:
      vdot = np.diag(self.robot_gv_kp).dot(q_robot_des - q_robot) + np.diag(self.robot_gv_kd).dot(np.reshape(v_robot_des - v_robot), (-1, 1))

    # Get the generalized inertia matrix.
    M = robot_plant.CalcMassMatrixViaInverseDynamics(self.robot_context)

    # Compute the contribution from force elements.
    link_wrenches = MultibodyForces(robot_plant)

    # Compute the external forces.
    fext = np.reshape(-robot_plant.CalcInverseDynamics(
        self.robot_context, np.zeros([self.nv_robot()]), link_wrenches), (-1, 1))

    # Compute inverse dynamics.
    return M.dot(vdot) - fext

  # Constructs the robot actuation matrix.
  def ConstructRobotActuationMatrix(self):
    # We assume that each degree of freedom is actuatable. There is no way to
    # verify this because we want to be able to use free bodies as "robots" too.

    # First zero out the generalized velocities for the whole multibody.
    v = np.zeros([self.nv_robot() + self.nv_ball()])

    # Now set the velocities in the generalized velocity array to ones.
    ones_nv_robot = np.ones([self.nv_robot()])
    self.robot_and_ball_plant.SetVelocitiesInArray(self.robot_instance, ones_nv_robot, v)

    # The matrix is of size nv_robot() + nv_ball() x nv_robot().
    # Only the robot is actuated.
    B = np.zeros([self.nv_robot() + self.nv_ball(), self.nv_robot()])
    col_index = 0
    for i in range(self.nv_robot() + self.nv_ball()):
        if abs(v[i]) > 0.5:
          B[i, col_index] = 1
          col_index += 1

    return B

  # Constructs the matrix that zeros angular velocities for the ball (and
  # does not change the linear velocities).
  def ConstructBallVelocityWeightingMatrix(self):
    # Get the indices of generalized velocity that correspond to the ball.
    nv = self.nv_ball() + self.nv_robot()
    v = np.zeros([nv])
    dummy_ball_v = np.array([0, 0, 0, 1, 1, 1])
    self.robot_and_ball_plant.SetVelocitiesInArray(self.ball_instance, dummy_ball_v, v)

    # Set the velocities weighting.
    P = np.zeros([3, nv])
    vball_index = 0
    for i in range(nv):
      if abs(v[i]) > 0.5:
        P[vball_index, i] = v[i]
        vball_index += 1

    return P

  # Computes the motor torques for ComputeActuationForContactDesiredAndContacting() using the no-separation contact
  # model. Specifically, this function optimizes:
  # argmin vdot_sub_des 1/2 * (vdot_sub_des - P * vdot)' * W *
  #                             (vdot_sub_des - P * vdot)
  # subject to:
  # Gn * vdot_sub_des + dotGn * v_sub_des >= 0
  # M * vdot = fext + Gn' * fn + Gs' * fs + Gt' * ft + B * u
  # fn >= 0
  #
  # and preconditions:
  # Nv = 0
  #
  # iM: the inverse of the joint robot/ball generalized inertia matrix
  # fext: the generalized external forces acting on the robot/ball
  # vdot_ball_des: the desired spatial acceleration on the ball
  # Z: the contact normal/tan1/tan2 Jacobian matrix (all normal rows come first, all first-tangent direction rows come
  #    next, all second-tangent direction rows come last). In the notation above, Z = [ Gn; Gs; Gt ].
  # N: the contact normal Jacobian matrix.
  # Ndot_v: the time derivative of the normal contact Jacobian matrix times the generalized velocities.
  #
  # Returns: a tuple containing (1) the actuation forces (u), (2) the contact force magnitudes fz (along the contact
  # normals, the first tangent direction, and the second contact tangent direction, respectively),
  def ComputeContactControlMotorTorquesNoSeparation(self, iM, fext, vdot_ball_des, Z, N, Ndot_v):
    # Construct the actuation and weighting matrices.
    B = self.ConstructRobotActuationMatrix()
    P = self.ConstructBallVelocityWeightingMatrix()

    # Primal variables are motor torques and contact force magnitudes.
    nv, nu = B.shape
    ncontact_variables = Z.shape[0]
    nc = N.shape[0]
    nprimal = nu + ncontact_variables

    # Construct the matrices necessary to construct the Hessian.
    Z_rows, Z_cols = Z.shape
    D = np.zeros([nv, nprimal])
    D[0:nv, 0:nu] = B
    D[-Z_cols:, -Z_rows:] = Z.T

    # Set the Hessian matrix for the QP.
    H = D.T.dot(iM.dot(P.T).dot(P.dot(iM.dot(D))))

    # Compute the linear terms.
    c = D.T.dot(iM.dot(P.T).dot(-vdot_ball_des + P.dot(iM.dot(fext))))

    # Set the affine constraint matrix.
    A = N.dot(iM.dot(D))
    b = -N.dot(iM.dot(fext)) - np.reshape(Ndot_v, (-1, 1))

    # Formulate the QP.
    prog = mathematicalprogram.MathematicalProgram()
    vars = prog.NewContinuousVariables(len(c), "vars")
    prog.AddQuadraticCost(H, c, vars)
    prog.AddLinearConstraint(A, b, b, vars)

    # Add a compressive-force constraint on the normal contact forces.
    lb = np.zeros(nprimal)
    lb[0:nu] = np.ones(nu) * -float('inf')
    lb[nu + nc:] = np.ones(nc * 2) * -float('inf')
    ub = np.ones(nprimal) * float('inf')
    prog.AddBoundingBoxConstraint(lb, ub, vars)

    # Solve the QP.
    result = prog.Solve()
    assert result == mathematicalprogram.SolutionResult.kSolutionFound
    z = prog.GetSolution(vars)

    # Get the actuation forces and the contact forces.
    u = np.reshape(z[0:nu], [-1, 1])
    fz = np.reshape(z[nu:nprimal], [-1, 1])

    # Get the normal forces and ensure that they are not tensile.
    fz_n = fz[0:nc]
    assert np.min(fz_n) >= -1e-8

    # Verify that the normal acceleration constraint is met.
    vdot = iM.dot(Z.T.dot(fz) + B.dot(u) + fext)
    assert np.linalg.norm(N.dot(vdot) + Ndot_v) < 1e-2
    logging.info('Objective function value: ' + str(np.linalg.norm(P.dot(vdot) - vdot_ball_des)))

    # Determine the friction coefficient for each point of contact.
    for i in range(nc):
      fs = fz[i+nc]
      ft = fz[i+nc*2]
      tan_force = math.sqrt(fs*fs + ft*ft)
      logging.info('Forces for contact ' + str(i) + ' normal: ' + str(fz[i]) + '  tangent: ' + str(tan_force))
      if tan_force < 1e-8:
          logging.info('Friction coefficient for contact ' + str(i) + ' unknown')
      else:
          logging.info('Friction coefficient for contact ' + str(i) + ' = ' + str(tan_force/fz[i]))

    return [u, fz]

  # Computes the motor torques for ComputeActuationForContactDesiredAndContacting() that minimize deviation from the
  # desired acceleration using no dynamics information and a gradient-based optimization strategy.
  # This controller uses the simulator to compute contact forces, rather than attempting to predict the contact forces
  # that the simulator will generate.
  def ComputeOptimalContactControlMotorTorques(self, controller_context, q, v, vdot_ball_des):

      P = self.ConstructBallVelocityWeightingMatrix()
      nu = self.ConstructRobotActuationMatrix().shape[1]

      # Get the current system positions and velocities.
      nv = len(v)

      # The objective function.
      def objective_function(u):
        vdot_approx = self.ComputeApproximateAcceleration(controller_context, q, v, u)
        delta = P.dot(vdot_approx) - vdot_ball_des
        return np.linalg.norm(delta)

      result = scipy.optimize.minimize(objective_function, np.random.normal(np.zeros([nu])))
      logging.info('scipy.optimize success? ' + str(result.success))
      logging.info('scipy.optimize message: ' +  result.message)
      logging.info('scipy.optimize result: ' + str(result.x))
      u_best = result.x

      return u_best

  # Computes the motor torques for ComputeActuationForContactDesiredAndContacting()
  # using the no-slip contact model. Specifically, this function optimizes:
  # argmin vdot_sub_des 1/2 * (vdot_sub_des - P * vdot)' * W *
  #                             (vdot_sub_des - P * vdot)
  # subject to:
  # Gn * vdot_sub_des + dotGn * v_sub_des >= 0
  # Gs * vdot_sub_des + dotGs * v_sub_des = 0
  # Gt * vdot_sub_des + dotGt * v_sub_des = 0
  # M * vdot = fext + Gn' * fn + Gs' * fs + Gt' * ft + B * u
  # fn >= 0
  #
  # and preconditions:
  # Nv = 0
  #
  # iM: the inverse of the joint robot/ball generalized inertia matrix
  # fext: the generalized external forces acting on the robot/ball
  # vdot_ball_des: the desired spatial acceleration on the ball
  # Z: the contact normal/tan1/tan2 Jacobian matrix (all normal rows come first, all first-tangent direction rows come
  # next, all second-tangent direction rows come last). In the notation above, Z = [ Gn; Gs; Gt ].
  # Zdot_v: the time derivative of the contact Jacobian matrices times the generalized velocities. In the notation
  # above, Zdot = [ dotGn; dotGs; dotGt ].
  #
  # Returns: a tuple containing (1) the actuation forces (u), (2) the contact force magnitudes fz (along the contact
  # normals, the first tangent direction, and the second contact tangent direction, respectively),
  def ComputeContactControlMotorTorquesNoSlip(self, iM, fext, vdot_ball_des, Z, Zdot_v, N, Ndot_v, S_ground, Sdot_v_ground, T_ground, Tdot_v_ground):
    # Construct the actuation and weighting matrices.
    B = self.ConstructRobotActuationMatrix()
    P = self.ConstructBallVelocityWeightingMatrix()

    # Primal variables are motor torques and contact force magnitudes.
    nv, nu = B.shape
    ncontact_variables = len(Zdot_v)
    nc = ncontact_variables/3
    nprimal = nu + ncontact_variables

    # Construct the matrices necessary to construct the Hessian.
    Z_rows, Z_cols = Z.shape
    D = np.zeros([nv, nprimal])
    D[0:nv, 0:nu] = B
    D[-Z_cols:, -Z_rows:] = Z.T

    # Set the Hessian matrix for the QP.
    H = D.T.dot(iM.dot(P.T).dot(P.dot(iM.dot(D))))

    # Compute the linear terms.
    c = D.T.dot(iM.dot(P.T).dot(-vdot_ball_des + P.dot(iM.dot(fext))))

    # Set the affine constraint matrix. There are nc + 1 constraints:
    # normal accelerations are zero (nc constraints),
    # no slip at the ground (2 constraints).
    A = np.zeros([nc + 2, nprimal])
    A[0:nc, :] = N.dot(iM.dot(D))
    A[nc, :] = S_ground.dot(iM.dot(D))
    A[nc+1, :] = T_ground.dot(iM.dot(D))
    b = np.zeros([nc+2, 1])
    b[0:nc] = -N.dot(iM.dot(fext)) - np.reshape(Ndot_v, (-1, 1))
    b[nc] = -S_ground.dot(iM.dot(fext)) - np.reshape(Sdot_v_ground, (-1, 1))
    b[nc+1] = -T_ground.dot(iM.dot(fext)) - np.reshape(Tdot_v_ground, (-1, 1))

    # Add a compressive-force constraint on the normal contact forces.
    prog = mathematicalprogram.MathematicalProgram()
    vars = prog.NewContinuousVariables(len(c), "vars")
    lb = np.zeros(nprimal)
    lb[0:nu] = np.ones(nu) * -float('inf')
    lb[nu + nc:] = np.ones(nc * 2) * -float('inf')
    ub = np.ones(nprimal) * float('inf')
    prog.AddBoundingBoxConstraint(lb, ub, vars)

    # Solve the QP.
    prog.AddQuadraticCost(H, c, vars)
    prog.AddLinearConstraint(A, b, np.ones([len(b), 1]) * 1e8, vars)
    result = prog.Solve()
    assert result == mathematicalprogram.SolutionResult.kSolutionFound
    z = prog.GetSolution(vars)

    # Get the actuation forces and the contact forces.
    u = np.reshape(z[0:nu], [-1, 1])
    fz = np.reshape(z[nu:nprimal], [-1, 1])

    # Determine the friction coefficient for each point of contact.
    for i in range(nc):
        fs = fz[i+nc]
        ft = fz[i+nc*2]
        tan_force = math.sqrt(fs*fs + ft*ft)
        logging.info('Forces for contact ' + str(i) + ' normal: ' + str(fz[i]) + '  tangent: ' + str(tan_force))
        if tan_force < 1e-8:
            logging.info('Friction coefficient for contact ' + str(i) + ' unknown')
        else:
            logging.info('Friction coefficient for contact ' + str(i) + ' = ' + str(tan_force/fz[i]))

    # Get the normal forces and ensure that they are not tensile.
    f_contact_n = fz[0:nc]
    assert np.min(f_contact_n) >= -1e-8

    return [u, fz]

  # Computes the approximate acceleration from a q, a v, and a u.
  def ComputeApproximateAcceleration(self, controller_context, q, v, u, dt=-1):
    # Update the step size, if necessary.
    if dt > 0:
      self.embedded_sim.delta_t = dt

    # Update the state in the embedded simulation.
    self.embedded_sim.UpdateTime(controller_context.get_time())
    self.embedded_sim.UpdatePlantPositions(q)
    self.embedded_sim.UpdatePlantVelocities(v)

    # Apply the controls to the embedded simulation and step the simulation
    # forward in time.
    B = self.ConstructRobotActuationMatrix()
    self.embedded_sim.ApplyControls(B.dot(u))
    self.embedded_sim.Step()

    # Get the new system velocity.
    vnew = self.embedded_sim.GetPlantVelocities()

    # Compute the estimated acceleration.
    return np.reshape((vnew - v) / self.embedded_sim.delta_t, (-1, 1))

  # Computes the motor torques for ComputeActuationForContactDesiredAndContacting()
  # using a "learned" dynamics model.
  # See ComputeContactControlMotorTorquesNoSlip() for description of parameters.
  def ComputeContactControlMotorTorquesUsingLearnedDynamics(self, controller_context, M, fext, vdot_ball_des, Z, Zdot_v):
    # Get the actuation matrix.
    B = self.ConstructRobotActuationMatrix()

    # Get the current system positions and velocities.
    q = self.get_q_all(controller_context)
    v = self.get_v_all(controller_context)
    nv = len(v)

    # Compute inverse(M)
    iM = np.linalg.inv(M)

    # Check N*v, S*v, T*v.
    logging.debug('Z*v: ' + str(Z.dot(v)))

    # Computes the reality gap.
    def reality_gap(epsilon):
      epsilon = np.reshape(epsilon, [-1, 1])

      # Call the no slip controller.
      [u, fz] = self.ComputeContactControlMotorTorquesNoSlip(iM, fext + epsilon, vdot_ball_des, Z, Zdot_v)

      # Make u and fz column vectors.
      fz = np.reshape(fz, [-1, 1])
      u = np.reshape(u, [-1, 1])

      # Compute the approximate acceleration.
      vdot_approx = self.ComputeApproximateAcceleration(controller_context, q, v, u)

      # Compute the difference between the dynamics computed by the no slip controller and the true dynamics.
      # The dynamics will be:
      # M * vdot = fext + Gn' * fn + Gs' * fs + Gt' * ft + B * u + epsilon.
      # The "reality gap" is the unaccounted for force.
      delta = M.dot(vdot_approx) - fext - Z.T.dot(fz) - B.dot(u) - epsilon
      logging.debug('reality gap: ' + str(np.reshape(delta, -1)))
      return np.reshape(delta, -1)

    # Attempt to solve the nonlinear system of equations.
    epsilon = np.reshape(scipy.optimize.fsolve(reality_gap, np.zeros([nv, 1])), [-1, 1])

    # Call the no slip controller.
    [u, fz] = self.ComputeContactControlMotorTorquesNoSlip(iM, fext + epsilon, vdot_ball_des, Z, Zdot_v)

    # Compute the estimated acceleration.
    vdot_approx = self.ComputeApproximateAcceleration(controller_context, q, v, u)

    logging.warning('Residual error norm: ' + str(np.linalg.norm(reality_gap(epsilon))))
    logging.debug('External forces and actuator forces: ' + str(-fext - B.dot(np.reshape(u, (-1, 1)))))
    logging.debug('Spatial contact force acting at the center-of-mass of the robot: ' + str(self.robot_and_ball_plant.GetVelocitiesFromArray(self.robot_instance, Z.T.dot(fz))))
    logging.debug('Spatial contact force acting at the center-of-mass of the ball: ' + str(self.robot_and_ball_plant.GetVelocitiesFromArray(self.ball_instance, Z.T.dot(fz))))
    logging.debug('Unmodeled forces: ' + str(epsilon))
    logging.debug('Forces on the ball: ' + str(self.robot_and_ball_plant.GetVelocitiesFromArray(self.ball_instance, -fext - B.dot(np.reshape(u, (-1, 1))) - epsilon)[-3:]))
    logging.debug('desired ball acceleration: ' + str(vdot_ball_des.T))
    logging.debug('ball acceleration from vdot_approx: ' + str(self.robot_and_ball_plant.GetVelocitiesFromArray(self.ball_instance, vdot_approx)))

    self.ball_accel_from_controller = self.robot_and_ball_plant.GetVelocitiesFromArray(self.ball_instance, vdot_approx)[-3:]

    return [u, fz]

  # Gets the index of contact between the ball and the ground.
  def GetBallGroundContactIndex(self, q, contacts):
     # Get the ball and ground bodies.
    ball_body = self.get_ball_from_robot_and_ball_plant()
    world_body = self.get_ground_from_robot_and_ball_plant()

    # Ensure that the context is set correctly.
    all_plant = self.robot_and_ball_plant
    all_context = self.robot_and_ball_context
    all_plant.SetPositions(all_context, q)

    # Evaluate scene graph's output port, getting a SceneGraph reference.
    query_object = self.robot_and_ball_plant.EvalAbstractInput(
      self.robot_and_ball_context, self.geometry_query_input_port.get_index()).get_value()
    inspector = query_object.inspector()

    # Get the tree corresponding to all bodies.
    all_plant = self.robot_and_ball_plant

    # Get the desired contact.
    ball_ground_contact_index = -1
    for i in range(len(contacts)):
      geometry_A_id = contacts[i].id_A
      geometry_B_id = contacts[i].id_B
      frame_A_id = inspector.GetFrameId(geometry_A_id)
      frame_B_id = inspector.GetFrameId(geometry_B_id)
      body_A = all_plant.GetBodyFromFrameId(frame_A_id)
      body_B = all_plant.GetBodyFromFrameId(frame_B_id)
      body_A_B_pair = self.MakeSortedPair(body_A, body_B)
      if body_A_B_pair == self.MakeSortedPair(ball_body, world_body):
        assert ball_ground_contact_index == -1
        ball_ground_contact_index = i

    return i
 
  # Computes the control torques when contact is desired and the robot and the
  # ball are in contact.
  def ComputeActuationForContactDesiredAndContacting(self, controller_context, contacts):
    # Alias the plant and its context.
    all_plant = self.robot_and_ball_plant
    all_context = self.robot_and_ball_context

    # Get the number of generalized positions and velocities.
    nv = all_plant.num_velocities()
    assert nv == self.nv_robot() + self.nv_ball()

    # Get the generalized positions and velocities.
    q = self.get_q_all(controller_context)
    v = self.get_v_all(controller_context)

    # Set the state in the "all plant" context.
    all_plant.SetPositions(all_context, q)
    all_plant.SetVelocities(all_context, v)

    # Get the generalized inertia matrix of the ball/robot system and compute
    # its Cholesky factorization.
    M = all_plant.CalcMassMatrixViaInverseDynamics(all_context)
    iM = np.linalg.inv(M)

    # Compute the contribution from force elements.
    link_wrenches = MultibodyForces(all_plant)
    all_plant.CalcForceElementsContribution(all_context, link_wrenches)

    # Compute the external forces.
    fext = -all_plant.CalcInverseDynamics(all_context, np.zeros([len(v)]), link_wrenches)
    fext = np.reshape(fext, [len(v), 1])

    # Get the desired ball *linear* acceleration.
    vdot_ball_des = self.plan.GetBallQVAndVdot(controller_context.get_time())[-3:]
    vdot_ball_des = np.reshape(vdot_ball_des, [-1, 1])

    # Get the Jacobians at the point of contact: N, S, T, and construct Z and
    # Zdot_v.
    nc = len(contacts)
    N, S, T, Ndot_v, Sdot_v, Tdot_v = self.ConstructJacobians(contacts, q, v)
    Z = np.zeros([N.shape[0] * 3, N.shape[1]])
    Z[0:nc,:] = N
    Z[nc:2*nc,:] = S
    Z[-nc:,:] = T

    # Get the Jacobians for ball/ground contact.
    ball_ground_contact_index = self.GetBallGroundContactIndex(q, contacts)
    S_ground = S[ball_ground_contact_index, :]
    T_ground = T[ball_ground_contact_index, :]
    Sdot_v_ground = Sdot_v[ball_ground_contact_index]
    Tdot_v_ground = Tdot_v[ball_ground_contact_index]

    # Set the time-derivatives of the Jacobians times the velocity.
    Zdot_v = np.zeros([nc * 3])
    Zdot_v[0:nc] = Ndot_v[:,0]
    Zdot_v[nc:2*nc] = Sdot_v[:, 0]
    Zdot_v[-nc:] = Tdot_v[:, 0]

    # Compute torques without applying any tangential forces.
    if self.controller_type == 'NoSlip':
      u, f_contact = self.ComputeContactControlMotorTorquesNoSlip(iM, fext, vdot_ball_des, Z, Zdot_v, N, Ndot_v, S_ground, Sdot_v_ground, T_ground, Tdot_v_ground)
    if self.controller_type == 'NoSeparation':
      u, f_contact = self.ComputeContactControlMotorTorquesNoSeparation(iM, fext, vdot_ball_des, Z, N, Ndot_v)
    if self.controller_type == 'BlackBoxDynamics':
      u = self.ComputeOptimalContactControlMotorTorques(controller_context, q, v, vdot_ball_des)

    # Compute the generalized contact forces.
    f_contact_generalized = None
    if self.controller_type == 'NoSlip' or self.controller_type == 'NoSeparation':
      f_contact_generalized = Z.T.dot(f_contact)

    # Output logging information.
    '''
    vdot = iM.dot(D.dot(zprimal) + fext)
    P_vdot = P.dot(vdot)
    logging.debug("N * v: " + str(N.dot(v)))
    logging.debug("S * v: " + str(S.dot(v)))
    logging.debug("T * v: " + str(T.dot(v)))
    logging.debug("Ndot * v: " + str(Ndot_v))
    logging.debug("Zdot * v: " + str(Zdot_v))
    logging.debug("fext: " + str(fext))
    logging.debug("M: ")
    logging.debug(M)
    logging.debug("P: ")
    logging.debug(P)
    logging.debug("D: ")
    logging.debug(D)
    logging.debug("B: ")
    logging.debug(B)
    logging.debug("N: ")
    logging.debug(N)
    logging.debug("Z: ")
    logging.debug(Z)
    logging.debug("contact forces: " + str(f_contact))
    logging.debug("vdot: " + str(vdot))
    logging.debug("vdot (desired): " + str(vdot_ball_des))
    logging.debug("P * vdot: " + str(P_vdot))
    logging.debug("torque: " + str(f_act))
    '''

    return u

  # Finds contacts only between the ball and the robot.
  def FindBallGroundContacts(self, all_q):
    # Get contacts between the robot and ball, and ball and the ground.
    contacts = self.FindContacts(all_q)

    # Get the ball and ground bodies.
    ball_body = self.get_ball_from_robot_and_ball_plant()
    world_body = self.get_ground_from_robot_and_ball_plant()

    # Evaluate scene graph's output port, getting a SceneGraph reference.
    query_object = self.robot_and_ball_plant.EvalAbstractInput(
      self.robot_and_ball_context, self.geometry_query_input_port.get_index()).get_value()
    inspector = query_object.inspector()

    # Get the tree corresponding to all bodies.
    all_plant = self.robot_and_ball_plant

    # Remove contacts between all but the ball and the ground.
    i = 0
    while i < len(contacts):
      geometry_A_id = contacts[i].id_A
      geometry_B_id = contacts[i].id_B
      frame_A_id = inspector.GetFrameId(geometry_A_id)
      frame_B_id = inspector.GetFrameId(geometry_B_id)
      body_A = all_plant.GetBodyFromFrameId(frame_A_id)
      body_B = all_plant.GetBodyFromFrameId(frame_B_id)
      body_A_B_pair = self.MakeSortedPair(body_A, body_B)
      if body_A_B_pair != self.MakeSortedPair(ball_body, world_body):
        contacts[i] = contacts[-1]
        del contacts[-1]
      else:
        i += 1

    return contacts

  # Finds contacts only between the ball and the robot.
  def FindRobotBallContacts(self, all_q):
    # Get contacts between the robot and ball, and ball and the ground.
    contacts = self.FindContacts(all_q)

    # Get the ball body and foot bodies.
    ball_body = self.get_ball_from_robot_and_ball_plant()
    foot_bodies = self.get_foot_links_from_robot_and_ball_plant()

    # Make sorted pairs to check.
    ball_foot_pairs = [0] * len(foot_bodies)
    for i in range(len(foot_bodies)):
      ball_foot_pairs[i] = self.MakeSortedPair(ball_body, foot_bodies[i])

    # Evaluate scene graph's output port, getting a SceneGraph reference.
    query_object = self.robot_and_ball_plant.EvalAbstractInput(
      self.robot_and_ball_context, self.geometry_query_input_port.get_index()).get_value()
    inspector = query_object.inspector()

    # Get the tree corresponding to all bodies.
    all_plant = self.robot_and_ball_plant

    # Remove contacts between all but the robot foot and the ball and the
    # ball and the ground.
    i = 0
    while i < len(contacts):
      geometry_A_id = contacts[i].id_A
      geometry_B_id = contacts[i].id_B
      frame_A_id = inspector.GetFrameId(geometry_A_id)
      frame_B_id = inspector.GetFrameId(geometry_B_id)
      body_A = all_plant.GetBodyFromFrameId(frame_A_id)
      body_B = all_plant.GetBodyFromFrameId(frame_B_id)
      body_A_B_pair = self.MakeSortedPair(body_A, body_B)
      if body_A_B_pair not in ball_foot_pairs:
        contacts[i] = contacts[-1]
        del contacts[-1]
      else:
        i += 1

    return contacts

  # Gets the vector of contacts.
  def FindContacts(self, all_q):
    # Set q in the context.
    self.UpdateRobotAndBallConfigurationForGeometricQueries(all_q)

    # Get the tree corresponding to all bodies.
    all_plant = self.robot_and_ball_plant

    # Evaluate scene graph's output port, getting a SceneGraph reference.
    query_object = self.robot_and_ball_plant.EvalAbstractInput(
        self.robot_and_ball_context, self.geometry_query_input_port.get_index()).get_value()
    inspector = query_object.inspector()

    # Determine the set of contacts.
    contacts = query_object.ComputePointPairPenetration()

    # Get the ball body and foot bodies.
    ball_body = self.get_ball_from_robot_and_ball_plant()
    foot_bodies = self.get_foot_links_from_robot_and_ball_plant()
    world_body = self.get_ground_from_robot_and_ball_plant()

    # Make sorted pairs to check.
    ball_foot_pairs = [0] * len(foot_bodies)
    for i in range(len(foot_bodies)):
      ball_foot_pairs[i] = self.MakeSortedPair(ball_body, foot_bodies[i])
    ball_world_pair = self.MakeSortedPair(ball_body, world_body)

    # Remove contacts between all but the robot foot and the ball and the
    # ball and the ground.
    i = 0
    while i < len(contacts):
      geometry_A_id = contacts[i].id_A
      geometry_B_id = contacts[i].id_B
      frame_A_id = inspector.GetFrameId(geometry_A_id)
      frame_B_id = inspector.GetFrameId(geometry_B_id)
      body_A = all_plant.GetBodyFromFrameId(frame_A_id)
      body_B = all_plant.GetBodyFromFrameId(frame_B_id)
      body_A_B_pair = self.MakeSortedPair(body_A, body_B)
      if body_A_B_pair not in ball_foot_pairs and body_A_B_pair != ball_world_pair:
        contacts[i] = contacts[-1]
        del contacts[-1]
      else:
        i += 1

    return contacts

  # Determines whether the ball and the ground are in contact.
  def IsBallContactingGround(self, q, contacts):
    # Get the ball and ground bodies.
    ball_body = self.get_ball_from_robot_and_ball_plant()
    ground_body = self.get_ground_from_robot_and_ball_plant()

    # Make sorted pairs to check.
    ball_ground_pair = self.MakeSortedPair(ball_body, ground_body)

    # Ensure that the context is set correctly.
    all_plant = self.robot_and_ball_plant
    all_context = self.robot_and_ball_context
    all_plant.SetPositions(all_context, q)

    # Evaluate scene graph's output port, getting a SceneGraph reference.
    query_object = self.robot_and_ball_plant.EvalAbstractInput(
      self.robot_and_ball_context,
      self.geometry_query_input_port.get_index()).get_value()
    inspector = query_object.inspector()

    for contact in contacts:
      geometry_A_id = contact.id_A
      geometry_B_id = contact.id_B
      frame_A_id = inspector.GetFrameId(geometry_A_id)
      frame_B_id = inspector.GetFrameId(geometry_B_id)
      body_A = self.robot_and_ball_plant.GetBodyFromFrameId(frame_A_id)
      body_B = self.robot_and_ball_plant.GetBodyFromFrameId(frame_B_id)
      body_A_B_pair = self.MakeSortedPair(body_A, body_B)
      if body_A_B_pair == ball_ground_pair:
        return True

    # No contact found.
    return False

  # Determines whether the ball and the robot are in contact.
  def IsRobotContactingBall(self, q, contacts):
    # Get the ball body and foot bodies.
    ball_body = self.get_ball_from_robot_and_ball_plant()
    foot_bodies = self.get_foot_links_from_robot_and_ball_plant()

    # Ensure that the context is set correctly.
    all_plant = self.robot_and_ball_plant
    all_context = self.robot_and_ball_context
    all_plant.SetPositions(all_context, q)

    # Make sorted pairs to check.
    ball_foot_pairs = [0] * len(foot_bodies)
    for i in range(len(foot_bodies)):
      ball_foot_pairs[i] = self.MakeSortedPair(ball_body, foot_bodies[i])

    # Evaluate scene graph's output port, getting a SceneGraph reference.
    query_object = self.robot_and_ball_plant.EvalAbstractInput(
      self.robot_and_ball_context,
      self.geometry_query_input_port.get_index()).get_value()
    inspector = query_object.inspector()

    for contact in contacts:
      geometry_A_id = contact.id_A
      geometry_B_id = contact.id_B
      frame_A_id = inspector.GetFrameId(geometry_A_id)
      frame_B_id = inspector.GetFrameId(geometry_B_id)
      body_A = self.robot_and_ball_plant.GetBodyFromFrameId(frame_A_id)
      body_B = self.robot_and_ball_plant.GetBodyFromFrameId(frame_B_id)
      body_A_B_pair = self.MakeSortedPair(body_A, body_B)
      if body_A_B_pair in ball_foot_pairs:
        return True

    # No contact found.
    return False

  # Calculate what torques to apply to the joints.
  def DoControlCalc(self, context, output):
    # Determine whether we're in a contacting or not-contacting phase.
    contact_desired = self.plan.IsContactDesired(context.get_time())

    # Get the generalized positions.
    q = self.get_q_all(context)

    # Look for full actuation.
    if self.fully_actuated:
        tau = self.ComputeFullyActuatedBallControlForces(context)

    else:  # "Real" control.
        # Compute tau.
        if contact_desired == True:
            # Find contacts.
            contacts = self.FindContacts(q)

            # Two cases: in the first, the robot and the ball are already in contact,
            # as desired. In the second, the robot desires to be in contact, but the
            # ball and robot are not contacting: the robot must intercept the ball.
            if self.IsRobotContactingBall(q, contacts):
                logging.info('Contact desired and contact detected at time ' + str(context.get_time()))
                tau = self.ComputeActuationForContactDesiredAndContacting(context, contacts)
            else:
                logging.info('Contact desired and no contact detected at time ' + str(context.get_time()))
                tau = self.ComputeActuationForContactDesiredButNoContact(context)
        else:
            # No contact desired.
            logging.info('Contact not desired at time ' + str(context.get_time()))
            tau = self.ComputeActuationForContactNotDesired(context)


    # Set the torque output.
    mutable_torque_out = output.get_mutable_value()
    mutable_torque_out[:] = np.zeros(mutable_torque_out.shape)

    if self.fully_actuated:
        mutable_torque_out[:] = tau
    else:
        self.robot_and_ball_plant.SetVelocitiesInArray(self.robot_instance, tau.flatten(), mutable_torque_out)

  def _DoCalcTimeDerivatives(self, context, derivatives):
    # Determine whether we're in a contacting or not-contacting phase.
    contact_intended = self.plan.IsContactDesired(context.get_time())

    if contact_intended:
      derivatives.get_mutable_vector().SetFromVector(np.zeros([self.nq_robot()]))
    else:
      # Get the desired robot configuration.
      q_robot_des = self.plan.GetRobotQVAndVdot(
          context.get_time())[0:nv_robot()-1]

      # Get the current robot configuration.
      q_robot = get_q_robot(context)
      derivatives.get_mutable_vector().SetFromVector(q_robot_des - q_robot)
