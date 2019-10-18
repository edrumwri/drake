#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/SVD>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>

#include <DR/common/exception.h>
#include <DR/common/logging.h>
#include <DR/tools/differential_inverse_kinematics.h>

namespace DR {

enum class TaskSpaceType {
  kCartesianOnly,    /// The regulated variables are Cartesian.
  kOrientationOnly,  /// The regulated variables are orientational.
  kFull,             /// The location and orientation of the frame is regulated.
};

/**
 Structure for defining goals that will be input to the impedance controller. Goals are specified using two frames,
 one (Frame S) offset from a link of the robot and one (Frame N) offset from a manipuland; the reference (desired)
 task space goals then represent a desired location, orientation, or pose for Frame S measured from, and expressed in,
 Frame N. For example, one can encode a pose goal, X_NS, as a 7-dimensional vector (four dimensional unit quaternion
 orientation and three dimensional Cartesian location). The "manipuland frame" need not be an object; it can
 correspond to the world frame.

 Drake's multibody functions often use a frame-fixed offset vector to define another frame, and TaskSpaceGoal
 accommodates this convention. Frames S is defined using a vector p_RS, expressed in Frame R, from the origin
 of R to the origin of S. Likewise, Frame N is defined using a vector p_MN, expressed in Frame M, from the origin
 of M to the origin of N.
 */
template <typename T>
class TaskSpaceGoal {
 public:
  TaskSpaceGoal(TaskSpaceType type, const drake::multibody::Frame<T>* robot_frame_R, const drake::Vector3<T>& p_RS,
                const drake::multibody::Frame<T>* manipuland_frame_M, const drake::Vector3<T>& p_MN)
      : type_(type),
        robot_frame_R_(*robot_frame_R),
        p_RS_(p_RS),
        manipuland_frame_M_(*manipuland_frame_M),
        p_MN_(p_MN) {
    // Verify pointers.
    DR_DEMAND(robot_frame_R);
    DR_DEMAND(manipuland_frame_M);
  }

  /// Gets the number of configuration (position) variables for this goal.
  int num_configuration_variables() const {
    switch (type_) {
      case TaskSpaceType::kCartesianOnly:
        return 3;
      case TaskSpaceType::kOrientationOnly:
        return 4;
      case TaskSpaceType::kFull:
        return 7;
    }

    DR_UNREACHABLE();
  }

  /// Gets the number of velocity variables for this goal.
  int num_velocity_variables() const {
    switch (type_) {
      case TaskSpaceType::kCartesianOnly:
      case TaskSpaceType::kOrientationOnly:
        return 3;

      case TaskSpaceType::kFull:
        return 6;
    }

    DR_UNREACHABLE();
  }

  /// Gets the robot frame (R).
  const drake::multibody::Frame<T>& robot_frame_R() const { return robot_frame_R_; }

  /// Gets the manipuland frame (M).
  const drake::multibody::Frame<T>& manipuland_frame_M() const { return manipuland_frame_M_; }

  /// Gets the vector from the origin of Frame M to the origin of Frame N, expressed in M.
  const drake::Vector3<T>& p_MN() const { return p_MN_; }

  /// Gets the vector from the origin of Frame R to the origin of Frame S, expressed in R.
  const drake::Vector3<T>& p_RS() const { return p_RS_; }

  /// Gets the number of acceleration variables for this goal. Note: the number of acceleration variables will
  /// *always* be equal to the number of velocity variables.
  int num_acceleration_variables() const { return num_velocity_variables(); }

  /// Gets the type of task space goal.
  TaskSpaceType type() const { return type_; }

 private:
  TaskSpaceType type_;

  // A frame (R) on the robot.
  const drake::multibody::Frame<T>& robot_frame_R_;

  // Offset vector from the origin of Frame R (robot frame) to the origin of Frame S, expressed in Frame R.
  const drake::Vector3<T> p_RS_;

  // A frame (M) on the manipuland.
  const drake::multibody::Frame<T>& manipuland_frame_M_;

  // Offset vector from the origin of Frame M (manipuland frame) to the origin of Frame N, expressed in Frame M.
  const drake::Vector3<T> p_MN_;
};

/**
 Implements a task space (Cartesian, orientation, or Cartesian and orientation) impedance controller. The
 controller can satisfy multiple task space "desireds" (e.g., frame location, frame velocity, frame
 acceleration), one set per frame, simultaneously. At present, only one goal frame can be specified per kinematic chain,
 meaning that, e.g., one goal is supported per arm of a multi-arm robot.

 Presently, unspecified goals (e.g., the robot has two arms but a goal is specified for only one arm) will lead to no
 actuations planned for the degrees-of-freedom that are left unspecified.

 Only Cartesian goals are supported at this time.
 */
template <typename T>
class TaskSpaceImpedanceController : public drake::systems::LeafSystem<T> {
 public:
  /// Constructs the impedance controller and the given task space type.
  TaskSpaceImpedanceController(const drake::multibody::MultibodyPlant<T>* universal_plant,
                               const drake::multibody::MultibodyPlant<T>* robot_plant,
                               std::vector<const TaskSpaceGoal<T>*>&& task_space_goals)
      : universal_plant_(*universal_plant), robot_plant_(*robot_plant), task_space_goals_(task_space_goals) {
    // Check the pointers.
    DR_DEMAND(universal_plant);
    DR_DEMAND(robot_plant);

    // Create the actuation matrix.
    B_ = universal_plant->MakeActuationMatrix();

    // Verify that the only actuators are the robot's actuators.
    const drake::MatrixX<T> B_robot = robot_plant->MakeActuationMatrix();
    DR_DEMAND(B_.cols() == B_robot.cols());

    // Verify that B is a binary matrix.
    for (int i = 0; i < B_.rows(); ++i) {
      for (int j = 0; j < B_.cols(); ++j) {
        if (B_(i, j) != 0 && B_(i, j) != 1) throw std::logic_error("Actuation matrix is not binary!");
      }
    }

    // Initialize the forces used for computing inverse dynamics.
    external_forces_ = std::make_unique<drake::multibody::MultibodyForces<T>>(universal_plant_);
    contact_forces_ = drake::VectorX<T>(universal_plant->num_velocities());

    // Create a Context for the universal plant.
    universal_plant_context_ = universal_plant->CreateDefaultContext();

    // Initialize the generalized accelerations for the robot.
    vdot_robot_ = drake::VectorX<T>::Zero(robot_plant->num_velocities());

    // Initialize Jacobian matrices used to avoid heap allocations. These Jacobians are in the size of the number of
    // velocity space variables of "the universe", because we typically need to control objects through contact.
    J_ = drake::MatrixX<T>(6, universal_plant->num_velocities());
    J_tmp_ = drake::MatrixX<T>(6, universal_plant->num_velocities());
    J_bias_ = drake::VectorX<T>(6);
    J_bias_tmp_ = drake::VectorX<T>(6);

    // Declare ports.
    universal_q_estimated_input_port_index_ =
        this->DeclareVectorInputPort("universal_q_estimated",
                                     drake::systems::BasicVector<T>(universal_plant->num_positions()))
            .get_index();
    universal_v_estimated_input_port_index_ =
        this->DeclareVectorInputPort("universal_v_estimated",
                                     drake::systems::BasicVector<T>(universal_plant->num_velocities()))
            .get_index();

    // Declare desired ports, one set per goal.
    // TODO(drum) Replace 'index' with a model instance name.
    int index = 0;
    for (const TaskSpaceGoal<T>* goal : task_space_goals_) {
      const std::string base_name =
          goal->robot_frame_R().name() + "/" + goal->manipuland_frame_M().name() + std::to_string(index);

      // Declare kinematic goals.
      x_NS_N_desired_input_port_index_[goal] =
          this->DeclareVectorInputPort(base_name + "_x_NS_N_desired",
                                       drake::systems::BasicVector<T>(goal->num_configuration_variables()))
              .get_index();
      xd_NS_N_desired_input_port_index_[goal] =
          this->DeclareVectorInputPort(base_name + "_xd_NS_N_desired",
                                       drake::systems::BasicVector<T>(goal->num_velocity_variables()))
              .get_index();
      xdd_NS_N_desired_input_port_index_[goal] =
          this->DeclareVectorInputPort(base_name + "_xdd_NS_N_desired",
                                       drake::systems::BasicVector<T>(goal->num_acceleration_variables()))
              .get_index();

      // Declare gain ports.
      task_space_kp_gain_input_port_index_[goal] =
          this->DeclareVectorInputPort(base_name + "_kp_ts_gain",
                                       drake::systems::BasicVector<T>(goal->num_velocity_variables()))
              .get_index();
      task_space_kd_gain_input_port_index_[goal] =
          this->DeclareVectorInputPort(base_name + "_kd_ts_gain",
                                       drake::systems::BasicVector<T>(goal->num_velocity_variables()))
              .get_index();

      ++index;
    }

    // Compute the number of goal velocity variables.
    num_goal_velocity_variables_ = 0;
    for (const TaskSpaceGoal<T>* goal : task_space_goals_)
      num_goal_velocity_variables_ += goal->num_velocity_variables();

    // Initialize the full Jacobian matrix- used for computing the Jacobian matrix with respect to every goal- to
    // avoid heap allocations. This matrix will comprise multiple stacked "universal" Jacobian matrices (i.e., J_).
    // The bias vector (J_bias_) will be formed the same way.
    J_full_ = drake::MatrixX<T>(num_goal_velocity_variables_, universal_plant->num_velocities());
    J_bias_full_ = drake::VectorX<T>(num_goal_velocity_variables_);

    // Do the same thing for the vector of goal accelerations.
    xddot_star_ = drake::VectorX<T>(num_goal_velocity_variables_);

    // Declare the output ports.
    actuation_output_port_index_ =
        this->DeclareVectorOutputPort("actuation", drake::systems::BasicVector<T>(robot_plant->num_actuators()),
                                      &TaskSpaceImpedanceController::CalcControlOutput)
            .get_index();
    residual_acceleration_output_port_index_ =
        this->DeclareVectorOutputPort("residual_acceleration",
                                      drake::systems::BasicVector<T>(num_goal_velocity_variables_),
                                      &TaskSpaceImpedanceController<T>::CalcResidualAcceleration)
            .get_index();
  }

  /// Gets whether estimated contacts factor into the control output.
  bool contact_affects_control() const { return contact_affects_control_; }

  /// Sets whether estimated contacts factor into the control output.
  void set_contact_affects_control(bool flag) { contact_affects_control_ = flag; }

  /// Gets the plant containing just the robot.
  const drake::multibody::MultibodyPlant<T>& robot_plant() const { return robot_plant_; }

  /// Gets the plant containing the robot and all bodies in the environment.
  const drake::multibody::MultibodyPlant<T>& universal_plant() const { return universal_plant_; }

  /// Gets the input port for the estimated generalized positions of every multibody in the environment.
  /// The vector is ordered according to the generalized positions of universal_plant().
  const drake::systems::InputPort<T>& universal_q_estimated_input_port() const {
    return drake::systems::System<T>::get_input_port(universal_q_estimated_input_port_index_);
  }

  /// Gets the input port for the estimated generalized velocities of every multibody in the environment.
  /// The vector is ordered according to the generalized velocities of universal_plant().
  const drake::systems::InputPort<T>& universal_v_estimated_input_port() const {
    return drake::systems::System<T>::get_input_port(universal_v_estimated_input_port_index_);
  }

  /// The task-space stiffness for the given goal.
  const drake::systems::InputPort<T>& task_space_kp_gain_input_port(const TaskSpaceGoal<T>& goal) const {
    return drake::systems::System<T>::get_input_port(task_space_kp_gain_input_port_index_.at(&goal));
  }

  /// The task-space viscous damping for the given goal.
  const drake::systems::InputPort<T>& task_space_kd_gain_input_port(const TaskSpaceGoal<T>& goal) const {
    return drake::systems::System<T>::get_input_port(task_space_kd_gain_input_port_index_.at(&goal));
  }

  /**
   Gets the input port for the desired task space configurations. The inputs correspond to a vector from
   the origin of N's frame to the origin of S's frame, expressed in N's frame.
  */
  const drake::systems::InputPort<T>& x_NS_N_desired_input_port(const TaskSpaceGoal<T>& goal) const {
    return drake::systems::System<T>::get_input_port(x_NS_N_desired_input_port_index_.at(&goal));
  }

  /** Gets the input port for the desired task space velocities.
   @see x_NS_N_desired_input_port() documentation for the interpretation of the input values.
  */
  const drake::systems::InputPort<T>& xd_NS_N_desired_input_port(const TaskSpaceGoal<T>& goal) const {
    return drake::systems::System<T>::get_input_port(xd_NS_N_desired_input_port_index_.at(&goal));
  }

  /** Gets the input port for the desired task space accelerations.
   @see x_NS_N_desired_input_port() documentation for the interpretation of the input values.
  */
  const drake::systems::InputPort<T>& xdd_NS_N_desired_input_port(const TaskSpaceGoal<T>& goal) const {
    return drake::systems::System<T>::get_input_port(xdd_NS_N_desired_input_port_index_.at(&goal));
  }

  /// Gets the port that provides commands to the actuators. Note that these commands are in actuator space, so this
  /// output should be wired to an ActuatorDemultiplexer if there are multiple model instances with actuators.
  const drake::systems::OutputPort<T>& actuation_output_port() const {
    return drake::systems::System<T>::get_output_port(actuation_output_port_index_);
  }

  // TODO(drum) Unit test this function.
  /// Gets the "residual" acceleration, which comes from solving the equation Jv̇ = ẋᵈᵉˢ + b for ̇v, where J is a Jacobian
  /// matrix, b is a known "bias" term, ẋᵈᵉˢ represents the kinematic goals, and v̇ are the universal plant generalized
  /// accelerations necessary to realize those task space goals.  If a ̇v does not exist such that the
  /// equation can be satisfied, the ̇v that minimizes the residual acceleration will be found. This port gives that
  /// residual acceleration.
  const drake::systems::OutputPort<T>& residual_acceleration_output_port() const {
    return drake::systems::System<T>::get_output_port(residual_acceleration_output_port_index_);
  }

 private:
  // Computes the residual acceleration from solving the least-squares problem Jv̇ = ẍ* - J̇v for generalized
  // velocities v̇, given desired task space accelerations ̈x* and Jacobian matrix J (and its time
  // derivative, J̇). If this residual error is not effectively zero, then the robotic system is unable to
  // accomplish its commanded accelerations. Thus, this function provides debugging output that tells when
  // the system does not have full control authority.
  void CalcResidualAcceleration(const drake::systems::Context<T>& context,
                                drake::systems::BasicVector<T>* residual_acceleration) const {
    // Get the q and v for the universe.
    Eigen::VectorBlock<const drake::VectorX<T>> q_universal =
        this->get_input_port(universal_q_estimated_input_port_index_).Eval(context);
    Eigen::VectorBlock<const drake::VectorX<T>> v_universal =
        this->get_input_port(universal_v_estimated_input_port_index_).Eval(context);

    // Set the positions and velocities in the universal plant context.
    universal_plant().SetPositions(universal_plant_context_.get(), q_universal);
    universal_plant().SetVelocities(universal_plant_context_.get(), v_universal);

    // Form the Jacobian matrix, bias term, and xddot*.
    FormJacobianAndTaskSpaceGoals(context);

    // Compute the desired acceleration for the universal plant.
    svd_.compute(J_full_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const drake::VectorX<T> vdot = svd_.solve(xddot_star_ - J_bias_full_);

    // Compute the residual acceleration.
    residual_acceleration->SetFromVector(J_full_ * vdot + J_bias_full_ - xddot_star_);
  }

  // Computes the 6ng x nv-dimensional Jacobian matrix J_full_, 6ng x 1 "acceleration-bias" vector J_bias_full_, and 6ng
  // x 1
  // desired-task-space acceleration xdd*, where ng is the number of goals and nv is the number of generalized velocity
  // variables in the universal plant.
  // @pre universal plant positions and velocities have been set in the universal plant context.
  void FormJacobianAndTaskSpaceGoals(const drake::systems::Context<T>& context) const {
    // Loop through all goals.
    int acceleration_goal_index = 0;
    for (const TaskSpaceGoal<T>* goal : task_space_goals_) {
      // Get the desireds.
      Eigen::VectorBlock<const drake::VectorX<T>> x_NS_N_des = x_NS_N_desired_input_port(*goal).Eval(context);
      Eigen::VectorBlock<const drake::VectorX<T>> xd_NS_N_des = xd_NS_N_desired_input_port(*goal).Eval(context);
      Eigen::VectorBlock<const drake::VectorX<T>> xdd_NS_N_des = xdd_NS_N_desired_input_port(*goal).Eval(context);

      // Get the complete xddot*, the frame acceleration with error feedback components.
      drake::VectorX<T> xddot_star = CalcXDDotStar(context, *goal, x_NS_N_des, xd_NS_N_des, xdd_NS_N_des);

      // Set it in the greater vector.
      xddot_star_.segment(acceleration_goal_index, goal->num_acceleration_variables()) = xddot_star;

      // Compute the Jacobian and the bias term.
      CalcJacobianAndBias(*goal, &J_, &J_bias_);

      // Set these terms in the greater matrix / vector.
      J_full_.block(acceleration_goal_index, 0, goal->num_acceleration_variables(), J_.cols()) = J_;
      J_bias_full_.segment(acceleration_goal_index, goal->num_acceleration_variables()) = J_bias_;

      // Update the index.
      acceleration_goal_index += goal->num_acceleration_variables();
    }
  }

  void CalcControlOutput(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const {
    // Get the q and v for the universe.
    Eigen::VectorBlock<const drake::VectorX<T>> q_universal =
        this->get_input_port(universal_q_estimated_input_port_index_).Eval(context);
    Eigen::VectorBlock<const drake::VectorX<T>> v_universal =
        this->get_input_port(universal_v_estimated_input_port_index_).Eval(context);

    // Set the positions and velocities in the universal plant context.
    universal_plant().SetPositions(universal_plant_context_.get(), q_universal);
    universal_plant().SetVelocities(universal_plant_context_.get(), v_universal);

    // Form the Jacobian matrix, bias term, and xddot*.
    FormJacobianAndTaskSpaceGoals(context);

    // Compute the desired acceleration for the universal plant.
    svd_.compute(J_full_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const drake::VectorX<T> vdot = svd_.solve(xddot_star_ - J_bias_full_);
    DR_LOG_DEBUG(DR::log(), "vdot: {}", vdot.transpose());
    DR_LOG_DEBUG(DR::log(), "J*vdot + dotJ*v - xddot*: {}", (J_full_ * vdot + J_bias_full_ - xddot_star_).transpose());

    // TODO(drum) Explore how we can turn off contacts between all but the robot and the manipuland (and all objects
    // that those are touching).
    // Get the contact forces acting on the plant, if desired.
    contact_forces_.setZero();
    if (contact_affects_control_) {
      // We can only compute the contact forces if the plant is continuous.
      DR_DEMAND(!universal_plant().is_discrete());

      for (drake::multibody::ModelInstanceIndex i(0); i < universal_plant().num_model_instances(); ++i) {
        contact_forces_ +=
            universal_plant().get_generalized_contact_forces_output_port(i).Eval(*universal_plant_context_);
      }
    }

    // TODO(drum) The code below is computing much more than what we need to compute the controls for the robot.
    //            Optimize.

    // Compute the force elements contribution (e.g., gravitational forces) to the external forces.
    universal_plant_.CalcForceElementsContribution(*universal_plant_context_, external_forces_.get());

    // Update the external forces vector with the contact forces.
    external_forces_->mutable_generalized_forces() += contact_forces_;

    // Compute the generalized forces necessary to realize the desired generalized accelerations:
    // tau_u = M(q)v̇ + C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W
    const drake::VectorX<T> tau_u =
        universal_plant_.CalcInverseDynamics(*universal_plant_context_, vdot, *external_forces_);

    // Compute the actuation forces.
    output->SetFromVector(B_.transpose() * tau_u);
    DR_LOG_DEBUG(DR::log(), "B' * f = u: {}", output->CopyToVector());
  }

  // Computes the actual frame acceleration (as a function of the open loop acceleration and PD error feedback terms).
  // @pre the positions and velocities in the universal plant context have been set to reflect the (estimated) current
  //      positions and velocities.
  drake::VectorX<T> CalcXDDotStar(const drake::systems::Context<T>& context, const TaskSpaceGoal<T>& goal,
                                  Eigen::VectorBlock<const drake::VectorX<T>> x_NoSo_N_des,
                                  Eigen::VectorBlock<const drake::VectorX<T>> xd_NoSo_N_des,
                                  Eigen::VectorBlock<const drake::VectorX<T>> xdd_NoSo_N_des) const {
    // TODO(drum) Alter this when there is more than just Cartesian acceleration.
    return CalcXDDotStarCartesianOnly(context, goal, x_NoSo_N_des, xd_NoSo_N_des, xdd_NoSo_N_des);
  }

  // Computes the desired acceleration of Frame N with respect to Frame S, expressed in Frame N.
  drake::VectorX<T> CalcXDDotStarCartesianOnly(const drake::systems::Context<T>& context, const TaskSpaceGoal<T>& goal,
                                               Eigen::VectorBlock<const drake::VectorX<T>> x_NoSo_N_des,
                                               Eigen::VectorBlock<const drake::VectorX<T>> xd_NoSo_N_des,
                                               Eigen::VectorBlock<const drake::VectorX<T>> xdd_NoSo_N_des) const {
    // Get the pose of Frame S in the world.
    const drake::math::RigidTransform<T> X_WR = goal.robot_frame_R().CalcPoseInWorld(*universal_plant_context_);
    const drake::math::RigidTransform<T> X_WS(X_WR.rotation(), X_WR * goal.p_RS());

    // Get the pose of Frame N in the world.
    const drake::math::RigidTransform<T> X_WM = goal.manipuland_frame_M().CalcPoseInWorld(*universal_plant_context_);
    const drake::math::RigidTransform<T> X_WN(X_WM.rotation(), X_WM * goal.p_MN());

    // The controller seeks for Frame N to be located at x_NoSo_N_des with respect to Frame S, expressed in Frame N.
    const drake::Vector3<T> x_NoSo_N =
        X_WN.rotation().transpose() *
        DifferentialInverseKinematics<T>::CalcPositionDifferential(X_WN.translation(), X_WS.translation());
    const drake::Vector3<T> x_NoSo_N_err =
        DifferentialInverseKinematics<T>::CalcPositionDifferential(x_NoSo_N, x_NoSo_N_des);
    DR_LOG_DEBUG(DR::log(), "x_NoSo_N: {}", x_NoSo_N.transpose());
    DR_LOG_DEBUG(DR::log(), "x_NoSo_N (des): {}", x_NoSo_N_des.transpose());
    DR_LOG_DEBUG(DR::log(), "x_NoSo_N (err): {}", x_NoSo_N_err.transpose());

    // Compute the velocity of Frame S in the world.
    // Note: this code block assumes that Frame R *is* a body frame. This assumption might need to be relaxed.
    DR_DEMAND(&goal.robot_frame_R().body().body_frame() == &goal.robot_frame_R());
    const drake::multibody::SpatialVelocity<T>& V_WR =
        universal_plant().EvalBodySpatialVelocityInWorld(*universal_plant_context_, goal.robot_frame_R().body());
    const drake::multibody::SpatialVelocity<T> V_WS = V_WR.Shift(X_WR.rotation() * goal.p_RS());

    // Compute the velocity of Frame M in the world.
    // Note: this code block assumes that Frame M *is* a body frame. This assumption might need to be relaxed.
    DR_DEMAND(&goal.manipuland_frame_M().body().body_frame() == &goal.manipuland_frame_M());
    const drake::multibody::SpatialVelocity<T>& V_WM =
        universal_plant().EvalBodySpatialVelocityInWorld(*universal_plant_context_, goal.manipuland_frame_M().body());
    const drake::multibody::SpatialVelocity<T> V_WN = V_WM.Shift(X_WM.rotation() * goal.p_MN());

    // Compute the current translational velocity of Frame N, measured from Frame S, and expressed in Frame N.
    const drake::Vector3<T> xd_NoSo_N = X_WN.rotation().transpose() * (V_WS.translational() - V_WN.translational());

    // Compute the velocity error.
    const drake::Vector3<T> xd_NoSo_N_err = xd_NoSo_N_des - xd_NoSo_N;
    DR_LOG_DEBUG(DR::log(), "xd_NoSo_N: {}", xd_NoSo_N.transpose());
    DR_LOG_DEBUG(DR::log(), "xd_NoSo_N (des): {}", xd_NoSo_N_des.transpose());
    DR_LOG_DEBUG(DR::log(), "xd_NoSo_N (err): {}", xd_NoSo_N_err.transpose());

    // Get the gains.
    Eigen::VectorBlock<const drake::VectorX<T>> kp_ts_gain =
        this->get_input_port(task_space_kp_gain_input_port_index_.at(&goal)).Eval(context);
    Eigen::VectorBlock<const drake::VectorX<T>> kd_ts_gain =
        this->get_input_port(task_space_kd_gain_input_port_index_.at(&goal)).Eval(context);

    DR_LOG_DEBUG(DR::log(), "desired end-effector acceleration (w/o error feedback): {}", xdd_NoSo_N_des.transpose());
    DR_LOG_DEBUG(DR::log(), "desired end-effector acceleration (with error feedback): {}",
                 (xdd_NoSo_N_des + Eigen::DiagonalMatrix<T, Eigen::Dynamic, 6>(kp_ts_gain) * x_NoSo_N_err +
                  Eigen::DiagonalMatrix<T, Eigen::Dynamic, 6>(kd_ts_gain) * xd_NoSo_N_err)
                     .transpose());

    // Return the desired end-effector acceleration.
    return xdd_NoSo_N_des + Eigen::DiagonalMatrix<T, Eigen::Dynamic, 6>(kp_ts_gain) * x_NoSo_N_err +
           Eigen::DiagonalMatrix<T, Eigen::Dynamic, 6>(kd_ts_gain) * xd_NoSo_N_err;
  }

  // Computes the Jacobian matrix and acceleration bias term that corresponds to a single task space goal. Since the
  // single task space goal is specified as x_NoSo_N_des, where S is a frame attached to the robot and N is a frame
  // attached to the manipuland, we have to perform these calculations.
  void CalcJacobianAndBias(const TaskSpaceGoal<T>& goal, drake::MatrixX<T>* J, drake::VectorX<T>* J_bias) const {
    drake::MatrixX<T>& J_tmp = (goal.type() == TaskSpaceType::kFull) ? *J : J_tmp_;
    drake::VectorX<T>& J_bias_tmp = (goal.type() == TaskSpaceType::kFull) ? *J_bias : J_bias_tmp_;

    // TODO(drum) Optimize this by computing Jacobian and acceleration bias only for things we care about (the robot +
    // manipuland?).
    // Compute the Jacobian matrix that transforms generalized velocities to the spatial velocity of Frame S
    // measured and expressed in Frame M. Per Drake documentation, measuring and expressing in Frame N is equivalent to
    // measuring and expressing in Frame M.
    J_tmp.resize(6, universal_plant_.num_velocities());
    universal_plant().CalcJacobianSpatialVelocity(*universal_plant_context_, drake::multibody::JacobianWrtVariable::kV,
                                                  goal.robot_frame_R(), goal.p_RS(), goal.manipuland_frame_M(),
                                                  goal.manipuland_frame_M(), &J_tmp);

    // Perform the same computation for the "acceleration bias" vector.
    J_bias_tmp = universal_plant().CalcBiasForJacobianSpatialVelocity(
        *universal_plant_context_, drake::multibody::JacobianWrtVariable::kV, goal.robot_frame_R(), goal.p_RS(),
        goal.manipuland_frame_M(), goal.manipuland_frame_M());

    // Pick the right part of this matrix and vector, if necessary.
    switch (goal.type()) {
      case TaskSpaceType::kFull:
        break;

      case TaskSpaceType::kCartesianOnly:
        *J = J_tmp.template bottomRows<3>();
        *J_bias = J_bias_tmp.template tail<3>();
        break;

      case TaskSpaceType::kOrientationOnly:
        *J = J_tmp.template topRows<3>();
        *J_bias = J_bias_tmp.template head<3>();
        break;
    }
  }

  // Port indices.
  drake::systems::InputPortIndex universal_q_estimated_input_port_index_{};
  drake::systems::InputPortIndex universal_v_estimated_input_port_index_{};
  std::unordered_map<const TaskSpaceGoal<T>*, drake::systems::InputPortIndex> task_space_kp_gain_input_port_index_{};
  std::unordered_map<const TaskSpaceGoal<T>*, drake::systems::InputPortIndex> task_space_kd_gain_input_port_index_{};
  std::unordered_map<const TaskSpaceGoal<T>*, drake::systems::InputPortIndex> x_NS_N_desired_input_port_index_{};
  std::unordered_map<const TaskSpaceGoal<T>*, drake::systems::InputPortIndex> xd_NS_N_desired_input_port_index_{};
  std::unordered_map<const TaskSpaceGoal<T>*, drake::systems::InputPortIndex> xdd_NS_N_desired_input_port_index_{};
  drake::systems::OutputPortIndex actuation_output_port_index_{};
  drake::systems::OutputPortIndex residual_acceleration_output_port_index_{};

  // Multibody forces used for computing inverse dynamics.
  std::unique_ptr<drake::multibody::MultibodyForces<T>> external_forces_;

  // The actuation matrix for the universal plant.
  drake::MatrixX<T> B_;

  // The "universal" plant, containing all objects in the environment.
  const drake::multibody::MultibodyPlant<T>& universal_plant_;

  // The plant for just the robot.
  const drake::multibody::MultibodyPlant<T>& robot_plant_;

  // This vector tells how to interpret each of the frame goals.
  std::vector<const TaskSpaceGoal<T>*> task_space_goals_;

  // Temporary variables used to minimize heap allocations. Altering these variables does not change the result of any
  // computations.
  mutable drake::MatrixX<T> J_, J_tmp_;
  mutable drake::MatrixX<T> J_full_;
  mutable drake::VectorX<T> J_bias_, J_bias_full_, J_bias_tmp_;
  mutable Eigen::JacobiSVD<drake::MatrixX<T>> svd_;
  mutable drake::VectorX<T> xddot_star_;
  mutable drake::VectorX<T> vdot_robot_;
  mutable std::unique_ptr<drake::systems::Context<T>> universal_plant_context_;
  mutable drake::VectorX<T> contact_forces_;

  // Port indices.
  drake::systems::InputPortIndex universal_vdot_desired_input_port_index_{};

  // The number of velocity variables in all goals.
  int num_goal_velocity_variables_{0};

  // Whether contact affects the control outputs. I hope this should be "yes" by default.
  bool contact_affects_control_{true};
};

}  // namespace DR
