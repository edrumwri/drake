#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/QR>

#include <drake/common/unused.h>
#include <drake/multibody/math/spatial_velocity.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>

#include <DR/interfaces/inverse_kinematics.h>
#include <DR/tools/differential_inverse_kinematics.h>
#include <DR/tools/sobol_sequence.h>

namespace DR {

/**
 A "driver", i.e., implementations of Chopstick-specific methods for kinematics
 and inverse kinematics.
 */
template <typename T>
class ChopstickKinematics : public InverseKinematics<T> {
 public:
  /**
   Constructs the given driver using a plant and a seed for inverse kinematics
   @param robot_plant the plant containing one or more chopsticks model instances. Other model instances
          (e.g., manipulands) can be present in the plant as well, but beware that this may slow the inverse kinematics
          method considerably.
   @param all_plant the plant containing all models in the environment, including the chopsticks models, manipulands,
          fixed objects, etc.
   @param random_seed the seed to use for the pseudorandom generator in the inverse kinematics solver. Setting this
          variable to a constant will cause the IK process to act deterministically (i.e., repeatably). Setting this
          value to, e.g., `time(NULL)` will cause the IK process to act randomly.
   */
  ChopstickKinematics(const drake::multibody::MultibodyPlant<T>* robot_plant, unsigned random_seed)
      : plant_(*robot_plant), seed_(random_seed) {
    // Create a context for the plant.
    plant_context_ = robot_plant->CreateDefaultContext();

    // How many model instances MBP reserves.
    const int num_builtin_model_instances = 2;

    // Initialize mutable variables.
    v_.resize(robot_plant->num_velocities());
    qdot_ = std::make_unique<drake::systems::BasicVector<double>>(robot_plant->num_positions());
    q_seed_.resize(plant_.num_positions());
    q_.resize(plant_.num_positions());
    q_model_.resize(robot_plant->num_positions() / (robot_plant->num_model_instances() - num_builtin_model_instances));
    delta_x_.resize(6 /* operational space dimension */);

    // Set the number of base poses.
    X_WB_.resize(robot_plant->num_model_instances());

    // Find the revolute joint indices and the relative poses for the mounted robots.
    revolute_joint_indices_ = std::vector<std::vector<drake::multibody::JointIndex>>(plant_.num_model_instances());
    for (drake::multibody::ModelInstanceIndex j(0); j < plant_.num_model_instances(); ++j) {
      std::vector<drake::multibody::JointIndex> joints = plant_.GetJointIndices(j);
      for (const auto& i : joints) {
        if (dynamic_cast<const drake::multibody::RevoluteJoint<T>*>(&plant_.get_joint(i))) {
          revolute_joint_indices_[j].push_back(i);
        } else {
          const auto* weld_joint = dynamic_cast<const drake::multibody::WeldJoint<T>*>(&plant_.get_joint(i));
          if (weld_joint) {
            if (weld_joint->child_body().name() == "base") {
              DRAKE_DEMAND(!X_WB_[j].has_value());
              X_WB_[j] = weld_joint->frame_on_parent().GetFixedPoseInBodyFrame();
            }
          }
        }
      }
    }
    for (const auto& i : revolute_joint_indices_) DR_DEMAND(i.size() == 2 || i.size() == 0);

    // Build the Body to body index map.
    for (drake::multibody::BodyIndex i(0); i < robot_plant->num_bodies(); ++i)
      body_to_body_index_map_[&robot_plant->get_body(i)] = i;

    // Build the body index to model instance index map.
    for (drake::multibody::ModelInstanceIndex i(0); i < robot_plant->num_model_instances(); ++i) {
      const std::vector<drake::multibody::BodyIndex> body_indices = robot_plant->GetBodyIndices(i);
      for (const auto& j : body_indices) body_index_to_model_instance_index_map_[j] = i;
    }

    // Set upper and lower limits.
    lower_limits_.resize(robot_plant->num_model_instances());
    upper_limits_.resize(robot_plant->num_model_instances());

    // First set all limits to +/- 1e2 (allows sampling much easier than +/- infinity).
    const double default_limit = 1e2;
    drake::VectorX<T> ll = drake::VectorX<T>::Ones(robot_plant->num_positions()) * -default_limit;
    drake::VectorX<T> ul = drake::VectorX<T>::Ones(robot_plant->num_positions()) * default_limit;

    // Now set individual limits.
    for (drake::multibody::ModelInstanceIndex i(0); i < robot_plant->num_model_instances(); ++i) {
      const std::vector<drake::multibody::JointIndex> joint_indices = robot_plant->GetJointIndices(i);
      for (const auto& j : joint_indices) {
        const drake::multibody::Joint<T>& joint = robot_plant->get_joint(j);
        const int position_start = joint.position_start();
        const int num_positions = joint.num_positions();
        ll.segment(position_start, num_positions) = joint.position_lower_limits();
        ul.segment(position_start, num_positions) = joint.position_upper_limits();
      }

      lower_limits_[i] = robot_plant->GetPositionsFromArray(i, ll);
      upper_limits_[i] = robot_plant->GetPositionsFromArray(i, ul);
      for (int j = 0; j < static_cast<int>(lower_limits_[i].size()); ++j) {
        lower_limits_[i][j] = std::max(lower_limits_[i][j], -default_limit);
        upper_limits_[i][j] = std::min(upper_limits_[i][j], +default_limit);
      }
    }

    // Get the left and right model instances.
    DR_DEMAND(robot_plant->num_model_instances() == 2 + num_builtin_model_instances);
    const drake::multibody::ModelInstanceIndex two(2);
    const drake::multibody::ModelInstanceIndex three(3);
    const std::string& two_name = robot_plant->GetModelInstanceName(two);
    const std::string& three_name = robot_plant->GetModelInstanceName(three);
    if (two_name.find("left") != std::string::npos) {
      left_chopstick_model_ = two;
      DR_DEMAND(three_name.find("right") != std::string::npos);
      right_chopstick_model_ = three;
    } else {
      DR_DEMAND(two_name.find("right") != std::string::npos);
      DR_DEMAND(three_name.find("left") != std::string::npos);
      left_chopstick_model_ = three;
      right_chopstick_model_ = two;
    }

    // The more seeds, the tighter the accuracy that can be obtained. The following number gives us 30 samples per
    // dimension, which seems to be about enough.
    num_seeds_to_evaluate_ = 900;

    // Given that the units of the chopstick are in meters, a tolerance of 4e-4 would correspond to 0.4 mm accuracy and
    // 2.4 x 10-3 radians (~0.14 degrees). To do better than this, we'd likely need a better seed to start from.
    ik_tolerance_ = 4e-4;

    // Use analytical IK by default.
    use_numerical_ik_ = false;

    // Initialize the Jacobian matrix for converting generalized velocities to link velocities.
    J_ = drake::MatrixX<T>(6, robot_plant->num_velocities());
  }

  virtual ~ChopstickKinematics() {}

  enum class InverseKinematicsType { kPositionOnly, kOrientationOnly, kPositionAndOrientation };

  /// Gets the type of inverse kinematics that will be performed.
  /// @note Currently only usable for numerical IK.
  InverseKinematicsType ik_type() const { return ik_type_; }

  /// Sets the type of inverse kinematics that will be performed.
  /// @note Currently only usable for numerical IK.
  void set_ik_type(InverseKinematicsType ik_type) { ik_type_ = ik_type; }

  /// Gets the number of seeds to evaluate; more seeds implies higher solution accuracy (to a point).
  /// @note only relevant when `use_numerical_ik()` is `true`.
  int num_seeds() const { return num_seeds_to_evaluate_; }

  /// Sets the number of seeds to evaluate.
  /// @note only relevant when `use_numerical_ik()` is `true`.
  void set_num_seeds(int num_seeds) { num_seeds_to_evaluate_ = num_seeds; }

  /// Gets the maximum number of iterations used in the inverse kinematics function's Newton-Raphson solver.
  /// @note only relevant when `use_numerical_ik()` is `true`.
  int ik_maximum_newton_raphson_iterations() const { return 12; }

  /// Gets the tolerance for the requested accuracy in the IK solution.
  /// @note only relevant when `use_numerical_ik()` is `true`.
  double ik_tolerance() const { return ik_tolerance_; }

  /// Sets the IK tolerance.
  /// @note only relevant when `use_numerical_ik()` is `true`.
  void set_ik_tolerance(double tol) { ik_tolerance_ = tol; }

  /// Gets the lower limits for the generalized positions of the given model instance.
  const drake::VectorX<T>& lower_limits(drake::multibody::ModelInstanceIndex model_instance) const {
    return lower_limits_[model_instance];
  }

  /// Gets the upper limits for the generalized positions of the given model instance.
  const drake::VectorX<T>& upper_limits(drake::multibody::ModelInstanceIndex model_instance) const {
    return upper_limits_[model_instance];
  }

  /** Sets the seed to use for inverse kinematics. If no value is specified, the inverse kinematics algorithm will
   search for a good seed.
   @note only relevant when `use_numerical_ik()` is `true`.
  */
  void set_numerical_inverse_kinematics_seed(drake::optional<drake::VectorX<T>> q_seed = {}) {
    if (q_seed.has_value()) {
      q_ik_seed_ = q_seed.value();
    } else {
      q_ik_seed_.reset();
    }
  }

  /// Gets whether resolved-motion rate control is used for inverse kinematics. Default is `false` (inverse kinematics
  /// is solved using a closed form solution).
  bool use_numerical_ik() const { return use_numerical_ik_; }

  /// Sets whether resolved-motion rate control is used for inverse kinematics.
  void set_use_numerical_ik(bool flag) { use_numerical_ik_ = flag; }

  /**
   Solves the velocity kinematics function V = f_G(q, v) for generalized velocity v given Frame F, spatial velocity V,
   generalized position q, and offset vector p_FG from Frame F to Frame G (expressed in Frame F). More precisely,
   `V_WG(q, v)` is the spatial velocity of frame `G` measured and expressed in the world frame W.
   */
  drake::VectorX<T> SolveVelocityInverseKinematics(const drake::multibody::Frame<T>& F, const drake::Vector3<T>& p_FG,
                                                   drake::multibody::ModelInstanceIndex, const drake::VectorX<T>& q,
                                                   const drake::VectorX<T>& V) const {
    // Set the generalized positions in the context.
    plant_.SetPositions(plant_context_.get(), q);

    // TODO(edrumwri) We need a Jacobian method that only considers the velocity variables from the model instance.
    // Set Jv_WG to the proper size, per MultibodyPlant documentation.
    Jv_WG_.resize(6, plant_.num_velocities());

    // Form the Jacobian matrix.
    const drake::multibody::Frame<double>& world_frame = plant_.world_frame();
    plant_.CalcJacobianSpatialVelocity(*plant_context_, drake::multibody::JacobianWrtVariable::kV, F, p_FG, world_frame,
                                       world_frame, &Jv_WG_);

    // Solve using least-squares QR factorization. We could perhaps do a full-rank pseudoinverse, but QR is likely
    // not as expensive and is robust to singularities.
    QR_Jv_WG_.compute(Jv_WG_);
    return QR_Jv_WG_.solve(V);
  }

  /**
   Solves the velocity kinematics function w = f_G(q, v) for generalized velocity v given Frame F, angular velocity w,
   generalized position q, and offset vector p_FG from Frame F to Frame G (expressed in Frame F). More precisely,
   `w_WG(q, v)` is the angular velocity of frame `G` expressed in the world frame W.
   */
  drake::VectorX<T> SolveOrientationVelocityInverseKinematics(const drake::multibody::Frame<T>& F,
                                                              const drake::Vector3<T>& p_FG,
                                                              drake::multibody::ModelInstanceIndex,
                                                              const drake::VectorX<T>& q,
                                                              const drake::Vector3<T>& w) const {
    // Set the generalized positions in the context.
    plant_.SetPositions(plant_context_.get(), q);

    // TODO(edrumwri) We need a Jacobian method that only considers the velocity variables from the model instance.
    // Set Jv_WG to the proper size, per MultibodyPlant documentation.
    Jv_WG_.resize(6, plant_.num_velocities());

    // TODO(edrumwri): Replace this with a call that only computes the needed components.
    // Form the Jacobian matrix.
    const drake::multibody::Frame<double>& world_frame = plant_.world_frame();
    plant_.CalcJacobianSpatialVelocity(*plant_context_, drake::multibody::JacobianWrtVariable::kV, F, p_FG, world_frame,
                                       world_frame, &Jv_WG_);

    // Solve using least-squares QR factorization. We could perhaps do a full-rank pseudoinverse, but QR is likely
    // not as expensive and is robust to singularities.
    QR_Jv_WG_.compute(Jv_WG_.template topRows<3>());
    return QR_Jv_WG_.solve(w);
  }

  /**
   Solves the velocity kinematics function xdot = f_G(q, v) for generalized velocity v given Frame F, translational,
   velocity xdot, generalized position q, and offset vector p_FG from Frame F to Frame G (expressed in Frame F). More
   precisely, `xdot_WG(q, v)` is the translational velocity of frame `G` measured and expressed in the world frame W.
   */
  drake::VectorX<T> SolveTranslationalVelocityInverseKinematics(const drake::multibody::Frame<T>& F,
                                                                const drake::Vector3<T>& p_FG,
                                                                drake::multibody::ModelInstanceIndex,
                                                                const drake::VectorX<T>& q,
                                                                const drake::Vector3<T>& xdot) const {
    // Set the generalized positions in the context.
    plant_.SetPositions(plant_context_.get(), q);

    // TODO(edrumwri) We need a Jacobian method that only considers the velocity variables from the model instance.
    // Set Jv_WG to the proper size, per MultibodyPlant documentation.
    Jv_WG_.resize(6, plant_.num_velocities());

    // TODO(edrumwri): Replace this with a call that only computes the needed components.
    // Form the Jacobian matrix.
    const drake::multibody::Frame<double>& world_frame = plant_.world_frame();
    plant_.CalcJacobianSpatialVelocity(*plant_context_, drake::multibody::JacobianWrtVariable::kV, F, p_FG, world_frame,
                                       world_frame, &Jv_WG_);

    // Solve using least-squares QR factorization. We could perhaps do a full-rank pseudoinverse, but QR is likely
    // not as expensive and is robust to singularities.
    QR_Jv_WG_.compute(Jv_WG_.template bottomRows<3>());
    return QR_Jv_WG_.solve(xdot);
  }

  /// Computes the operational space differential between rotation matrices.
  drake::Vector3<T> CalcOrientationDifferential(const drake::VectorX<T>& q, const drake::multibody::Frame<T>& F,
                                                const drake::math::RotationMatrix<T>& R_FA,
                                                const drake::math::RotationMatrix<T>& R_FB) const {
    const drake::Vector3<T> omega = DifferentialInverseKinematics<T>::CalcOrientationDifferential(R_FA, R_FB);

    // Get the body to which this frame is attached.
    const drake::multibody::Body<T>& chopstick_body = F.body();

    // Get the direction of the chopstick's longitudinal axis (the +x axis in the chopstick body frame)
    // in the world frame. We rely upon some inside knowledge to do this: we know that the forward kinematics
    // function is always going to be called before this function, so we know that the generalized positions used in
    // the forward kinematics computation are the generalized positions that correspond to the chopstick pose we want.
    plant_.SetPositions(plant_context_.get(), q);
    const drake::math::RotationMatrix<T>& R_WC = plant_.EvalBodyPoseInWorld(*plant_context_, chopstick_body).rotation();
    const drake::Vector3<T> direction = R_WC * drake::Vector3<T>(1, 0, 0);

    return DifferentialInverseKinematics<T>::RemoveAngularVelocityComponentsAlongDirection(omega, direction);
  }

  /** Implements InverseKinematics<T>::SolveInverseKinematics().
   */
  drake::VectorX<T> SolveInverseKinematics(const drake::math::RigidTransform<T>& X_WG_target,
                                           const drake::Vector3<T>& p_FG,
                                           const drake::multibody::Frame<T>& F) const final {
    if (use_numerical_ik_) {
      return SolveInverseKinematicsNumerical(X_WG_target, p_FG, F);
    } else {
      return SolveInverseKinematicsAnalytical(X_WG_target, p_FG, F);
    }
  }

  /** Convenience function for computing the forward kinematics of a Frame G rigidly attached to Frame F.
   @param q the configuration of the robot for which the frame velocity should be computed.
   @param p_FG the vector offset from the origin of Frame F to Frame G, expressed in F's frame.
   @param F the frame of interest.
   */
  drake::math::RigidTransform<T> CalcForwardKinematics(const drake::VectorX<T>& q, const drake::Vector3<T>& p_FG,
                                                       const drake::multibody::Frame<T>& F) const {
    plant_.SetPositions(plant_context_.get(), q);

    // We want X_WG = X_WF * X_FG, where X_FG represents an identity orientation with p_FG as the translation
    // vector. We use that property to optimize the computation of X_WG:
    // X_WG = | R_WF  p_WoFo | * | I  p_FoGo |
    //        | 0     1      | * | 0  1      |
    //
    // For clarity, we have used the longform of the monogram notation, so p_FoGo is equivalent to p_FG. Therefore:
    // X_WG = | R_WF  R_WF * pFoGo + p_WoFo |
    //        | 0     1                     |
    const drake::math::RigidTransform<T> X_WF = F.CalcPoseInWorld(*plant_context_);
    return drake::math::RigidTransform<T>(X_WF.rotation(), X_WF * p_FG);
  }

  /** Convenience function for computing the velocity of a Frame G rigidly attached to Frame F.
   @param q the configuration of the robot for which the frame velocity should be computed.
   @param v the velocity of the robot for which the frame velocity should be computed.
   @param p_FG the vector offset from the origin of Frame F to Frame G, expressed in F's frame.
   @param F the frame of interest.
   */
  drake::multibody::SpatialVelocity<T> CalcFrameVelocity(const drake::VectorX<T>& q, const drake::VectorX<T>& v,
                                                         const drake::Vector3<T>& p_FG,
                                                         const drake::multibody::Frame<T>& F) const {
    plant_.SetPositions(plant_context_.get(), q);
    plant_.SetVelocities(plant_context_.get(), v);

    const drake::math::RigidTransform<T>& X_WF = plant_.EvalBodyPoseInWorld(*plant_context_, F.body());
    const drake::multibody::SpatialVelocity<T>& X_WFo_W =
        plant_.EvalBodySpatialVelocityInWorld(*plant_context_, F.body());
    const drake::Vector3<T> p_FoGo_W = X_WF.rotation() * p_FG;
    return X_WFo_W.Shift(p_FoGo_W);
  }

  // TODO(edrumwri) This function needs to be unit tested.
  /** Convenience function for computing the acceleration of a Frame G rigidly attached to Frame F.
   @param q the configuration of the robot for which the frame acceleration should be computed.
   @param v the velocity of the robot for which the frame acceleration should be computed.
   @param vdot the acceleration of the robot for which the frame acceleration should be computed.
   @param p_FG the vector offset from the origin of Frame F to Frame G, expressed in F's frame.
   @param F the frame of interest.
   */
  drake::Vector6<T> CalcFrameAcceleration(const drake::VectorX<T>& q, const drake::VectorX<T>& v,
                                          const drake::VectorX<T>& vdot, const drake::Vector3<T>& p_FG,
                                          const drake::multibody::Frame<T>& F) const {
    plant_.SetPositions(plant_context_.get(), q);
    plant_.SetVelocities(plant_context_.get(), v);

    const drake::multibody::Frame<double>& world_frame = plant_.world_frame();
    plant_.CalcJacobianSpatialVelocity(*plant_context_, drake::multibody::JacobianWrtVariable::kV, F, p_FG, world_frame,
                                       world_frame, &J_);
    const drake::Vector6<T> Jdot_v = plant_.CalcBiasForJacobianSpatialVelocity(
        *plant_context_, drake::multibody::JacobianWrtVariable::kV, F, p_FG, world_frame, world_frame);
    return J_ * vdot + Jdot_v;
  }

 private:
  // Analytical IK solutions computed using sympy.
  // Note: This approach only computes inverse kinematics for the end-link, which is all we need with the current robot
  // design.
  drake::VectorX<T> SolveInverseKinematicsAnalytical(const drake::math::RigidTransform<T>& X_WG_target,
                                                     const drake::Vector3<T>& p_FG,
                                                     const drake::multibody::Frame<T>& F) const {
    using std::acos;
    using std::asin;
    using std::pow;
    using std::sqrt;

    // TODO(edrumwri) Implement position-only and orientation-only IK approaches to eliminate this demand.
    // Verify that the ik type is both position and orientation.
    DR_DEMAND(ik_type_ == InverseKinematicsType::kPositionAndOrientation);

    // Note that each robot is mounted to a particular base frame, B. The transformation X_WB gives the relative pose
    // of B in the world frame frame. We want the analytical solution to compute the result q such that X_WG- the end
    // effector frame with respect to the world frame- is equal to X_WG_target. We define X_WG_target = X_WB * X_BG;
    // however, the inverse kinematics method computes the solution for X_BG. So, to get it to compute the correct
    // solution, we need to compute X_BG_target = X_WB⁻¹ * X_WG_target.

    // Get the body to which this frame is attached.
    const drake::multibody::Body<T>& chopstick_body = F.body();
    DRAKE_DEMAND(chopstick_body.name() == "end_effector");

    // Find out which model instance this belongs to.
    const drake::multibody::BodyIndex body_index = body_to_body_index_map_.at(&chopstick_body);
    const drake::multibody::ModelInstanceIndex model_instance = body_index_to_model_instance_index_map_.at(body_index);

    // Compute X_BG target.
    const drake::math::RigidTransform<T> X_BG_target = X_WB_[model_instance].value().inverse() * X_WG_target;

    // Get the relevant parts of the transformation matrix.
    const double x1 = X_BG_target.translation()[0];
    const double x2 = X_BG_target.translation()[1];
    const double x3 = X_BG_target.translation()[2];
    const drake::Matrix3<double> R_BG_target = X_BG_target.rotation().matrix();
    const double r11 = R_BG_target(0, 0);
    const double r12 = R_BG_target(0, 1);
    const double r13 = R_BG_target(0, 2);
    const double r21 = R_BG_target(1, 0);
    const double r22 = R_BG_target(1, 1);
    const double r23 = R_BG_target(1, 2);
    const double r31 = R_BG_target(2, 0);
    const double r32 = R_BG_target(2, 1);
    const double r33 = R_BG_target(2, 2);
    const double p_FG1 = p_FG[0];
    const double p_FG2 = p_FG[1];
    const double p_FG3 = p_FG[2];

    // Seems like we should be using some of these, perhaps with atan2 instead
    // of asin/acos?
    drake::unused(r12);
    drake::unused(r13);
    drake::unused(r22);
    drake::unused(r21);
    drake::unused(r23);
    drake::unused(r33);

    // The zero tolerance will depend on the accuracy with which the rotation matrix is computed as well as the
    // trigonometric operations below. This should be about right.
    const double zero_tolerance = 1e-14;

    // Note: steps for changing the model:
    // 1. Update the poses in closed_form_ik.py and resolve.
    // 2. Copy one of the solutions for each chopstick to below. Remove multiples of M_PI*2, as it is unnecessary.
    // 3. Replace pow(x, 2) and pow(x, 3) with x*x and x*x*x for faster computation.
    // 4. Update the ordering of the q_model_ array below, as necessary.
    // 5. Build and run unit tests; if an error is observed corresponding to no solution existing, there is likely
    //    an added/subtracted M_PI that should be removed from the roll variable computation.
    // 6. Tests still not passing? Check the forward kinematics function of closed_form_ik.py against the forward
    //    forward kinematics function in Drake.

    // Determine which model instance is used.
    if (model_instance == left_chopstick_model_) {
      // Before attempting to compute the solution, ensure that the roll will be zero.
      const double roll = asin(r32 / sqrt(-r31 * r31 + 1.0));
      if (std::abs(roll) > zero_tolerance) throw std::runtime_error("No solution exists for the desired target pose.");

      // Otherwise, compute the solution.
      const double x = 0.03125 * (32.0 * p_FG2 * pow(-r31 * r31 + 1.0, 5.0L / 2.0L) *
                                      (r11 * r31 * r32 -
                                       r31 * r31 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) *
                                           sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) +
                                       sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) *
                                           sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0))) -
                                  32.0 * p_FG3 * pow(r31 * r31 - 1.0, 2) *
                                      (r11 * r31 * r31 * r31 * sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) -
                                       r11 * r31 * sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) -
                                       r31 * r31 * r32 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) +
                                       r32 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0))) +
                                  32.0 * pow(-r31 * r31 + 1.0, 7.0L / 2.0L) * (-p_FG1 * r11 + x1) +
                                  3.0 * pow(-r31 * r31 + 1.0, 5.0L / 2.0L) *
                                      (-r11 * r31 * r32 +
                                       r31 * r31 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) *
                                           sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) -
                                       sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) *
                                           sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)))) /
                       pow(-r31 * r31 + 1.0, 7.0L / 2.0L);
      const double y =
          0.03125 *
          (-32.0 * p_FG3 * pow(-r31 * r31 + 1.0, 3.0L / 2.0L) *
               (r11 * r32 -
                r31 * r31 * r31 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) *
                    sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) +
                r31 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) *
                    sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0))) +
           32.0 * pow(-r31 * r31 + 1.0, 5.0L / 2.0L) *
               (p_FG1 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) * sqrt(-r31 * r31 + 1.0) - x2) +
           (r31 * r31 - 1.0) *
               (32.0 * p_FG2 * r11 * r31 * r31 * sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) -
                32.0 * p_FG2 * r11 * sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) -
                32.0 * p_FG2 * r31 * r31 * r31 * r32 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) +
                32.0 * p_FG2 * r31 * r32 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) -
                3.0 * r11 * r31 * r31 * sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) +
                3.0 * r11 * sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) +
                3.0 * r31 * r31 * r31 * r32 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) -
                3.0 * r31 * r32 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)))) /
          pow(-r31 * r31 + 1.0, 5.0L / 2.0L);
      const double z = -p_FG1 * r31 - p_FG2 * r32 -
                       p_FG3 * sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) * sqrt(-r31 * r31 + 1.0) +
                       0.09375 * r32 + x3;
      const double pitch = asin(r31);
      const double yaw = acos(r11 / sqrt(-r31 * r31 + 1.0));

      // Set the joint positions for the model. Note that this is predicated on the layout as specified in the SDF file.
      q_model_[0] = y;
      q_model_[1] = x;
      q_model_[2] = yaw;
      q_model_[3] = z;
      q_model_[4] = pitch;
    } else {
      DR_DEMAND(model_instance == right_chopstick_model_);
      const double roll = asin(r32 / sqrt(-r31 * r31 + 1.0));
      if (std::abs(roll) > zero_tolerance) throw std::runtime_error("No solution exists for the desired target pose.");

      // Otherwise, compute the solution.
      const double x = 0.03125 * (32.0 * p_FG2 * pow(-r31 * r31 + 1.0, 5.0L / 2.0L) *
                                      (r11 * r31 * r32 -
                                       r31 * r31 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) *
                                           sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) +
                                       sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) *
                                           sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0))) -
                                  32.0 * p_FG3 * pow(r31 * r31 - 1.0, 2) *
                                      (r11 * r31 * r31 * r31 * sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) -
                                       r11 * r31 * sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) -
                                       r31 * r31 * r32 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) +
                                       r32 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0))) +
                                  32.0 * pow(-r31 * r31 + 1.0, 7.0L / 2.0L) * (-p_FG1 * r11 + x1) +
                                  3.0 * pow(-r31 * r31 + 1.0, 5.0L / 2.0L) *
                                      (r11 * r31 * r32 -
                                       r31 * r31 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) *
                                           sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) +
                                       sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) *
                                           sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)))) /
                       pow(-r31 * r31 + 1.0, 7.0L / 2.0L);
      const double y =
          0.03125 *
          (32.0 * p_FG3 * pow(-r31 * r31 + 1.0, 3.0L / 2.0L) *
               (r11 * r32 -
                r31 * r31 * r31 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) *
                    sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) +
                r31 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) *
                    sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0))) +
           32.0 * pow(-r31 * r31 + 1.0, 5.0L / 2.0L) *
               (-p_FG1 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) * sqrt(-r31 * r31 + 1.0) + x2) -
           (r31 * r31 - 1.0) *
               (32.0 * p_FG2 * r11 * r31 * r31 * sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) -
                32.0 * p_FG2 * r11 * sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) -
                32.0 * p_FG2 * r31 * r31 * r31 * r32 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) +
                32.0 * p_FG2 * r31 * r32 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) +
                3.0 * r11 * r31 * r31 * sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) -
                3.0 * r11 * sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) -
                3.0 * r31 * r31 * r31 * r32 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)) +
                3.0 * r31 * r32 * sqrt((r11 * r11 + r31 * r31 - 1.0) / (r31 * r31 - 1.0)))) /
          pow(-r31 * r31 + 1.0, 5.0L / 2.0L);
      const double z = -p_FG1 * r31 - p_FG2 * r32 -
                       p_FG3 * sqrt((r31 * r31 + r32 * r32 - 1.0) / (r31 * r31 - 1.0)) * sqrt(-r31 * r31 + 1.0) -
                       0.09375 * r32 + x3;
      const double pitch = -asin(r31);
      const double yaw = acos(r11 / sqrt(-r31 * r31 + 1.0));

      // Set the joint positions for the model. Note that this is predicated on the layout as specified in the SDF file.
      q_model_[0] = y;
      q_model_[1] = x;
      q_model_[2] = yaw;
      q_model_[3] = z;
      q_model_[4] = pitch;
    }

    return q_model_;
  }

  /*
   Performs a numerical IK procedure for orientation only.
   @note The current implementation seems to be limited by the inability of the orientation differential function to
         remove all angular differential components not reachable from the current configuration, with the result being
         that solutions may not always be found. Increasing the number of seeds can help the solver find a solution to
         tolerance, though usually an increase of one or two orders of magnitude over the nominal number of
         seeds will be necessary.
 */
  drake::VectorX<T>& SolveInverseKinematicsNumericalOrientationOnly(
      const drake::math::RigidTransform<T>& X_WG_target, const drake::Vector3<T>& p_FG,
      const drake::multibody::Frame<T>& F, const drake::multibody::Body<T>& chopstick_body) const {
    // Set the lambda for computing the full operational space differential.
    const auto f_diff = [this, &chopstick_body](const drake::math::RigidTransform<T>& X_FA,
                                                const drake::math::RigidTransform<T>& X_FB) -> drake::VectorX<T> {
      // Compute the normal rotational differential.
      delta_x_ = DifferentialInverseKinematics<T>::CalcOrientationDifferential(X_FA.rotation(), X_FB.rotation());

      // Get the direction of the chopstick's longitudinal axis (the +x axis in the chopstick body frame)
      // in the world frame. We rely upon some inside knowledge to do this: we know that the forward kinematics
      // function is always going to be called before this function, so we know that the generalized positions used in
      // the forward kinematics computation are the generalized positions that correspond to the chopstick pose we want.
      const drake::math::RigidTransform<T>& X_WC = plant_.EvalBodyPoseInWorld(*plant_context_, chopstick_body);
      const drake::Vector3<T> direction = X_WC.rotation() * drake::Vector3<T>(1, 0, 0);

      // Remove the components aligned with this direction from delta_x.
      return DifferentialInverseKinematics<T>::RemoveAngularVelocityComponentsAlongDirection(delta_x_, direction);
    };

    // Call the differential IK solver on the orientation only using the best seed found.
    q_ = SolveOrientationalInverseKinematics(X_WG_target, p_FG, F, q_ik_seed_);

    // Check whether the solution lies within the given tolerance.
    const double squared_tol = ik_tolerance() * ik_tolerance();
    const double solution_error = f_diff(CalcForwardKinematics(q_, p_FG, F), X_WG_target).squaredNorm();
    if (solution_error > squared_tol) throw std::runtime_error("Could not find a solution to the desired tolerance.");

    return q_;
  }

  /* Leverages the information that the degrees of freedom that control chopstick orientation in our model are located
   closer to the chopstick than the prismatic DoFs. This means that we can solve for generalized coordinates that attain
   the desired orientation first; solving for the desired translation thereafter will not affect the orientation
   (barring nastiness in the Jacobian pseudoinversion).
   @note See note in SolveInverseKinematicsNumericalOrientationOnly().
  */
  drake::VectorX<T>& SolveInverseKinematicsNumericalPositionAndOrientation(
      const drake::math::RigidTransform<T>& X_WG_target, const drake::Vector3<T>& p_FG,
      const drake::multibody::Frame<T>& F, const drake::multibody::Body<T>& chopstick_body) const {
    // Set the lambda for computing the full operational space differential.
    const auto f_diff = [this, &chopstick_body](const drake::math::RigidTransform<T>& X_FA,
                                                const drake::math::RigidTransform<T>& X_FB) -> drake::VectorX<T> {
      // Compute the normal operational space differential.
      const bool angular_components_on_top = true;  // Drake's Jacobians are ordered this way.
      delta_x_ =
          DifferentialInverseKinematics<T>::CalcOperationalSpaceDifferential(X_FA, X_FB, angular_components_on_top);

      // Get the direction of the chopstick's longitudinal axis (the +x axis in the chopstick body frame)
      // in the world frame. We rely upon some inside knowledge to do this: we know that the forward kinematics
      // function is always going to be called before this function, so we know that the generalized positions used in
      // the forward kinematics computation are the generalized positions that correspond to the chopstick pose we want.
      const drake::math::RigidTransform<T>& X_WC = plant_.EvalBodyPoseInWorld(*plant_context_, chopstick_body);
      const drake::Vector3<T> direction = X_WC.rotation() * drake::Vector3<T>(1, 0, 0);

      // Remove the components aligned with this direction from delta_x.
      const drake::Vector3<T> omega = delta_x_.template head<3>();
      delta_x_.template head<3>() =
          DifferentialInverseKinematics<T>::RemoveAngularVelocityComponentsAlongDirection(omega, direction);
      return delta_x_;
    };

    // Call the differential IK solver on the orientation only using the best seed found.
    q_seed_ = SolveOrientationalInverseKinematics(X_WG_target, p_FG, F, q_ik_seed_);

    // Call the differential IK solver on the full problem, using the solution for orientation as the seed.
    q_ = SolveTranslationalInverseKinematics(X_WG_target, p_FG, F, q_seed_);

    // Check whether the solution lies within the given tolerance.
    const double squared_tol = ik_tolerance() * ik_tolerance();
    const double solution_error = f_diff(CalcForwardKinematics(q_, p_FG, F), X_WG_target).squaredNorm();
    if (solution_error > squared_tol) throw std::runtime_error("Could not find a solution to the desired tolerance.");

    return q_;
  }

  // Solves only the translational part of the IK problem.
  drake::VectorX<T> SolveTranslationalInverseKinematics(const drake::math::RigidTransform<T>& X_WG_target,
                                                        const drake::Vector3<T>& p_FG,
                                                        const drake::multibody::Frame<T>& F,
                                                        const drake::VectorX<T>& q_seed) const {
    q_ = drake::VectorX<T>::Zero(plant_.num_positions());

    // Get the body to which this frame is attached.
    const drake::multibody::Body<T>& chopstick_body = F.body();

    // Define the translational space differential function.
    auto f_diff_translate = [this, &chopstick_body](const drake::VectorX<T>&,
                                                    const drake::math::RigidTransform<T>& X_FA,
                                                    const drake::math::RigidTransform<T>& X_FB) -> drake::VectorX<T> {
      return DifferentialInverseKinematics<T>::CalcPositionDifferential(X_FA.translation(), X_FB.translation());
    };

    // Get the model instance that corresponds to this frame.
    const drake::multibody::ModelInstanceIndex model_instance =
        body_index_to_model_instance_index_map_.at(body_to_body_index_map_.at(&F.body()));

    // Set the solver function.
    auto f_qdiff = [this, model_instance, &p_FG, &F](const drake::VectorX<T>& q,
                                                     const drake::VectorX<T>& xdot) -> drake::VectorX<T> {
      v_ = SolveTranslationalVelocityInverseKinematics(F, p_FG, model_instance, q, xdot);
      plant_.SetPositions(plant_context_.get(), q);
      plant_.MapVelocityToQDot(*plant_context_, v_, qdot_.get());
      return qdot_->CopyToVector();
    };

    // Set constants.
    const double tol = ik_tolerance();
    const int max_iterations = ik_maximum_newton_raphson_iterations();

    // Call the differential IK solver on the full problem, using the solution for orientation as the seed.
    return differential_ik_.SolveInverseKinematics(
        X_WG_target, [this, &p_FG, &F](const drake::VectorX<T>& q) { return CalcForwardKinematics(q, p_FG, F); },
        f_diff_translate, f_qdiff, q_seed, max_iterations, tol);
  }

  // Solves only the orientational part of the IK problem.
  drake::VectorX<T> SolveOrientationalInverseKinematics(const drake::math::RigidTransform<T>& X_WG_target,
                                                        const drake::Vector3<T>& p_FG,
                                                        const drake::multibody::Frame<T>& F,
                                                        const drake::optional<drake::VectorX<T>>& q_seed) const {
    q_ = drake::VectorX<T>::Zero(plant_.num_positions());

    // Define the orientational space differential function.
    auto f_diff_ori = [this, &F](const drake::VectorX<T>& q, const drake::math::RigidTransform<T>& X_FA,
                                 const drake::math::RigidTransform<T>& X_FB) -> drake::Vector3<T> {
      return CalcOrientationDifferential(q, F, X_FA.rotation(), X_FB.rotation());
    };

    // Get the model instance that corresponds to this frame.
    const drake::multibody::ModelInstanceIndex model_instance =
        body_index_to_model_instance_index_map_.at(body_to_body_index_map_.at(&F.body()));

    // Set the solver function.
    auto f_qdiff = [this, model_instance, &p_FG, &F](const drake::VectorX<T>& q,
                                                     const drake::VectorX<T>& xdot) -> drake::VectorX<T> {
      v_ = SolveOrientationVelocityInverseKinematics(F, p_FG, model_instance, q, xdot);
      plant_.SetPositions(plant_context_.get(), q);
      plant_.MapVelocityToQDot(*plant_context_, v_, qdot_.get());
      return qdot_->CopyToVector();
    };

    // Set constants.
    const double tol = ik_tolerance();
    const int max_iterations = ik_maximum_newton_raphson_iterations();

    // Use the provided seed, if any.
    if (q_seed.has_value()) {
      best_seed_ = q_seed.value();
    } else {
      // Find the best seed.
      SobolSequence<T> sequence(2 /* Number of degrees of freedom to sample over */);
      double best_seed_eval = std::numeric_limits<double>::max();
      q_seed_.setZero(plant_.num_positions());
      for (int i = 0; i < num_seeds(); ++i) {
        const drake::Vector2<T> sample = sequence.Sample();
        for (int j = 0; j < static_cast<int>(revolute_joint_indices_[model_instance].size()); ++j)
          q_seed_[plant_.get_joint(revolute_joint_indices_[model_instance][j]).position_start()] =
              -M_PI + sample[j] * (2.0 * M_PI);

        double eval = f_diff_ori(q_seed_, CalcForwardKinematics(q_seed_, p_FG, F), X_WG_target).squaredNorm();
        if (eval < best_seed_eval) {
          best_seed_eval = eval;
          best_seed_ = q_seed_;
        }
      }
    }

    // Call the differential IK solver.
    return differential_ik_.SolveInverseKinematics(
        X_WG_target, [this, &p_FG, &F](const drake::VectorX<T>& q) { return CalcForwardKinematics(q, p_FG, F); },
        f_diff_ori, f_qdiff, best_seed_, max_iterations, tol);
  }

  drake::VectorX<T> SolveInverseKinematicsNumerical(const drake::math::RigidTransform<T>& X_WG_target,
                                                    const drake::Vector3<T>& p_FG,
                                                    const drake::multibody::Frame<T>& F) const {
    // Get the body to which this frame is attached.
    const drake::multibody::Body<T>& chopstick_body = F.body();

    drake::VectorX<T>* q = nullptr;
    switch (ik_type_) {
      case InverseKinematicsType::kPositionOnly:
        q = &SolveInverseKinematicsNumericalPositionOnly(X_WG_target, p_FG, F, chopstick_body);
        break;

      case InverseKinematicsType::kOrientationOnly:
        q = &SolveInverseKinematicsNumericalOrientationOnly(X_WG_target, p_FG, F, chopstick_body);
        break;

      case InverseKinematicsType::kPositionAndOrientation:
        q = &SolveInverseKinematicsNumericalPositionAndOrientation(X_WG_target, p_FG, F, chopstick_body);
        break;
    }
    DR_DEMAND(q);

    // Get just the part that corresponds to this model.
    const drake::multibody::BodyIndex body_index = body_to_body_index_map_.at(&chopstick_body);
    const drake::multibody::ModelInstanceIndex model_instance = body_index_to_model_instance_index_map_.at(body_index);
    q_model_ = plant_.GetPositionsFromArray(model_instance, *q);

    return q_model_;
  }

  drake::VectorX<T>& SolveInverseKinematicsNumericalPositionOnly(
      const drake::math::RigidTransform<T>& X_WG_target, const drake::Vector3<T>& p_FG,
      const drake::multibody::Frame<T>& F, const drake::multibody::Body<T>& chopstick_body) const {
    const auto f_diff = [this, &chopstick_body](const drake::math::RigidTransform<T>& X_FA,
                                                const drake::math::RigidTransform<T>& X_FB) -> drake::VectorX<T> {
      return DifferentialInverseKinematics<T>::CalcPositionDifferential(X_FA.translation(), X_FB.translation());
    };

    // Call the differential IK solver on the full problem, using the solution for orientation as the seed.
    q_seed_.setZero();
    q_ = SolveTranslationalInverseKinematics(X_WG_target, p_FG, F, q_seed_);

    // Check whether the solution lies within the given tolerance.
    const double squared_tol = ik_tolerance() * ik_tolerance();
    const double solution_error = f_diff(CalcForwardKinematics(q_, p_FG, F), X_WG_target).squaredNorm();
    if (solution_error > squared_tol) throw std::runtime_error("Could not find a solution to the desired tolerance.");

    return q_;
  }

  // The plant that contains the robot.
  const drake::multibody::MultibodyPlant<T>& plant_;

  // Note: these variables are mutable because changing their values will not alter the value of any computations: they
  // are temporary variable intended to minimize heap allocations.
  mutable drake::MatrixX<T> Jv_WG_;
  mutable Eigen::ColPivHouseholderQR<drake::MatrixX<T>> QR_Jv_WG_;
  mutable std::unique_ptr<drake::systems::Context<T>> plant_context_;
  mutable drake::VectorX<T> v_;
  mutable std::unique_ptr<drake::systems::BasicVector<T>> qdot_;
  mutable drake::VectorX<T> q_seed_, q_, best_seed_, q_model_;
  mutable drake::VectorX<T> delta_x_;
  mutable DifferentialInverseKinematics<T> differential_ik_;
  mutable unsigned seed_;
  mutable drake::optional<drake::VectorX<T>> q_ik_seed_;

  // Whether an IK solution is computed using a numerical approach.
  bool use_numerical_ik_;

  // The number of seeds to evaluate.
  int num_seeds_to_evaluate_;

  // The error tolerance at which a candidate configuration is considered to solve an IK problem.
  double ik_tolerance_;

  // The indices of the revolute joints, ordered per model instance.
  std::vector<std::vector<drake::multibody::JointIndex>> revolute_joint_indices_;

  // Allows quickly determining which model index corresponds to a body index.
  std::unordered_map<drake::multibody::BodyIndex, drake::multibody::ModelInstanceIndex>
      body_index_to_model_instance_index_map_;

  // Allows quickly determining which body index corresponds to a Body.
  std::unordered_map<const drake::multibody::Body<T>*, drake::multibody::BodyIndex> body_to_body_index_map_;

  // The limits on the generalized positions of the robot, per model instance.
  std::vector<drake::VectorX<T>> lower_limits_, upper_limits_;

  // The model instance indices per chopstick.
  drake::multibody::ModelInstanceIndex left_chopstick_model_, right_chopstick_model_;

  // Relative poses between the robot bases and the world.
  std::vector<drake::optional<drake::math::RigidTransform<T>>> X_WB_;

  // The type of numerical inverse kinematics that is used.
  InverseKinematicsType ik_type_{InverseKinematicsType::kPositionAndOrientation};

  // Jacobian matrix used for mapping generalized velocities to link velocities. Note that this is mutable to avoid heap
  // allocations; changing this variable will not change the result of any computations.
  mutable drake::MatrixX<T> J_;
};

}  // namespace DR
