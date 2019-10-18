#include <cstdlib>

#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>

#include <DR/common/environment.h>
#include <DR/drivers/chopstick/chopstick_config.h>
#include <DR/drivers/chopstick/chopstick_kinematics.h>
#include <DR/simulation/model_generator.h>

#include <gtest/gtest.h>

using drake::Vector3;
using drake::VectorX;
using drake::math::RigidTransform;
using drake::math::RotationMatrix;
using Framed = drake::multibody::Frame<double>;
using RigidTransformd = RigidTransform<double>;
using RotationMatrixd = RotationMatrix<double>;
using Vector3d = Vector3<double>;
using VectorXd = VectorX<double>;
using MatrixXd = drake::MatrixX<double>;

namespace DR {
namespace {

enum ChopstickId { kLeft = 0, kRight = 1 };

class ChopstickTest : public ::testing::TestWithParam<ChopstickId> {
 public:
  // Gets the model instance corresponding to a particular chopstick.
  drake::multibody::ModelInstanceIndex model_instance(ChopstickId id) {
    switch (id) {
      case kLeft:
        return left_instance_;
      case kRight:
        return right_instance_;
    }

    DRAKE_UNREACHABLE();
  }

  const drake::multibody::MultibodyPlant<double>& plant() const { return *plant_; }
  drake::systems::Context<double>& context() const { return *context_; }
  const ChopstickKinematics<double>& kinematics() const { return *kinematics_; }
  ChopstickKinematics<double>& kinematics() { return *kinematics_; }

  Vector3d CalcDifferential(const RotationMatrixd& R1, const RotationMatrixd& R2) const {
    // R1 + dR = R2, where dR = \skew{omega}*R1
    //        \skew{omega}*R1 = (R2 - R1)
    //           \skew{omega} = (R2 - R1)*R1'
    const drake::Matrix3<double> skew_omega = (R2.matrix() - R1.matrix()) * R1.matrix().transpose();
    const double s_xy = skew_omega(0, 1);
    const double s_xz = skew_omega(0, 2);
    const double s_yx = skew_omega(1, 0);
    const double s_yz = skew_omega(1, 2);
    const double s_zx = skew_omega(2, 0);
    const double s_zy = skew_omega(2, 1);
    return 0.5 * Vector3d(s_zy - s_yz, s_xz - s_zx, s_yx - s_xy);
  }

  drake::multibody::SpatialVelocity<double> CalcDifferential(const RigidTransformd& P1,
                                                             const RigidTransformd& P2) const {
    return drake::multibody::SpatialVelocity<double>(CalcDifferential(P1.rotation(), P2.rotation()),
                                                     P2.translation() - P1.translation());
  }

  drake::multibody::SpatialVelocity<double> CalcFrameVelocityViaJacobian(const VectorXd& q, const VectorXd& v,
                                                                         const Vector3d& p_FG, const Framed& F) const {
    plant().SetPositions(context_.get(), q);

    const Framed& world_frame = plant().world_frame();
    MatrixXd J(6, plant().num_velocities());
    plant().CalcJacobianSpatialVelocity(*context_, drake::multibody::JacobianWrtVariable::kV, F, p_FG, world_frame,
                                        world_frame, &J);
    return drake::multibody::SpatialVelocity<double>(J * v);
  }

 private:
  void SetUp() override {
    // Construct the plant.
    plant_ = std::make_unique<drake::multibody::MultibodyPlant<double>>();

    // Add robots to plant.
    std::tie(left_instance_, right_instance_) = AddChopsticksToMBP(plant_.get());
    // Finalize the plant.
    plant_->Finalize();
    context_ = plant_->CreateDefaultContext();

    // Since random behavior is present, initialize the random seed
    // deterministically so that our test is no longer random.
    unsigned seed = 0;

    // Construct the kinematics system.
    kinematics_ = std::make_unique<ChopstickKinematics<double>>(plant_.get(), seed);
  }

  drake::multibody::ModelInstanceIndex left_instance_, right_instance_;
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  std::unique_ptr<ChopstickKinematics<double>> kinematics_;
};  // namespace

// Tests the velocity kinematics.
TEST_P(ChopstickTest, Velocity) {
  // Get which chopstick this is.
  ChopstickId chopstick = GetParam();

  // Set the chopstick to an arbitrary configuration.
  drake::multibody::ModelInstanceIndex chopstick_model_instance = model_instance(chopstick);
  VectorXd q_model(plant().num_positions(chopstick_model_instance));
  for (int i = 0; i < q_model.size(); ++i) q_model[i] = static_cast<double>(i + 1) * 0.1;

  VectorXd q = VectorXd::Zero(plant().num_positions());
  plant().SetPositionsInArray(chopstick_model_instance, q_model, &q);
  plant().SetPositions(&context(), q);

  // Set v arbitrarily.
  const VectorXd v = q * 10;

  // Get the frame.
  const drake::multibody::Body<double>& chopstick_body =
      plant().GetBodyByName("end_effector", chopstick_model_instance);
  const auto& F = dynamic_cast<const drake::multibody::Frame<double>&>(chopstick_body.body_frame());

  // Use the expected offset between F and G.
  const Vector3d p_FG(1.0, 0, 0);

  // Evaluate the pose of the chopstick at q.
  const RigidTransformd X_WG = kinematics().CalcForwardKinematics(q, p_FG, F);

  // Evaluate the pose of the chopstick at q + dt * v.
  const double dt = 1e-6;
  const RigidTransformd X_WG_prime = kinematics().CalcForwardKinematics(q + dt * v, p_FG, F);

  // Get the differential between the two.
  const drake::multibody::SpatialVelocity<double> V_WG_approx = CalcDifferential(X_WG, X_WG_prime) * (1.0 / dt);

  // Compare against the velocity.
  const drake::multibody::SpatialVelocity<double> V_WG = kinematics().CalcFrameVelocity(q, v, p_FG, F);

  // Double-check against the velocity computed via a Jacobian.
  const drake::multibody::SpatialVelocity<double> V_WG_prime = CalcFrameVelocityViaJacobian(q, v, p_FG, F);
  EXPECT_LT((V_WG.translational() - V_WG_prime.translational()).norm(), 10 * std::numeric_limits<double>::epsilon());
  EXPECT_LT((V_WG.rotational() - V_WG_prime.rotational()).norm(), 10 * std::numeric_limits<double>::epsilon());

  EXPECT_LT((V_WG_approx.translational() - V_WG.translational()).norm(), dt * 20);
  EXPECT_LT((V_WG_approx.rotational() - V_WG.rotational()).norm(), dt * 20);
}

// Tests the ability of the inverse kinematics solver to find a solution.
TEST_P(ChopstickTest, InverseKinematics) {
  // Get which chopstick this is.
  ChopstickId chopstick = GetParam();

  // Set the chopstick to an arbitrary configuration.
  drake::multibody::ModelInstanceIndex chopstick_model_instance = model_instance(chopstick);
  VectorXd q_model(plant().num_positions(chopstick_model_instance));
  for (int i = 0; i < q_model.size(); ++i) q_model[i] = static_cast<double>(i + 1) * 0.1;

  VectorXd q = VectorXd::Zero(plant().num_positions());
  plant().SetPositionsInArray(chopstick_model_instance, q_model, &q);
  plant().SetPositions(&context(), q);

  // Get the frame.
  const drake::multibody::Body<double>& chopstick_body =
      plant().GetBodyByName("end_effector", chopstick_model_instance);
  const auto& F = dynamic_cast<const drake::multibody::Frame<double>&>(chopstick_body.body_frame());

  // Use the expected offset between F and G.
  const Vector3d p_FG(1.0, 0, 0);

  // Record the target pose.
  const RigidTransformd X_WF = F.CalcPoseInWorld(context());
  const RigidTransformd X_WG(X_WF.rotation(), X_WF * p_FG);

  // This tolerance seems to be sufficient.
  const double zero_tol = 1e-14;

  // Solve using analytical IK (closed form).
  EXPECT_FALSE(kinematics().use_numerical_ik());
  const VectorXd qstar_cf = kinematics().SolveInverseKinematics(X_WG, p_FG, F);

  // Check the IK solution in joint-space. We do this to ensure that there are
  // not extraneous M_PI's added in the closed form IK solution.
  EXPECT_LT((qstar_cf - q_model).norm(), zero_tol);

  // Check the difference between the poses.
  plant().SetPositionsInArray(chopstick_model_instance, qstar_cf, &q);
  const RigidTransformd X_WG_star_cf = kinematics().CalcForwardKinematics(q, p_FG, F);
  const VectorXd xdiff_cf = DifferentialInverseKinematics<double>::CalcOperationalSpaceDifferential(X_WG, X_WG_star_cf);
  EXPECT_LT(xdiff_cf.norm(), zero_tol);

  // Attempt to solve again, now using RMRC with a bunch of seeds and solving for position and orientation.
  kinematics().set_num_seeds(1e3);
  kinematics().set_use_numerical_ik(true);
  kinematics().set_ik_type(ChopstickKinematics<double>::InverseKinematicsType::kPositionAndOrientation);
  EXPECT_TRUE(kinematics().use_numerical_ik());
  const VectorXd qstar_rmrc = kinematics().SolveInverseKinematics(X_WG, p_FG, F);

  // Check the difference between the poses. Note that the IK solver will claim success finding a solution to a tighter
  // tolerance than the operational space differential indicates because the differential used in the former removes
  // the roll degree of freedom from the differential. So we back off the tolerance by a factor of 10 (hence the 0.1
  // below).
  plant().SetPositionsInArray(chopstick_model_instance, qstar_rmrc, &q);
  const RigidTransformd X_WG_star_rmrc = kinematics().CalcForwardKinematics(q, p_FG, F);
  const VectorXd xdiff_rmrc =
      DifferentialInverseKinematics<double>::CalcOperationalSpaceDifferential(X_WG, X_WG_star_rmrc);
  EXPECT_LT(0.1 * xdiff_rmrc.norm(), kinematics().ik_tolerance());

  // Attempt to solve again, now for just the translation. We should be able to get essentially perfect accuracy
  // since there are no orientation components.
  kinematics().set_ik_type(ChopstickKinematics<double>::InverseKinematicsType::kPositionOnly);
  const VectorXd qstar_rmrc_translation = kinematics().SolveInverseKinematics(X_WG, p_FG, F);
  plant().SetPositionsInArray(chopstick_model_instance, qstar_rmrc, &q);
  const RigidTransformd X_WG_star_rmrc_translation = kinematics().CalcForwardKinematics(q, p_FG, F);
  const VectorXd xdiff_rmrc_translation = DifferentialInverseKinematics<double>::CalcPositionDifferential(
      X_WG.translation(), X_WG_star_rmrc_translation.translation());
  EXPECT_LT(xdiff_rmrc_translation.norm(), kinematics().ik_tolerance());

  // Attempt to solve one last time, now for just the orientation.
  kinematics().set_ik_type(ChopstickKinematics<double>::InverseKinematicsType::kOrientationOnly);
  const VectorXd qstar_rmrc_orientation = kinematics().SolveInverseKinematics(X_WG, p_FG, F);
  plant().SetPositionsInArray(chopstick_model_instance, qstar_rmrc_orientation, &q);
  const RigidTransformd X_WG_star_rmrc_orientation = kinematics().CalcForwardKinematics(q, p_FG, F);
  const VectorXd xdiff_rmrc_orientation = DifferentialInverseKinematics<double>::CalcOrientationDifferential(
      X_WG.rotation(), X_WG_star_rmrc_orientation.rotation());
  EXPECT_LT(xdiff_rmrc_orientation.norm(), kinematics().ik_tolerance());
}

INSTANTIATE_TEST_CASE_P(InverseKinematicsTest, ChopstickTest, ::testing::Values(kLeft, kRight));

}  // namespace
}  // namespace DR
