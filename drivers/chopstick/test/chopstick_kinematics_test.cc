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

// Tests the ability of the inverse kinematics solver to find a solution.
TEST_P(ChopstickTest, InverseKinematics) {
  // Get which chopstick this is.
  ChopstickId chopstick = GetParam();

  // Set the chopstick to an arbitrary configuration.
  drake::multibody::ModelInstanceIndex chopstick_model_instance = model_instance(chopstick);
  VectorX<double> q_model(plant().num_positions(chopstick_model_instance));
  for (int i = 0; i < q_model.size(); ++i) q_model[i] = static_cast<double>(i + 1) * 0.1;

  VectorX<double> q = VectorX<double>::Zero(plant().num_positions());
  plant().SetPositionsInArray(chopstick_model_instance, q_model, &q);
  plant().SetPositions(&context(), q);

  // Get the frame.
  const drake::multibody::Body<double>& chopstick_body =
      plant().GetBodyByName("end_effector", chopstick_model_instance);
  const auto& F = dynamic_cast<const drake::multibody::Frame<double>&>(chopstick_body.body_frame());

  // Use the expected offset between F and G.
  const Vector3<double> p_FG(1.0, 0, 0);

  // Record the target pose.
  const RigidTransform<double> X_WF = F.CalcPoseInWorld(context());
  const RigidTransform<double> X_WG(X_WF.rotation(), X_WF * p_FG);

  // This tolerance seems to be sufficient.
  const double zero_tol = 1e-14;

  // Solve using analytical IK (closed form).
  EXPECT_FALSE(kinematics().use_numerical_ik());
  const VectorX<double> qstar_cf = kinematics().SolveInverseKinematics(X_WG, p_FG, F);

  // Check the IK solution in joint-space. We do this to ensure that there are
  // not extraneous M_PI's added in the closed form IK solution.
  EXPECT_LT((qstar_cf - q_model).norm(), zero_tol);

  // Check the difference between the poses.
  plant().SetPositionsInArray(chopstick_model_instance, qstar_cf, &q);
  const RigidTransform<double> X_WG_star_cf = kinematics().CalcForwardKinematics(q, p_FG, F);
  const VectorX<double> xdiff_cf =
      DifferentialInverseKinematics<double>::CalcOperationalSpaceDifferential(X_WG, X_WG_star_cf);
  EXPECT_LT(xdiff_cf.norm(), zero_tol);

  // Attempt to solve again, now using RMRC with a bunch of seeds and solving for position and orientation.
  kinematics().set_num_seeds(1e3);
  kinematics().set_use_numerical_ik(true);
  kinematics().set_ik_type(ChopstickKinematics<double>::InverseKinematicsType::kPositionAndOrientation);
  EXPECT_TRUE(kinematics().use_numerical_ik());
  const VectorX<double> qstar_rmrc = kinematics().SolveInverseKinematics(X_WG, p_FG, F);

  // Check the difference between the poses. Note that the IK solver will claim success finding a solution to a tighter
  // tolerance than the operational space differential indicates because the differential used in the former removes
  // the roll degree of freedom from the differential. So we back off the tolerance by a factor of 10 (hence the 0.1
  // below).
  plant().SetPositionsInArray(chopstick_model_instance, qstar_rmrc, &q);
  const RigidTransform<double> X_WG_star_rmrc = kinematics().CalcForwardKinematics(q, p_FG, F);
  const VectorX<double> xdiff_rmrc =
      DifferentialInverseKinematics<double>::CalcOperationalSpaceDifferential(X_WG, X_WG_star_rmrc);
  EXPECT_LT(0.1 * xdiff_rmrc.norm(), kinematics().ik_tolerance());

  // Attempt to solve again, now for just the translation. We should be able to get essentially perfect accuracy
  // since there are no orientation components.
  kinematics().set_ik_type(ChopstickKinematics<double>::InverseKinematicsType::kPositionOnly);
  const VectorX<double> qstar_rmrc_translation = kinematics().SolveInverseKinematics(X_WG, p_FG, F);
  plant().SetPositionsInArray(chopstick_model_instance, qstar_rmrc, &q);
  const RigidTransform<double> X_WG_star_rmrc_translation = kinematics().CalcForwardKinematics(q, p_FG, F);
  const VectorX<double> xdiff_rmrc_translation = DifferentialInverseKinematics<double>::CalcPositionDifferential(
      X_WG.translation(), X_WG_star_rmrc_translation.translation());
  EXPECT_LT(xdiff_rmrc_translation.norm(), kinematics().ik_tolerance());

  // Attempt to solve one last time, now for just the orientation.
  kinematics().set_ik_type(ChopstickKinematics<double>::InverseKinematicsType::kOrientationOnly);
  const VectorX<double> qstar_rmrc_orientation = kinematics().SolveInverseKinematics(X_WG, p_FG, F);
  plant().SetPositionsInArray(chopstick_model_instance, qstar_rmrc_orientation, &q);
  const RigidTransform<double> X_WG_star_rmrc_orientation = kinematics().CalcForwardKinematics(q, p_FG, F);
  const VectorX<double> xdiff_rmrc_orientation = DifferentialInverseKinematics<double>::CalcOrientationDifferential(
      X_WG.rotation(), X_WG_star_rmrc_orientation.rotation());
  EXPECT_LT(xdiff_rmrc_orientation.norm(), kinematics().ik_tolerance());
}

INSTANTIATE_TEST_CASE_P(InverseKinematicsTest, ChopstickTest, ::testing::Values(kLeft, kRight));

}  // namespace
}  // namespace DR
