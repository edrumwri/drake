#include <cstdlib>

#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>

#include <DR/drivers/chopstick_config.h>
#include <DR/drivers/chopstick_driver.h>
#include <DR/simulation/model_generator.h>

#include <gtest/gtest.h>

using drake::math::RigidTransform;
using drake::math::RotationMatrix;
using drake::VectorX;
using drake::Vector3;

namespace DR {
namespace {

enum ChopstickId { kLeft = 0, kRight = 1 };

class ChopstickTest : public ::testing::TestWithParam<ChopstickId> {
 public:
  // Gets the model instance corresponding to a particular chopstick.
  drake::multibody::ModelInstanceIndex model_instance(ChopstickId id) {
    switch (id) {
      case kLeft: return left_instance_;
      case kRight: return right_instance_;
    }

    DRAKE_UNREACHABLE();
  }

  const drake::multibody::MultibodyPlant<double>& plant() const { return *plant_; }
  drake::systems::Context<double>& context() const { return *context_; }
  const ChopstickDriver<double>& driver() const { return *driver_; }
  ChopstickDriver<double>& driver() { return *driver_; }

 private:
  void SetUp() override {
    // Get the absolute model path from an environment variable.
    const char* absolute_model_path_env_var = std::getenv("DR_ABSOLUTE_MODEL_PATH");
    ASSERT_NE(absolute_model_path_env_var, nullptr);
    std::string absolute_model_path = std::string(absolute_model_path_env_var);

    // Add a trailing slash if necessary.
    if (absolute_model_path.back() != '/')
      absolute_model_path += '/';

    // Construct the plant.
    plant_ = std::make_unique<drake::multibody::MultibodyPlant<double>>();

    // Add robots to plant.
    ModelGenerator<double> model_generator;
    std::vector<RobotInstanceConfig> robots = CreateChopstickRobotsConfig();
    ASSERT_EQ(robots.front().name(), "chopstick_left");
    ASSERT_EQ(robots.back().name(), "chopstick_right");
    left_instance_ = model_generator.AddRobotToMBP(robots.front(), plant_.get());
    right_instance_ = model_generator.AddRobotToMBP(robots.back(), plant_.get());

    // Finalize the plant.
    plant_->Finalize();
    context_ = plant_->CreateDefaultContext();

    // Since random behavior is present, initialize the random seed
    // deterministically so that our test is no longer random.
    unsigned seed = 0;

    // Construct the driver.
    driver_ = std::make_unique<ChopstickDriver<double>>(plant_.get(), seed);
  }

  drake::multibody::ModelInstanceIndex left_instance_, right_instance_;
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  std::unique_ptr<ChopstickDriver<double>> driver_;
};

// Tests the ability of the inverse kinematics solver to find a solution.
TEST_P(ChopstickTest, InverseKinematics) {
  // Get which chopstick this is.
  ChopstickId chopstick = GetParam();

  // Set the chopstick to an arbitrary configuration.
  drake::multibody::ModelInstanceIndex chopstick_model_instance = model_instance(chopstick);
  VectorX<double> q_model(plant().num_positions(chopstick_model_instance));
  for (int i = 0; i < q_model.size(); ++i)
    q_model[i] = static_cast<double>(i + 1) * 0.1;

  VectorX<double> q = VectorX<double>::Zero(plant().num_positions());
  plant().SetPositionsInArray(chopstick_model_instance, q_model, &q);
  plant().SetPositions(&context(), q);

  // Get the frame.
  const drake::multibody::Body<double>&
      chopstick_body = plant().GetBodyByName("end_effector", chopstick_model_instance);
  const auto& F = dynamic_cast<const drake::multibody::Frame<double>&>(chopstick_body.body_frame());

  // Use the expected offset between F and G.
  const Vector3<double> p_FG(1.0, 0, 0);

  // Record the target pose.
  const RigidTransform<double> X_WF = F.CalcPoseInWorld(context());
  const RigidTransform<double> X_WG(X_WF.rotation(), X_WF * p_FG);

  // This tolerance seems to be sufficient.
  const double zero_tol = 1e-14;

  // Solve using analytical IK (closed form).
  EXPECT_FALSE(driver().use_numerical_ik());
  const VectorX<double> qstar_cf = driver().SolveInverseKinematics(X_WG, p_FG, F);

  // Check the IK solution in joint-space. We do this to ensure that there are
  // not extraneous M_PI's added in the closed form IK solution.
  EXPECT_LT((qstar_cf - q_model).norm(), zero_tol);

  // Check the difference between the poses.
  plant().SetPositionsInArray(chopstick_model_instance, qstar_cf, &q);
  const RigidTransform<double> X_WG_star_cf = driver().CalcForwardKinematics(q, p_FG, F);
  const VectorX<double> xdiff_cf =
      DifferentialInverseKinematics<double>::CalcOperationalSpaceDifferential(X_WG, X_WG_star_cf);
  EXPECT_LT(xdiff_cf.norm(), zero_tol);

  // Attempt to solve again, now using RMRC with a bunch of seeds.
  driver().set_num_seeds(1e3);
  driver().set_use_numerical_ik(true);
  EXPECT_TRUE(driver().use_numerical_ik());
  const VectorX<double> qstar_rmrc = driver().SolveInverseKinematics(X_WG, p_FG, F);

  // Check the difference between the poses. Note that the IK solver will claim success finding a solution to a tighter
  // tolerance than the operational space differential indicates because the differential used in the former removes
  // the roll degree of freedom from the differential. So we back off the tolerance by a factor of 10 (hence the 0.1
  // below).
  plant().SetPositionsInArray(chopstick_model_instance, qstar_rmrc, &q);
  const RigidTransform<double> X_WG_star_rmrc = driver().CalcForwardKinematics(q, p_FG, F);
  const VectorX<double> xdiff_rmrc =
      DifferentialInverseKinematics<double>::CalcOperationalSpaceDifferential(X_WG, X_WG_star_rmrc);
  EXPECT_LT(0.1 * xdiff_rmrc.norm(), driver().ik_tolerance());
}

INSTANTIATE_TEST_CASE_P(InverseKinematicsTest,
                        ChopstickTest,
                        ::testing::Values(kLeft, kRight));


}  // namespace
}  // namespace DR

