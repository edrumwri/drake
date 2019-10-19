#include <DR/simulation/container_unloading/loading_dock_generator.h>

#include <DR/common/environment.h>
#include <DR/simulation/config.h>

#include <gtest/gtest.h>

#include <drake/common/random.h>

#include "../../test/body_simulation_driver.h"

namespace DR {
const double kInitialGroundClearance = 1.7;
const double kMaxSimulationTime = 0.1;

/*
 The LoadingDockGeneratorTest creates a stack of boxes then uses the SimulationFacade to simulate them.  This test
 focuses on testing whether the loading dock can be built in simulation.
*/
template <typename T>
class LoadingDockGeneratorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::string absolute_model_path = GetModelDirectoryFromEnvironment();
    loading_dock_generator_ = std::make_unique<LoadingDockGenerator>(absolute_model_path);

    // Set distribution parameters.
    const double kMaxConveyorExtension = 5.0;
    const double kMaxTranslationError = 0.1;
    const double kDefaultConveyorHeight = 1.0;
    const double kDefaultRobotConveyorOverlap = 1.0;
    const double kDefaultRobotConveyorZOffset = -kDefaultConveyorHeight + kMaxTranslationError * 2.0;

    loading_dock_generator_->set_attribute_range(
        LoadingDockGenerator::AttributeType::kTrailerPoseTranslation,
        drake::Vector3<double>{0.0, -kMaxTranslationError, -kMaxTranslationError},
        drake::Vector3<double>{kMaxTranslationError, kMaxTranslationError, kMaxTranslationError});
    loading_dock_generator_->set_attribute_range(
        LoadingDockGenerator::AttributeType::kConveyorPoseTranslation,
        drake::Vector3<double>{0.0, -kMaxTranslationError, kDefaultConveyorHeight - kMaxTranslationError},
        drake::Vector3<double>{kMaxTranslationError, kMaxTranslationError,
                               kDefaultConveyorHeight + kMaxTranslationError});
    loading_dock_generator_->set_attribute_range(
        LoadingDockGenerator::AttributeType::kRobotMountPoseTranslation,
        drake::Vector3<double>{-kDefaultRobotConveyorOverlap - kMaxTranslationError, -kMaxTranslationError,
                               kDefaultRobotConveyorZOffset - kMaxTranslationError},
        drake::Vector3<double>{-kDefaultRobotConveyorOverlap + kMaxTranslationError, kMaxTranslationError,
                               kDefaultRobotConveyorZOffset + kMaxTranslationError});

    // Trailer bed size 52' L x 100" W (15.8496 m x 2.54 m).
    // Trailer height 111" H (2.8194 m).
    loading_dock_generator_->set_attribute_range(LoadingDockGenerator::AttributeType::kTrailerSize,
                                                 drake::Vector3<double>{kMaxConveyorExtension + 2.0, 2.0, 2.0},
                                                 drake::Vector3<double>{15.8496, 2.54, 2.8194});

    loading_dock_generator_->set_attribute_range(LoadingDockGenerator::AttributeType::kTrailerCoulombFriction,
                                                 drake::Vector2<double>{0.5, 0.25}, drake::Vector2<double>{1.0, 0.5});

    loading_dock_generator_->set_attribute_range(LoadingDockGenerator::AttributeType::kConveyorCoulombFriction,
                                                 drake::Vector2<double>{0.5, 0.25}, drake::Vector2<double>{1.0, 0.5});

    loading_dock_generator_->set_attribute_range(LoadingDockGenerator::AttributeType::kConveyorExtension,
                                                 drake::Vector1<double>{0.0},
                                                 drake::Vector1<double>{kMaxConveyorExtension});
    loading_dock_generator_->set_attribute_range(LoadingDockGenerator::AttributeType::kConveyorBeltWidth,
                                                 drake::Vector1<double>{0.5}, drake::Vector1<double>{1.0});
  }

  void CheckIsSimulable() {
    // Get simulator instance and advance to the target time.
    ASSERT_NO_THROW(driver().simulator(simulator_instance_index_).AdvanceTo(kMaxSimulationTime));
  }

  void Build(std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>>&& bodies) {
    // Test SimulatorFacade derived class.
    ASSERT_NO_THROW(driver_ = std::make_unique<BodySimulationDriver<T>>(std::move(bodies)));

    // Check build.
    ASSERT_NO_THROW(simulator_instance_index_ = driver().CreateSimulatedSystem());

    // Get simulator instance and advance to the target time.
    ASSERT_NO_THROW(driver().simulator(simulator_instance_index_).Initialize());
  }

  LoadingDockGenerator& loading_dock_generator() { return *loading_dock_generator_.get(); }
  BodySimulationDriver<T>& driver() { return *driver_.get(); }

 private:
  SimulatorInstanceIndex simulator_instance_index_{};

  std::unique_ptr<BodySimulationDriver<T>> driver_;
  std::unique_ptr<LoadingDockGenerator> loading_dock_generator_;
};

#if GTEST_HAS_TYPED_TEST

using testing::Types;

typedef Types<double> Implementations;

TYPED_TEST_SUITE(LoadingDockGeneratorTest, Implementations);

/*
 Check whether a few dynamic bodies placed in the trailer simulation can be simulated for any interval of time.
 */
TYPED_TEST(LoadingDockGeneratorTest, SimulatedLoadingDock) {
  drake::RandomGenerator random_generator;
  std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> bodies;

  // Dynamic body on trailer floor.
  const double kCubeSize = 0.5;
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("dynamic_cube1");
    body->set_mass_and_possibly_make_static(10.0);
    body->SetCoulombFriction(0.8, 0.6);
    body->SetBoxGeometry(kCubeSize, kCubeSize, kCubeSize);
    body->set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
        drake::Vector3<double>{1.0, 0.0, kCubeSize * 0.5 + kInitialGroundClearance}));
    bodies.insert(std::move(body));
  }

  // Dynamic body between trailer and loading dock.
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("dynamic_cube2");
    body->set_mass_and_possibly_make_static(10.0);
    body->SetCoulombFriction(0.8, 0.6);
    body->SetBoxGeometry(kCubeSize, kCubeSize, kCubeSize);
    body->set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
        drake::Vector3<double>{-0.4 * kCubeSize, 0.0, kCubeSize * 0.5 + kInitialGroundClearance}));
    bodies.insert(std::move(body));
  }

  // Dynamic body on conveyor.
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("dynamic_cube3");
    body->set_mass_and_possibly_make_static(10.0);
    body->SetCoulombFriction(0.8, 0.6);
    body->SetBoxGeometry(kCubeSize, kCubeSize, kCubeSize);
    body->set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
        drake::Vector3<double>{-1.0, 0.0, kCubeSize * 0.5 + kInitialGroundClearance}));
    bodies.insert(std::move(body));
  }

  // Generate loading dock bodies.
  bodies.merge(this->loading_dock_generator().CreateLoadingDockConfigs(&random_generator));

  // Build model, diagram & simulation.
  ASSERT_NO_FATAL_FAILURE(this->Build(std::move(bodies)));
  // Test simulation works.
  ASSERT_NO_FATAL_FAILURE(this->CheckIsSimulable());
}

#endif  // GTEST_HAS_TYPED_TEST

}  // namespace DR
