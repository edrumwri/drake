#include <DR/model_generator.h>

#include <gtest/gtest.h>

#include <drake/common/autodiff.h>

#include <DR/config.h>

namespace DR {
namespace {

/**
 * Model Generator Test
 * Generates permutations of UnloadingTaskConfig (excluding the robot model) for
 * testing. This includes:
 *
 * An instance of BodyInstanceConfig for all supported
 * derived classes of drake:geometry:Shape, currently:
 *    -- Box {floating (manipuland body), fixed (environment body)}
 *    -- Sphere {floating, fixed}
 *    -- Cylinder {floating, fixed}
 *
 * An instance of EnvironmentInstanceConfig for all supported EnvironmentType,
 * currently:
 *    -- Floor
 *    -- Trailer
 *
 * The permutations of UnloadingTaskConfig are tested against ModelGenerator<T>
 * for supported drake default scalar types:
 *    -- T = double
 *    -- T = drake::AutoDiffXd
 *
 * see include/DR/config.h for documentation on UnloadingTaskConfig
 */

template <typename T>
class ModelGeneratorTest : public ::testing::Test {
 public:
  ModelGenerator<T>* model_generator() { return model_generator_.get(); }
  drake::systems::DiagramBuilder<T>* builder() { return builder_.get(); }

 protected:
  void SetUp() override {
    model_generator_ = std::make_unique<ModelGenerator<T>>();
    builder_ = std::make_unique<drake::systems::DiagramBuilder<T>>();
  }

 private:
  std::unique_ptr<ModelGenerator<T>> model_generator_;
  std::unique_ptr<drake::systems::DiagramBuilder<T>> builder_;
};

#if GTEST_HAS_TYPED_TEST

using testing::Types;

typedef Types<double, drake::AutoDiffXd> Implementations;

TYPED_TEST_SUITE(ModelGeneratorTest, Implementations);

// Verifies that model generator works for important permutations of
// UnloadingTaskConfig settings.
TYPED_TEST(ModelGeneratorTest, UnloadingTaskConfig) {
  // Test simulation options (Integration Scheme).
  std::vector<SimulatorInstanceConfig> test_simulators;
  {
    SimulatorInstanceConfig test_sim;
    // Setup config shared by all sims.
    test_sim.set_simulation_time(0.1);

    test_sim.set_integration_scheme(
        SimulatorInstanceConfig::kImplicitEulerIntegrationScheme);
    test_simulators.push_back(test_sim);

    test_sim.set_integration_scheme(
        SimulatorInstanceConfig::kRK2IntegrationScheme);
    test_simulators.push_back(test_sim);

    test_sim.set_integration_scheme(
        SimulatorInstanceConfig::kRK3IntegrationScheme);
    test_simulators.push_back(test_sim);

    test_sim.set_integration_scheme(
        SimulatorInstanceConfig::kSemiExplicitEulerIntegrationScheme);
    test_simulators.push_back(test_sim);
  }

  // Test floating manipuland bodies of supported types.
  std::vector<std::vector<BodyInstanceConfig>> test_manipulands;
  {
    BodyInstanceConfig test_obj;
    // Setup config shared by all environments.
    test_obj.set_mass(10.0);
    test_obj.set_is_floating(true);
    // Set pose of model in world.
    test_obj.set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
        drake::Vector3<double>(1.0, 0.0, 1.0)));
    test_obj.SetCoulombFriction(0.8, 0.6);

    // Setup Sphere manipuland attributes.
    test_obj.set_name(std::move("manip_sphere_1"));
    test_obj.SetSphereGeometry(0.1);
    test_manipulands.emplace_back();
    test_manipulands.back().push_back(test_obj);
    // Setup Box manipuland attributes.
    test_obj.set_name(std::move("manip_box_1"));
    test_obj.SetBoxGeometry(0.2, 0.3, 0.1);
    test_manipulands.emplace_back();
    test_manipulands.back().push_back(test_obj);
    // Setup Cylinder manipuland attributes.
    test_obj.set_name(std::move("manip_cylinder_1"));
    test_obj.SetCylinderGeometry(0.05, 0.3);
    test_manipulands.emplace_back();
    test_manipulands.back().push_back(test_obj);
  }

  // Test environment options of supported type and environment bodies.
  std::vector<EnvironmentInstanceConfig> test_environments;
  {
    // Test static Environment Bodies.
    std::vector<std::vector<BodyInstanceConfig>> test_env_bodies;
    {
      BodyInstanceConfig test_obj;
      test_obj.set_is_floating(false);
      // Setup config shared by both environments.
      // Set location of model in world.
      test_obj.set_pose(drake::math::RigidTransform<double>(
          drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
          drake::Vector3<double>(1.0, 0.0, 0.5)));
      test_obj.SetCoulombFriction(0.8, 0.6);

      // Setup Sphere manipuland attributes.
      test_obj.set_name(std::move("env_sphere_1"));
      test_obj.SetSphereGeometry(0.1);
      test_env_bodies.emplace_back();
      test_env_bodies.back().push_back(test_obj);

      // Setup Box manipuland attributes.
      test_obj.set_name(std::move("env_box_1"));
      test_obj.SetBoxGeometry(0.2, 0.3, 0.1);
      test_env_bodies.emplace_back();
      test_env_bodies.back().push_back(test_obj);

      // Setup Cylinder manipuland attributes.
      test_obj.set_name(std::move("env_cylinder_1"));
      test_obj.SetCylinderGeometry(0.05, 0.3);
      test_env_bodies.emplace_back();
      test_env_bodies.back().push_back(test_obj);
    }

    EnvironmentInstanceConfig test_env;
    // Setup config shared by both environments.
    test_env.SetCoulombFriction(0.8, 0.6);

    for (const auto& env_bodies : test_env_bodies) {
      test_env.SetBodyInstanceConfigs(env_bodies);
      // Setup floor plane environment.
      test_env.SetFloorEnvironment();
      test_environments.push_back(test_env);
      // Setup trailer environment.
      test_env.SetTrailerEnvironment();
      test_environments.push_back(test_env);
    }
  }

  // Run test for all permutations.
  UnloadingTaskConfig config;
  for (const auto& simulator : test_simulators) {
    config.set_simulator_instance_config(simulator);
    for (const auto& environment : test_environments) {
      config.set_environment_instance_config(environment);
      for (const auto& manipulands : test_manipulands) {
        config.set_manipuland_instance_config(manipulands);

        // Make sure config is good.
        ASSERT_NO_THROW(config.ValidateConfig());

        // Test that we can successfully build the world.
        ASSERT_NO_THROW(this->model_generator()->CreateSceneAndRobot(
            config, this->builder()));
      }
    }
  }
}

#endif  // GTEST_HAS_TYPED_TEST
}  // namespace
}  // namespace DR
