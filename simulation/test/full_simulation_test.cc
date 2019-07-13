#include <gflags/gflags.h>

#include <drake/common/autodiff.h>

#include <DR/simulation/config.h>
#include <DR/simulation/controller_generator.h>
#include <DR/simulation/model_generator.h>
#include <DR/simulation/simulation_generator.h>
#include <DR/simulation/state_setter.h>

#define GTEST_HAS_EXCEPTIONS 1
#include <gtest/gtest.h>

namespace DR {
namespace {

/*
 The FullSimulationTest creates a drake::systems::DiagramBuilder,
 ModelGenerator, ControllerGenerator, StateSetter, and SimulationGenerator tests
 implementing this class then use the stored classes to generate and run a
 simulation according to configuration parameters provided in an
 UnloadingTaskConfig.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
         Options for T are the following: (☑ = supported)
         ☑ double
         ☐ drake::AutoDiffXd
         ☐ drake::symbolic::Expression
 */
template <typename T>
class FullSimulationTest : public ::testing::Test {
 public:
  ModelGenerator<T>* model_generator() { return model_generator_.get(); }
  ControllerGenerator<T>* controller_generator() { return controller_generator_.get(); }
  StateSetter<T>* state_setter() { return state_setter_.get(); }
  SimulationGenerator<T>* simulation_generator() { return simulation_generator_.get(); }

  drake::systems::DiagramBuilder<T>* builder() { return builder_.get(); }

  void ResetPointers() {
    model_generator_ = std::make_unique<ModelGenerator<T>>();
    controller_generator_ = std::make_unique<ControllerGenerator<T>>();
    state_setter_ = std::make_unique<StateSetter<T>>();
    simulation_generator_ = std::make_unique<SimulationGenerator<T>>();

    builder_ = std::make_unique<drake::systems::DiagramBuilder<T>>();
  }

  void BuildAndSim(const UnloadingTaskConfig& config) {
    this->ResetPointers();
    // Make sure config is good.
    ASSERT_NO_THROW(config.ValidateConfig());

    // Test that we can successfully build the world.
    typename ModelGenerator<T>::ModelWithSceneGraph model_scenegraph;
    ASSERT_NO_THROW(model_scenegraph = this->model_generator()->CreateSceneAndRobot(  //
                        config,                                                       //
                        this->builder()));

    ASSERT_NE(std::get<0>(model_scenegraph), nullptr);
    ASSERT_NE(std::get<1>(model_scenegraph), nullptr);

    // Test that we can successfully connect all parts of the control diagram.
    typename ControllerGenerator<T>::DiagramWithContext diagram_context;
    ASSERT_NO_THROW(diagram_context = this->controller_generator()->CreateDiagramWithContext(  //
                        config,                                                                //
                        *std::get<0>(model_scenegraph),                                        //
                        *std::get<1>(model_scenegraph),                                        //
                        this->builder()));

    ASSERT_NE(std::get<0>(diagram_context).get(), nullptr);
    ASSERT_NE(std::get<1>(diagram_context).get(), nullptr);

    // Test that we can successfully set all free floating body poses and
    // robot joint angles.
    ASSERT_NO_THROW(this->state_setter()->SetState(config,                               //
                                                   *std::get<0>(model_scenegraph),       //
                                                   *std::get<0>(diagram_context).get(),  //
                                                   std::get<1>(diagram_context).get()));

    // Test that we can successfully simulate the full test duration.
    std::unique_ptr<drake::systems::Simulator<double>> simulator;
    ASSERT_NO_THROW(simulator = this->simulation_generator()->BuildSimulator(  //
                        config.simulator_instance_config(),                    //
                        *std::get<0>(diagram_context).get(),                   //
                        std::move(std::get<1>(diagram_context)),               //
                        std::get<0>(model_scenegraph)));

    ASSERT_NE(simulator.get(), nullptr);

    ASSERT_NO_THROW(simulator->AdvanceTo(config.simulator_instance_config().simulation_time()));

    ASSERT_NEAR(simulator->get_context().get_time(), config.simulator_instance_config().simulation_time(),
                config.simulator_instance_config().step_size());
  }

 protected:
  void SetUp() override { ResetPointers(); }

 private:
  std::unique_ptr<ModelGenerator<T>> model_generator_;
  std::unique_ptr<ControllerGenerator<T>> controller_generator_;
  std::unique_ptr<StateSetter<T>> state_setter_;
  std::unique_ptr<SimulationGenerator<T>> simulation_generator_;

  std::unique_ptr<drake::systems::DiagramBuilder<T>> builder_;
};

#ifndef GTEST_HAS_TYPED_TEST_P
#error "This test requires GTEST_HAS_TYPED_TEST_P"
#else  // GTEST_HAS_TYPED_TEST_P

TYPED_TEST_SUITE_P(FullSimulationTest);

/*
 Passive Bodies Simulation Test
 Generates a simulation with robot controller, dynamic and static bodies and
 contact.  The robot is meant to hold position over the duration of this test.
 An instance of BodyInstanceConfig for all supported
 derived classes of drake:geometry:Shape, currently:
    -- Box {floating (manipuland body), fixed (environment body)}
    -- Sphere {floating, fixed}
    -- Cylinder {floating, fixed}
 An instance of EnvironmentInstanceConfig for all supported EnvironmentType,
 currently:
    -- Floor
    -- Trailer
 see include/DR/simulation/config.h for documentation on UnloadingTaskConfig
 */
TYPED_TEST_P(FullSimulationTest, PassiveBodies) {
  // Test simulation options (Integration Scheme).
  std::vector<SimulatorInstanceConfig> test_simulators;
  {
    SimulatorInstanceConfig test_sim;
    // Setup config shared by all sims.
    test_sim.set_simulation_time(1.0);
    test_sim.set_target_realtime_rate(100.0);

    test_sim.set_integration_scheme(SimulatorInstanceConfig::kImplicitEulerIntegrationScheme);
    test_simulators.push_back(test_sim);

    test_sim.set_integration_scheme(SimulatorInstanceConfig::kRK2IntegrationScheme);
    test_simulators.push_back(test_sim);

    test_sim.set_integration_scheme(SimulatorInstanceConfig::kRK3IntegrationScheme);
    test_simulators.push_back(test_sim);

    test_sim.set_integration_scheme(SimulatorInstanceConfig::kSemiExplicitEulerIntegrationScheme);
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
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(), drake::Vector3<double>(1.0, 0.0, 1.0)));
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
      // Set pose of model in world.
      test_obj.set_pose(drake::math::RigidTransform<double>(
          drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(), drake::Vector3<double>(1.0, 0.0, 0.5)));
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
      test_env.set_body_instance_configs(env_bodies);
      // Setup floor plane environment.
      test_env.set_floor_environment();
      test_environments.push_back(test_env);
      // Setup trailer environment.
      test_env.set_trailer_environment();
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
        config.set_manipuland_instance_configs(manipulands);

        ASSERT_NO_FATAL_FAILURE(this->BuildAndSim(config));
      }
    }
  }
}

REGISTER_TYPED_TEST_SUITE_P(FullSimulationTest, PassiveBodies);

typedef ::testing::Types<double> DrakeScalarTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(SimulationSupportedScalars, FullSimulationTest, DrakeScalarTypes);

#endif  // GTEST_HAS_TYPED_TEST_P
}  // namespace
}  // namespace DR
