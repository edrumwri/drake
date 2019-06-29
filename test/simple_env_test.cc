/**
 * Simple Environment Building Test
 * Uses the ModelGenerator Class to create a MultibodyPlant (world model) for
 * each permutation of the UnloadingTaskConfig, this includes: An instance of
 * BodyInstanceConfig for all supported derived classes of drake:geometry:Shape,
 * currently:
 *    -- Box {floating (manipuland body), fixed (environment body)}
 *    -- Sphere {floating, fixed}
 *    -- Cylinder {floating, fixed}
 * An instance of EnvironmentInstanceConfig for all supported EnvironmentType,
 * currently:
 *    -- Floor
 *    -- Trailer
 * see include/PCESystems/config.h for documentation of UnloadingTaskConfig and
 * sub-classes" BodyInstanceConfig, EnvironmentInstanceConfig
 */

#include <gflags/gflags.h>

#include <drake/common/text_logging_gflags.h>

#include <PCESystems/model_generator.h>

namespace DR {
namespace {

int do_main() {
  UnloadingTaskConfig config;

  SimulatorInstanceConfig test_sims;
  test_sims.set_integration_scheme(kImplicitEulerIntegrationScheme);
  config.set_simulator_instance_config(test_sims);

  std::vector<BodyInstanceConfig> test_objs;
  // Test floating manipuland Bodies.
  {
    BodyInstanceConfig test_obj;
    // Setup config shared by both environments.
    test_obj.set_mass(10.0);
    // Set location of model in world.
    test_obj.set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
        drake::Vector3<double>(1.0, 1.0, 1.0)));
    test_obj.SetCoulombFriction(0.8, 0.6);

    // Setup Sphere manipuland attributes.
    test_obj.set_name(std::move("sphere_1"));
    test_obj.SetSphereGeometry(0.1);
    test_objs.push_back(test_obj);
    // Setup Box manipuland attributes.
    test_obj.set_name(std::move("box_1"));
    test_obj.SetBoxGeometry(0.2, 0.3, 0.1);
    test_objs.push_back(test_obj);
    // Setup Cylinder manipuland attributes.
    test_obj.set_name(std::move("cylinder_1"));
    test_obj.SetCylinderGeometry(0.05, 0.3);
    test_objs.push_back(test_obj);
  }

  // Test static Environment Bodies.
  std::vector<BodyInstanceConfig> test_env_objs;
  {
    BodyInstanceConfig test_obj;
    // Setup config shared by both environments.
    // Set location of model in world.
    test_obj.set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
        drake::Vector3<double>(1.0, 1.0, 1.0)));
    test_obj.SetCoulombFriction(0.8, 0.6);

    // Setup Sphere manipuland attributes.
    test_obj.set_name(std::move("sphere_1"));
    test_obj.SetSphereGeometry(0.1);
    test_env_objs.push_back(test_obj);
    // Setup Box manipuland attributes.
    test_obj.set_name(std::move("box_1"));
    test_obj.SetBoxGeometry(0.2, 0.3, 0.1);
    test_env_objs.push_back(test_obj);
    // Setup Cylinder manipuland attributes.
    test_obj.set_name(std::move("cylinder_1"));
    test_obj.SetCylinderGeometry(0.05, 0.3);
    test_env_objs.push_back(test_obj);
  }

  // Test environment type options.
  std::vector<EnvironmentInstanceConfig> test_envs;
  {
    EnvironmentInstanceConfig test_env;
    // Setup config shared by both environments.
    test_env.SetCoulombFriction(0.8, 0.6);
    // Setup floor plane environment.
    test_env.SetFloorEnvironment();
    test_envs.push_back(test_env);
    // Setup trailer environment.
    test_env.SetTrailerEnvironment();
    test_envs.push_back(test_env);
  }

  // Test floating Manipuland bodies in each environment.
  for (int i = 0; i < test_envs.size(); i++) {
    config.set_environment_instance_config(test_envs[i]);
    std::cout << "  Test Environment #" << i << " of " << test_envs.size()
              << std::endl;
    for (int j = 0; j < test_objs.size(); j++) {
      std::cout << "    Test Manipuland #" << j << " of " << test_objs.size()
                << std::endl;
      config.set_manipuland_instance_config(
          std::vector<BodyInstanceConfig>{test_objs[j]});

      std::cout << " ======= Create World Model ======= " << std::endl;

      drake::systems::DiagramBuilder<double> builder;

      auto* mbp = ModelGenerator::CreateSceneAndRobot(config, &builder);
    }
  }

  // Test static Environment bodies in each environment.
  for (int i = 0; i < test_envs.size(); i++) {
    std::cout << "  Test Environment #" << i << " of " << test_envs.size()
              << std::endl;
    for (int j = 0; j < test_env_objs.size(); j++) {
      std::cout << "    Test Environment Object #" << j << " of "
                << test_env_objs.size() << std::endl;

      test_envs[i].SetBodyInstanceConfigs(
          std::vector<BodyInstanceConfig>{test_env_objs[j]});
      config.set_environment_instance_config(test_envs[i]);

      config.set_manipuland_instance_config(std::vector<BodyInstanceConfig>{});

      std::cout << " ======= Create World Model ======= " << std::endl;

      drake::systems::DiagramBuilder<double> builder;

      auto* mbp = ModelGenerator::CreateSceneAndRobot(config, &builder);
    }
  }

  std::cout << " **** Completed successfully!* *** " << std::endl;
  return 0;
}

}  // namespace
}  // namespace DR

int main(int argc, char* argv[]) {
  std::cout << "Running Program: " << std::endl;
  std::copy(argv, argv + argc,
            std::ostream_iterator<const char*>(std::cout, " "));
  std::cout << std::endl;

  gflags::SetUsageMessage(
      "Main function driving the 'Chopstick' robot in simulation "
      "using Drake's drake::multibody::MultibodyPlant, with "
      "geometry::SceneGraph visualization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return DR::do_main();
}
