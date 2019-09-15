#include <gflags/gflags.h>

#include <drake/common/autodiff.h>

#include <DR/drivers/chopstick/chopstick_config.h>
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
  ModelGenerator<T> model_gen;
  ControllerGenerator<T> controller_gen;
  StateSetter<T> state_gen;
  SimulationGenerator<T> sim_gen;

  drake::systems::DiagramBuilder<T> builder_;

  drake::multibody::MultibodyPlant<T>* model_;
  drake::geometry::SceneGraph<T>* scenegraph_;
  std::unique_ptr<drake::systems::Diagram<T>> diagram_;
  std::unique_ptr<drake::systems::Simulator<double>> simulator_;

  std::unique_ptr<drake::systems::Context<T>> context_;
  drake::optional<drake::systems::Context<T>*> sim_context_ = {};

 public:
  using MyParamType = T;

  const drake::systems::Context<T>& context() {
    if (sim_context_.has_value()) {
      return *sim_context_.value();
    }
    return *context_.get();
  }

  const drake::multibody::MultibodyPlant<T>& model() { return *model_; }

  void BuildSystem(const UnloadingTaskConfig& config) {
    try {
      // Make sure config is good.
      config.ValidateConfig();

      // Test that we can successfully build the world.
      std::tie(model_, scenegraph_) = model_gen.CreateSceneAndRobot(config, &builder_);

      ASSERT_NE(model_, nullptr);
      ASSERT_NE(scenegraph_, nullptr);

      // Test that we can successfully connect all parts of the control diagram.
      std::tie(diagram_, context_) = controller_gen.CreateDiagramWithContext(config, *model_, *scenegraph_, &builder_);

      ASSERT_NE(diagram_.get(), nullptr);
      ASSERT_NE(context_.get(), nullptr);

      // Test that we can successfully set all free floating body poses and
      // robot joint angles.
      state_gen.SetState(config, *model_, *diagram_.get(), context_.get());

      // Create the simulation.
      simulator_ =
          sim_gen.BuildSimulator(config.simulator_instance_config(), *diagram_.get(), std::move(context_), model_);
      ASSERT_NE(simulator_.get(), nullptr);

      sim_context_ = &simulator_->get_mutable_context();
      ASSERT_EQ(sim_context_.has_value(), true);
      ASSERT_NE(sim_context_.value(), nullptr);
    } catch (const std::exception& e) {
      drake::log()->critical("BuildSystem threw an error: {}", e.what());
      throw std::runtime_error(e.what());
    }
  }
  void SimulateSystem(const SimulatorInstanceConfig& config) {
    try {
      // Make sure config is good.
      config.ValidateConfig();

      do {
        // Advance one simulation step.
        simulator_->AdvanceTo(this->context().get_time() + config.step_size());
      } while (this->context().get_time() < config.simulation_time());
    } catch (const std::exception& e) {
      drake::log()->critical("SimulateSystem threw an error: {}", e.what());
      throw std::runtime_error(e.what());
    }
  }
  drake::VectorX<T> GetControlledChopsticksAcceleration() {
    // Pair of chopstick robots.
    std::vector<RobotInstanceConfig> robots = CreateChopstickRobotsConfig();
    // Set floating bases.
    for (auto& robot : robots) {
      robot.set_is_floating(false);
    }

    // Create Robot plant.
    auto mbp = std::make_unique<drake::multibody::MultibodyPlant<double>>();

    // Add robots to plant and set gravity.
    for (const auto& robot : robots) {
      DR::ModelGenerator<double>().AddRobotToMBP(robot, mbp.get());
      mbp->mutable_gravity_field().set_gravity_vector(robot.gravity());
    }
    mbp->Finalize();

    auto robot_context = mbp->CreateDefaultContext();

    drake::log()->debug("Time = {}", this->context().get_time());

    // Get positions and velocities from discrete state plant.
    auto robot_q = model_->GetPositions(this->context());
    auto robot_v = model_->GetVelocities(this->context());

    drake::log()->debug("robot_q = {}", robot_q.transpose());
    drake::log()->debug("robot_v = {}", robot_v.transpose());

    // Set positions and velocities in continuous state plant.
    mbp->SetPositions(robot_context.get(), robot_q);
    mbp->SetVelocities(robot_context.get(), robot_v);

    // Set commanded torques in continuous state plant from diagram with universal, discrete state plant.
    for (auto& robot : robots) {
      drake::multibody::ModelInstanceIndex model_instance = model_->GetModelInstanceByName(robot.name());
      drake::multibody::ModelInstanceIndex mbp_instance = mbp->GetModelInstanceByName(robot.name());
      const auto& model_context = diagram_->GetSubsystemContext(*model_, *sim_context_.value());
      drake::VectorX<double> control_input_vector =
          model_->get_actuation_input_port(model_instance).Eval(model_context);
      drake::log()->debug("control_input_vector ({}) = {}", robot.name(), control_input_vector.transpose());

      auto control_input = std::make_unique<drake::systems::BasicVector<double>>(mbp->num_actuated_dofs(mbp_instance));
      control_input->get_mutable_value() << control_input_vector;
      robot_context->FixInputPort(mbp->get_actuation_input_port(mbp_instance).get_index(), std::move(control_input));
    }

    // Calculate accelerations of continuous state plant resulting from control input.
    std::unique_ptr<drake::systems::ContinuousState<T>> derivatives = mbp->AllocateTimeDerivatives();
    mbp->CalcTimeDerivatives(*robot_context.get(), derivatives.get());

    // Get and return accelerations.
    auto robot_v_dot = derivatives->get_generalized_velocity().CopyToVector();
    auto robot_q_dot = derivatives->get_generalized_position().CopyToVector();
    drake::log()->debug("robot_q_dot (norm = {}) = {}", robot_q_dot.norm(), robot_q_dot.transpose());
    drake::log()->debug("robot_v_dot (norm = {}) = {}", robot_v_dot.norm(), robot_v_dot.transpose());
    return robot_v_dot;
  }

  void BuildAndSimulate(const UnloadingTaskConfig& config) {
    this->BuildSystem(config);
    this->SimulateSystem(config.simulator_instance_config());
  }
};

#ifndef GTEST_HAS_TYPED_TEST_P
#error "This test requires GTEST_HAS_TYPED_TEST_P"
#else  // GTEST_HAS_TYPED_TEST_P

TYPED_TEST_SUITE_P(FullSimulationTest);

/**
 * Stationary Full Robot Simulation Test
 * Generates a simulation with robot controller, dynamic and static bodies and
 * contact.  The robot is meant to hold position over the duration of this test.
 *
 * A static and dynamic box instance of BodyInstanceConfig:
 *    -- Box {floating (manipuland body), fixed (environment body)}
 *
 * An instance of EnvironmentInstanceConfig for all supported EnvironmentType,
 * currently:
 *    -- Trailer
 *
 * Two robot instances ("chopstick_{left,right}") in each simulation with the
 * ControlScheme:
 *    -- Stationary (holds at initial position)
 *
 * see include/DR/simulation/config.h for documentation on UnloadingTaskConfig
 */
TYPED_TEST_P(FullSimulationTest, StationaryRobot) {
  SimulatorInstanceConfig simulator;
  // The simulation will run for 1 second of virtual time.
  simulator.set_simulation_time(1.0);
  simulator.set_target_realtime_rate(100.0);
  simulator.set_integration_scheme(SimulatorInstanceConfig::kImplicitEulerIntegrationScheme);

  std::vector<BodyInstanceConfig> manipulands;
  // Floating manipuland Bodies.
  {
    BodyInstanceConfig manipuland;
    manipuland.set_mass(10.0);
    manipuland.set_is_floating(true);
    manipuland.SetCoulombFriction(0.8, 0.6);
    // Setup Box manipuland attributes.
    manipuland.SetBoxGeometry(0.2, 0.3, 0.1);

    // Set pose of manipuland in world.
    manipuland.set_name(std::move("manip_box_1"));
    manipuland.set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(), drake::Vector3<double>(1.0, 0.0, 0.7)));
    manipulands.push_back(manipuland);

    // Set pose of manipuland in world.
    manipuland.set_name(std::move("manip_box_2"));
    manipuland.set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(), drake::Vector3<double>(1.0, 0.0, 0.8)));
    manipulands.push_back(manipuland);
  }

  // Pair of chopstick robots.
  std::vector<RobotInstanceConfig> robots = CreateChopstickRobotsConfig();
  for (auto& robot : robots) {
    robot.set_control_scheme(RobotInstanceConfig::kStationaryControlScheme);
  }

  // Setup Environment options.
  EnvironmentInstanceConfig environment;
  {
    // Static Environment Bodies.
    std::vector<BodyInstanceConfig> env_bodies;
    {
      BodyInstanceConfig env_body;
      env_body.set_is_floating(false);
      // Set pose of static body in world.
      env_body.set_pose(
          drake::math::RigidTransform<double>(drake::math::RollPitchYaw<double>(0.0, M_PI_2, 0.0).ToRotationMatrix(),
                                              drake::Vector3<double>(1.0, 0.0, 0.4)));
      env_body.SetCoulombFriction(0.8, 0.6);

      // Setup Cylinder attributes.
      env_body.set_name(std::move("env_cylinder_1"));
      env_body.SetCylinderGeometry(0.05, 0.3);
      env_bodies.push_back(env_body);
    }
    environment.set_body_instance_configs(env_bodies);

    // Use trailer environment.
    environment.set_trailer_environment();
    environment.SetCoulombFriction(0.8, 0.6);
  }

  UnloadingTaskConfig config;
  config.set_simulator_instance_config(simulator);
  config.set_manipuland_instance_configs(manipulands);
  config.set_robot_instance_configs(robots);
  config.set_environment_instance_config(environment);

  ASSERT_NO_THROW(this->BuildAndSimulate(config));
  ASSERT_NEAR(this->context().get_time(), config.simulator_instance_config().simulation_time(),
              config.simulator_instance_config().step_size());
}

/**
 * Stationary Robot Simulation Test (Empty Scene)
 * Tests whether the simulator can simulate a pair of chopstick robots holding a fixed position for one simulation step
 * under the influence of gravity in an empty scene with a floor plane.
 */
TYPED_TEST_P(FullSimulationTest, StationaryRobotNoBodies) {
  SimulatorInstanceConfig simulator;
  // The simulation will run for 1 second of virtual time.
  simulator.set_simulation_time(1.0);
  simulator.set_target_realtime_rate(100.0);
  simulator.set_integration_scheme(SimulatorInstanceConfig::kImplicitEulerIntegrationScheme);

  // Pair of chopstick robots.
  std::vector<RobotInstanceConfig> robots = CreateChopstickRobotsConfig();
  for (auto& robot : robots) {
    robot.set_control_scheme(RobotInstanceConfig::kStationaryControlScheme);
  }

  // Setup Environment options.
  EnvironmentInstanceConfig environment;
  // Use trailer environment.
  environment.set_floor_environment();
  environment.SetCoulombFriction(0.8, 0.6);

  UnloadingTaskConfig config;
  config.set_simulator_instance_config(simulator);
  config.set_robot_instance_configs(robots);
  config.set_environment_instance_config(environment);

  // Setup robot model in simulation.
  this->BuildSystem(config);

  // Get system initial state.
  drake::VectorX<typename TestFixture::MyParamType> world_q = this->model().GetPositions(this->context());

  const double test_tolerance = 1e-10;

  // Check that the robot is not being commanded to accelerate.
  drake::VectorX<typename TestFixture::MyParamType> world_v_dot = this->GetControlledChopsticksAcceleration();
  EXPECT_NEAR(world_v_dot.norm(), 0.0, test_tolerance);

  // Simulate for one simulation step
  this->SimulateSystem(config.simulator_instance_config());

  // Check that the robot has held position for the test duration.

  // Check that the robot's joint velocities are zero.
  drake::VectorX<typename TestFixture::MyParamType> new_world_v = this->model().GetVelocities(this->context());
  EXPECT_NEAR(new_world_v.norm(), 0.0, test_tolerance);

  // Check that the robot's current joint positions are near initial joint positions.
  drake::VectorX<typename TestFixture::MyParamType> new_world_q = this->model().GetPositions(this->context());
  EXPECT_NEAR((new_world_q - world_q).norm(), 0.0, test_tolerance);
}

/**
 * Passive Robot Simulation Test (Empty Scene)
 * Tests whether the simulator can simulate a pair of chopstick robots with no active controller (actuators maintain
 * zero effort) falling under the influence of gravity in an empty scene with a floor plane.
 */
TYPED_TEST_P(FullSimulationTest, PassiveRobotNoBodies) {
  SimulatorInstanceConfig simulator;
  // The simulation will run for 1 second of virtual time.
  simulator.set_simulation_time(1.0);
  simulator.set_target_realtime_rate(100.0);
  simulator.set_integration_scheme(SimulatorInstanceConfig::kImplicitEulerIntegrationScheme);

  // Pair of chopstick robots.
  std::vector<RobotInstanceConfig> robots = CreateChopstickRobotsConfig();

  // Setup Environment options.
  EnvironmentInstanceConfig environment;
  {
    // Use trailer environment.
    environment.set_trailer_environment();
    environment.SetCoulombFriction(0.8, 0.6);
  }

  UnloadingTaskConfig config;
  config.set_simulator_instance_config(simulator);
  config.set_robot_instance_configs(robots);
  config.set_environment_instance_config(environment);

  ASSERT_NO_THROW(this->BuildAndSimulate(config));
  ASSERT_NEAR(this->context().get_time(), config.simulator_instance_config().simulation_time(),
              config.simulator_instance_config().step_size());
}

/**
 Passive Bodies Simulation Test
 Generates a stack of floating and fixed base Boxes.
 The boxes are situated in a "Trailer" instance of EnvironmentInstanceConfig.
 See include/DR/simulation/config.h for documentation on UnloadingTaskConfig.
 */
// TODO(samzapo): Replace this test with a test of the clutter generation code.
TYPED_TEST_P(FullSimulationTest, PassiveBodies) {
  SimulatorInstanceConfig simulator;
  // The simulation will run for 1 second of virtual time.
  simulator.set_simulation_time(1.0);
  simulator.set_target_realtime_rate(100.0);
  simulator.set_integration_scheme(SimulatorInstanceConfig::kImplicitEulerIntegrationScheme);

  double box_height = 0.1;

  std::vector<BodyInstanceConfig> manipulands;
  // Floating manipuland Bodies.
  {
    BodyInstanceConfig manipuland;
    manipuland.set_mass(10.0);
    manipuland.set_is_floating(true);
    manipuland.SetCoulombFriction(0.8, 0.6);
    // Setup Box manipuland attributes.
    manipuland.SetBoxGeometry(0.2, 0.3, box_height);
    int max_manipulands = 3;
    for (int i = 0; i < max_manipulands; ++i) {
      // Set pose of manipuland in world.
      manipuland.set_name("manip_box_" + std::to_string(i));
      double box_z_offset = box_height * 0.5 + box_height * i;
      manipuland.set_pose(
          drake::math::RigidTransform<double>(drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                              drake::Vector3<double>(1.0, 0.0, 1.0 + box_z_offset)));
      manipulands.push_back(manipuland);
    }
  }

  // Setup Environment options.
  EnvironmentInstanceConfig environment;
  {
    // Static Environment Bodies.
    std::vector<BodyInstanceConfig> env_bodies;
    // Floating manipuland Bodies.
    {
      BodyInstanceConfig env_body;
      env_body.set_is_floating(false);
      env_body.SetCoulombFriction(0.8, 0.6);
      // Setup Box env_body attributes.
      env_body.SetBoxGeometry(0.2, 0.3, box_height);
      int max_env_body = 10;
      for (int i = 0; i < max_env_body; ++i) {
        // Set pose of env_body in world.
        env_body.set_name("env_body_box_" + std::to_string(i));
        double box_z_offset = box_height * 0.5 + box_height * i;
        env_body.set_pose(
            drake::math::RigidTransform<double>(drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                                drake::Vector3<double>(1.0, 0.0, 0.0 + box_z_offset)));
        env_bodies.push_back(env_body);
      }
    }
    environment.set_body_instance_configs(env_bodies);

    // Use trailer environment.
    environment.set_trailer_environment();
    environment.SetCoulombFriction(0.8, 0.6);
  }

  UnloadingTaskConfig config;
  config.set_simulator_instance_config(simulator);
  config.set_manipuland_instance_configs(manipulands);
  config.set_environment_instance_config(environment);

  ASSERT_NO_THROW(this->BuildAndSimulate(config));
  ASSERT_NEAR(this->context().get_time(), config.simulator_instance_config().simulation_time(),
              config.simulator_instance_config().step_size());
}

REGISTER_TYPED_TEST_SUITE_P(FullSimulationTest, StationaryRobot, StationaryRobotNoBodies, PassiveRobotNoBodies,
                            PassiveBodies);

typedef ::testing::Types<double> DrakeScalarTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(SimulationSupportedScalars, FullSimulationTest, DrakeScalarTypes);

#endif  // GTEST_HAS_TYPED_TEST_P
}  // namespace
}  // namespace DR
