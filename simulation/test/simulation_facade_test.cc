#include <DR/simulation/simulation_facade.h>

#include <DR/drivers/chopstick/chopstick_config.h>
#include <DR/simulation/config.h>

#define GTEST_HAS_EXCEPTIONS 1
#include <gtest/gtest.h>

namespace DR {
namespace {

/*
 The SimulationFacadeTests create a child class of SimulationFacade to generate and
 run a simulation with static and dynamic single bodies and robots configured according to configuration
 parameters provided by SingleBodyInstanceConfig and RobotSimulationDriver from 'DR/simulation/config.h'.
 */
template <typename T>
class BodySimulationDriver final : public SimulationFacade<T> {
 public:
  BodySimulationDriver() {}
  virtual ~BodySimulationDriver() {}

  explicit BodySimulationDriver(std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> body_instance_configs)
      : body_instance_configs_(std::move(body_instance_configs)) {}

  virtual void DoPopulateModelAndScenegraph() {
    for (const auto& body_config : body_instance_configs_) {
      AddBodyToMBP(*body_config.get(), &this->universal_plant());
    }
  }

  virtual void DoConstructAndConnectDiagram() {}

  // Create a context from the diagram and set state according to the configuration.
  virtual void DoSetState(drake::systems::Context<T>* context) {
    drake::systems::Context<T>& mbp_context =
        this->diagram().GetMutableSubsystemContext(this->universal_plant(), context);

    // StateSetter should address all states.  Abstract states are not currently addressed because there are none. This
    // check will trigger an error if abstract states are added but not addressed here.
    // TODO(samzapo): Set initial abstract states.
    DR_DEMAND(mbp_context.num_abstract_states() == 0);

    // NOTE: StateSetter does not address check num_discrete_state_groups.  Discrete states specifically left out
    // because MBP can run in two modes: continuous mode, where the equations of motions are integrated using an ODE
    // solver, and discrete mode, where the equations of motion are integrated using a time-stepping type scheme. In the
    // latter case, the kinematic state will be discrete variables.

    // Sets the position and velocity of the dynamic bodies in the world.
    for (const auto& body : body_instance_configs_) {
      // Set the pose and spatial velocity of this floating body in world frame.
      SetBodyPoseAndVelocity(this->universal_plant().GetBodyByName(body->name()), body->pose(),
                             body->spatial_velocity(), this->universal_plant(), &mbp_context);
    }
  }

 private:
  const std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> body_instance_configs_;
};
}  // namespace

using T = double;
/**
 * Passive Bodies test.
 *
 * Tests whether the simulation_facade child class can create a simulation for and then simulate dynamic single bodies
 * falling under the influence of gravity and coming into contact with other static or dynamic single bodies.
 */
TEST(SimulationFacadeTest, PassiveBodies) {
  std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> bodies;

  double box_height = 0.1;
  double top_box_height = 1.0;

  // Boxes are balanced on cylinder.
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("dynamic_box_top");
    body->set_mass_and_possibly_make_static(10.0);
    body->SetCoulombFriction(0.8, 0.6);
    body->SetBoxGeometry(0.2, 0.3, box_height);
    body->set_pose(
        drake::math::RigidTransform<double>(drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                            drake::Vector3<double>(1.0, 0.0, top_box_height)));
    bodies.insert(std::move(body));
  }
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("dynamic_box_bottom");
    body->set_mass_and_possibly_make_static(10.0);
    body->SetCoulombFriction(0.8, 0.6);
    body->SetBoxGeometry(0.2, 0.3, box_height);
    body->set_pose(
        drake::math::RigidTransform<double>(drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                            drake::Vector3<double>(1.0, 0.0, top_box_height - box_height)));
    bodies.insert(std::move(body));
  }
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("static_cylinder");
    body->set_mass_and_possibly_make_static(0.0 /* static */);
    body->SetCoulombFriction(0.8, 0.6);
    body->SetCylinderGeometry(box_height * 0.5, 0.3);
    // Pitched forward 90 deg, axis of cylinder is now +x.
    body->set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, M_PI_2, 0.0).ToRotationMatrix(),
        drake::Vector3<double>(1.0, 0.0, top_box_height - box_height * 2.0 - 0.01)));
    body->SetCoulombFriction(0.8, 0.6);
    bodies.insert(std::move(body));
  }

  // Box is sitting on floor.
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("dynamic_box_floor");
    body->set_mass_and_possibly_make_static(10.0);
    body->SetCoulombFriction(0.8, 0.6);
    body->SetBoxGeometry(0.2, 0.3, box_height);
    body->set_pose(
        drake::math::RigidTransform<double>(drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                            drake::Vector3<double>(1.0, 0.0, box_height * 0.5 + 0.01)));
    bodies.insert(std::move(body));
  }

  // Static body of Floor (large box geometry).
  {
    const double kFloorThickness = 0.1;
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("static_box_floor");
    body->set_mass_and_possibly_make_static(0.0 /* static */);
    body->SetCoulombFriction(0.8, 0.6);
    body->SetBoxGeometry(100.0, 100.0, kFloorThickness);
    body->set_pose(
        drake::math::RigidTransform<double>(drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                            drake::Vector3<double>(0.0, 0.0, -kFloorThickness * 0.5)));
    bodies.insert(std::move(body));
  }

  // Test SimulatorFacade derived class.
  std::unique_ptr<BodySimulationDriver<T>> driver;
  ASSERT_NO_THROW(driver = std::make_unique<BodySimulationDriver<T>>(std::move(bodies)));

  const double simulation_time = 0.1;

  // Check build.
  SimulatorInstanceIndex simulator_instance_index;
  ASSERT_NO_THROW(simulator_instance_index = driver->CreateSimulatedSystem());

  // Get simulator instance and advance to the target time.
  ASSERT_NO_THROW(driver->simulator(simulator_instance_index).AdvanceTo(simulation_time));
}

}  // namespace DR
