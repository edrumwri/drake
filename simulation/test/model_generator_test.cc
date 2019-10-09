#include <gtest/gtest.h>

#include <drake/common/autodiff.h>

#include <DR/drivers/chopstick/chopstick_config.h>
#include <DR/simulation/config.h>
#include <DR/simulation/model_generator.h>
#include <DR/tools/output_stream_operators.h>

namespace DR {
namespace {

/*
 The ModelGeneratorTest generates a universal plant for
 several permutations of world models.

 TODO(samzapo): Replace the tests implemented in this file with a test of the clutter generation code.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.
         Options for T are the following: (☑ = supported)
         ☑ double
         ☐ drake::AutoDiffXd
         ☐ drake::symbolic::Expression
*/
template <typename T>
class ModelGeneratorTest : public ::testing::Test {
 protected:
  using MyParamType = T;

  drake::multibody::MultibodyPlant<T>* CreateMBP(drake::systems::DiagramBuilder<T>* builder, double step_size = 0.001) {
    drake::multibody::AddMultibodyPlantSceneGraphResult<T> result = drake::multibody::AddMultibodyPlantSceneGraph(
        builder, std::make_unique<drake::multibody::MultibodyPlant<T>>(step_size));
    drake::multibody::MultibodyPlant<T>* mbp = &(result.plant);

    return mbp;
  }
};

#if GTEST_HAS_TYPED_TEST

using testing::Types;

typedef Types<double> Implementations;

TYPED_TEST_SUITE(ModelGeneratorTest, Implementations);

/*
 Model Generator Test
 Generates permutations of SingleBodyInstanceConfig.
 This includes:
 A fixed and floating base instance of SingleBodyInstanceConfig for all supported
 derived classes of drake::geometry::Shape, currently:
    -- Box
    -- Sphere
    -- Cylinder
 See include/DR/simulation/config.h for documentation on SingleBodyInstanceConfig.
 TODO(samzapo) continue to update for each new mesh.
 */
TYPED_TEST(ModelGeneratorTest, SingleBodyInstanceConfig) {
  std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> bodies;

  // Sphere attributes.
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("sphere");
    body->set_mass_and_possibly_make_static(10.0);
    body->set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(), drake::Vector3<double>(1.0, 0.0, 1.0)));
    body->SetSphereGeometry(0.1);
    bodies.insert(std::move(body));
  }
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("sphere");
    body->set_mass_and_possibly_make_static(0.0 /* static */);
    body->set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(), drake::Vector3<double>(1.0, 0.0, 1.0)));
    body->SetSphereGeometry(0.1);
    bodies.insert(std::move(body));
  }

  // Box attributes.
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("box");
    body->set_mass_and_possibly_make_static(10.0);
    body->set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(), drake::Vector3<double>(1.0, 0.0, 1.0)));
    body->SetBoxGeometry(0.2, 0.3, 0.1);
    bodies.insert(std::move(body));
  }
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("box");
    body->set_mass_and_possibly_make_static(0.0 /* static */);
    body->set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(), drake::Vector3<double>(1.0, 0.0, 1.0)));
    body->SetBoxGeometry(0.2, 0.3, 0.1);
    bodies.insert(std::move(body));
  }

  // Cylinder attributes.
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("cylinder");
    body->set_mass_and_possibly_make_static(10.0);
    body->set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(), drake::Vector3<double>(1.0, 0.0, 1.0)));
    body->SetCylinderGeometry(0.05, 0.3);
    bodies.insert(std::move(body));
  }
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("cylinder");
    body->set_mass_and_possibly_make_static(0.0 /* static */);
    body->set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(), drake::Vector3<double>(1.0, 0.0, 1.0)));
    body->SetCylinderGeometry(0.05, 0.3);
    bodies.insert(std::move(body));
  }

  // Static-only Shapes.
  // Setup HalfSpace attributes.
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("half_space");
    body->set_mass_and_possibly_make_static(0.0 /* static */);
    body->set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(), drake::Vector3<double>(1.0, 0.0, 0.0)));
    body->SetHalfSpaceGeometry();
    bodies.insert(std::move(body));
  }

  // Run test for all permutations.
  for (const auto& body : bodies) {
    drake::systems::DiagramBuilder<typename TestFixture::MyParamType> builder;
    auto* mbp = this->CreateMBP(&builder);

    // Add a floating body to the plant so it can finalize (requires nv > 0).
    auto token_floating_body = std::make_unique<SingleBodyInstanceConfig>();
    token_floating_body->set_name("token_floating_body");
    token_floating_body->set_mass_and_possibly_make_static(10.0);
    token_floating_body->set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(), drake::Vector3<double>(1.0, 1.0, 1.0)));
    token_floating_body->SetSphereGeometry(0.1);
    AddBodyToMBP(*token_floating_body.get(), mbp);

    // Add the tested body to the plant.
    drake::multibody::ModelInstanceIndex model_instance = AddBodyToMBP(*body.get(), mbp);

    mbp->Finalize();

    EXPECT_EQ(mbp->num_actuated_dofs(model_instance), 0);
    if (body->is_static()) {
      EXPECT_EQ(mbp->num_positions(model_instance), 0);
      EXPECT_EQ(mbp->num_velocities(model_instance), 0);
    } else {
      EXPECT_EQ(mbp->num_positions(model_instance), 7);
      EXPECT_EQ(mbp->num_velocities(model_instance), 6);
    }
  }
}

/*
 Model Generator Test
 Generates bad permutations of SingleBodyInstanceConfig that will cause the program to abort.
 This includes:
    -- Weightless floating body.
    -- Floating mesh with no collision geometry.
    -- Mesh with a bad model path.
    -- Floating HalfSpace.
 see include/DR/simulation/config.h for documentation on SingleBodyInstanceConfig
 */
TYPED_TEST(ModelGeneratorTest, IncorrectSingleBodyInstanceConfigThrow) {
  std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> bodies;

  // Floating HalfSpace.
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("half_space");
    // Set mass, non-static.
    body->set_mass_and_possibly_make_static(10.0);
    body->set_pose(drake::math::RigidTransform<double>(
        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(), drake::Vector3<double>(1.0, 0.0, 1.0)));
    body->SetHalfSpaceGeometry();
    bodies.insert(std::move(body));
  }

  // Run test for all permutations.
  for (const auto& body : bodies) {
    EXPECT_THROW(
        {
          drake::systems::DiagramBuilder<typename TestFixture::MyParamType> builder;
          auto* mbp = this->CreateMBP(&builder);
          AddBodyToMBP(*body.get(), mbp);
          mbp->Finalize();
        },
        std::exception);
  }
}

// This test checks for the correct number of degrees of freedom after creating two floating base chopstick
// robots ( 5 actuated degrees of freedom + base degrees of freedom each) with `ModelGenerator`.
TYPED_TEST(ModelGeneratorTest, FloatingBaseRobotTest) {
  // Pair of chopstick robots.
  std::unordered_set<std::unique_ptr<RobotInstanceConfig>> robots =
      CreateChopstickRobotsConfig("robot_", drake::math::RigidTransform<double>::Identity());
  // Set floating base.
  for (auto& robot : robots) {
    robot->set_is_floating(true);
  }

  // Create Robot plant.
  drake::systems::DiagramBuilder<typename TestFixture::MyParamType> builder;
  auto* mbp = this->CreateMBP(&builder);

  // Add robots to plant.
  for (const auto& robot : robots) {
    AddRobotToMBP(*robot.get(), mbp);
  }
  mbp->Finalize();

  int plant_num_joint_dofs = 0;
  int plant_num_position_dofs = 0;
  int plant_num_velocity_dofs = 0;
  for (const auto& robot : robots) {
    drake::multibody::ModelInstanceIndex model_index = mbp->GetModelInstanceByName(robot->name());
    // TODO(samzapo): Correctness of this test is predicated on single-dof joints.
    // Expected num joint dofs in robot.
    int robot_num_joint_dofs = robot->joint_instance_configs().size();
    // 3 linear + 4 quaternion + joint DOFs.
    int robot_num_position_dofs = 7 + robot_num_joint_dofs;
    // 3 linear + 3 angular + joint DOFs.
    int robot_num_velocity_dofs = 6 + robot_num_joint_dofs;

    // Total dofs expected in plant.
    plant_num_joint_dofs += robot_num_joint_dofs;
    plant_num_position_dofs += robot_num_position_dofs;
    plant_num_velocity_dofs += robot_num_velocity_dofs;

    // Check for correct joint DOFs.
    EXPECT_EQ(mbp->num_actuated_dofs(model_index), robot_num_joint_dofs);
    EXPECT_EQ(mbp->num_positions(model_index), robot_num_position_dofs);
    EXPECT_EQ(mbp->num_velocities(model_index), robot_num_velocity_dofs);
  }

  EXPECT_EQ(mbp->num_actuated_dofs(), plant_num_joint_dofs);
  EXPECT_EQ(mbp->num_positions(), plant_num_position_dofs);
  EXPECT_EQ(mbp->num_velocities(), plant_num_velocity_dofs);
}

// This test checks for the correct number of degrees of freedom after creating two fixed base chopstick
// robots ( 5 actuated degrees of freedom + base degrees of freedom each) with `ModelGenerator`.
TYPED_TEST(ModelGeneratorTest, FixedBaseRobotTest) {
  // Pair of chopstick robots.
  std::unordered_set<std::unique_ptr<RobotInstanceConfig>> robots =
      CreateChopstickRobotsConfig("robot_", drake::math::RigidTransform<double>::Identity());

  // Create Robot plant.
  drake::systems::DiagramBuilder<typename TestFixture::MyParamType> builder;
  auto* mbp = this->CreateMBP(&builder);

  // Add robots to plant.
  for (const auto& robot : robots) {
    AddRobotToMBP(*robot.get(), mbp);
  }
  mbp->Finalize();

  int plant_num_joint_dofs = 0;
  for (const auto& robot : robots) {
    drake::multibody::ModelInstanceIndex model_index = mbp->GetModelInstanceByName(robot->name());
    // TODO(samzapo): Correctness of this test is predicated on single-dof joints.
    // Expected num joint dofs in robot.
    int robot_num_joint_dofs = 0;
    for (const JointInstanceConfig& joint : robot->joint_instance_configs()) {
      robot_num_joint_dofs += joint.dofs();
    }

    // Total dofs expected in plant.
    plant_num_joint_dofs += robot_num_joint_dofs;

    // Check for correct joint DOFs from model instance in plant.
    EXPECT_EQ(mbp->num_actuated_dofs(model_index), robot_num_joint_dofs);
    EXPECT_EQ(mbp->num_positions(model_index), robot_num_joint_dofs);
    EXPECT_EQ(mbp->num_velocities(model_index), robot_num_joint_dofs);
  }

  EXPECT_EQ(mbp->num_actuated_dofs(), plant_num_joint_dofs);
  EXPECT_EQ(mbp->num_positions(), plant_num_joint_dofs);
  EXPECT_EQ(mbp->num_velocities(), plant_num_joint_dofs);
}

#endif  // GTEST_HAS_TYPED_TEST
}  // namespace
}  // namespace DR
