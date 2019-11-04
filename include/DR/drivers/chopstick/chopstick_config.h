#pragma once

#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include <DR/common/environment.h>
#include <DR/simulation/config.h>
#include <DR/simulation/model_generator.h>
#include <DR/tools/optional.h>
#include <DR/tools/output_stream_operators.h>

namespace DR {

// This default value specifies the fixed-base separation distance between chopsticks models where two robots are placed
// side-by-side in a trailer and is specific to trailer unloading task.
// This parameter should not be sampleable via Monte Carlo because it is used to set an initial condition, and is not a
// modeling parameter.
const double kDefaultChopsticksSeparationWidth = 2.54;

/**
 Configuration function for the Chopstick robot.  Will create a valid RobotInstanceConfig with optional parameters for
 initial state and control gains.
 @param name a unique name for the robot.
 @param model_directory the absolute path to the directory with the robot model file.
 @param model_file a relative path from `model_directory` to the robot's SDF.
 @param base_pose (optional) the 6d pose of the robot's base link.
 @param base_spatial_velocity (optional) the 6d spatial velocity of the robot's base link.
 @param joint_positions (optional) a map of initial joint positions, keys are the joint names.  Missing keys will use
        the default values and extra keys won't be accessed.
 @param joint_velocities (optional) a map of initial joint velocities, keys are the joint names.  Missing keys will use
        the default values and extra keys won't be accessed.
 @param joint_kps (optional) a map of joint positional feedback gains, keys are the joint names.  Missing keys will use
        the default values and extra keys won't be accessed.
 @return a unique_ptr to the configuration for the chopstick robot with optional parameters applied.
 */
std::unique_ptr<RobotInstanceConfig> CreateSingleChopstickRobotConfig(
    const std::string& name, const std::string& model_file_path,
    const std::optional<drake::math::RigidTransform<double>>& base_pose = {},
    const std::optional<drake::multibody::SpatialVelocity<double>>& base_spatial_velocity = {},
    const std::optional<std::map<std::string, double>>& joint_positions = {},
    const std::optional<std::map<std::string, double>>& joint_velocities = {},
    const std::optional<std::map<std::string, double>>& joint_kps = {}) {
  // Set fixed valued specific to Chopstick robot.
  std::vector<std::string> joint_names{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5"};

  std::map<std::string, JointInstanceConfig::JointType> joint_type{
      {"joint_1", JointInstanceConfig::JointType::kPrismaticJointType},
      {"joint_2", JointInstanceConfig::JointType::kPrismaticJointType},
      {"joint_3", JointInstanceConfig::JointType::kRevoluteJointType},
      {"joint_4", JointInstanceConfig::JointType::kPrismaticJointType},
      {"joint_5", JointInstanceConfig::JointType::kRevoluteJointType}};

  // Set default values.
  const std::map<std::string, double> default_joint_kps{
      {"joint_1", 10.0}, {"joint_2", 10.0}, {"joint_3", 10.0}, {"joint_4", 10.0}, {"joint_5", 10.0}};

  // Revolute joints are set to zero (chopsticks pointing straight in the +x-axis direction) and prismatic joints are
  // set to place the chopsticks at the center of the workspace.
  const std::map<std::string, double> default_joint_positions{
      {"joint_1", 0.5}, {"joint_2", 0.6}, {"joint_3", 0.0}, {"joint_4", 1.1}, {"joint_5", 0.0}};

  const std::map<std::string, double> default_joint_velocities{
      {"joint_1", 0.0}, {"joint_2", 0.0}, {"joint_3", 0.0}, {"joint_4", 0.0}, {"joint_5", 0.0}};

  const drake::math::RigidTransform<double> default_base_pose = drake::math::RigidTransform<double>::Identity();

  const drake::multibody::SpatialVelocity<double> default_base_spatial_velocity =
      drake::multibody::SpatialVelocity<double>::Zero();

  // Configure joints.
  std::vector<JointInstanceConfig> joints;
  for (const std::string& joint_name : joint_names) {
    joints.emplace_back(
        joint_name, joint_type.at(joint_name),
        GetOptionalValueOrDefault(joint_positions, joint_name,
                                  default_joint_positions.at(joint_name)) /* joint position */,
        GetOptionalValueOrDefault(joint_velocities, joint_name,
                                  default_joint_velocities.at(joint_name)) /* joint velocity */,
        GetOptionalValueOrDefault(joint_kps, joint_name, default_joint_kps.at(joint_name)) /* joint kp gain */);
  }

  // Create robot config and return.
  auto robot_instance_config = std::make_unique<RobotInstanceConfig>(name, model_file_path, joints);

  robot_instance_config->set_pose(GetOptionalValueOrDefault(base_pose, default_base_pose));
  robot_instance_config->set_spatial_velocity(
      GetOptionalValueOrDefault(base_spatial_velocity, default_base_spatial_velocity));

  return robot_instance_config;
}

/**
  A helper function for configuring a pair of chopstick robots for the Unloading Task.
  @return std::vector<RobotInstanceConfig> a vector of two chopstick robot configurations.
*/
std::unordered_set<std::unique_ptr<RobotInstanceConfig>> CreateChopstickRobotsConfig(
    const std::string& name = "",
    const drake::math::RigidTransform<double>& pose = drake::math::RigidTransform<double>::Identity()) {
  std::unordered_set<std::unique_ptr<RobotInstanceConfig>> robots;

  std::string absolute_model_path = GetModelDirectoryFromEnvironment();

  // Offset chopstick robots by half the combined robot's width.
  drake::math::RigidTransform<double> left_pose =
      pose *
      drake::math::RigidTransform<double>(drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                          drake::Vector3<double>(0.0, kDefaultChopsticksSeparationWidth * 0.5, 0.0));
  drake::math::RigidTransform<double> right_pose =
      pose *
      drake::math::RigidTransform<double>(drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                          drake::Vector3<double>(0.0, -kDefaultChopsticksSeparationWidth * 0.5, 0.0));

  {
    std::unique_ptr<RobotInstanceConfig> robot =
        CreateSingleChopstickRobotConfig(name + "chopstick_left", absolute_model_path + "/chopstick/chopstick_left.sdf",
                                         left_pose, {} /* default spatial velocity */, {} /* default joint position */,
                                         {} /* default joint velocity */, {} /* default kp */);
    robots.insert(std::move(robot));
  }
  {
    std::unique_ptr<RobotInstanceConfig> robot = CreateSingleChopstickRobotConfig(
        name + "chopstick_right", absolute_model_path + "/chopstick/chopstick_right.sdf", right_pose,
        {} /* default spatial velocity */, {} /* default joint position */, {} /* default joint velocity */,
        {} /* default kp */);
    robots.insert(std::move(robot));
  }

  return robots;
}

struct ChopsticksInstances {
  drake::multibody::ModelInstanceIndex left_instance;
  drake::multibody::ModelInstanceIndex right_instance;
  /// For assignment to a std::tie of values.
  operator std::tuple<drake::multibody::ModelInstanceIndex&, drake::multibody::ModelInstanceIndex&>() {
    return std::tie(left_instance, right_instance);
  }
};

/**
  A helper function for adding a pre-configured pair of chopstick robots to a multibody plant.  Returns a pair
  containing the left and right chopstick model instances, respectively.
  @mbp a pointer to a multibody plant.  The chopsticks models will be added to this plant.
  @returns a pir of model instances for the left and right chopstick robot, respectively.
*/
template <typename T>
ChopsticksInstances AddChopsticksToMBP(drake::multibody::MultibodyPlant<T>* mbp) {
  ChopsticksInstances chopstick_instances;
  std::unordered_set<std::unique_ptr<RobotInstanceConfig>> robots = CreateChopstickRobotsConfig();
  for (auto& robot : robots) {
    if (robot->name() == "chopstick_left") {
      chopstick_instances.left_instance = AddRobotToMBP(*robot.get(), mbp);
    } else {
      DR_DEMAND(robot->name() == "chopstick_right");
      chopstick_instances.right_instance = AddRobotToMBP(*robot.get(), mbp);
    }
  }
  return chopstick_instances;
}

}  // namespace DR
