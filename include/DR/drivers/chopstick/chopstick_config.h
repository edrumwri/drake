#pragma once

#include <map>
#include <string>
#include <vector>

#include <DR/simulation/config.h>
#include <DR/tools/optional.h>

namespace DR {

/**
 Configuration function for the Chopstick robot.  Will create a valid RobotInstanceConfig with optional parameters for
 initial state and control gains.
 @param name a unique name for the robot.
 @param model_directory the absolute path to the directory with the robot model file.
 @param model_file a relative path from `model_directory` to the robot's SDF.
 @param base_pose the 6d pose of the robot's base link.
 @param base_spatial_velocity the 6d spatial velocity iof the robot's base link.
 @param joint_positions an Nd vector of initial joint positions.
 @param joint_velocities an Nd vector of initial joint velocities.
 @param joint_kps an Nd vector of joint positional feedback gains.
 @return RobotInstanceConfig the configuration for the chopstick robot with optional parameters applied.
 */
RobotInstanceConfig CreateChopstickRobotInstanceConfig(
    const std::string& name, const std::string& model_directory, const std::string& model_file,
    const drake::optional<drake::math::RigidTransform<double>>& base_pose = {},
    const drake::optional<drake::multibody::SpatialVelocity<double>>& base_spatial_velocity = {},
    const drake::optional<std::map<std::string, double>>& joint_positions = {},
    const drake::optional<std::map<std::string, double>>& joint_velocities = {},
    const drake::optional<std::map<std::string, double>>& joint_kps = {}) {
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
  RobotInstanceConfig robot_instance_config(name, model_directory, model_file, joints);

  robot_instance_config.set_pose(GetOptionalValueOrDefault(base_pose, default_base_pose));
  robot_instance_config.set_spatial_velocity(
      GetOptionalValueOrDefault(base_spatial_velocity, default_base_spatial_velocity));

  return robot_instance_config;
}

/**
 A helper function for configuring a pair of chopstick robots for the Unloading Task.
 @return std::vector<RobotInstanceConfig> a vector of two chopstick robot configurations.
 */
std::vector<RobotInstanceConfig> CreateChopstickRobotsConfig() {
  std::vector<RobotInstanceConfig> robots;

  // Get the absolute model path from an environment variable.
  const char* absolute_model_path_env_var = std::getenv("DR_ABSOLUTE_MODEL_PATH");
  DR_DEMAND(absolute_model_path_env_var);
  std::string absolute_model_path = std::string(absolute_model_path_env_var);

  // Add a trailing slash if necessary.
  if (absolute_model_path.back() != '/')
    absolute_model_path += '/';

  // Get default trailer width.
  // TODO(samzapo): Build the environment model in this file as well.
  EnvironmentInstanceConfig default_environment;
  default_environment.set_trailer_environment();
  double default_trailer_width = default_environment.trailer_size()[1];

  // Offset left chopstick half the trailer width to the left.
  robots.push_back(CreateChopstickRobotInstanceConfig(
      "chopstick_left", absolute_model_path, "/chopstick/chopstick_left.sdf",
      drake::math::RigidTransform<double>(drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                          drake::Vector3<double>(0.0, default_trailer_width / 2.0, 0.0))));

  // Offset right chopstick half the trailer width to the right.
  robots.push_back(CreateChopstickRobotInstanceConfig(
      "chopstick_right", absolute_model_path, "/chopstick/chopstick_right.sdf",
      drake::math::RigidTransform<double>(drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                          drake::Vector3<double>(0.0, -default_trailer_width / 2.0, 0.0))));

  // Robots have a fixed base by default.
  for (auto& robot : robots) {
    robot.set_is_floating(false);
  }

  return robots;
}
}  // namespace DR
