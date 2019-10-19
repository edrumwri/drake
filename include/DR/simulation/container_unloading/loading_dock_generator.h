#pragma once

#include <algorithm>
#include <map>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <drake/common/random.h>
#include <drake/common/text_logging.h>

#include <DR/common/logging.h>
#include <DR/simulation/config.h>
#include <DR/tools/output_stream_operators.h>
#include <DR/tools/uniform_vector_distribution.h>

namespace DR {
/**
 A class for generating the relevent bodies to simulate a loading dock with a mated trailer.
 Random variables for the loading dock include:
 - Coulomb Friction {trailer floor, conveyor} x {static, dynamic}.
 - Trailer size {length, width, height}.
 - Conveyor translation w.r.t. dock floor dofs: {x, y, z}.
 // TODO(samzapo): Randomly sample orientation within set ranges {roll, pitch, yaw}.
 - Robot mount-point translation w.r.t. conveyor dofs: {x, y, z}.
 // TODO(samzapo): Randomly sample orientation within set ranges {roll, pitch, yaw}.
 - Trailer translation w.r.t. dock floor dofs: {x, y, z}.
 // TODO(samzapo): Randomly sample orientation within set ranges {roll, pitch, yaw}.
 // TODO(samzapo): Randomly sample color of: {trailer wall, trailer floor, conveyor, dock floor} x {r, g, b}.
 - Conveyor length-wise extension into trailer.
 // TODO(samzapo): model conveyor as robot w/ extension prismatic joint.
 - Conveyor belt width.
 */
class LoadingDockGenerator {
 public:
  // Types of random variables for the loading dock:
  enum class AttributeType {
    kTrailerPoseTranslation,
    kConveyorPoseTranslation,
    kRobotMountPoseTranslation,
    kTrailerSize,
    kTrailerCoulombFriction,
    kConveyorCoulombFriction,
    kConveyorExtension,
    kConveyorBeltWidth,
  };

  // No copy-construction, or copy-assignment.
  LoadingDockGenerator(const LoadingDockGenerator&) = delete;
  void operator=(const LoadingDockGenerator&) = delete;

  // No move-construction, or move-assignment.
  LoadingDockGenerator(LoadingDockGenerator&&) = delete;
  void operator=(LoadingDockGenerator&&) = delete;

  // Default destructor.
  ~LoadingDockGenerator() = default;

  LoadingDockGenerator() = delete;

  /**
   @param model_directory the absolute path to the directory of mesh models for objects.
   */
  explicit LoadingDockGenerator(const std::string& model_directory) : model_directory_(model_directory) {}

  /**
   Creates configurations for a collection of static bodies to represent truck trailer.
   The trailer is a hollow box with its -x end open.
   The trailer origin is located on the trailer floor (height-wise), at the rear gate (length-wise), and on the
   center-line of the truck (width-wise).
   */
  std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> CreateTrailerConfigs(
      drake::RandomGenerator* random_generator) {
    std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> bodies;

    const std::string name("trailer");

    drake::Vector3<double> size = EvalAttribute(AttributeType::kTrailerSize, random_generator);
    drake::math::RigidTransform<double> trailer_pose = drake::math::RigidTransform<double>::Identity();
    trailer_pose.set_translation(EvalAttribute(AttributeType::kTrailerPoseTranslation, random_generator));

    drake::Vector2<double> cf_coefs = EvalAttribute(AttributeType::kTrailerCoulombFriction, random_generator);
    drake::multibody::CoulombFriction<double> coulomb_friction(cf_coefs[0], cf_coefs[1]);

    const double& length = size[0];
    const double& width = size[1];
    const double& height = size[2];

    // Dimensions of the boxes that will be the walls of the trailer.
    const drake::Vector3<double> side_wall_dimensions{
        length,
        kTrailerWallThickness,
        height,
    };
    const drake::Vector3<double> end_wall_dimensions{
        kTrailerWallThickness,
        width + 2.0 * kTrailerWallThickness,
        height + 2.0 * kTrailerWallThickness,
    };
    const drake::Vector3<double> ceiling_floor_dimensions{
        length,
        width + 2.0 * kTrailerWallThickness,
        kTrailerWallThickness,
    };

    bodies.insert(SingleBodyInstanceConfig::CreateStaticBoxConfig(
        name + "_floor", ceiling_floor_dimensions,
        trailer_pose * drake::math::RigidTransform<double>(
                           drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                           drake::Vector3<double>{length * 0.5, 0.0, -0.5 * kTrailerWallThickness}),
        coulomb_friction, kTrailerColorRGBA));

    bodies.insert(SingleBodyInstanceConfig::CreateStaticBoxConfig(
        name + "_ceiling", ceiling_floor_dimensions,
        trailer_pose * drake::math::RigidTransform<double>(
                           drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                           drake::Vector3<double>{length * 0.5, 0.0, height + 0.5 * kTrailerWallThickness}),
        coulomb_friction, kTrailerColorRGBA));

    bodies.insert(SingleBodyInstanceConfig::CreateStaticBoxConfig(
        name + "_wall_left", side_wall_dimensions,
        trailer_pose *
            drake::math::RigidTransform<double>(
                drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                drake::Vector3<double>{length * 0.5, width * 0.5 + 0.5 * kTrailerWallThickness, height * 0.5}),
        coulomb_friction, kTrailerColorRGBA));

    bodies.insert(SingleBodyInstanceConfig::CreateStaticBoxConfig(
        name + "_wall_right", side_wall_dimensions,
        trailer_pose *
            drake::math::RigidTransform<double>(
                drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                drake::Vector3<double>{length * 0.5, -width * 0.5 - 0.5 * kTrailerWallThickness, height * 0.5}),
        coulomb_friction, kTrailerColorRGBA));

    bodies.insert(SingleBodyInstanceConfig::CreateStaticBoxConfig(
        name + "_wall_front", end_wall_dimensions,
        trailer_pose * drake::math::RigidTransform<double>(
                           drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                           drake::Vector3<double>{length + 0.5 * kTrailerWallThickness, 0.0, height * 0.5}),
        coulomb_friction, kTrailerColorRGBA));

    {  // Truck
      auto body = std::make_unique<SingleBodyInstanceConfig>();
      body->set_name(name + "_truck_vehicle");
      body->set_pose(trailer_pose * drake::math::RigidTransform<double>(
                                        drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                        drake::Vector3<double>(0.0, 0.0, kPavementHeight)));
      std::string absolute_model_path = GetModelDirectoryFromEnvironment();
      body->SetMeshGeometry(absolute_model_path + "/objects/truck.obj", 1.0);
      bodies.insert(std::move(body));
    }
    return bodies;
  }

  std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> CreateConveyorConfigs(
      drake::RandomGenerator* random_generator) {
    std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> bodies;

    const std::string name("conveyor");

    drake::math::RigidTransform<double> conveyor_pose = drake::math::RigidTransform<double>::Identity();
    conveyor_pose.set_translation(EvalAttribute(AttributeType::kConveyorPoseTranslation, random_generator));

    const double conveyor_extension = EvalAttribute(AttributeType::kConveyorExtension, random_generator)[0];

    const double conveyor_width = EvalAttribute(AttributeType::kConveyorBeltWidth, random_generator)[0];

    drake::math::RigidTransform<double> robot_mount_pose = drake::math::RigidTransform<double>::Identity();
    robot_mount_pose.set_translation(EvalAttribute(AttributeType::kRobotMountPoseTranslation, random_generator));

    drake::Vector2<double> cf_coefs = EvalAttribute(AttributeType::kConveyorCoulombFriction, random_generator);
    drake::multibody::CoulombFriction<double> coulomb_friction(cf_coefs[0], cf_coefs[1]);

    // Conveyor length is amount extended into trailer + token length.
    const double conveyor_length = kConveyorDockGap + conveyor_extension;

    drake::Vector3<double> robot_mount_dims{0.05, 0.05, 0.05};
    drake::Vector3<double> conveyor_dims{conveyor_length, conveyor_width, kConveyorThickness};

    bodies.insert(SingleBodyInstanceConfig::CreateStaticBoxConfig(
        name + "_belt", conveyor_dims,
        conveyor_pose *
            drake::math::RigidTransform<double>(
                drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                drake::Vector3<double>{0.5 * conveyor_length - kConveyorDockGap, 0.0, -kConveyorThickness * 0.5}),
        coulomb_friction, kConveyorColorRGBA));

    bodies.insert(SingleBodyInstanceConfig::CreateStaticBoxConfig(
        name + "_robot_mount", robot_mount_dims,
        conveyor_pose *
            drake::math::RigidTransform<double>(drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                                drake::Vector3<double>{conveyor_extension, 0.0, 0.0}) *
            robot_mount_pose,
        coulomb_friction, kConveyorColorRGBA));

    return bodies;
  }

  // Creates configurations for the static loading dock floor and other building elements.
  std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> CreateBuildingConfigs() {
    std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> bodies;

    const std::string name("loading_dock");
    const double kBuildingConveyorSize = 0.5;
    drake::Vector4<double> kConveyorRegionColorRGBA{0.25, 0.25, 0.25, 1.0};

    bodies.insert(SingleBodyInstanceConfig::CreateStaticBoxConfig(
        name + "_small_belt", drake::Vector3<double>{kDockConveyorLength, 1.0, kBuildingConveyorSize},
        drake::math::RigidTransform<double>(
            drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
            drake::Vector3<double>{-0.5 * kDockConveyorLength - kConveyorDockGap, 0.0, 0.5 * kBuildingConveyorSize}),
        {}, kConveyorRegionColorRGBA));

    bodies.insert(SingleBodyInstanceConfig::CreateStaticBoxConfig(
        name + "_big_belt", drake::Vector3<double>{kDockPlatformSize, 2.0, kBuildingConveyorSize},
        drake::math::RigidTransform<double>(
            drake::math::RollPitchYaw<double>(0.0, 0.0, M_PI_2).ToRotationMatrix(),
            drake::Vector3<double>{-kDockConveyorLength - kConveyorDockGap, 0.0, 0.5 * kBuildingConveyorSize}),
        {}, kConveyorRegionColorRGBA));

    bodies.emplace(SingleBodyInstanceConfig::CreateStaticBoxConfig(
        name + "_floor", drake::Vector3<double>{kDockPlatformSize, kDockPlatformSize, kDockPlatformThickness},
        drake::math::RigidTransform<double>(
            drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
            drake::Vector3<double>(-0.5 * kDockPlatformSize, 0.0, -0.5 * kDockPlatformThickness)),
        {}, drake::Vector4<double>{0.72, 0.70, 0.67, 1.0} /* concrete gray */));

    return bodies;
  }

  std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> CreateLoadingDockConfigs(
      drake::RandomGenerator* random_generator) {
    std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> body_instance_configs;

    body_instance_configs.merge(CreateTrailerConfigs(random_generator));
    body_instance_configs.merge(CreateConveyorConfigs(random_generator));
    body_instance_configs.merge(CreateBuildingConfigs());
    return body_instance_configs;
  }

  void set_attribute_range(AttributeType attribute_type, const drake::VectorX<double>& lower_bound,
                           const drake::VectorX<double>& upper_bound) {
    drake::log()->debug("{} has range=[[{}] to [{}]] interval_size=[{}]", attribute_type, lower_bound.transpose(),
                        upper_bound.transpose(), (upper_bound - lower_bound).transpose());
    attribute_distribution_.at(attribute_type)
        .param(UniformVectorDistribution<double>::param_type(lower_bound, upper_bound));
  }

  UniformVectorDistribution<double>::param_type attribute_param(AttributeType attribute_type) {
    return attribute_distribution_.at(attribute_type).param();
  }

  // The following are variables of the loading dock that are either not sampled or not important to the task.
  // Color of trailer and conveyor visual geometry.
  drake::Vector4<double> kConveyorColorRGBA{0.25, 0.25, 0.25, 1.0};
  drake::Vector4<double> kTrailerColorRGBA{1.0, 1.0, 1.0, 0.1};
  // Dock floor size.
  const double kDockPlatformThickness = 0.1;
  const double kDockPlatformSize = 20.0;
  // Size of the conveyors in the loading dock.
  const double kConveyorThickness = 0.1;
  const double kDockConveyorLength = 3.0;
  // The space between the end of the fully retracted conveyor and the edge of the loading dock.
  const double kConveyorDockGap = 2.0;
  // Pavement height is the height where the trailer truck model's wheels meet the ground.
  const double kPavementHeight = -1.95;
  const double kTrailerWallThickness = 0.05;

 private:
  drake::VectorX<double> EvalAttribute(AttributeType attribute_type, drake::RandomGenerator* random_generator) {
    return attribute_distribution_.at(attribute_type)(*random_generator);
  }

  const std::string model_directory_{""};

  // Attribute type -> Distribution
  std::map<AttributeType, UniformVectorDistribution<double>> attribute_distribution_{
      {AttributeType::kTrailerPoseTranslation, UniformVectorDistribution<double>(3)},
      {AttributeType::kConveyorPoseTranslation, UniformVectorDistribution<double>(3)},
      {AttributeType::kRobotMountPoseTranslation, UniformVectorDistribution<double>(3)},
      {AttributeType::kTrailerSize, UniformVectorDistribution<double>(3)},
      {AttributeType::kTrailerCoulombFriction, UniformVectorDistribution<double>(2)},
      {AttributeType::kConveyorCoulombFriction, UniformVectorDistribution<double>(2)},
      {AttributeType::kConveyorExtension, UniformVectorDistribution<double>(1)},
      {AttributeType::kConveyorBeltWidth, UniformVectorDistribution<double>(1)},
  };
};

std::ostream& operator<<(std::ostream& os, LoadingDockGenerator::AttributeType attribute_type) {
  switch (attribute_type) {
    case LoadingDockGenerator::AttributeType::kTrailerPoseTranslation:
      os << "kTrailerPoseTranslation";
      break;
    case LoadingDockGenerator::AttributeType::kConveyorPoseTranslation:
      os << "kConveyorPoseTranslation";
      break;
    case LoadingDockGenerator::AttributeType::kRobotMountPoseTranslation:
      os << "kRobotMountPoseTranslation";
      break;
    case LoadingDockGenerator::AttributeType::kTrailerSize:
      os << "kTrailerSize";
      break;
    case LoadingDockGenerator::AttributeType::kTrailerCoulombFriction:
      os << "kTrailerCoulombFriction";
      break;
    case LoadingDockGenerator::AttributeType::kConveyorCoulombFriction:
      os << "kConveyorCoulombFriction";
      break;
    case LoadingDockGenerator::AttributeType::kConveyorExtension:
      os << "kConveyorExtension";
      break;
    case LoadingDockGenerator::AttributeType::kConveyorBeltWidth:
      os << "kConveyorBeltWidth";
      break;
    default:
      break;
  }
  return os;
}

}  // namespace DR
