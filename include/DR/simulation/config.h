/*
This file provides data storage and validation classes that help DR configure models in the Drake, they:
-- store model information so that functions in 'model_generator.h' can populate a multibody plant with models.
-- store robot, joint, actuation, and controller information to tools in 'controller_generator.h' that connect the
   diagrams controlling those models.
-- store state information for those models so they can be set by tools in 'model_generator.h' and 'state_setter.h'.
*/

#pragma once

#include <experimental/filesystem>  // requires linking to stdc++fs
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <drake/common/text_logging.h>
#include <drake/geometry/shape_specification.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/math/spatial_velocity.h>
#include <drake/multibody/plant/coulomb_friction.h>
#include <drake/multibody/shapes/geometry.h>

#include <DR/common/environment.h>
#include <DR/common/exception.h>
#include <DR/tools/output_stream_operators.h>

namespace DR {

// Since these are only default parameters, they will never be sampled for Monte Carlo simulation.
// Theses default values are meant as sane stand-ins when uncertain about a value.
// If a parameter is measured or important then it should be set explicitly--- even if it matches these defaults.
const drake::multibody::CoulombFriction<double> kDefaultCoulombFriction(0.8, 0.6);
const drake::Vector4<double> kTransparentWhite{1.0, 1.0, 1.0, 0.05};
const drake::Vector4<double> kOpaqueWhite{1.0, 1.0, 1.0, 1.0};

// This is the gravity vector applied to all multibody plants created by 'model_generator.h'.
// the default value is usually best.
drake::Vector3<double> kDefaultGravityVector{0.0, 0.0, -9.8};

/**
 The base interface for all configuration specifications.
 */
class ConfigBase {
 public:
  ConfigBase(const ConfigBase&) = default;
  ConfigBase& operator=(const ConfigBase&) = default;
  ConfigBase(ConfigBase&&) = default;
  ConfigBase& operator=(ConfigBase&&) = default;
  ConfigBase() = default;
  virtual ~ConfigBase() = default;

  // TODO(samzapo): Write a parser and exporter to YAML/JSON for derived classes.

  // TODO(samzapo): Add virtual serialize & deserialize functions for derived classes.

  const std::string& name() const { return name_; }

  void set_name(const std::string& name) {
    DR_DEMAND(!name.empty());
    name_ = name;
  }

 private:
  // A unique name for the configuration of the configured object.
  std::string name_{"NOT_SET"};
};

/**
 The interface for all configuration of models.
 Stores general information about the state of the floating or fixed base of the model.
 */
class ModelInstanceConfig : public ConfigBase {
 public:
  ModelInstanceConfig(const ModelInstanceConfig&) = default;
  ModelInstanceConfig& operator=(const ModelInstanceConfig&) = default;
  ModelInstanceConfig(ModelInstanceConfig&&) = default;
  ModelInstanceConfig& operator=(ModelInstanceConfig&&) = default;
  ModelInstanceConfig() = default;
  virtual ~ModelInstanceConfig() = default;

  const drake::math::RigidTransform<double>& pose() const { return pose_; }

  void set_pose(const drake::math::RigidTransform<double>& pose) { pose_ = pose; }

  const drake::multibody::SpatialVelocity<double>& spatial_velocity() const { return spatial_velocity_; }

  void set_spatial_velocity(const drake::multibody::SpatialVelocity<double>& spatial_velocity) {
    DR_DEMAND(std::isfinite(spatial_velocity.get_coeffs().norm()));
    spatial_velocity_ = spatial_velocity;
  }

 private:
  // The current (floating==true) or permanent (floating==false) location of
  // this object.
  // World Frame:  +z "Up" (from floor toward ceiling of trailer)
  //               |  / +x "Forward" (from floor center toward front of trailer)
  //               | /
  //         ------  +y "Left" (from center toward the left wall of the trailer)
  drake::math::RigidTransform<double> pose_ = drake::math::RigidTransform<double>::Identity();

  drake::multibody::SpatialVelocity<double> spatial_velocity_ = drake::multibody::SpatialVelocity<double>::Zero();
};

/**
 SingleBodies include all dynamic and static (fixed to the world) bodies.
 */
class SingleBodyInstanceConfig final : public ModelInstanceConfig {
 public:
  SingleBodyInstanceConfig(const SingleBodyInstanceConfig&) = default;
  SingleBodyInstanceConfig& operator=(const SingleBodyInstanceConfig&) = default;
  SingleBodyInstanceConfig(SingleBodyInstanceConfig&&) = default;
  SingleBodyInstanceConfig& operator=(SingleBodyInstanceConfig&&) = default;
  SingleBodyInstanceConfig() {}
  virtual ~SingleBodyInstanceConfig() = default;

  double mass() const { return mass_; }

  /// To make static, set mass to 0.0.  Otherwise, dynamic bodies have positive mass.
  void set_mass_and_possibly_make_static(double mass) {
    DR_DEMAND(0.0 <= mass && std::isfinite(mass));
    mass_ = mass;
  }

  bool is_static() const { return (mass() == 0.0); }

  const drake::multibody::CoulombFriction<double>& coulomb_friction() const { return coulomb_friction_; }

  void SetCoulombFriction(double static_friction, double dynamic_friction) {
    set_coulomb_friction(drake::multibody::CoulombFriction<double>(static_friction, dynamic_friction));
  }

  void set_coulomb_friction(const drake::multibody::CoulombFriction<double>& coulomb_friction) {
    coulomb_friction_ = coulomb_friction;
  }

  const drake::Vector4<double>& color() const { return color_of_visual_geometry_; }

  void set_color(double red, double green, double blue, double alpha) {
    set_color(drake::Vector4<double>(red, green, blue, alpha));
  }

  void set_color(const drake::Vector4<double>& color) {
    DR_DEMAND(0.0 <= color.minCoeff() && color.maxCoeff() <= 1.0);
    color_of_visual_geometry_ = color;
  }

  void SetBoxGeometry(double length, double width, double height) {
    DR_DEMAND(0.0 < width && std::isfinite(width));
    DR_DEMAND(0.0 < height && std::isfinite(height));
    DR_DEMAND(0.0 < length && std::isfinite(length));
    this->set_visual_geometry(drake::geometry::Box(length, width, height));
    this->set_collision_geometry(drake::geometry::Box(length, width, height));
  }

  void SetSphereGeometry(double radius) {
    DR_DEMAND(0.0 < radius && std::isfinite(radius));
    this->set_visual_geometry(drake::geometry::Sphere(radius));
    this->set_collision_geometry(drake::geometry::Sphere(radius));
  }

  void SetCylinderGeometry(double radius, double length) {
    DR_DEMAND(0.0 < radius && std::isfinite(radius));
    DR_DEMAND(0.0 < length && std::isfinite(length));
    this->set_visual_geometry(drake::geometry::Cylinder(radius, length));
    this->set_collision_geometry(drake::geometry::Cylinder(radius, length));
  }

  void SetMeshGeometry(const std::string& absolute_filename, double scale = 1.0) {
    DR_DEMAND(0.0 < scale && std::isfinite(scale));
    std::experimental::filesystem::path path(absolute_filename);
    DR_DEMAND(exists(path) && is_regular_file(path));
    this->set_visual_geometry(drake::geometry::Mesh(absolute_filename, scale));
    // TODO(samzapo) support mesh collision geometries.
  }

  void SetHalfSpaceGeometry() {
    this->set_visual_geometry(drake::geometry::HalfSpace());
    this->set_collision_geometry(drake::geometry::HalfSpace());
  }

  template <typename T>
  void set_visual_geometry(const T& shape) {
    static_assert(std::is_base_of<drake::geometry::Shape, T>::value, "T must derive from drake::geometry::Shape");
    visual_geometry_ = std::make_shared<T>(shape);
  }

  const drake::geometry::Shape& visual_geometry() const {
    DR_DEMAND(this->has_visual_geometry());
    return *visual_geometry_.get();
  }

  template <typename T>
  void set_collision_geometry(const T& shape) {
    static_assert(std::is_base_of<drake::geometry::Shape, T>::value, "T must derive from drake::geometry::Shape");
    collision_geometry_ = std::make_shared<T>(shape);
  }

  const drake::geometry::Shape& collision_geometry() const {
    DR_DEMAND(this->has_collision_geometry());
    return *collision_geometry_.get();
  }

  bool has_collision_geometry() const { return (collision_geometry_.get() != nullptr); }

  bool has_visual_geometry() const { return (visual_geometry_.get() != nullptr); }

  /**
  Creates a configuration for a static box.
  @param name Sets the name of the model instance (must be unique).
  @param size the dimensions of the box.
  @param pose Sets the pose of the box.
  @param coulomb_friction (optional) sets the static and dynamic Coulomb friction coefficients of the box.
  @param color (optional) RGBA color of the box's visual geometry.
  */
  static std::unique_ptr<SingleBodyInstanceConfig> CreateStaticBoxConfig(
      const std::string& name, const drake::Vector3<double>& size, const drake::math::RigidTransform<double>& pose,
      const std::optional<drake::multibody::CoulombFriction<double>>& coulomb_friction,
      const std::optional<drake::Vector4<double>>& color) {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name(name);
    body->set_mass_and_possibly_make_static(0.0 /* static body */);
    body->SetBoxGeometry(size[0], size[1], size[2]);
    body->set_pose(pose);

    if (coulomb_friction.has_value()) body->set_coulomb_friction(coulomb_friction.value());
    if (color.has_value()) body->set_color(color.value());

    return body;
  }

 private:
  // see: drake/geometry/shape_specification.h for details
  std::shared_ptr<drake::geometry::Shape> visual_geometry_;
  std::shared_ptr<drake::geometry::Shape> collision_geometry_;

  // The mass of the manipuland (only for "floating" objects).
  // Typically represents the weight of the manipuland in kilograms.
  // The default (0.0) indicates that the body is to be treated as static.
  double mass_{0.0};

  // color {red, green, blue, alpha} in interval [0,1].
  drake::Vector4<double> color_of_visual_geometry_ = {0.5, 0.5, 0.5, 1.0};

  // The frictional properties of the object.
  // These are the friction properties for this specific object;
  // they'll be combined with friction properties for another object when
  // frictional forces need to be calculated (e.g., when the robot or another
  // object, or the environment collides with this object).
  drake::multibody::CoulombFriction<double> coulomb_friction_ = kDefaultCoulombFriction;
};

/**
 Configuration class for a robot's actuated joints each configuration is meant to describe a unique joint on a
 robot, but not required to be unique in the simulated world.
 JointInstanceConfig stores information describing a robot's joints. This
 configuration will be used by the ModelGenerator and StateSetter to build and initialize the robot model and used by
 ControllerGenerator to configure the robot control system.  During simulation, this config can be used by controllers
 and planners as a reference for joint names, types, and other relevant parameters.

  NOTE: This class should only be used for serialization.

  TODO(samzapo): Write a parser and exporter to YAML/JSON for this class.
 */
class JointInstanceConfig final : public ConfigBase {
 public:
  enum class JointType {
    kUnknownJointType,
    kRevoluteJointType,
    kPrismaticJointType,
  };

  JointInstanceConfig(const JointInstanceConfig&) = default;
  JointInstanceConfig& operator=(const JointInstanceConfig&) = default;
  JointInstanceConfig(JointInstanceConfig&&) = default;
  JointInstanceConfig& operator=(JointInstanceConfig&&) = default;
  virtual ~JointInstanceConfig() = default;

  explicit JointInstanceConfig(JointType type = JointType::kUnknownJointType) : type_(type) {
    DR_DEMAND(type != JointType::kUnknownJointType);
  }

  JointInstanceConfig(const std::string& name, JointType type, std::optional<double> position,
                      std::optional<double> velocity, std::optional<double> kp)
      : JointInstanceConfig(type) {
    this->set_name(name);
    if (position.has_value()) this->set_position(position.value());
    if (velocity.has_value()) this->set_velocity(velocity.value());
    if (kp.has_value()) this->set_kp(kp.value());
  }

  int dofs() const {
    switch (type()) {
      case JointType::kRevoluteJointType:
        return 1;
      case JointType::kPrismaticJointType:
        return 1;
      default:
        throw std::logic_error("Requested the dofs of an unknown joint type!");
        break;
    }
    DR_UNREACHABLE();
  }

  JointType type() const { return type_; }

  double kp() const { return kp_; }

  void set_kp(double kp) {
    DR_DEMAND(0.0 < kp && std::isfinite(kp));
    kp_ = kp;
  }

  double position() const { return position_; }

  void set_position(double position) {
    DR_DEMAND(std::isfinite(position_));
    position_ = position;
  }

  double velocity() const { return velocity_; }

  void set_velocity(double velocity) {
    DR_DEMAND(std::isfinite(velocity));
    velocity_ = velocity;
  }

 private:
  JointType type_;

  double kp_{0.0};
  double position_{0.0};
  double velocity_{0.0};
};

/**
 Configuration class for an instance of a robot, each configuration is meant to describe a unique robot in the
 simulated world.

 RobotInstanceConfig stores information describing a controlled multibody or "robot". This configuration will be used
 by the ModelGenerator and StateSetter to build and initialize the robot model and used by ControllerGenerator to
 configure the robot control system.  During simulation, this config can be used by controllers and planners as a
 reference for the robot's name, joint configurations, and the path to its model file to generate additional,
 controller-owned robot models.
 */
class RobotInstanceConfig final : public ModelInstanceConfig {
 public:
  RobotInstanceConfig(const RobotInstanceConfig&) = default;
  RobotInstanceConfig& operator=(const RobotInstanceConfig&) = default;
  RobotInstanceConfig(RobotInstanceConfig&&) = default;
  RobotInstanceConfig& operator=(RobotInstanceConfig&&) = default;
  virtual ~RobotInstanceConfig() = default;

  explicit RobotInstanceConfig(const std::string& model_file_path = "NOT_SET") : model_file_path_(model_file_path) {
    std::experimental::filesystem::path path(model_file_path_);
    DR_DEMAND(exists(path) && is_regular_file(path));
  }

  /**
   ControlScheme describes what type of controllers to configure for this robot instance.
    NOTE: When a target control-system architecture is better known, this enum will be replaced with a ControllerConfig
    class.
   */
  enum class ControlScheme {
    // The robot is not controlled, all joints are set to zero torque.
    kPassiveControlScheme,
    // The robot is controlled and configured to hold its initial position.
    kStationaryControlScheme,
  };

  RobotInstanceConfig(const std::string& robot_name, const std::string& model_file_path,
                      const std::vector<JointInstanceConfig>& joint_configs)
      : RobotInstanceConfig(model_file_path) {
    this->set_name(robot_name);
    this->set_joint_instance_configs(joint_configs);
  }

  bool is_floating() const { return is_floating_; }

  void set_is_floating(bool is_floating) { is_floating_ = is_floating; }

  int num_joints() const { return joint_instance_configs_.size(); }

  const std::string& model_file_path() const { return model_file_path_; }

  const std::vector<JointInstanceConfig>& joint_instance_configs() const { return joint_instance_configs_; }

  std::vector<JointInstanceConfig>& mutable_joint_instance_configs() { return joint_instance_configs_; }

  void set_joint_instance_configs(const std::vector<JointInstanceConfig>& joint_instance_configs) {
    joint_instance_configs_ = joint_instance_configs;
  }

  ControlScheme control_scheme() const { return control_scheme_; }

  void set_control_scheme(ControlScheme control_scheme) { control_scheme_ = control_scheme; }

 private:
  // Path to SDF.
  std::string model_file_path_;

  // Determines whether the root link of the robot is affixed to the environment.
  //  true: floating base, NOT affixed to the environment
  //  false: fixed base, affixed to the environment
  bool is_floating_{false};

  // Joint configuration.
  std::vector<JointInstanceConfig> joint_instance_configs_;

  // Control Scheme for robot.
  // TODO(samzapo) replace with more detailed control configuration.
  ControlScheme control_scheme_{ControlScheme::kPassiveControlScheme};
};

std::ostream& operator<<(std::ostream& os, const ConfigBase& config) {
  os << "ConfigBase{ ";
  os << "name_ = " << config.name() << ", ";
  os << "};";
  return os;
}

std::ostream& operator<<(std::ostream& os, const ModelInstanceConfig& config) {
  os << "ModelInstanceConfig{ ";
  os << static_cast<ConfigBase>(config) << ", ";
  os << "pose_ = " << config.pose() << ", ";
  os << "spatial_velocity_ = " << config.spatial_velocity() << ", ";
  os << "};";
  return os;
}

std::ostream& operator<<(std::ostream& os, const SingleBodyInstanceConfig& config) {
  os << "config = SingleBodyInstanceConfig{ ";
  os << static_cast<ModelInstanceConfig>(config) << ", ";
  os << "mass_ = " << config.mass() << ", ";
  os << "is_static: " << config.is_static() << ", ";
  os << "has_collision_geometry = " << config.has_collision_geometry() << ", ";
  os << "has_visual_geometry = " << config.has_visual_geometry() << ", ";
  os << "color_of_visual_geometry_ = " << config.color().transpose() << ", ";
  os << "coulomb_friction_.static = " << config.coulomb_friction().static_friction() << ", ";
  os << "coulomb_friction_.dynamic = " << config.coulomb_friction().dynamic_friction() << ", ";
  os << " };";
  return os;
}

}  // namespace DR
