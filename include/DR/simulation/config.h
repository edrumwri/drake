#pragma once
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <drake/geometry/shape_specification.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/math/spatial_velocity.h>
#include <drake/multibody/plant/coulomb_friction.h>
#include <drake/multibody/shapes/geometry.h>

#include <DR/common/exception.h>

namespace DR {

/*
 NOTE: Configuration files are not templatized because they do not store
 parameters that vary with respect to the scalar type of a
 drake::multibody::MultibodyPlant<T>.  Example:
 ```
 template <typename T>
 geometry::GeometryId MultibodyPlant<T>::RegisterVisualGeometry(
  const Body<T>& body, const math::RigidTransform<double>& X_BG,
  const geometry::Shape& shape, const std::string& name,
  const Vector4<double>& diffuse_color,
  SceneGraph<T>* scene_graph)
 ```
 the Pose `math::RigidTransform<double>` remains a double despite the
 templated scalar type of MultibodyPlant.
 */

/*
 The base interface for all configuration specifications.  It requires that
 all derived classes to implement a ValidateConfig function that will check
 the member variables of the config file for validity.
 */
class ConfigBase {
 public:
  virtual ~ConfigBase() {}

  /**
   This function much be implemented by all derived classes of ConfigBase
   class. This function should throw an error if one or more of the member
   variables is not set correctly (e.g., out of valid range, not set, NaN).
   */
  virtual void ValidateConfig() const = 0;
};

/**
 This class stores simulation-specific parameters.
 The member variables of SimulatorInstanceConfig affect the performance of
 physical simulation of the universal plant.

 NOTE: Some of these simulation parameters are used even if a MultibodyPlant
 will not be simulated (e.g., step_size is used to configure a
 MultibodyPlant).

 NOTE: This class should only be used for serialization.

  TODO(samzapo): Write a parser and exporter to YAML/JSON for this class.
 */
class SimulatorInstanceConfig final : public ConfigBase {
 public:
  /// Enum types for selecting the simulation integration scheme.
  enum IntegrationScheme {
    // The integration scheme was not set.
    kUnknownIntegrationScheme = 0,
    // A first-order, semi-explicit Euler integrator.
    kSemiExplicitEulerIntegrationScheme = 1,
    // A second-order, explicit Runge Kutta integrator.
    kRK2IntegrationScheme = 2,
    // A third-order Runge Kutta integrator with a third order error estimate.
    kRK3IntegrationScheme = 3,
    // A first-order, fully implicit integrator with second order error estimation.
    kImplicitEulerIntegrationScheme = 4,
  };

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SimulatorInstanceConfig)

  SimulatorInstanceConfig() {}
  virtual ~SimulatorInstanceConfig() {}
  void ValidateConfig() const final {
    DR_DEMAND(target_accuracy_ > 0.0);
    DR_DEMAND(target_realtime_rate_ > 0.0);
    DR_DEMAND(simulation_time_ > 0.0);
    DR_DEMAND(integration_scheme_ != kUnknownIntegrationScheme);
  }

  void set_target_accuracy(double target_accuracy) { target_accuracy_ = target_accuracy; }

  void set_target_realtime_rate(double target_realtime_rate) { target_realtime_rate_ = target_realtime_rate; }

  void set_simulation_time(double simulation_time) { simulation_time_ = simulation_time; }

  void set_step_size(double step_size) { step_size_ = step_size; }

  void set_integration_scheme(const std::string& integration_scheme) {
    if (integration_scheme.compare("semi_explicit_euler") == 0) {
      integration_scheme_ = kSemiExplicitEulerIntegrationScheme;
    } else if (integration_scheme.compare("runge_kutta2") == 0) {
      integration_scheme_ = kRK2IntegrationScheme;
    } else if (integration_scheme.compare("runge_kutta3") == 0) {
      integration_scheme_ = kRK3IntegrationScheme;
    } else if (integration_scheme.compare("implicit_euler") == 0) {
      integration_scheme_ = kImplicitEulerIntegrationScheme;
    } else {
      throw std::runtime_error(
          "Integration scheme was an invalid value. Expected: "
          "{'semi_explicit_euler','runge_kutta2','runge_"
          "kutta3','implicit_euler'}, was: '" +
          integration_scheme + "'");
    }
  }

  void set_integration_scheme(IntegrationScheme integration_scheme) { integration_scheme_ = integration_scheme; }

  double target_accuracy() const { return target_accuracy_; }

  double target_realtime_rate() const { return target_realtime_rate_; }

  double simulation_time() const { return simulation_time_; }

  double step_size() const { return step_size_; }

  IntegrationScheme integration_scheme() const { return integration_scheme_; }

 private:
  // The target accuracy determines the size of the actual time steps taken
  // whenever a variable time step integrator is used.
  double target_accuracy_{0.001};
  // Desired rate relative to real time.  See documentation for
  // drake::systems::Simulator::set_target_realtime_rate() for details.
  double target_realtime_rate_{1.0};
  // Desired duration of the simulation in seconds.
  double simulation_time_{10.0};
  // If greater than zero, the plant is modeled as a system with
  // discrete updates and period equal to this time_step.
  // If 0, the plant is modeled as a continuous system.
  double step_size_{0.001};
  // Integration scheme to be used, see enum IntegrationScheme above.
  // Available options are:
  // 'semi_explicit_euler','runge_kutta2','runge_kutta3','implicit_euler'
  IntegrationScheme integration_scheme_{kUnknownIntegrationScheme};
};

/**
 Bodies include all dynamic and static (fixed to the world) bodies that the
 robot can touch.
 NOTE: This group excludes the environment (floor, trailer of truck, and the
 robot model) but includes static objects.

 NOTE: This class should only be used for serialization.

  TODO(samzapo): Write a parser and exporter to YAML/JSON for this class.
 */
class BodyInstanceConfig final : public ConfigBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BodyInstanceConfig)

  BodyInstanceConfig() {}
  virtual ~BodyInstanceConfig() {}
  void ValidateConfig() const final {
    DR_DEMAND(geometry_.get());
    DR_DEMAND(!name_.empty());
    if (is_floating()) {
      DR_DEMAND(mass_ > 0);
    }

    DR_DEMAND(std::isfinite(pose_.GetMaximumAbsoluteDifference(drake::math::RigidTransform<double>::Identity())));
  }

  void set_name(const std::string& name) { name_ = name; }

  void set_is_floating(bool is_floating) { is_floating_ = is_floating; }

  void set_pose(const drake::math::RigidTransform<double>& pose) { pose_ = pose; }

  void SetBoxGeometry(double length, double width, double height) {
    geometry_ = std::make_shared<drake::geometry::Box>(length, width, height);
  }

  void SetSphereGeometry(double radius) { geometry_ = std::make_shared<drake::geometry::Sphere>(radius); }

  void SetCylinderGeometry(double radius, double length) {
    geometry_ = std::make_shared<drake::geometry::Cylinder>(radius, length);
  }

  void set_mass(double mass) { mass_ = mass; }

  void SetCoulombFriction(double static_friction, double dynamic_friction) {
    coulomb_friction_ = drake::multibody::CoulombFriction<double>(static_friction, dynamic_friction);
  }

  void set_spatial_velocity(const drake::multibody::SpatialVelocity<double>& spatial_velocity) {
    spatial_velocity_ = spatial_velocity;
  }

  const std::string& name() const { return name_; }

  bool is_floating() const { return is_floating_; }

  const drake::math::RigidTransform<double>& pose() const { return pose_; }

  const drake::geometry::Shape& geometry() const { return *geometry_.get(); }

  double mass() const { return mass_; }

  const drake::multibody::CoulombFriction<double>& coulomb_friction() const { return coulomb_friction_; }

  const drake::multibody::SpatialVelocity<double>& spatial_velocity() const { return spatial_velocity_; }

 protected:
  // A unique name for this manipuland.
  std::string name_{""};

  // see: drake/geometry/shape_specification.h for details
  // implemented:  { Box, Sphere }
  std::shared_ptr<drake::geometry::Shape> geometry_;

  // Determines whether the object is considered static or dynamic.
  //  true: object is dynamic&  NOT fixed to the environment
  //  false: object is static&  fixed to the environment
  bool is_floating_{true};

  // The current (floating==true) or permanent (floating==false) location of
  // this object.
  // World Frame:  +z "Up" (from floor toward ceiling of trailer)
  //               |  / +x "Forward" (from floor center toward front of trailer)
  //               | /
  //         ------  +y "Left" (from center toward the left wall of the trailer)
  drake::math::RigidTransform<double> pose_ = drake::math::RigidTransform<double>::Identity();

  drake::multibody::SpatialVelocity<double> spatial_velocity_ = drake::multibody::SpatialVelocity<double>::Zero();

  // The mass of the manipuland (only for "floating" objects).
  // Typically represents the weight of the manipuland in kilograms
  double mass_{0.0};

  // The frictional properties of the object.
  // These are the friction properties for this specific object;
  // they'll be combined with friction properties for another object when
  // frictional forces need to be calculated (e.g., when the robot or another
  // object, or the environment collides with this object).
  drake::multibody::CoulombFriction<double> coulomb_friction_;
};

/**
 The static floor or trailer is referred to as the "environment".
 There is only ever one environment because it is composed of open
 geometries (e.g. HalfSpace).

 NOTE: This class should only be used for serialization.

  TODO(samzapo): Write a parser and exporter to YAML/JSON for this class.
*/

class EnvironmentInstanceConfig final : public ConfigBase {
 public:
  enum EnvironmentType {
    // The environment type was not set.
    kUnknownEnvironmentType = 0,
    // 'Trailer' is the shape of an open back of a trailer truck represented by 5 HalfSpaces.  The origin is at the
    // center of the trailer floor with +X pointing into the truck,  +Y pointing to the left, and +Z as up.
    kTrailerEnvironmentType = 1,
    // 'Floor' is an infinite horizontal plane represented by a HalfSpace at the origin with a +Z normal (up).
    kFloorEnvironmentType = 2,
  };

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EnvironmentInstanceConfig)

  EnvironmentInstanceConfig() {}
  virtual ~EnvironmentInstanceConfig() {}
  void ValidateConfig() const final {
    DR_DEMAND(std::isfinite(gravity_.norm()));
    DR_DEMAND(std::isfinite(trailer_size_.norm()));
    DR_DEMAND(trailer_size_.minCoeff() > 0.0);
    DR_DEMAND(type_ != kUnknownEnvironmentType);

    for (const auto& env_body : body_instance_configs()) {
      env_body.ValidateConfig();
      DR_DEMAND(!env_body.is_floating());
    }
  }

  void set_gravity(double x, double y, double z) { gravity_ = drake::Vector3<double>(x, y, z); }

  void set_floor_environment() { type_ = kFloorEnvironmentType; }

  void set_trailer_environment() { type_ = kTrailerEnvironmentType; }

  bool is_floor_environment() const { return type_ == kFloorEnvironmentType; }

  bool is_trailer_environment() const { return type_ == kTrailerEnvironmentType; }
  EnvironmentType type() const { return type_; }

  void SetTrailerSize(double length, double width, double height) {
    set_trailer_environment();
    trailer_size_ = drake::Vector3<double>(length, width, height);
  }

  void SetCoulombFriction(double static_friction, double dynamic_friction) {
    coulomb_friction_ = drake::multibody::CoulombFriction<double>(static_friction, dynamic_friction);
  }

  const drake::Vector3<double>& gravity() const { return gravity_; }

  const drake::Vector3<double>& trailer_size() const {
    DR_DEMAND(type_ == kTrailerEnvironmentType);
    return trailer_size_;
  }

  const drake::multibody::CoulombFriction<double>& coulomb_friction() const { return coulomb_friction_; }

  const std::vector<BodyInstanceConfig>& body_instance_configs() const { return body_instance_configs_; }

  void set_body_instance_configs(const std::vector<BodyInstanceConfig>& body_instance_configs) {
    body_instance_configs_ = body_instance_configs;
  }

 private:
  /** TODO(samzapo): the environment should be defined in some frame, Y, which
   would normally be identity. But when it were not identity, the
   transformation would allow the trailer to assume a different location or
   orientation in the world.
   e.g., add member variable: drake::math::RigidTransform<double> pose_
   */

  /**
   EnvironmentBodies include all static bodies (fixed to the world) that the
   robot can touch and can be reprersented with the BodyInstanceConfig class.
   Bodies of this type must be fixed to the world and have geometric
   properties (inertial properties are not important).
   */
  std::vector<BodyInstanceConfig> body_instance_configs_;

  // This is the gravity vector of this environment (the default value is
  // usually best).
  drake::Vector3<double> gravity_{0.0, 0.0, -9.8};

  // Environment type (must be set):
  EnvironmentType type_{kUnknownEnvironmentType};

  // The frictional properties of the environment.
  // these are the friction properties for this specific object,
  // they'll be combined with friction properties for another object when
  // frictional forces need to be calculated (e.g., when the robot or another
  // object collides with the environment).
  drake::multibody::CoulombFriction<double> coulomb_friction_;

  // Max trailer bed size 52' L x 100" W (15.8496 m x 2.54 m)
  // Max trailer height 111" H (2.8194 m)
  drake::Vector3<double> trailer_size_{15.8496, 2.54, 2.8194};
};

/**
 Configuration class for a robot's actuated joints each configuration is meant to describe a unique joint on a
 robot, but not required to be unique in the simulated world.
 JointInstanceConfig stores information describing a robot's joints. This
 configuraton will be used by the ModelGenerator and StateSetter to build and initialize the robot model and used by
 ControllerGenerator to configure the robot control system.  During simulation, this config can be used by controllers
 and planners as a reference for joint names, types, and other relevant parameters.

  NOTE: This class should only be used for serialization.

  TODO(samzapo): Write a parser and exporter to YAML/JSON for this class.
 */
class JointInstanceConfig final : public ConfigBase {
 public:
  enum class JointType {
    kUnknownJointType = 0,
    kRevoluteJointType = 1,
    kPrismaticJointType = 2,
  };
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(JointInstanceConfig)

  JointInstanceConfig() {}
  JointInstanceConfig(const std::string& joint_name, JointType joint_type, drake::optional<double> position,
                      drake::optional<double> velocity, drake::optional<double> kp) {
    this->set_name(joint_name);
    this->set_type(joint_type);

    if (position.has_value()) this->set_position(position.value());
    if (velocity.has_value()) this->set_velocity(velocity.value());
    if (kp.has_value()) this->set_kp(kp.value());
  }
  ~JointInstanceConfig() {}
  void ValidateConfig() const final {
    DR_DEMAND(!name_.empty());
    DR_DEMAND(type_ != JointType::kUnknownJointType);
    DR_DEMAND(kp_ >= 0.0);
    DR_DEMAND(std::isfinite(position_));
    DR_DEMAND(std::isfinite(velocity_));
  }

  const std::string& name() const { return name_; }
  void set_name(const std::string& name) { name_ = name; }

  JointType type() const { return type_; }
  void set_type(JointType type) { type_ = type; }

  double kp() const { return kp_; }
  void set_kp(double kp) { kp_ = kp; }

  double position() const { return position_; }
  void set_position(double position) { position_ = position; }

  double velocity() const { return velocity_; }
  void set_velocity(double velocity) { velocity_ = velocity; }

 private:
  std::string name_{""};
  JointType type_{JointType::kUnknownJointType};
  double kp_{0.0};
  double position_{std::numeric_limits<double>::quiet_NaN()};
  double velocity_{0.0};
};

/**
 Configuration class for an instance of a robot, each configuration is meant to describe a unique robot in the
 simulated world.

 RobotInstanceConfig stores information describing a controlled multiobody or "robot". This configuraton will be used by
 the ModelGenerator and StateSetter to build and initialize the robot model and used by ControllerGenerator to configure
 the robot control system.  During simulation, this config can be used by controllers and planners as a reference for
 the robot's name, joint configurations, and the path to its model file to generate additional, controller-owned robot
 models.

  NOTE: This class should only be used for serialization.

  TODO(samzapo): Write a parser and exporter to YAML/JSON for this class.
 */
class RobotInstanceConfig final : public ConfigBase {
 public:
  /**
   ControlScheme describes what type of controllers to configure for this robot instance.
    NOTE: When a target control-system architecture is better known this Enum will be replaced with a ControllerConfig
    class.
   */
  enum ControlScheme {
    // The robot is not controlled, all joints are set to zero torque.
    kPassiveControlScheme = 1,
    // The robot is controlled and configured to hold its initial position.
    kStationaryControlScheme = 2,
    // The robot is controlled and configured to perform the TruckUnload task.
    kPrimitiveControlScheme = 3,
  };
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RobotInstanceConfig)

  RobotInstanceConfig() {}
  RobotInstanceConfig(const std::string& robot_name, const std::string& model_directory, const std::string& model_file,
                      const std::vector<JointInstanceConfig>& joint_configs) {
    this->set_name(robot_name);
    this->set_model_directory(model_directory);
    this->set_model_file(model_file);
    this->set_joint_instance_configs(joint_configs);
  }
  ~RobotInstanceConfig() {}
  void ValidateConfig() const final {
    DR_DEMAND(!name_.empty());
    DR_DEMAND(!model_file_.empty());
    DR_DEMAND(!model_directory_.empty());
  }

  int num_joints() const { return joint_instance_configs_.size(); }

  const std::string& name() const { return name_; }
  void set_name(const std::string& name) { name_ = name; }

  const std::string& model_file() const { return model_file_; }
  void set_model_file(const std::string& model_file) { model_file_ = model_file; }

  const std::string& model_directory() const { return model_directory_; }
  void set_model_directory(const std::string& model_directory) { model_directory_ = model_directory; }

  bool is_floating() const { return is_floating_; }
  void set_is_floating(bool is_floating) { is_floating_ = is_floating; }

  const std::vector<JointInstanceConfig>& joint_instance_configs() const { return joint_instance_configs_; }
  void set_joint_instance_configs(const std::vector<JointInstanceConfig>& joint_instance_configs) {
    joint_instance_configs_ = joint_instance_configs;
  }

  ControlScheme control_scheme() const { return control_scheme_; }
  void set_control_scheme(ControlScheme control_scheme) { control_scheme_ = control_scheme; }

  const drake::math::RigidTransform<double>& pose() const { return pose_; }
  void set_pose(const drake::math::RigidTransform<double>& pose) { pose_ = pose; }

  const drake::multibody::SpatialVelocity<double>& spatial_velocity() const { return spatial_velocity_; }
  void set_spatial_velocity(const drake::multibody::SpatialVelocity<double>& spatial_velocity) {
    spatial_velocity_ = spatial_velocity;
  }

  void set_gravity(double x, double y, double z) { gravity_ = drake::Vector3<double>(x, y, z); }
  const drake::Vector3<double>& gravity() const { return gravity_; }

 private:
  // This is the gravity vector of this robot;s controllers (the default value is
  // usually best).
  // NOTE: This value will be checked against EnvironmentInstanceConfig::gravity_
  drake::Vector3<double> gravity_{0.0, 0.0, -9.8};

  std::string name_{""};

  std::vector<JointInstanceConfig> joint_instance_configs_;

  std::string model_file_{""};
  std::string model_directory_{""};

  bool is_floating_{false};

  drake::math::RigidTransform<double> pose_ = drake::math::RigidTransform<double>::Identity();

  drake::multibody::SpatialVelocity<double> spatial_velocity_ = drake::multibody::SpatialVelocity<double>::Zero();

  ControlScheme control_scheme_{kPassiveControlScheme};
};

/**
 Configuration class describing the parameters for generating a
 trailer unloading task scenario.
 includes:
   1 Static Environment {Floor, Trailer}
   N Static or Dynamic Manipulands {Box, Sphere}

 NOTE: This class should only be used for serialization.

 TODO(samzapo): Write a parser and exporter to YAML/JSON for this class.
 */

class UnloadingTaskConfig final : public ConfigBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(UnloadingTaskConfig)

  UnloadingTaskConfig() {}
  virtual ~UnloadingTaskConfig() {}
  void ValidateConfig() const final {
    simulator_instance_config().ValidateConfig();
    // Set up unique name check.
    std::set<std::string> unique_names;
    std::pair<std::set<std::string>::iterator, bool> ret;

    for (const auto& manipuland : manipuland_instance_configs()) {
      manipuland.ValidateConfig();
      DR_DEMAND(manipuland.is_floating());

      // Guarantee that all bodies have a unique name.
      ret = unique_names.insert(manipuland.name());
      DR_DEMAND(ret.second, "A manipuland with name " + manipuland.name() + " already exists!");
    }

    environment_instance_config().ValidateConfig();

    for (const auto& env_body : environment_instance_config().body_instance_configs()) {
      // guarantee that all bodies have a unique name.
      ret = unique_names.insert(env_body.name());
      DR_DEMAND(ret.second, "A body with name " + env_body.name() + " already exists!");
    }

    for (const auto& robot : robot_instance_configs()) {
      // guarantee that all bodies have a unique name.
      ret = unique_names.insert(robot.name());
      DR_DEMAND(ret.second, "A robot with name " + robot.name() + " already exists!");

      // Make sure that gravity of world and robot match.
      DR_DEMAND(environment_instance_config().gravity() == robot.gravity());
    }
  }

  const SimulatorInstanceConfig& simulator_instance_config() const { return simulator_instance_config_; }

  const std::vector<BodyInstanceConfig>& manipuland_instance_configs() const { return manipuland_instance_configs_; }

  const EnvironmentInstanceConfig& environment_instance_config() const { return environment_instance_config_; }

  void set_simulator_instance_config(const SimulatorInstanceConfig& simulator_instance_config) {
    simulator_instance_config_ = simulator_instance_config;
  }

  void set_manipuland_instance_configs(const std::vector<BodyInstanceConfig>& manipuland_instance_configs) {
    manipuland_instance_configs_ = manipuland_instance_configs;
  }

  void set_environment_instance_config(const EnvironmentInstanceConfig& environment_instance_config) {
    environment_instance_config_ = environment_instance_config;
  }

  const std::vector<RobotInstanceConfig>& robot_instance_configs() const { return robot_instance_configs_; }
  void set_robot_instance_configs(const std::vector<RobotInstanceConfig>& robot_instance_configs) {
    robot_instance_configs_ = robot_instance_configs;
  }

 private:
  SimulatorInstanceConfig simulator_instance_config_;

  /**
   Manipulands include all dynamic bodies that the robot can touch and move.
   Bodies of this type must NOT be fixed to the world and have inertial and
   geometric properties.
   NOTE: The set of manipulands does not include any part of the robot.
   */
  std::vector<BodyInstanceConfig> manipuland_instance_configs_;

  std::vector<RobotInstanceConfig> robot_instance_configs_;

  EnvironmentInstanceConfig environment_instance_config_;
};

}  // namespace DR
