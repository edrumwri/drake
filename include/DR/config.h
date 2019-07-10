#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <drake/geometry/shape_specification.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/plant/coulomb_friction.h>
#include <drake/multibody/shapes/geometry.h>

namespace DR {

/**
 * NOTE: Configuration files are not templatized because they do not store
 * parameters that vary with respect to the scalar type of a
 * drake::multibody::MultibodyPlant<T>.  Example:
 * ```
 * template <typename T>
 * geometry::GeometryId MultibodyPlant<T>::RegisterVisualGeometry(
 *  const Body<T>& body, const math::RigidTransform<double>& X_BG,
 *  const geometry::Shape& shape, const std::string& name,
 *  const Vector4<double>& diffuse_color,
 *  SceneGraph<T>* scene_graph)
 * ```
 * the Pose `math::RigidTransform<double>` remains a double despite the
 * templated scalar type of MultibodyPlant.
 */

/**
 * The base interface for all configuration specifications.  It requires that
 * all derived classes to implement a ValidateConfig function that will check
 * the member variables of the config file for validity.
 */
class ConfigBase {
 public:
  virtual ~ConfigBase() {}

  /**
   * This function much be implemented by all derived classes of ConfigBase
   * class. This function should throw an error if one or more of the member
   * variables is not set correctly (e.g., out of valid range, not set, NaN).
   */
  virtual void ValidateConfig() const = 0;
};

/**
 * This class stores simulation-specific parameters.
 * The member variables of SimulatorInstanceConfig affect the performance of
 * physical simulation of the world model.
 *
 * NOTE: Some of these simulation parameters are used even if a MultibodyPlant
 * will not be simulated (e.g., step_size is used to configure a
 * MultibodyPlant).
 */
class SimulatorInstanceConfig : public ConfigBase {
 public:
  /**
   * Enum types for simulation integration schemes:
   *
   * 0) kUnknownIntegrationScheme: No integration scheme was set.
   *
   * 1) kSemiExplicitEulerIntegrationScheme: A first-order, semi-explicit Euler
   * integrator.
   *
   * 2) kRK2IntegrationScheme: A second-order, explicit Runge Kutta integrator.
   *
   * 3) kRK3IntegrationScheme: A third-order Runge Kutta integrator with a third
   * order error estimate.
   *
   * 4) kImplicitEulerIntegrationScheme: A first-order, fully implicit
   * integrator with second order error estimation.
   */
  enum IntegrationScheme {
    kUnknownIntegrationScheme = 0,
    kSemiExplicitEulerIntegrationScheme = 1,
    kRK2IntegrationScheme = 2,
    kRK3IntegrationScheme = 3,
    kImplicitEulerIntegrationScheme = 4,
  };

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SimulatorInstanceConfig)

  SimulatorInstanceConfig() {}
  virtual ~SimulatorInstanceConfig() {}
  void ValidateConfig() const final {
    DRAKE_DEMAND(target_accuracy_ > 0.0);
    DRAKE_DEMAND(target_realtime_rate_ > 0.0);
    DRAKE_DEMAND(simulation_time_ > 0.0);
    DRAKE_DEMAND(integration_scheme_ != kUnknownIntegrationScheme);
  }

  void set_target_accuracy(double target_accuracy) {
    target_accuracy_ = target_accuracy;
  }

  void set_target_realtime_rate(double target_realtime_rate) {
    target_realtime_rate_ = target_realtime_rate;
  }

  void set_simulation_time(double simulation_time) {
    simulation_time_ = simulation_time;
  }

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

  void set_integration_scheme(IntegrationScheme integration_scheme) {
    integration_scheme_ = integration_scheme;
  }

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
 * Bodies include all dynamic and static (fixed to the world) bodies that the
 * robot can touch.
 * NOTE: This group excludes the environment (floor, trailer of truck, and the
 * robot model) but includes static objects.
 */

class BodyInstanceConfig : public ConfigBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BodyInstanceConfig)

  BodyInstanceConfig() {}
  virtual ~BodyInstanceConfig() {}
  void ValidateConfig() const final {
    DRAKE_DEMAND(geometry_.get());
    DRAKE_DEMAND(!name_.empty());
    if (is_floating()) {
      DRAKE_DEMAND(mass_ > 0);
    }

    DRAKE_DEMAND(std::isfinite(pose_.GetMaximumAbsoluteDifference(
        drake::math::RigidTransform<double>::Identity())));
  }

  void set_name(const std::string& name) { name_ = name; }

  void set_is_floating(bool is_floating) { is_floating_ = is_floating; }

  void set_pose(const drake::math::RigidTransform<double>& pose) {
    pose_ = pose;
  }

  void SetBoxGeometry(double length, double width, double height) {
    geometry_ = std::make_shared<drake::geometry::Box>(length, width, height);
  }

  void SetSphereGeometry(double radius) {
    geometry_ = std::make_shared<drake::geometry::Sphere>(radius);
  }

  void SetCylinderGeometry(double radius, double length) {
    geometry_ = std::make_shared<drake::geometry::Cylinder>(radius, length);
  }

  void set_mass(double mass) { mass_ = mass; }

  void SetCoulombFriction(double static_friction, double dynamic_friction) {
    coulomb_friction_ = drake::multibody::CoulombFriction<double>(
        static_friction, dynamic_friction);
  }

  const std::string& name() const { return name_; }

  bool is_floating() const { return is_floating_; }

  const drake::math::RigidTransform<double>& pose() const { return pose_; }

  const drake::geometry::Shape& geometry() const { return *geometry_.get(); }

  double mass() const { return mass_; }

  const drake::multibody::CoulombFriction<double>& coulomb_friction() const {
    return coulomb_friction_;
  }

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
  drake::math::RigidTransform<double> pose_;

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
 * The static floor or trailer is referred to as the "environment".
 * There is only ever one environment because it is composed of open
 * geometries (e.g. HalfSpace).
 */

class EnvironmentInstanceConfig : public ConfigBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EnvironmentInstanceConfig)

  EnvironmentInstanceConfig() {}
  virtual ~EnvironmentInstanceConfig() {}
  void ValidateConfig() const final {
    DRAKE_DEMAND(std::isfinite(gravity_.norm()));
    DRAKE_DEMAND(std::isfinite(trailer_size_.norm()));
    DRAKE_DEMAND(trailer_size_.minCoeff() > 0.0);
    DRAKE_DEMAND(type_ != kUnknownEnvironmentType);

    for (const auto& env_body : body_instance_configs()) {
      env_body.ValidateConfig();
      DRAKE_DEMAND(!env_body.is_floating());
    }
  }

  void set_gravity(double x, double y, double z) {
    gravity_ = drake::Vector3<double>(x, y, z);
  }

  void SetFloorEnvironment() { type_ = kFloorEnvironmentType; }

  void SetTrailerEnvironment() { type_ = kTrailerEnvironmentType; }

  bool IsFloorEnvironment() const { return type_ == kFloorEnvironmentType; }

  bool IsTrailerEnvironment() const { return type_ == kTrailerEnvironmentType; }

  void SetTrailerSize(double length, double width, double height) {
    SetTrailerEnvironment();
    trailer_size_ = drake::Vector3<double>(length, width, height);
  }

  void SetCoulombFriction(double static_friction, double dynamic_friction) {
    coulomb_friction_ = drake::multibody::CoulombFriction<double>(
        static_friction, dynamic_friction);
  }

  const drake::Vector3<double>& gravity() const { return gravity_; }

  const drake::Vector3<double>& trailer_size() const {
    DRAKE_DEMAND(type_ == kTrailerEnvironmentType);
    return trailer_size_;
  }

  const drake::multibody::CoulombFriction<double>& coulomb_friction() const {
    return coulomb_friction_;
  }

  const std::vector<BodyInstanceConfig>& body_instance_configs() const {
    return body_instance_configs_;
  }

  void set_body_instance_configs(
      const std::vector<BodyInstanceConfig>& body_instance_configs) {
    body_instance_configs_ = body_instance_configs;
  }

 private:
  /** TODO(samzapo): the environment should be defined in some frame, Y, which
   * would normally be identity. But when it were not identity, the
   * transformation would allow the trailer to assume a different location or
   * orientation in the world.
   * e.g., add member variable: drake::math::RigidTransform<double> pose_
   * NOTE: Does this set the parent frame of all bodies in
   * body_instance_configs_?
   */

  /**
   * EnvironmentBodies include all static bodies (fixed to the world) that the
   * robot can touch and can be reprersented with the BodyInstanceConfig class.
   * Bodies of this type must be fixed to the world and have geometric
   * properties, (inertial properties are not important).
   */
  std::vector<BodyInstanceConfig> body_instance_configs_;

  // This is the gravity vector of this environment (the default value is
  // usually best).
  drake::Vector3<double> gravity_{0.0, 0.0, -9.8};

  // Environment type (must be set):
  // 'Trailer' is the shape of an open back of a trailer truck represented
  //           by 5 HalfSpaces the origin is at the center of the trailer
  //           floor with +X pointing into the truck,
  //           +Y pointing to the left, and +Z as up.
  // 'Floor' is an infinite horizontal plane represented by a HalfSpace at the
  //         origin with a +Z normal (up).
  enum EnvironmentType {
    kUnknownEnvironmentType = 0,
    kTrailerEnvironmentType = 1,
    kFloorEnvironmentType = 2,
  } type_{kUnknownEnvironmentType};

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
 * Configuration class describing the parameters for generating a
 * trailer unloading task scenario.
 * includes:
 *   1 Static Environment {Floor, Trailer}
 *   N Static or Dynamic Manipulands {Box, Sphere}
 */

class UnloadingTaskConfig : public ConfigBase {
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
      DRAKE_DEMAND(manipuland.is_floating());

      // Guarantee that all bodies have a unique name.
      ret = unique_names.insert(manipuland.name());
      if (ret.second == false) {
        std::cout << "A manipuland with name " << manipuland.name()
                  << " already exists" << std::endl;
      }
      DRAKE_DEMAND(ret.second);
    }

    environment_instance_config().ValidateConfig();

    for (const auto& env_body :
         environment_instance_config().body_instance_configs()) {
      // guarantee that all bodies have a unique name.
      ret = unique_names.insert(env_body.name());
      if (ret.second == false) {
        std::cout << "A body with name " << env_body.name() << " already exists"
                  << std::endl;
      }
      DRAKE_DEMAND(ret.second);
    }
  }

  const SimulatorInstanceConfig& simulator_instance_config() const {
    return simulator_instance_config_;
  }

  const std::vector<BodyInstanceConfig>& manipuland_instance_configs() const {
    return manipuland_instance_configs_;
  }

  const EnvironmentInstanceConfig& environment_instance_config() const {
    return environment_instance_config_;
  }

  void set_simulator_instance_config(
      const SimulatorInstanceConfig& simulator_instance_config) {
    simulator_instance_config_ = simulator_instance_config;
  }

  void set_manipuland_instance_configs(
      const std::vector<BodyInstanceConfig>& manipuland_instance_configs) {
    manipuland_instance_configs_ = manipuland_instance_configs;
  }

  void set_environment_instance_config(
      const EnvironmentInstanceConfig& environment_instance_config) {
    environment_instance_config_ = environment_instance_config;
  }

 private:
  SimulatorInstanceConfig simulator_instance_config_;

  /**
   * Manipulands include all dynamic bodies that the robot can touch and move.
   * Bodies of this type must NOT be fixed to the world and have inertial and
   * geometric properties.
   * NOTE: The set of manipulands does not include any part of the robot.
   */
  std::vector<BodyInstanceConfig> manipuland_instance_configs_;

  EnvironmentInstanceConfig environment_instance_config_;
};

}  // namespace DR
