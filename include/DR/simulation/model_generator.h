/*
 This file defines functions that add models to an un-finalized MultibodyPlant according to configuration
 parameters provided by SingleBodyInstanceConfig & RobotInstanceConfig from 'DR/simulation/config.h'.
 */
#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <utility>

#include <drake/common/drake_optional.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>

#include <DR/common/exception.h>
#include <DR/simulation/config.h>
#include <DR/simulation/shape_to_unit_inertia.h>
#include <DR/tools/output_stream_operators.h>

namespace DR {

/**
 Moves the model to the given pose in the world and then generates a weld
 joint between the world frame and the model for states that will remain
 fixed.
 @param body_name the base link of the robot or object (should be either
        "base" for the root link of a robot, or the object name for fixed
        objects).
 @param model_instance the instance index of the target model.
 @param X_WM the transform from the world frame to the intended pose of
        the model.
 @param mbp raw pointer to the MultiBodyPlant representing the world.
 */
template <typename T>
void WeldToWorldAtPose(std::string body_name, const drake::multibody::ModelInstanceIndex& model_instance,
                       const drake::math::RigidTransform<double>& X_WM, drake::multibody::MultibodyPlant<T>* mbp) {
  // This adds an offset to the named body in the model instance and then
  // welds the body to world.
  const drake::multibody::Body<T>& world_body = mbp->world_body();
  const drake::multibody::Body<T>& model_body = mbp->GetBodyByName(body_name, model_instance);
  mbp->template AddJoint<drake::multibody::WeldJoint>(body_name + "_world_weld", world_body, X_WM, model_body,
                                                      drake::math::RigidTransform<double>::Identity() /* X_CJ */,
                                                      drake::math::RigidTransform<double>::Identity() /* X_JpJc */);
}

/**
 Generates a robot model described by RobotInstanceConfig and adds it to the MultibodyPlant.
 @param RobotInstanceConfig configuration class with file path to the robot description file and other parameters for
        the robot instance.
 @param MultibodyPlant raw pointer to the universal plant, the robot model will be added to this system.
 @return the index of the newly created model instance.

 NOTE: The robot's base link must be named
 */
template <typename T>
drake::multibody::ModelInstanceIndex AddRobotToMBP(const RobotInstanceConfig& config,
                                                   drake::multibody::MultibodyPlant<T>* mbp) {
  mbp->mutable_gravity_field().set_gravity_vector(kDefaultGravityVector);

  // TODO(samzapo): Implement for T = AutoDiff
  // The class `drake::multibody::Parser(mbp)` is only defined
  // for `drake::multibody::MultibodyPlant<T>*` for `T = double` not `T = AutoDiff`
  static_assert(std::is_same<T, double>::value, "ModelGenerator<T>::AddRobotToMBP is only implemented for T = double");

  DR_DEMAND(mbp);
  drake::multibody::ModelInstanceIndex model_instance =
      drake::multibody::Parser(mbp).AddModelFromFile(config.model_file_path(), config.name());

  // TODO(samzapo) Remove this constraint.  Find the name of the root link of the robot model programmatically.
  DR_DEMAND(mbp->HasBodyNamed("base", model_instance), "Robot model must have link named 'base'");

  if (!config.is_floating()) {
    WeldToWorldAtPose("base", model_instance, config.pose(), mbp);
  }

  return model_instance;
}

/**
 Generates a body and places it in the MultibodyPlant according to the
 config (e.g., a box that can be picked-up, a static box on the floor).
 @param config the SingleBodyInstanceConfig describing the attributes of the
        dynamic object (e.g., friction, mass, geometry).
 @param mbp a raw pointer to the MultibodyPlant to populate with the model.
 */
template <typename T>
drake::multibody::ModelInstanceIndex AddBodyToMBP(const SingleBodyInstanceConfig& config,
                                                  drake::multibody::MultibodyPlant<T>* mbp) {
  mbp->mutable_gravity_field().set_gravity_vector(kDefaultGravityVector);

  const drake::math::RigidTransform<double>& X_WM = config.pose();

  // A 'unit inertia' is not an identity inertia tensor. Instead it is the body's inertia tensor divided by its mass.
  // See drake::multibody::UnitInertia".
  drake::multibody::UnitInertia<T> rotational_inertia;

  // Some static-only geometries can't produce a UnitInertia.
  // Applies a token inertia (unit solid sphere) for static bodies.
  if (config.is_static()) {
    rotational_inertia = drake::multibody::UnitInertia<T>::SolidSphere(1.0);
  } else {
    // If a body has a floating base, then it must have a collision geometry.
    // Otherwise, will fall forever under the influence of gravity.
    DR_DEMAND(config.has_collision_geometry());

    // Get inertia of collision geometry.
    // TODO(samzapo): Add an inertia configuration param to SingleBodyInstanceConfig.
    ShapeToUnitInertia<double> reifier;
    config.collision_geometry().Reify(&reifier);
    rotational_inertia = reifier.unit_inertia();
  }

  // TODO(samzapo): Support "Center of mass" offset from body origin.
  drake::multibody::SpatialInertia<double> M_Bcm(config.mass(), drake::Vector3<double>::Zero() /* center of mass */,
                                                 rotational_inertia);

  drake::multibody::ModelInstanceIndex model_instance = mbp->AddModelInstance(config.name());

  const drake::multibody::RigidBody<T>& object = mbp->AddRigidBody(config.name(), model_instance, M_Bcm);

  // Pose of geometry in body frame.
  const drake::math::RigidTransform<double> X_GB = drake::math::RigidTransform<double>::Identity();

  if (config.has_collision_geometry()) {
    // Add geometry for the object.
    mbp->RegisterCollisionGeometry(object, X_GB, config.collision_geometry(), config.name() + "_collision",
                                   config.coulomb_friction());
  }
  // Add visual for the object.
  mbp->RegisterVisualGeometry(object, X_GB, config.visual_geometry(), config.name() + "_visual", config.color());

  if (config.is_static()) {
    WeldToWorldAtPose(config.name(), model_instance, X_WM, mbp);
    drake::log()->debug("Affixed model {} to the environment at pose {}.", config.name(), StreamToString(X_WM));
  }
  // Else, do nothing, initial state will be set in context after model is finalized and diagram is built.

  return model_instance;
}

/**
 A convenience function that generates a new MultibodyPlant to represent the robot described by the RobotInstanceConfig.
 The returned MultibodyPlant can be used for, e.g., getting generalized inertia of just the robot, not every body in a
 simulation.
 @param config a description of the robot model.
 @return a unique pointer to the newly created and finalized robot plant.
 */
template <typename T>
std::unique_ptr<typename drake::multibody::MultibodyPlant<T>> CreateStandaloneRobotPlant(
    const RobotInstanceConfig& config) {
  auto mbp = std::make_unique<typename drake::multibody::MultibodyPlant<T>>();

  // Add robot model to the new multibody plant instance.
  AddRobotToMBP(config, mbp.get());

  mbp->Finalize();
  return mbp;
}
}  // namespace DR
