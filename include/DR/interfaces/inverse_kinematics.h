#pragma once

#include <drake/common/eigen_types.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/tree/frame.h>

namespace DR {

/**
 A virtual class for providing robot-specific inverse kinematics solutions in such a way that control software can
 be written to be robot-agnostic.
 */
template <typename T>
class InverseKinematics {
 public:
  virtual ~InverseKinematics() {}

  /**
   A generic method for computing inverse kinematics solutions for a desired position and orientation. In the interest
   of applications to both non-redundant and redundant manipulators, this API provides no mechanism for selecting from
   potentially multiple solutions.
   @param X_WG_target the target pose of Frame G on the robot (relative to the world frame)
   @param p_FG a position vector from Frame F to Frame G, which is aligned identically to Frame F
   @param F the material frame on the robot
   @return the generalized position q such that `||X_WG_target - f_G(q)|| < tol`, where f_G(q) is the forward kinematics
           function for `G`
   @throws std::runtime_error if a solution cannot be found to the given tolerance
   */
  virtual drake::VectorX<T> SolveInverseKinematics(const drake::math::RigidTransform<T>& X_WG_target,
      const drake::Vector3<T>& p_FG, const drake::multibody::Frame<T>& F) const = 0;
};

}  // namespace DR

