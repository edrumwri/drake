#pragma once

namespace DR {

/**
 A virtual class for providing robot-specific inverse kinematics solutions in such a way that control software can
 be written to be robot-agnostic.
 */ 
template <typename T>
class InverseKinematics {
 public:
  /**
   A generic method for computing inverse kinematics solutions for a desired position and orientation. In the interest
   of applications to both non-redundant and redundant manipulators, this API provides no mechanism for selecting from
   potentially multiple solutions.  
   @param X_WF_target the target pose of a frame on the robot (relative to the world frame)
   @param frame the material frame on the robot
   @param tol the tolerance to which the Euclidean norm of the difference between the target and the pose 
          corresponding to the solution must be satisfied 
   @return the generalized position q such that `||X_WF_target - f(q)|| < tol`, where f(q) is the forward kinematics
           function for `frame`
   @throws std::runtime_error if a solution cannot be found to the given tolerance
   */ 
  virtual VectorX<T> Solve(const drake::math::RigidTransform<T>& X_WF_target,
                           const drake::multibody::Frame<T>& frame,
                           double tol) const = 0;
};

}  // namespace DR

