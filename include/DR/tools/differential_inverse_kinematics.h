#pragma once

#include <drake/math/rigid_transform.h>

namespace DR {

/**
 Class for methods related to differential inverse kinematics.
 */
template <typename T>
class DifferentialInverseKinematics {
 public:
  /**
   Computes the spatial differential ΔP between two poses P1 and P2 such that P1 + ΔP = P2. 
   */
  static Eigen::Matrix<T, 6, 1> CalcOperationalSpaceDifferential(
      const drake::math::RigidTransform<T>& P1,
      const drake::math::RigidTransform<T>& P2,
      bool angular_components_on_top = true) {
    const drake::Vector3<T> position_differential = CalcPositionDifferential(P1.translation(), P2.translation());
    const drake::Vector3<T> orientation_differential = CalcOrientationDifferential(P1.rotation(), P2.rotation());
    Eigen::Matrix<T, 6, 1> delta_P;
    if (angular_components_on_top) {
      delta_P.template head<3>() = orientation_differential;
      delta_P.template tail<3>() = position_differential;
    } else {
      delta_P.template head<3>() = position_differential;
      delta_P.template tail<3>() = orientation_differential;
    }
    return delta_P;
  }

  /**
   Computes the position differential Δv between two three-dimensional vectors v1 and v2 such that v1 + Δv = v2.
   */
  static drake::Vector3<T> CalcPositionDifferential(const drake::Vector3<T>& v1, const drake::Vector3<T>& v2) {
    return v2 - v1;
  }

  /**
   Computes the orientation differential ω between two rotation matrices R1 and R2 such that R1 + ΔR = R2, where
   ΔR = skew(ω)R1.  
   */
  static drake::Vector3<T> CalcOrientationDifferential(
      const drake::math::RotationMatrix<T>& R1, const drake::math::RotationMatrix<T>& R2) {
    const drake::Matrix3<T> R1m = R1.matrix();
    const drake::Matrix3<T>& delta_R = R2.matrix() - R1m;
    const drake::Matrix3<T> skew_omega = delta_R * R1m.transpose();
    return drake::Vector3<T>(skew_omega(2, 1) - skew_omega(1, 2),
                             skew_omega(0, 2) - skew_omega(2, 0),
                             skew_omega(1, 0) - skew_omega(0, 1)) * 0.5;
  }

  /**
   Removes the components from an orientation differential that are aligned with the given direction. Put another way,
   any instantaneous angular velocity around the direction vector d is removed from the vector `omega`.
   @throws std::runtime_error if the direction vector d is exactly zero-length. 
   */
  static drake::Vector3<T> RemoveAngularVelocityComponentsAlongDirection(
     const drake::Vector3<T>& omega, const drake::Vector3<T>& d) {
    const T norm_d = d.norm();
    if (norm_d == 0.0)
      throw std::runtime_error("Direction vector is zero length.");
    const drake::Vector3<T> dN = d / norm_d;
    return omega - dN * dN.dot(omega);
  }

  /**
   A function for computing an inverse kinematics solution using differential kinematics. This method provides a trivial
   implementation of Newton-Raphson. It does not account for joint limits or geometric intersections with the
   environment: the caller is responsible for checking that the result is valid in these respects. The abstract
   implementation of this method allows for solving in various ways Jacobian transpose, damped least squares, etc.

   @param X_WF_target the target pose of some frame 
   @param X_WF_fn a function that returns the frame pose given a generalized position as input
   @param diff_fn a function that returns the differential ΔP between two poses P1 and P2 such that P1 + ΔP = P2
   @param delta_q_fn a function that computes a direction in generalized position space (Δq) given a configuration q and
          pose differential (ΔP), e.g., by solving JΔq = ΔP (where J(q) is the Jacobian matrix of partial derivatives of
          poses P taken with respect to generalized positions q). Note that the implementer is responsible for
          computing Jacobian matrices (if necessary/desired).
   @param q_seed the configuration to start iterating from
   @param max_iterations the maximum number of iterations to take before failing 
   @param tol the tolerance to which the Euclidean norm of the difference between the target and the pose 
          corresponding to the solution must be satisfied 
   @return the generalized position q such that `||X_WF_target - f(q)|| < tol`, where f(q) is the forward kinematics
           function for the given frame 
   @throws std::runtime_error if a solution cannot be found to the given tolerance
   */
  drake::VectorX<T> Solve(
      const drake::math::RigidTransform<T>& X_WF_target,
      const std::function<drake::math::RigidTransform<T>(const drake::VectorX<T>&)>& X_WF_fn,
      const std::function<drake::VectorX<T>(
          const drake::math::RigidTransform<T>&, const drake::math::RigidTransform<T>&)>& diff_fn,
      const std::function<drake::VectorX<T>(const drake::VectorX<T>&, const drake::VectorX<T>&)>& delta_q_fn,
      const drake::VectorX<T>& q_seed,
      int max_iterations = 100,
      double tol = 1e-8) {
    // Use a squared tolerance to avoid expensive square roots.
    const double squared_tol = tol * tol;

    // Declare here to avoid heap allocations.
    drake::VectorX<T> delta_P;

    // Set initial value of q.
    drake::VectorX<T> q = q_seed;

    for (int i = 0; i < max_iterations; ++i) {
      // Compute the operational space differential.
      delta_P = diff_fn(X_WF_fn(q), X_WF_target);

      // Check for convergence.
      const double delta_P_squared_norm = delta_P.squaredNorm();
      if (delta_P_squared_norm < squared_tol)
        return q;

      // A scaling factor that minimizes large changes when far away from the solution.
      const double alpha = atan(1.0 / delta_P_squared_norm) / M_PI_2;

      // Update q.
      q += delta_q_fn(q, delta_P) * alpha;
    }

    throw std::runtime_error(
        "Could not find a solution to the requested tolerance within the given number of iterations.");
  }
};

}  // namespace DR

// Instantiate templates.
extern template class DR::DifferentialInverseKinematics<double>;

