#pragma once

#include <Eigen/Core>

#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace rod2d {

/// The descriptor for a rigid contact between two rigid bodies.
struct RigidContact {
  /// The state of two potentially intersecting features (resulting, in this case,
  /// in a single point of contact) between two rigid bodies. This point of
  /// contact would ideally be active only if the distance between the
  /// corresponding features on the rigid bodies were zero; because of floating
  /// point error (truncation and rounding error), we often wish for the related
  /// contact constraints to be treated as active even when the distance between
  /// the corresponding features is greater than zero.
  enum class ContactState {
    /// The bodies are to be considered to be not-in-contact at these two
    /// features.
        kNotContacting,

    /// The bodies are to be considered as in-contact and sliding at these
    /// two features.
        kContactingAndSliding,

    /// The bodies are to be considered as in-contact and not sliding at these
    /// two features.
        kContactingWithoutSliding,
  };

  /// The state of the contact.
  ContactState state;

  /// The vector from the center-of-mass of the rigid body to the point of
  /// contact, defined in the body frame of the rigid body.
  // TODO(edrumwri): This vector will need to be updated to account for the
  //                 fact that two rigid bodies are (generally) involved in a
  //                 rigid contact.
  Eigen::Vector3d u;

  /// The coefficient of friction at this contact point.
  double mu{0};
};

/// Structure for holding rigid contact data for computing rigid contact
/// problems at the acceleration-level.
template <class T>
struct RigidContactAccelProblemData {
  /// Flag that indicates whether one or more points of contact is transitioning
  /// from not-sliding to sliding, indicating whether LCP solver must be used.
  bool transitioning_contacts{false};

  /// The indices of the sliding contacts (those contacts at which there is
  /// non-zero relative velocity between bodies in the plane tangent to the
  /// point of contact) in the vector of possible contacts.
  std::vector<int> sliding_contacts;

  /// The indices of the non-sliding contacts (those contacts at which there
  /// is zero relative velocity between bodies in the plane tangent to the
  /// point of contact) in the vector of possible contacts. This group also
  /// includes those contacts which have a sliding mode but for which the
  /// tangential velocity is momentarily below the floating point tolerance for
  /// zero.
  std::vector<int> non_sliding_contacts;

  /// Coefficients of friction for the sliding contacts.
  VectorX<T> mu_sliding;

  /// Coefficients of friction for the non-sliding contacts.
  VectorX<T> mu_non_sliding;

  /// The ℝⁿˣᵐ Jacobian matrix that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into velocities projected along the
  /// contact normals at the n contact points.
  MatrixX<T> N;

  /// The time derivative of the matrix N defined above.
  MatrixX<T> Ndot;

  /// The ℝⁿᵏˣᵐ Jacobian matrix that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into velocities projected along the
  /// k vector that span the contact tangents (used to linearize the friction
  /// cone) at the n *non-sliding* contact points. For contact problems in two
  /// dimensions, k will be one. For a friction pyramid in three dimensions, k
  /// would be two.
  MatrixX<T> F;

  /// The time derivative of the matrix F defined above.
  MatrixX<T> Fdot;

  /// The ℝⁿˣᵐ Jacobian matrix that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into velocities projected along the
  /// directions of sliding at the n *sliding* contact points.
  MatrixX<T> Q;

  /// The ℝⁿˣᵐ matrix (N - μQ) that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into velocities projected along the
  /// contact normals at the n contact points. The μQ factor indicates that
  /// any normal forces applied using this Jacobian will yield frictional
  /// affects for sliding contacts.
  MatrixX<T> N_minus_mu_Q;

  /// The ℝᵐˣⁿᵏ matrix M⁻¹Fᵀ that transforms forces along the k spanning vectors
  /// in the n non-sliding contact planes to generalized accelerations.
  MatrixX<T> M_inv_x_FT;
};

/// Structure for holding rigid contact data for computing rigid contact
/// problems at the velocity-level (impacts and time stepping).
template <class T>
struct RigidContactVelProblemData {
  /// The indices of the active contacts in the vector of possible contacts.
  std::vector<int> active_contacts;

  /// Coefficients of friction (identical for static and dynamic).
  VectorX<T> mu;

  /// The ℝⁿˣᵐ Jacobian matrix that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into velocities projected along the
  /// contact normals at the n contact points.
  MatrixX<T> N;

  /// The ℝⁿᵏˣᵐ Jacobian matrix that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into velocities projected along the
  /// k vector that span the contact tangents (used to linearize the friction
  /// cone) at the n contact points. For contact problems in two dimensions, k
  /// will be one. For a friction pyramid in three dimensions, k would be two
  /// (greater k will allow more faithful approximations)..
  MatrixX<T> F;
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake
