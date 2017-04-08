#pragma once

#include <Eigen/Core>

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

  /// The vector from the center-of-mass of the rod to the point of contact,
  /// defined in the body frame of the rod.
  Eigen::Vector2d u;

  /// The coefficient of friction at this contact point.
  double mu{0};
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake
