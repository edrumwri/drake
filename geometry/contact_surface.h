#pragma once

template <class T>
class ContactSurfaceVertex {
  Eigen::Vector3<T> location;
  Eigen::Vector3<T> traction;
  Eigen::Vector2<T> slip_velocity;
};

template <class T>
class ContactSurface {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactSurface)

  /// The id of the first geometry in the contact.
  GeometryId id_A;

  /// The id of the second geometry in the contact.
  GeometryId id_B;

  /// Vertices comprising the contact surface.
  std::vector<ContactSurfaceVertex<T>> vertices;
};