#pragma once

#include <Eigen/Core>

namespace drake {

template <class T>
class Vertex {
  Eigen::Vector3<T> location;
  Eigen::Vector3<T> traction;
  Eigen::Vector2<T> slip_velocity;
};

template <class T>
class PolygonalMesh {

};

}
