/*
  Operator overloads for various data structures to simplify logging code.
*/
#pragma once

#include <iostream>
#include <map>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include <drake/math/rigid_transform.h>
#include <drake/multibody/math/spatial_velocity.h>

namespace DR {

/// Both types `K` and `V` must provide `ostream` overloads.
template <typename K, typename V>
std::ostream& operator<<(std::ostream& os, const std::map<K, V>& m) {
  os << "{";
  for (const auto& p : m) {
    os << p << std::endl;
  }
  os << "};";
  return os;
}

/// Type `T` must provide an `ostream` overload.
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec) {
  os << "{";
  for (const auto& v : vec) {
    os << v << std::endl;
  }
  os << "};";
  return os;
}

/// Type `T` must provide an `ostream` overload.
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::set<T>& vec) {
  os << "{";
  for (const auto& v : vec) {
    os << v << std::endl;
  }
  os << "};";
  return os;
}

/// Type `T` must provide an `ostream` overload.
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::unordered_set<T>& vec) {
  os << "{";
  for (const auto& v : vec) {
    os << v << std::endl;
  }
  os << "};";
  return os;
}

/// Both types `V1` and `V2` must provide `ostream` overloads.
template <typename V1, typename V2>
std::ostream& operator<<(std::ostream& os, const std::pair<V1, V2>& pair) {
  os << "{" << pair.first << "," << pair.second << "}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const drake::math::RigidTransform<double>& pose) {
  os << "translation=[ " << pose.translation().transpose() << " ], rotation=[ " << pose.rotation().ToQuaternion().w()
     << " < " << pose.rotation().ToQuaternion().vec().transpose() << " > ]";
  return os;
}

std::ostream& operator<<(std::ostream& os, const drake::multibody::SpatialVelocity<double>& spatial_velocity) {
  os << "rotational=[ " << spatial_velocity.rotational().transpose() << " ], translational=[ "
     << spatial_velocity.translational().transpose() << " ]";
  return os;
}

/// Type `T` must provide an `ostream` overload.
/// NOTE: This method is required to output `drake::math::RigidTransform` via `fmt::format`.
template <typename T>
std::string StreamToString(const T& value) {
  std::stringstream ss;
  ss << value;
  return ss.str();
}

}  // namespace DR
