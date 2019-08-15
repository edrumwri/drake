#pragma once

#include <map>

#include <drake/common/drake_optional.h>

namespace DR {
template <typename T, typename K>
const T& GetOptionalValueOrDefault(const drake::optional<std::map<K, T>>& optional_map, const K& map_key,
                                   const T& default_value) {
  if (optional_map.has_value()) {
    auto it = optional_map.value().find(map_key);
    if (it != optional_map.value().end()) {
      return it->second;
    }
  }
  return default_value;
}

template <typename T>
const T& GetOptionalValueOrDefault(const drake::optional<T>& optional_value, const T& default_value) {
  if (optional_value.has_value()) {
    return optional_value.value();
  }
  return default_value;
}
}  // namespace DR
