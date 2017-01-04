#pragma once

#include <functional>
#include <limits>
#include <utility>

namespace drake {
namespace systems {

/// Functions for finding roots of univariate functions.
template <class T>
class UnivariateRootFinder {
 public:

  // Disable copy, assign, and move.
  UnivariateRootFinder(const UnivariateRootFinder<T>& other) = delete;
  UnivariateRootFinder& operator=(const UnivariateRootFinder<T>& other) =
      delete;

  static std::pair<T, bool> FindRootWithBisection(
      std::function<T(T)> f,
      T x_left, T x_right,
      T tol_left, T tol_right,
      int max_iter = std::numeric_limits<int>::max());
};
}  // namespace systems
}  // namespace drake
