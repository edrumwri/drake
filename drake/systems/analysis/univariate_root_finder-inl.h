#pragma once

/// @file
/// Template method implementations for univariate_root_finder.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include <stdexcept>
#include <utility>

#include "drake/systems/analysis/univariate_root_finder.h"

namespace drake {
namespace systems {

/// Uses the bisection method to locate a root known to lie between x_left and
/// x_right. The root (on return), will be refined until
/// tol_left <= f(root) <= tol_right (or the maximum number of iterations is
/// exceeded).
/// @param f The univariate function.
/// @param x_left The left-hand side of the interval to search.
/// @param x_right The right-hand side of the interval to search.
/// @param tol_left The left handed tolerance (typically should be a negative
///        number, must not be positive).
/// @param tol_right The right handed tolerance (typically should be a positive
///        number, must not be negative).
/// @param max_iter The maximum number of iterations of bisection.
/// @returns the root, subject to the given tolerance, or the best approximation
///          obtainable within the maximum number of iterations.
/// @pre The root has been bracketed, meaning that f(x_left)*f(x_right) <= 0.
/// @throws std::logic_error if the root has not been bracketed, tol_left is
///         positive, or tol_right is negative.
template <class T>
std::pair<T, bool> UnivariateRootFinder<T>::FindRootWithBisection(
                                                 std::function<T(T)> f,
                                                 T x_left, T x_right,
                                                 T tol_left, T tol_right,
                                                 int max_iter) {
  // Evaluate the function at both endpoints.
  T f_left = f(x_left);
  T f_right = f(x_right);
  if (f_left * f_right > 0)
    throw std::logic_error("Root has not been bracketed.");

  // Check validity of tolerances
  if (tol_left > 0)
    throw std::logic_error("Left tolerance is positive.");
  if (tol_right < 0)
    throw std::logic_error("Right tolerance is negative.");

  // Check for immediate solution.
  if (f_left == 0)
    return std::make_pair(x_left, true);
  if (f_right == 0)
    return std::make_pair(x_right, true);

  // Search.
  for (int i=0; i< max_iter; ++i) {
    // Do the bisection and interval reassignment.
    T x_mid = (x_left + x_right)/2;
    T f_mid = f(x_mid);
    if (f_mid <= tol_right && f_mid >= tol_left)
      return std::make_pair(x_mid, true);

    // Do the interval reassignment.
    if (f_left * f_mid <= 0.0) {
      x_right = x_mid;
      f_right = f_mid;
    } else {
      x_left = x_mid;
      f_left = f_mid;
    }
  }

  // Recompute the midpoint.
  return std::make_pair((x_left + x_right)/2, false);
}

}  // namespace systems
}  // namespace drake
