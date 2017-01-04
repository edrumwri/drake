#include <cmath>
#include "drake/systems/analysis/univariate_root_finder.h"
#include "gtest/gtest.h"


namespace drake {
namespace systems {
namespace {

GTEST_TEST(UnivariateRootFinderTest, CubicFunction) {
  // Setup a lambda function that computes a cubic function.
  std::function<double(double)> cub_poly = [](double t) { return t*t*t; };

  // Test bisection with symmetric tolerance
  const double symm_tol = 1e-6;
  std::pair<double, bool> result =
      UnivariateRootFinder<double>::FindRootWithBisection(cub_poly,
                                                          -1.0, // left bracket
                                                          4.0,  // right bracket
                                                          -symm_tol, symm_tol);
  EXPECT_TRUE(result.second);
  EXPECT_NEAR(cub_poly(result.first), 0.0, symm_tol);

  // Test bisection with asymmetric tolerance
  const double asymm_tol_left = 0.0;
  const double asymm_tol_right = 1e-6;
  result = UnivariateRootFinder<double>::FindRootWithBisection(cub_poly,
                                                      -1.0, // left bracket
                                                      4.0,  // right bracket
                                                      asymm_tol_left,
                                                      asymm_tol_right);

  EXPECT_TRUE(result.second);
  EXPECT_GE(cub_poly(result.first), asymm_tol_left);
  EXPECT_LE(cub_poly(result.first), asymm_tol_right);
}

}  // namespace
}  // namespace systems
}  // namespace drake
