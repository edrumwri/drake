#include "drake/examples/fibonacci/fibonacci.h"

#include <memory>
#include <sstream>

#include <gtest/gtest.h>

#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace examples {
namespace fibonacci {
namespace {

// Verify that we get the right sequence for one sequence length.
GTEST_TEST(Fibonacci, CheckSequence) {
  std::ostringstream out;
  FibonacciDifferenceEquation fibonacci(out);

  systems::Simulator<double> simulator(fibonacci);

  // Set the initial conditions.
  fibonacci.Initialize(&simulator.get_mutable_context());

  // Simulate forward to fibonacci(6): 0 1 1 2 3 5 8
  simulator.StepTo(6 * FibonacciDifferenceEquation::kPeriod);

  EXPECT_EQ(out.str(), "0: 0\n1: 1\n2: 1\n3: 2\n4: 3\n5: 5\n6: 8\n");
}

}  // namespace
}  // namespace fibonacci
}  // namespace examples
}  // namespace drake
