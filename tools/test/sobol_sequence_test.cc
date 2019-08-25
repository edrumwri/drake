#include <DR/tools/sobol_sequence.h>

#include <gtest/gtest.h>

using drake::VectorX;
using Vector2d = drake::Vector2<double>;

namespace DR {
namespace {

// Tests the Sobol sequence in one dimension.
GTEST_TEST(SobolSequence, OneD) {
  SobolSequence<double> s(1);
  EXPECT_EQ(s.Sample()[0], 0.5);
  EXPECT_EQ(s.Sample()[0], 0.25);
  EXPECT_EQ(s.Sample()[0], 0.75);
  EXPECT_EQ(s.Sample()[0], 0.375);
  EXPECT_EQ(s.Sample()[0], 0.875);
}

// Tests the Sobol sequence in two dimensions.
GTEST_TEST(SobolSequence, TwoD) {
  SobolSequence<double> s(2);
  EXPECT_EQ(s.Sample(), Vector2d(0.5, 0.5));
  EXPECT_EQ(s.Sample(), Vector2d(0.25, 0.75));
  EXPECT_EQ(s.Sample(), Vector2d(0.75, 0.25));
  EXPECT_EQ(s.Sample(), Vector2d(0.375, 0.625));
  EXPECT_EQ(s.Sample(), Vector2d(0.875, 0.125));
}

}  // namespace
}  // namespace DR

