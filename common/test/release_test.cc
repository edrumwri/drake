// This file tests whether the debugging tools in this project work as expected when in release mode.
// Force this source file into release mode.
#ifndef NDEBUG
#define NDEBUG
#define NDEBUG_DEF 1
#endif

#ifndef NDEBUG
#error "This file must be compiled in release mode"
#endif

#include <DR/common/exception.h>

#include <exception> /* runtime_error, logic_error */

#include <gtest/gtest.h>

namespace DR {
TEST(ReleaseDebugToolsTest, AssertFalse) {
  // Assert should NOT result in death when in release mode.
  EXPECT_NO_THROW({ assert(false); });
  EXPECT_NO_THROW({ DRAKE_ASSERT(false); });
  EXPECT_NO_THROW({ DR_ASSERT(false); });
  EXPECT_NO_THROW({ DR_ASSERT(false, "Message"); });
}
TEST(ReleaseDebugToolsTest, DemandFalse) {
  // Demand should result in death when in release mode.
  EXPECT_DEATH({ DRAKE_DEMAND(false); }, ".*");
  EXPECT_DEATH({ DR_DEMAND(false); }, ".*");
  EXPECT_DEATH({ DR_DEMAND(false, "Message"); }, ".*");
}
TEST(ReleaseDebugToolsTest, SanityCheck) {
  EXPECT_EXIT({ exit(0); }, ::testing::ExitedWithCode(0), ".*");
  EXPECT_EXIT({ exit(1); }, ::testing::ExitedWithCode(1), ".*");
  EXPECT_DEATH({ std::abort(); }, ".*");
  EXPECT_THROW({ throw std::exception(); }, std::exception);
  EXPECT_THROW({ throw std::runtime_error("runtime_error"); }, std::runtime_error);
  EXPECT_THROW({ throw std::logic_error("logic_error"); }, std::logic_error);
}
TEST(ReleaseDebugToolsTest, NoExit) {
  EXPECT_NO_THROW(assert(true));
  EXPECT_NO_THROW(DR_ASSERT(true));
  EXPECT_NO_THROW(DR_ASSERT(true, "Message"));
  EXPECT_NO_THROW(DR_DEMAND(true));
  EXPECT_NO_THROW(DR_DEMAND(true, "Message"));
}
}  // namespace DR

#ifdef NDEBUG_DEF
#undef NDEBUG
#endif
