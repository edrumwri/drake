// This file tests whether the debugging tools in this project work as expected when built in debug mode.
// Force this source file into debug mode.
#ifdef NDEBUG
#undef NDEBUG
#define NDEBUG_UNDEF 1
#endif

#include <DR/common/exception.h>

#include <gtest/gtest.h>

namespace DR {
TEST(DebugToolsTest, AssertFalse) {
  EXPECT_DEATH({ assert(false); }, ".*");
  EXPECT_DEATH({ DRAKE_ASSERT(false); }, ".*");
  EXPECT_DEATH({ DR_ASSERT(false); }, ".*");
  EXPECT_DEATH({ DR_ASSERT(false, "Message"); }, ".*");
}
TEST(DebugToolsTest, DemandFalse) {
  EXPECT_DEATH({ DRAKE_DEMAND(false); }, ".*");
  EXPECT_DEATH({ DR_DEMAND(false); }, ".*");
  EXPECT_DEATH({ DR_DEMAND(false, "Message"); }, ".*");
}
TEST(DebugToolsTest, SanityCheck) {
  EXPECT_EXIT({ exit(0); }, ::testing::ExitedWithCode(0), ".*");
  EXPECT_EXIT({ exit(1); }, ::testing::ExitedWithCode(1), ".*");
  EXPECT_DEATH({ std::abort(); }, ".*");
  EXPECT_THROW({ throw std::exception(); }, std::exception);
  EXPECT_THROW({ throw std::runtime_error("runtime_error"); }, std::runtime_error);
  EXPECT_THROW({ throw std::logic_error("logic_error"); }, std::logic_error);
}
TEST(DebugToolsTest, NoExit) {
  EXPECT_NO_THROW(assert(true));
  EXPECT_NO_THROW(DR_ASSERT(true));
  EXPECT_NO_THROW(DR_ASSERT(true, "Message"));
  EXPECT_NO_THROW(DR_DEMAND(true));
  EXPECT_NO_THROW(DR_DEMAND(true, "Message"));
}
}  // namespace DR

#ifdef NDEBUG_UNDEF
#define NDEBUG
#endif
