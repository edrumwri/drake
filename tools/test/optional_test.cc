#include <DR/tools/optional.h>

#include <gtest/gtest.h>

namespace DR {

// Tests GetOptionalValueOrDefault for optional maps with a desired key.
TEST(OptionalTest, MapWithDefault) {
  // Init optional map.
  std::map<std::string, int> test_map{{"one", 1}, {"two", 2}};
  std::optional<std::map<std::string, int>> optional_map = test_map;

  // Set default value.
  int default_value = 0;

  // Key in map, gets value in map.
  EXPECT_EQ(GetOptionalValueOrDefault(optional_map, std::string("one"), default_value), optional_map.value().at("one"));

  // Key not in map, gets default.
  EXPECT_EQ(GetOptionalValueOrDefault(optional_map, std::string("three"), default_value), default_value);

  // Optional map does not have value, gets default.
  EXPECT_EQ(GetOptionalValueOrDefault({}, std::string("one"), default_value), default_value);
}

// Tests GetOptionalValueOrDefault for optional values.
TEST(OptionalTest, ValueWithDefault) {
  // Init optional value.
  std::optional<int> optional_value{1};

  // Set default value.
  int default_value = 0;

  // Optional has value, gets optional value.
  EXPECT_EQ(GetOptionalValueOrDefault(optional_value, default_value), optional_value.value());

  // Optional does not have value, gets default value.
  EXPECT_EQ(GetOptionalValueOrDefault({}, default_value), default_value);
}

}  // namespace DR
