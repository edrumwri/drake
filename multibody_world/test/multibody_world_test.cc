#include "drake/multibody_world/multibody_world.h"

#include <utility>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "drake/common/find_resource.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

using drake::multibody::parsing::AddModelFromSdfFile;

namespace drake {
namespace geometry {
namespace internal {

class MultibodyWorldClassTest : public ::testing::Test {
 protected:
  void SetUp() {}

  MultibodyWorld<double> mbp_sg_;
};

// Verifies that attempting to use the system (by accessing a port) before it is
// finalized is not allowed.
TEST_F(MultibodyWorldClassTest, NoFinalizeYieldsFailure) {
  ASSERT_FALSE(mbp_sg_.is_finalized());

  // We check only the pose bundle output; others are dependent on *how* the
  // plant has been set up.
  EXPECT_THROW(mbp_sg_.get_pose_bundle_output_port(), std::logic_error);
}

// Verifies that attempting to finalize the system without registering
// geometries results in an error.
TEST_F(MultibodyWorldClassTest, FinalizeWithoutRegisteringGeomsFails) {
  EXPECT_THROW(mbp_sg_.Finalize(), std::logic_error);
}

// Tests that finalizing allows the system to be used.
// finalized is not allowed.
TEST_F(MultibodyWorldClassTest, FinalizeEqualsSuccess) {
  auto& cart_pole = mbp_sg_.mutable_multibody_plant();

  // Make and add the cart_pole model.
  const std::string full_name = FindResourceOrThrow(
      "drake/examples/multibody/cart_pole/cart_pole.sdf");
  AddModelFromSdfFile(
      full_name, &cart_pole, &mbp_sg_.mutable_scene_graph());

  // Add gravity to the model.
  cart_pole.AddForceElement<multibody::UniformGravityFieldElement>(
      -9.81 * Vector3<double>::UnitZ());

  // Now the model is complete.
  mbp_sg_.Finalize();

  ASSERT_TRUE(mbp_sg_.is_finalized());

  // We check only the pose bundle output; others are dependent on *how* the
  // plant has been set up.
  EXPECT_NO_THROW(mbp_sg_.get_pose_bundle_output_port());
}

// Tests that attempting to use the system (by accessing a port) after it is
// finalized is allowed.

// Tests that connecting to DrakeVisualizer works as expected.

// Tests that getting the MultibodyPlant context works.

//

// Tests that AutoDiff construction is possible.
GTEST_TEST(MultibodyWorldTest, AutoDiff) {
  EXPECT_NO_THROW(MultibodyWorld<AutoDiffXd>());
}


}  // namespace internal
}  // namespace geometry
}  // namespace drake
