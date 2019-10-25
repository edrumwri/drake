#include <DR/simulation/container_unloading/parcel_stack_generator.h>

#include <DR/common/environment.h>
#include <DR/simulation/config.h>

#include <gtest/gtest.h>

#include <drake/common/random.h>

#include "../../test/body_simulation_driver.h"

namespace DR {
// Clearance between the floor and the parcel stack at simulation start.
const double kInitialGroundClearance = 0.001;

// Number of stacks to a side of the square grid in test `CombinedStack`.
const int kCombinedStackSize = 2;

// Interval of time to simulate to check whether stack is simulable.
const double kSimulationTime = 0.01;

// Permit a maximum maximum permissible interpenetration depth.
// NOTE: `SimulationFacade` sets the penetration allowance to `kDefaultSimulatorParameters.target_accuracy`.
const double kMaxPenetrationDepth = kDefaultSimulatorParameters.target_accuracy;

// Length, width, height of stack in meters, cube size is comparable to the width and height of a truck trailer.
const double kStackWidth = 2.0;
const double kStackLength = 2.4;
const double kStackHeight = 2.0;
const double kGripperWidth = 0.0;
const drake::Vector3<double> kStackSize{kStackLength, kStackWidth, kStackHeight};

// Thickness (z dimension) of static box representing floor.
const double kFloorThickness = 0.1;

/*
 The ParcelStackGeneratorTest creates a stack of boxes then uses the UnloadingTaskDriver to simulate them.  This test
 focuses on testing whether the parcel stack conforms to its described constraints:
 - No bodies intersect one another within a single stack.
 - No bodies belonging to a stack will leave the area bounded by the stack dimensions and intersect adjacent stacks.
*/
template <typename T>
class ParcelStackGeneratorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::string absolute_model_path = GetModelDirectoryFromEnvironment();
    parcel_stack_generator_ = std::make_unique<ParcelStackGenerator>(absolute_model_path);

    // Set distribution parameters.
    parcel_stack_generator_->set_parcel_attribute_range(ParcelStackGenerator::AttributeType::kPoseTranslation,
                                                        drake::Vector3<double>{-0.01, -0.01, -0.0},
                                                        drake::Vector3<double>{0.01, 0.01, 0.0});
    parcel_stack_generator_->set_parcel_attribute_range(ParcelStackGenerator::AttributeType::kPoseYawRotation,
                                                        drake::Vector1<double>{-0.1}, drake::Vector1<double>{0.1});
    parcel_stack_generator_->set_parcel_attribute_range(ParcelStackGenerator::AttributeType::kSize,
                                                        drake::Vector3<double>{0.3, 0.3, 0.1},
                                                        drake::Vector3<double>{0.67, 0.46, 0.5});
    parcel_stack_generator_->set_parcel_attribute_range(ParcelStackGenerator::AttributeType::kColor,
                                                        drake::Vector4<double>{0.7, 0.45, 0.0, 1.0},
                                                        drake::Vector4<double>{0.9, 0.65, 0.2, 1.0});
    parcel_stack_generator_->set_parcel_attribute_range(ParcelStackGenerator::AttributeType::kCoulombFriction,
                                                        drake::Vector2<double>{0.5, 0.25},
                                                        drake::Vector2<double>{1.0, 0.5});
    parcel_stack_generator_->set_parcel_attribute_range(ParcelStackGenerator::AttributeType::kMass,
                                                        drake::Vector1<double>{1.0}, drake::Vector1<double>{40.0});
    std::vector<double> parcel_type_distribution_weights(static_cast<int>(ParcelGeometries::kNumTypes));
    parcel_type_distribution_weights[static_cast<int>(ParcelGeometries::kBox)] = 0.80;
    parcel_type_distribution_weights[static_cast<int>(ParcelGeometries::kMailTote)] = 0.10;
    parcel_type_distribution_weights[static_cast<int>(ParcelGeometries::kCarTire)] = 0.10;
    parcel_stack_generator_->set_parcel_type_distribution_weights(parcel_type_distribution_weights);
  }

  double CalcMaxInterpenetration() {
    // Check pairwise distances.
    std::unique_ptr<drake::AbstractValue> state_value = driver().scene_graph().get_query_output_port().Allocate();
    EXPECT_NO_THROW(state_value->template get_value<drake::geometry::QueryObject<T>>());
    const drake::geometry::QueryObject<T>& query_object =
        state_value->template get_value<drake::geometry::QueryObject<T>>();

    const auto& scene_graph_context = driver().diagram().GetSubsystemContext(
        driver().scene_graph(), driver().simulator(simulator_instance_index_).get_context());

    driver().scene_graph().get_query_output_port().Calc(scene_graph_context, state_value.get());

    std::vector<drake::geometry::PenetrationAsPointPair<double>> point_pair_penetration =
        query_object.ComputePointPairPenetration();

    double max_depth = 0.0;
    // There should be no interpenetration in the stack.
    for (const auto& p : point_pair_penetration) {
      if (p.depth >= kMaxPenetrationDepth) {
        std::cout << "Bodies in the generated parcel stack intersect at initialization!" << std::endl;
        std::cout << "p.depth = " << p.depth << std::endl;
        std::cout << "p.normal * p.depth = " << (p.depth * p.nhat_BA_W).transpose() << std::endl;
        std::cout << "p.p_WCb = " << p.p_WCb.transpose() << std::endl;
        std::cout << "p.p_WCa = " << p.p_WCa.transpose() << std::endl;
      }
      if (p.depth > max_depth) max_depth = p.depth;
    }
    return max_depth;
  }

  void CheckIsSimulable() {
    // Get simulator instance and advance to the target time.
    ASSERT_NO_THROW(driver().simulator(simulator_instance_index_).AdvanceTo(kSimulationTime));
  }

  void Build(std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>>&& bodies) {
    // Test SimulatorFacade derived class.
    ASSERT_NO_THROW(driver_ = std::make_unique<BodySimulationDriver<T>>(std::move(bodies)));

    // Check build.
    ASSERT_NO_THROW(simulator_instance_index_ = driver().CreateSimulatedSystem());

    // Get simulator instance and advance to the target time.
    ASSERT_NO_THROW(driver().simulator(simulator_instance_index_).Initialize());
  }

  ParcelStackGenerator& parcel_stack_generator() { return *parcel_stack_generator_.get(); }
  BodySimulationDriver<T>& driver() { return *driver_.get(); }

 private:
  SimulatorInstanceIndex simulator_instance_index_{};

  std::unique_ptr<BodySimulationDriver<T>> driver_;
  std::unique_ptr<ParcelStackGenerator> parcel_stack_generator_;
};

#if GTEST_HAS_TYPED_TEST

using testing::Types;

typedef Types<double> Implementations;

TYPED_TEST_SUITE(ParcelStackGeneratorTest, Implementations);

/*
 Test to see whether a stack of non-intersecting floating bodies can be created.
 */
TYPED_TEST(ParcelStackGeneratorTest, CheckMaxDepth) {
  std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> bodies;

  // Static body of floor (large box geometry).
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("static_box_floor");
    body->set_mass_and_possibly_make_static(0.0 /* static */);
    body->SetCoulombFriction(0.8, 0.6);
    body->SetBoxGeometry(100.0, 100.0, kFloorThickness);
    body->set_pose(
        drake::math::RigidTransform<double>(drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                            drake::Vector3<double>(0.0, 0.0, -kFloorThickness * 0.5)));
    bodies.insert(std::move(body));
  }

  // Dynamic body on floor.
  {
    const double kCubeSize = 1.0;
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("dynamic_cube");
    body->set_mass_and_possibly_make_static(10.0);
    body->SetCoulombFriction(0.8, 0.6);
    body->SetBoxGeometry(kCubeSize, kCubeSize, kCubeSize);
    body->set_pose(
        drake::math::RigidTransform<double>(drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                            drake::Vector3<double>(0.0, 0.0, kCubeSize * 0.5 - kMaxPenetrationDepth)));
    bodies.insert(std::move(body));
  }

  // Build model, diagram & simulation.
  ASSERT_NO_FATAL_FAILURE(this->Build(std::move(bodies)));
  // Test for self intersection.
  ASSERT_NEAR(this->CalcMaxInterpenetration(), kMaxPenetrationDepth, 1e-7);
  // Test simulation works.
  ASSERT_NO_FATAL_FAILURE(this->CheckIsSimulable());
}

/*
 Test to see whether a stack of non-intersecting floating bodies can be created.
 */
TYPED_TEST(ParcelStackGeneratorTest, ManipulandStack) {
  std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> bodies;
  drake::RandomGenerator random_generator;
  // Place stack origin at world origin.
  drake::math::RigidTransform<double> stack_origin(drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                                   drake::Vector3<double>(0.0, 0.0, kInitialGroundClearance));

  // Add floating bodies (manipulands) to the configuration.
  bodies.merge(this->parcel_stack_generator().FillRegionWithParcels(
      stack_origin, true /* floating bodies */, kStackSize, 0.0 /* add gripper size between boxes */,
      "_stack" /* stack name in body names */, &random_generator));

  // Static body of Floor (large box geometry).
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("static_box_floor");
    body->set_mass_and_possibly_make_static(0.0 /* static */);
    body->SetCoulombFriction(0.8, 0.6);
    body->SetBoxGeometry(100.0, 100.0, kFloorThickness);
    body->set_pose(
        drake::math::RigidTransform<double>(drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                            drake::Vector3<double>(0.0, 0.0, -kFloorThickness * 0.5)));
    bodies.insert(std::move(body));
  }

  // Build model, diagram & simulation.
  ASSERT_NO_FATAL_FAILURE(this->Build(std::move(bodies)));
  // Test for self intersection.
  ASSERT_LT(this->CalcMaxInterpenetration(), kMaxPenetrationDepth);
  // Test simulation works.
  ASSERT_NO_FATAL_FAILURE(this->CheckIsSimulable());
}

/*
 Test to see whether multiple stacks of non-intersecting floating and static bodies can be created.
 */
TYPED_TEST(ParcelStackGeneratorTest, CombinedStack) {
  std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> bodies;
  drake::RandomGenerator random_generator;

  // Arrange multiple stacks in a pattern on the xy-plane.
  // Pattern: ☒ = floating, manipuland stack , ☐ = static, environment body stack
  //
  //        j          |+x (forward)
  //       1 0         |
  // i   1 ☐ ☒    +y---+
  //     0 ☒ ☐  (left)
  //         ↑
  // The world origin (indicated by `↑`) is at the base of the furthest -y and -x stack.
  // NOTE: double-braces required in C++11 prior to the CWG 1270 revision
  // See https://en.cppreference.com/w/cpp/container/array.
  std::array<std::array<bool, kCombinedStackSize>, kCombinedStackSize> stack_is_floating{{
      {{false, true}},
      {{true, false}},
  }};

  for (int i = 0; i < kCombinedStackSize; ++i) {
    for (int j = 0; j < kCombinedStackSize; ++j) {
      // Place each stack in its own footprint.
      drake::math::RigidTransform<double> stack_pose(
          drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
          drake::Vector3<double>(kStackLength * i, kStackWidth * j, kInitialGroundClearance));
      // Unique name of stack according to location in arrangement.
      std::string stack_name = "_stack" + std::to_string(i) + std::to_string(j);
      // Determine which collection of bodies to populate.
      bodies.merge(this->parcel_stack_generator().FillRegionWithParcels(
          stack_pose, stack_is_floating[j /* length-wise */][i /* width-wise */] /* floating manipuland */, kStackSize,
          kGripperWidth, stack_name, &random_generator));
    }
  }

  // Static body of Floor (large box geometry).
  {
    auto body = std::make_unique<SingleBodyInstanceConfig>();
    body->set_name("static_box_floor");
    body->set_mass_and_possibly_make_static(0.0 /* static */);
    body->SetCoulombFriction(0.8, 0.6);
    body->SetBoxGeometry(100.0, 100.0, kFloorThickness);
    body->set_pose(
        drake::math::RigidTransform<double>(drake::math::RollPitchYaw<double>(0.0, 0.0, 0.0).ToRotationMatrix(),
                                            drake::Vector3<double>(0.0, 0.0, -kFloorThickness * 0.5)));
    bodies.insert(std::move(body));
  }

  // Build model, diagram & simulation.
  ASSERT_NO_FATAL_FAILURE(this->Build(std::move(bodies)));
  // Test for self intersection.
  ASSERT_LT(this->CalcMaxInterpenetration(), kMaxPenetrationDepth);
  // Test simulation works.
  ASSERT_NO_FATAL_FAILURE(this->CheckIsSimulable());
}

#endif  // GTEST_HAS_TYPED_TEST

}  // namespace DR
