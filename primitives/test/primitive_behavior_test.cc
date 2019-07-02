#include <DR/primitives/primitive_behavior.h>

#include <gtest/gtest.h>

#include <drake/multibody/benchmarks/acrobot/make_acrobot_plant.h>
#include <drake/systems/analysis/simulator.h>

using drake::multibody::benchmarks::acrobot::AcrobotParameters;
using drake::multibody::benchmarks::acrobot::MakeAcrobotPlant;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::VectorX;

namespace DR {
namespace {

// A plan consisting of sinusoidal motion.
class SimplePlan : public Plan<double> {
 public:
  SimplePlan(double start_time, double end_time) : start_time_(start_time), end_time_(end_time) { }
  const double& start_time() const final { return start_time_; }
  const double& end_time() const final { return end_time_; }
  VectorX<double> Evaluate(const double& t) const final {
    VectorX<double> val(1);
    val[0] = std::cos(t - start_time_);
    return val; 
  } 

 private:
  std::unique_ptr<Plan<double>> DoClone() const final {
    return std::make_unique<SimplePlan>(start_time_, end_time_);
  }

  double start_time_{};
  double end_time_{};
};

// A simple derived behavior.
class DerivedBehavior : public PrimitiveBehavior<double> {
 public:
  DerivedBehavior(const MultibodyPlant<double>* plant, double freq, double planning_time, double execution_time) :
      PrimitiveBehavior(plant, freq), planning_time_(planning_time), execution_time_(execution_time) { }

 private:
  std::unique_ptr<Plan<double>> DoComputePlan(const Context<double>& context) const final {
    // Sleep to make it seem like the planning takes some time.
    sleep(planning_time_);

    // Return the simple plan.
    return std::make_unique<SimplePlan>(context.get_time(), context.get_time() + execution_time_);
  }

  void DoCalcControlOutput(const Context<double>& context, BasicVector<double>* output) const final {
    ASSERT_EQ(output->size(), 1);

    // Get the active plan, if any.
    const Plan<double>* plan = this->active_plan(context);
    if (plan) {
      output->SetFromVector(plan->Evaluate(context.get_time()));
    } else {
      (*output)[0] = 123.0;  // Set the output to a unique value.
    }
  }

  double planning_time_{};
  double execution_time_{};
};

class PrimitiveBehaviorTest : public ::testing::Test {

 public:
  const MultibodyPlant<double>& plant() const { return *plant_; }

 protected:
  void SetUp() override {
    // Load in the MultibodyPlant for an Acrobot.
    AcrobotParameters default_parameters;
    plant_ = MakeAcrobotPlant(default_parameters, true /* build and finalize */);

    // Create the Context.
    context_ = plant_->CreateDefaultContext();
  }

 private:
  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<Context<double>> context_;
};

// Verifies that the control frequency is set as commanded.
TEST_F(PrimitiveBehaviorTest, Freq) {
  // Construct the behavior without any planning or execution time and with
  // a unique frequency.
  const double freq = 123.0;          // Hz.
  const double planning_time = 0.0;   // Time in seconds.
  const double execution_time = 0.0;  // Time in seconds.
  DerivedBehavior behavior(&plant(), freq, planning_time, execution_time);
  EXPECT_EQ(behavior.copy_freq(), freq);
}

// Verifies that the output ports can be queried.
TEST_F(PrimitiveBehaviorTest, Ports) {
  const double planning_time = 0.0;   // Time in seconds.
  const double execution_time = 0.0;  // Time in seconds.
  DerivedBehavior behavior(&plant(), 1000 /* Hz */, planning_time, execution_time);
  std::unique_ptr<Context<double>> context = behavior.CreateDefaultContext();
  context->FixInputPort(behavior.operation_signal_input_port().get_index(),
      drake::AbstractValue::Make(PrimitiveBehavior<double>::kActive));
  drake::Value<BasicVector<double>> output(plant().num_actuators());
  ASSERT_NO_THROW(behavior.generalized_effort_output_port().Calc(*context, &output)); 
  behavior.generalized_effort_output_port().Eval(*context);

  // TODO(edrumwri): Determine why this is necessary and eliminate it. 
  // Wait until planning is complete.
  behavior.BlockOnPlanningThreadTermination();
}

// Verifies that the MultibodyPlant can be queried.
TEST_F(PrimitiveBehaviorTest, QueryPlant) {
  const double planning_time = 0.0;   // Time in seconds.
  const double execution_time = 0.0;  // Time in seconds.
  DerivedBehavior behavior(&plant(), 1000 /* Hz */, planning_time, execution_time);

  // There are no exceptions thrown by this method currently- this statement 
  // is just communicating that we want to check that this method is callable
  // without problems.
  EXPECT_NO_THROW(behavior.robot_plant());
}

// Verifies that the control output is zero if the behavior is inactive.
TEST_F(PrimitiveBehaviorTest, ControlOutputZeroOnInactiveBehavior) {
  // Ensure that, if the planner starts, we will identify that before it
  // terminates.
  const double planning_time = 1.0;   // Time in seconds.
  const double execution_time = 0.0;  // Time in seconds.
  DerivedBehavior behavior(&plant(), 1000 /* Hz */, planning_time, execution_time);
  std::unique_ptr<Context<double>> context = behavior.CreateDefaultContext();
  context->FixInputPort(behavior.operation_signal_input_port().get_index(),
      drake::AbstractValue::Make(PrimitiveBehavior<double>::kInactive));
  const Eigen::VectorBlock<const VectorX<double>> output = behavior.generalized_effort_output_port().Eval(*context);
  ASSERT_EQ(output.size(), 1);
  EXPECT_EQ(output[0], 0.0); 

  // Verify that the behavior never started planning.
  EXPECT_FALSE(behavior.is_planning(*context));
}

// Verifies that no plan is active before planning has started.
TEST_F(PrimitiveBehaviorTest, InitialPlanningStatus) {
  // Construct the behavior without any planning or execution time. 
  const double planning_time = 0.0;   // Time in seconds.
  const double execution_time = 0.0;  // Time in seconds.
  DerivedBehavior behavior(&plant(), 1000 /* Hz */, planning_time,
      execution_time);
  auto context = behavior.CreateDefaultContext();

  EXPECT_EQ(behavior.active_plan(*context), nullptr);
}	

// Verifies that planning is active when planning is expected.
TEST_F(PrimitiveBehaviorTest, PlanningInProgressStatus) {
  // Construct the behavior to ensure that planning lasts long enough. The
  // value below (in seconds) may need to be adjusted upward to pass CI.
  const double planning_time = 1.0;
  const double execution_time = 0.0;  // Time in seconds.
  DerivedBehavior behavior(&plant(), 1000 /* Hz */, planning_time,
      execution_time);

  // Trigger the planning process by evaluating the control output.
  std::unique_ptr<Context<double>> context = behavior.CreateDefaultContext();
  context->FixInputPort(behavior.operation_signal_input_port().get_index(),
      drake::AbstractValue::Make(PrimitiveBehavior<double>::kActive));
  behavior.generalized_effort_output_port().Eval(*context); 

  // Verify the status.
  EXPECT_TRUE(behavior.is_planning(*context));
  EXPECT_EQ(behavior.active_plan(*context), nullptr);

  // Wait until planning is complete.
  behavior.BlockOnPlanningThreadTermination();
}

// Verify that completed plans are copied to the State and that planning can occur when there is an active plan in the
// State.
TEST_F(PrimitiveBehaviorTest, PlanningComplete) {
  // The copy rate, in Hz.
  const double copy_rate = 1000.0;

  // Plan for a minimal period of time; execute for the maximum period of time.
  const double planning_time = 0;
  const double execution_time = std::numeric_limits<double>::max();
  DerivedBehavior behavior(
      &plant(), copy_rate, planning_time, execution_time); 

  // Trigger the planning process by evaluating the control output.
  std::unique_ptr<Context<double>> context = behavior.CreateDefaultContext();
  context->FixInputPort(behavior.operation_signal_input_port().get_index(),
      drake::AbstractValue::Make(PrimitiveBehavior<double>::kActive));
  behavior.generalized_effort_output_port().Eval(*context); 

  // Wait until planning is complete.
  behavior.BlockOnPlanningThreadTermination();

  // Simulate forward in time by a single iteration to allow the plan to be
  // copied to the behavior state.
  const Context<double>* raw_context = context.get();
  drake::systems::Simulator<double> simulator(behavior, std::move(context));
  simulator.AdvanceTo(1.0/copy_rate);

  // Verify the planning status and that the plan contains the expected
  // execution time.
  const Plan<double>* plan = behavior.active_plan(*raw_context);
  ASSERT_NE(plan, nullptr);
  EXPECT_EQ(plan->end_time(), std::numeric_limits<double>::max());
}

// Verify that the control output is consistent with an active plan.
TEST_F(PrimitiveBehaviorTest, ControlExpected) {
  // The copy rate, in Hz.
  const double copy_rate = 1000.0;

  // Plan for one second (at a time); execute for the maximum period of time.
  const double planning_time = 1.0;
  const double execution_time = std::numeric_limits<double>::max();
  DerivedBehavior behavior(
      &plant(), copy_rate, planning_time, execution_time); 

  // Trigger the planning process by evaluating the control output.
  std::unique_ptr<Context<double>> context = behavior.CreateDefaultContext();
  context->FixInputPort(behavior.operation_signal_input_port().get_index(),
      drake::AbstractValue::Make(PrimitiveBehavior<double>::kActive));
  behavior.generalized_effort_output_port().Eval(*context); 

  // Wait until planning is complete.
  behavior.BlockOnPlanningThreadTermination();

  // Simulate forward to one second in time. 
  const Context<double>* raw_context = context.get();
  drake::systems::Simulator<double> simulator(behavior, std::move(context));
  simulator.AdvanceTo(1.0);

  // Verify the planning status.
  const Plan<double>* plan = behavior.active_plan(*raw_context);
  EXPECT_NE(plan, nullptr);

  // Verify that the output is what we expect when a plan is active.
  Eigen::VectorBlock<const VectorX<double>> new_output = behavior.generalized_effort_output_port().Eval(*raw_context); 
  EXPECT_EQ(new_output[0], std::cos(raw_context->get_time() - plan->start_time()));

  // Verify that the behavior is planning again.
  EXPECT_TRUE(behavior.is_planning(*raw_context));

  // Wait until planning is complete.
  behavior.BlockOnPlanningThreadTermination();
}

}  // namespace
}  // namespace DR

