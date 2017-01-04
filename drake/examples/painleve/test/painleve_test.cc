#include "drake/examples/painleve/painleve.h"
#include "drake/systems/analysis/simulator.h"

#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace painleve {
namespace {

/// Class for testing the Painleve Paradox example using a piecewise DAE
/// approach.
class PainleveDAETest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<Painleve<double>>();
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

  systems::VectorBase<double>* continuous_state() {
    return context_->get_mutable_continuous_state_vector();
  }

  // Sets a secondary initial Painleve configuration.
  void SetSecondInitialConfig() {
    const double half_len = dut_->get_rod_length() / 2;
    const double r22 = std::sqrt(2) / 2;
    systems::ContinuousState<double>& v =
        *context_->get_mutable_continuous_state();

    // This configuration is symmetric to the default Painleve configuration.
    v[0] = -half_len * r22;
    v[1] = half_len * r22;
    v[2] = 3 * M_PI / 4.0;
    v[3] = 1.0;
    v[4] = 0.0;
    v[5] = 0.0;
  }

  std::unique_ptr<Painleve<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

/// Class for testing the Painleve Paradox example using a first order time
/// stepping approach.
class PainleveTimeSteppingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const double dt = 1e-3;
    dut_ = std::make_unique<Painleve<double>>(dt);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
  }

  systems::BasicVector<double>* discrete_state() {
    return context_->get_mutable_discrete_state(0);
  }

  // Sets a secondary initial Painleve configuration.
  void SetSecondInitialConfig() {
    const double half_len = dut_->get_rod_length() / 2;
    const double r22 = std::sqrt(2) / 2;
    auto v = discrete_state()->get_mutable_value();

    // This configuration is symmetric to the default Painleve configuration.
    v(0) = -half_len * r22;
    v(1) = half_len * r22;
    v(2) = 3 * M_PI / 4.0;
    v(3) = 1.0;
    v(4) = 0.0;
    v(5) = 0.0;
  }

  std::unique_ptr<Painleve<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

/// Verify the Painleve configuration occurs.
TEST_F(PainleveDAETest, Inconsistent) {
  EXPECT_THROW(dut_->DoCalcTimeDerivatives(*context_, derivatives_.get()),
               std::runtime_error);
}

// Verify the second Painleve configuration occurs.
TEST_F(PainleveDAETest, Inconsistent2) {
  SetSecondInitialConfig();
  EXPECT_THROW(dut_->DoCalcTimeDerivatives(*context_, derivatives_.get()),
               std::runtime_error);
}

// Verify that the (non-impacting) Painleve configuration does not result in a
// state change.
TEST_F(PainleveDAETest, ImpactNoChange) {
  // Setup state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec));
  dut_->HandleImpact(*context_, &new_cstate);
  for (int i = 0; i < state_dim; ++i)
    EXPECT_EQ(new_cstate[i], (*context_->get_continuous_state())[i]);
}

/// Verify that impacting configuration results in non-impacting configuration.
TEST_F(PainleveDAETest, InfFrictionImpactThenNoImpact) {
  // Setup writable state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec),
                                              state_dim / 2, state_dim / 2, 0);

  // Cause the initial state to be impacting
  (*context_->get_mutable_continuous_state())[4] = -1.0;

  // Set the coefficient of friction to infinite. This forces the Painleve code
  // to go through the first impact path.
  dut_->set_mu_Coulomb(std::numeric_limits<double>::infinity());

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, &new_cstate);
  context_->get_mutable_continuous_state()->SetFrom(new_cstate);

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, &new_cstate);

  // Verify that there is no further change from *another* impact.
  for (int i = 0; i < state_dim; ++i)
    EXPECT_NEAR(new_cstate[i], (*context_->get_continuous_state())[i],
                std::numeric_limits<double>::epsilon());
}

/// Verify that impacting configuration results in non-impacting configuration.
TEST_F(PainleveDAETest, NoFrictionImpactThenNoImpact) {
  // Setup writable state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec),
                                              state_dim / 2, state_dim / 2, 0);

  // Set the initial state to be impacting
  systems::ContinuousState<double>& v =
      *context_->get_mutable_continuous_state();
  v[3] = -1.0;

  // Set the coefficient of friction to zero. This forces the Painleve code
  // to go through the second impact path.
  dut_->set_mu_Coulomb(0.0);

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, &new_cstate);
  context_->get_mutable_continuous_state()->SetFrom(new_cstate);

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, &new_cstate);

  // Verify that there is no further change from *another* impact.
  for (int i = 0; i < state_dim; ++i)
    EXPECT_NEAR(new_cstate[i], (*context_->get_continuous_state())[i],
                std::numeric_limits<double>::epsilon());
}

// Verify that no exceptions thrown for a non-sliding configuration.
TEST_F(PainleveDAETest, NoSliding) {
  const double half_len = dut_->get_rod_length() / 2;
  const double r22 = std::sqrt(2) / 2;
  systems::ContinuousState<double>& v =
      *context_->get_mutable_continuous_state();

  // Set the coefficient of friction to zero (triggering the case on the
  // edge of the friction cone).
  dut_->set_mu_Coulomb(0.0);

  // This configuration has no sliding velocity.
  v[0] = -half_len * r22;
  v[1] = half_len * r22;
  v[2] = 3 * M_PI / 4.0;
  v[3] = 0.0;
  v[4] = 0.0;
  v[5] = 0.0;

  // No exceptions should be thrown.
  EXPECT_NO_THROW(dut_->DoCalcTimeDerivatives(*context_, derivatives_.get()));

  // Set the coefficient of friction to effective no-slip (triggering the
  // case strictly inside the friction cone).
  dut_->set_mu_Coulomb(std::numeric_limits<double>::infinity());

  // No exceptions should be thrown.
  EXPECT_NO_THROW(dut_->DoCalcTimeDerivatives(*context_, derivatives_.get()));
}

// Test multiple (two-point) contact configurations.
TEST_F(PainleveDAETest, MultiPoint) {
  systems::ContinuousState<double>& v =
      *context_->get_mutable_continuous_state();

  // This configuration has no sliding velocity. It should throw no exceptions.
  v[0] = 0;
  v[1] = 0;
  v[2] = 0;
  v[3] = 0.0;
  v[4] = 0.0;
  v[5] = 0.0;
  EXPECT_NO_THROW(dut_->DoCalcTimeDerivatives(*context_, derivatives_.get()));

  // This configuration has sliding velocity. It should throw an exception.
  v[0] = 0;
  v[1] = 0;
  v[2] = 0;
  v[3] = 1.0;
  v[4] = 0.0;
  v[5] = 0.0;
  EXPECT_THROW(dut_->DoCalcTimeDerivatives(*context_, derivatives_.get()),
               std::logic_error);
}

/// Verify that Painleve configuration does not result in a state change.
TEST_F(PainleveDAETest, ImpactNoChange2) {
  SetSecondInitialConfig();

  // Setup state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec));
  dut_->HandleImpact(*context_, &new_cstate);
  for (int i = 0; i < state_dim; ++i)
    EXPECT_EQ(new_cstate[i], (*context_->get_continuous_state())[i]);
}

/// Verify that impacting configuration results in non-impacting configuration.
TEST_F(PainleveDAETest, InfFrictionImpactThenNoImpact2) {
  SetSecondInitialConfig();

  // Setup writable state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec),
                                              state_dim / 2, state_dim / 2, 0);

  // Cause the initial state to be impacting
  (*context_->get_mutable_continuous_state())[4] = -1.0;

  // Set the coefficient of friction to infinite. This forces the Painleve code
  // to go through the first impact path.
  dut_->set_mu_Coulomb(std::numeric_limits<double>::infinity());

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, &new_cstate);
  context_->get_mutable_continuous_state()->SetFrom(new_cstate);

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, &new_cstate);

  // Verify that there is no further change from *another* impact.
  for (int i = 0; i < state_dim; ++i)
    EXPECT_NEAR(new_cstate[i], (*context_->get_continuous_state())[i],
                std::numeric_limits<double>::epsilon());
}

/// Verify that impacting configuration results in non-impacting configuration.
TEST_F(PainleveDAETest, NoFrictionImpactThenNoImpact2) {
  SetSecondInitialConfig();

  // Setup writable state.
  const int state_dim = 6;
  auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
  systems::ContinuousState<double> new_cstate(std::move(cstate_vec),
                                              state_dim / 2, state_dim / 2, 0);

  // Cause the initial state to be impacting
  (*context_->get_mutable_continuous_state())[4] = -1.0;

  // Set the coefficient of friction to zero. This forces the Painleve code
  // to go through the second impact path.
  dut_->set_mu_Coulomb(0.0);

  // Handle the impact and copy the result to the context.
  dut_->HandleImpact(*context_, &new_cstate);
  context_->get_mutable_continuous_state()->SetFrom(new_cstate);

  // Do one more impact- there should now be no change.
  dut_->HandleImpact(*context_, &new_cstate);

  // Verify that there is no further change from *another* impact.
  for (int i = 0; i < state_dim; ++i)
    EXPECT_NEAR(new_cstate[i], (*context_->get_continuous_state())[i],
                std::numeric_limits<double>::epsilon());
}

/// Checks that the Painleve Paradox does not occur as the result of an impact.
/*
TEST_F(PainleveDAETest, NoPainleveParadoxAfterImpact) {
  for (int i=0; i< 100000; ++i) {
    SetSecondInitialConfig();

    // Cause the initial state to be impacting
    (*context_->get_mutable_continuous_state())[3] = (double) rand()/RAND_MAX*2
        -1;
    (*context_->get_mutable_continuous_state())[4] = -(double) rand()/RAND_MAX;
    (*context_->get_mutable_continuous_state())[5] = (double) rand()/RAND_MAX*2
        -1;

    // Handle the impact and copy the result to the context.
    const int state_dim = 6;
    auto cstate_vec = std::make_unique<systems::BasicVector<double>>(state_dim);
    systems::ContinuousState<double> new_cstate(std::move(cstate_vec),
                                             state_dim / 2, state_dim / 2, 0);
    dut_->HandleImpact(*context_, &new_cstate);
    context_->get_mutable_continuous_state()->SetFrom(new_cstate);

    // Verify there is no inconsistent configuration remaining.
    EXPECT_NO_THROW(dut_->CalcTimeDerivatives(*context_, derivatives_.get()));
  }
}
*/

/// Verify that Painleve Paradox system can be effectively simulated using
/// first-order time stepping approach.
TEST_F(PainleveTimeSteppingTest, TimeStepping) {
  // Set the initial state to an inconsistent configuration.
  SetSecondInitialConfig();

  // Init the simulator.
  systems::Simulator<double> simulator(*dut_, std::move(context_));

  // Integrate forward to a point where the rod should be at rest.
  const double t_final = 2.5;
  simulator.StepTo(t_final);

  // Get angular orientation and velocity.
  const auto v = simulator.get_context().get_discrete_state(0)->get_value();
  const double theta = v(2);
  const double theta_dot = v(5);

  // After sufficiently long, theta should be 0 or M_PI and the velocity
  // should be nearly zero.
  EXPECT_TRUE(std::fabs(theta) < 1e-6 || std::fabs(theta - M_PI) < 1e-6);
  EXPECT_NEAR(theta_dot, 0.0, 1e-6);

  // TODO(edrumwri): Introduce more extensive tests that cross-validates the
  // time-stepping based approach against the piecewise DAE-based approach.
}


}  // namespace
}  // namespace painleve
}  // namespace drake
