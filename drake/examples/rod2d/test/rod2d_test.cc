#include "drake/examples/rod2d/rod2d.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"

using drake::systems::VectorBase;
using drake::systems::BasicVector;
using drake::systems::ContinuousState;
using drake::systems::State;
using drake::systems::SystemOutput;
using drake::systems::AbstractValues;
using drake::systems::Simulator;
using drake::systems::SemiExplicitEulerIntegrator;
using drake::systems::Context;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;

namespace drake {
namespace examples {
namespace rod2d {

// Class for testing the Rod2D example using a piecewise DAE
// approach.
class Rod2DDAETest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SimulationType::kPiecewiseDAE, 0.0);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();

    // Use a non-unit mass.
    dut_->set_rod_mass(2.0);

    // Set the cfm to be very small.
    dut_->set_cfm(10 * std::numeric_limits<double>::epsilon());

    // Set a zero input force (this is the default).
    std::unique_ptr<BasicVector<double>> ext_input =
        std::make_unique<BasicVector<double>>(3);
    ext_input->SetAtIndex(0, 0.0);
    ext_input->SetAtIndex(1, 0.0);
    ext_input->SetAtIndex(2, 0.0);
    context_->FixInputPort(0, std::move(ext_input));
  }

  std::unique_ptr<State<double>> CloneState() const {
    return context_->CloneState();
  }

  VectorBase<double> *continuous_state() {
    return context_->get_mutable_continuous_state_vector();
  }

  // Sets the rod to an upright, "resting" state (no initial velocity).
  void SetUprightRestingState() {
    // Set the initial state to sustained contact with zero tangential velocity
    // at the point of contact.
    const double half_len = dut_->get_rod_half_length();
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    xc[0] = 0.0;
    xc[1] = half_len;
    xc[2] = M_PI_2;
    xc[3] = 0.0;
    xc[4] = 0.0;
    xc[5] = 0.0;

    // Prepare to set contact states.
    std::vector<RigidContact>& contacts =
        dut_->get_contacts(context_->get_mutable_state());
    EXPECT_EQ(contacts.size(), 2);

    // First point is Rl in Rod Frame (see class documentation); in this
    // new configuration, it contacts the half-space without sliding.
    contacts[0].state = RigidContact::ContactState::kContactingWithoutSliding;
    contacts[0].mu = dut_->get_mu_coulomb();
    contacts[0].u = Eigen::Vector3d(-dut_->get_rod_half_length(), 0, 0);

    // Second point is Rr in Rod Frame. In this new configuration; in this
    // new configuration, it does not contact the half-space..
    contacts[1].state = RigidContact::ContactState::kNotContacting;
    contacts[1].mu = dut_->get_mu_coulomb();
    contacts[1].u = Eigen::Vector3d(dut_->get_rod_half_length(), 0, 0);
  }

  // Sets a secondary initial Painlevé configuration.
  void SetSecondInitialConfig() {
    // Set the configuration to an inconsistent (Painlevé) type state with
    // the rod at a 135 degree counter-clockwise angle with respect to the
    // x-axis. The rod in [Stewart, 2000] is at a 45 degree counter-clockwise
    // angle with respect to the x-axis.
    // * [Stewart, 2000]  D. Stewart, "Rigid-Body Dynamics with Friction and
    //                    Impact". SIAM Rev., 42(1), 3-39, 2000.
    using std::sqrt;
    const double half_len = dut_->get_rod_half_length();
    const double r22 = std::sqrt(2) / 2;
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    xc[0] = -half_len * r22;
    xc[1] = half_len * r22;
    xc[2] = 3 * M_PI / 4.0;
    xc[3] = 1.0;
    xc[4] = 0.0;
    xc[5] = 0.0;

    // Contact states should be identical to defaults.
  }

  // Sets the rod to a state that corresponds to ballistic motion.
  void SetBallisticState() {
    const double half_len = dut_->get_rod_half_length();
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    xc[0] = 0.0;
    xc[1] = 10 * half_len;
    xc[2] = M_PI_2;
    xc[3] = 1.0;
    xc[4] = 2.0;
    xc[5] = 3.0;

    // Prepare to set the contact mode to ballistic.
    std::vector<RigidContact>& contacts =
        dut_->get_contacts(context_->get_mutable_state());
    EXPECT_EQ(contacts.size(), 2);

    // First point is Rl in Rod Frame (see class documentation); in this
    // new configuration, it does not contact the half-space..
    contacts[0].state = RigidContact::ContactState::kNotContacting;
    contacts[0].mu = dut_->get_mu_coulomb();
    contacts[0].u = Eigen::Vector3d(-dut_->get_rod_half_length(), 0, 0);

    // Second point is Rr in Rod Frame. In this new configuration; in this
    // new configuration, it does not contact the half-space..
    contacts[1].state = RigidContact::ContactState::kNotContacting;
    contacts[1].mu = dut_->get_mu_coulomb();
    contacts[1].u = Eigen::Vector3d(dut_->get_rod_half_length(), 0, 0);
  }

  // Sets the rod to an interpenetrating configuration without modifying the
  // velocity or any mode variables.
  void SetInterpenetratingConfig() {
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    // Configuration has the rod on its side.
    xc[0] = 0.0;    // com horizontal position
    xc[1] = -1.0;   // com vertical position
    xc[2] = 0.0;    // rod rotation
  }

  // Sets the rod to a resting horizontal configuration without modifying the
  // velocity or any mode variables.
  void SetRestingHorizontalConfig() {
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    // Configuration has the rod on its side.
    xc[0] = 0.0;     // com horizontal position
    xc[1] = 0.0;     // com vertical position
    xc[2] = 0.0;     // rod rotation
  }

  // Sets the rod to an arbitrary impacting state.
  void SetImpactingState() {
    // This state is identical to that obtained from SetSecondInitialConfig()
    // but with the vertical component of velocity set such that the state
    // corresponds to an impact.
    SetSecondInitialConfig();
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    xc[4] = -1.0;    // com vertical velocity.
  }

  std::unique_ptr<Rod2D<double>> dut_;  //< The device under test.
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
};

// Checks that the output port represents the state.
TEST_F(Rod2DDAETest, Output) {
  const ContinuousState<double>& xc = *context_->get_continuous_state();
  std::unique_ptr<SystemOutput<double>> output =
      dut_->AllocateOutput(*context_);
  dut_->CalcOutput(*context_, output.get());
  for (int i = 0; i < xc.size(); ++i)
    EXPECT_EQ(xc[i], output->get_vector_data(0)->get_value()(i));
}

// Verifies that setting dut to an impacting state actually results in an
// impacting state.
TEST_F(Rod2DDAETest, ImpactingState) {
  SetImpactingState();
  EXPECT_TRUE(dut_->IsImpacting(context_->get_state()));
}

// Tests parameter getting and setting.
TEST_F(Rod2DDAETest, Parameters) {
  // Set parameters to non-default values.
  const double g = -1.0;
  const double mass = 0.125;
  const double mu = 0.5;
  const double h = 0.03125;
  const double J = 0.25;
  dut_->set_gravitational_acceleration(g);
  dut_->set_rod_mass(mass);
  dut_->set_mu_coulomb(mu);
  dut_->set_rod_half_length(h);
  dut_->set_rod_moment_of_inertia(J);
  EXPECT_EQ(dut_->get_gravitational_acceleration(), g);
  EXPECT_EQ(dut_->get_rod_mass(), mass);
  EXPECT_EQ(dut_->get_mu_coulomb(), mu);
  EXPECT_EQ(dut_->get_rod_half_length(), h);
  EXPECT_EQ(dut_->get_rod_moment_of_inertia(), J);
}

// Verify that impact handling works as expected.
TEST_F(Rod2DDAETest, ImpactWorks) {
  // Cause the initial state to be impacting, with center of mass directly
  // over the point of contact.
  const double half_len = dut_->get_rod_half_length();
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();
  xc[0] = 0.0;
  xc[1] = half_len;
  xc[2] = M_PI_2;
  xc[3] = 0.0;
  xc[4] = -1.0;
  xc[5] = 0.0;

  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

  // Prepare to set contact states.
  std::vector<RigidContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  EXPECT_EQ(contacts.size(), 2);

  // First point is Rl in Rod Frame (see class documentation); in this
  // new configuration, it contacts the half-space without sliding.
  contacts[0].state = RigidContact::ContactState::kContactingWithoutSliding;
  contacts[0].mu = dut_->get_mu_coulomb();
  contacts[0].u = Eigen::Vector3d(-dut_->get_rod_half_length(), 0, 0);

  // Second point is Rr in Rod Frame. In this new configuration; in this
  // new configuration, it does not contact the half-space..
  contacts[1].state = RigidContact::ContactState::kNotContacting;
  contacts[1].mu = dut_->get_mu_coulomb();
  contacts[1].u = Eigen::Vector3d(dut_->get_rod_half_length(), 0, 0);

  // Rod should be impacting.
  EXPECT_TRUE(dut_->IsImpacting(context_->get_state()));

  // Handle the impact.
  dut_->ModelImpact(new_state.get());
  context_->get_mutable_state()->SetFrom(*new_state);

  // Verify that the state has been modified such that the body is no longer
  // in an impacting state and the configuration has not been modified.
  const double tol = 10 * dut_->get_cfm();
  EXPECT_NEAR(xc[0], 0.0, tol);
  EXPECT_NEAR(xc[1], half_len, tol);
  EXPECT_NEAR(xc[2], M_PI_2, tol);
  EXPECT_NEAR(xc[3], 0.0, tol);
  EXPECT_NEAR(xc[4], 0.0, tol);
  EXPECT_NEAR(xc[5], 0.0, tol);
}

// Verify that derivatives match what we expect from a non-inconsistent,
// ballistic configuration.
TEST_F(Rod2DDAETest, ConsistentDerivativesBallistic) {
  // Set the initial state to ballistic motion.
  SetBallisticState();

  // Calculate the derivatives.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // Verify that the derivatives match what we expect for this non-inconsistent
  // ballistic system.
  const double tol = std::numeric_limits<double>::epsilon();
  const double g = dut_->get_gravitational_acceleration();
  const ContinuousState<double>& xc = *context_->get_continuous_state();
  EXPECT_NEAR((*derivatives_)[0], xc[3], tol);  // qdot = v ...
  EXPECT_NEAR((*derivatives_)[1], xc[4], tol);  // ... for this ...
  EXPECT_NEAR((*derivatives_)[2], xc[5], tol);  // ... system.
  EXPECT_NEAR((*derivatives_)[3], 0.0, tol);   // Zero horizontal acceleration.
  EXPECT_NEAR((*derivatives_)[4], g, tol);     // Gravitational acceleration.
  EXPECT_NEAR((*derivatives_)[5], 0.0, tol);   // Zero rotational acceleration.
}

// Verify that derivatives match what we expect from a non-inconsistent
// contacting configuration.
TEST_F(Rod2DDAETest, ConsistentDerivativesContacting) {
  // Calculate the derivatives.
  SetUprightRestingState();
  std::vector<RigidContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  EXPECT_EQ(contacts.size(), 2);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // Verify that derivatives match what we expect from a non-inconsistent
  // contacting configuration. In this case, there is no initial sliding,
  // velocity and the rod is oriented vertically, so we expect no sliding
  // to begin to occur.
  const double tol = 10 * dut_->get_cfm();
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();
  EXPECT_NEAR((*derivatives_)[0], xc[3], tol);
  EXPECT_NEAR((*derivatives_)[1], xc[4], tol);
  EXPECT_NEAR((*derivatives_)[2], xc[5], tol);
  EXPECT_NEAR((*derivatives_)[3], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[4], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[5], 0.0, tol);

  // Set the coefficient of friction to zero, update the sliding velocity,
  // and try again. Derivatives should be exactly the same because no frictional
  // force can be applied.
  xc[3] = -1.0;
  contacts[0].state = RigidContact::ContactState::kContactingAndSliding;
  dut_->set_mu_coulomb(0.0);
  contacts[0].mu = contacts[1].mu = dut_->get_mu_coulomb();
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR((*derivatives_)[0], xc[3], tol);
  EXPECT_NEAR((*derivatives_)[1], xc[4], tol);
  EXPECT_NEAR((*derivatives_)[2], xc[5], tol);
  EXPECT_NEAR((*derivatives_)[3], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[4], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[5], 0.0, tol);

  // Add a large upward force and ensure that the rod accelerates upward.
  const double fup = 100.0;
  std::unique_ptr<BasicVector<double>> ext_input =
      std::make_unique<BasicVector<double>>(3);
  ext_input->SetAtIndex(0, 0.0);
  ext_input->SetAtIndex(1, fup);
  ext_input->SetAtIndex(2, 0.0);
  context_->FixInputPort(0, std::move(ext_input));

  // First, redetermine the acceleration-level active set.
  dut_->DetermineAccelLevelActiveSet(*context_, context_->get_mutable_state());

  // Verify that the contact point is no longer to be considered in contact.
  EXPECT_EQ(contacts[0].state, RigidContact::ContactState::kNotContacting);

  // Now calculate the derivatives.
  const double ydd_computed = dut_->get_gravitational_acceleration() +
      fup/dut_->get_rod_mass();
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR((*derivatives_)[3], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[4], ydd_computed, tol);
  EXPECT_NEAR((*derivatives_)[5], 0.0, tol);
}

// Verify that derivatives match what we expect from a sticking contact
// configuration.
TEST_F(Rod2DDAETest, DerivativesContactingAndSticking) {
  // Set the initial state to sustained contact with zero tangential velocity
  // at the point of contact.
  SetUprightRestingState();
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();

  // Prepare to set contact states.
  std::vector<RigidContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  EXPECT_EQ(contacts.size(), 2);

  // First point is Rl in Rod Frame (see class documentation); in this
  // new configuration, it contacts the half-space without sliding.
  contacts[0].state = RigidContact::ContactState::kContactingWithoutSliding;
  contacts[0].mu = dut_->get_mu_coulomb();
  contacts[0].u = Eigen::Vector3d(-dut_->get_rod_half_length(), 0, 0);

  // Second point is Rr in Rod Frame. In this new configuration; in this
  // new configuration, it does not contact the half-space..
  contacts[1].state = RigidContact::ContactState::kNotContacting;
  contacts[1].mu = dut_->get_mu_coulomb();
  contacts[1].u = Eigen::Vector3d(dut_->get_rod_half_length(), 0, 0);

  // Set a constant horizontal input force, as if applied at the bottom of
  // the rod.
  std::unique_ptr<BasicVector<double>> ext_input =
      std::make_unique<BasicVector<double>>(3);
  const double f_x = 1.0;
  const double f_y = -1.0;
  ext_input->SetAtIndex(0, f_x);
  ext_input->SetAtIndex(1, f_y);
  ext_input->SetAtIndex(2, f_x * dut_->get_rod_half_length());
  const Vector3<double> fext = ext_input->CopyToVector();
  context_->FixInputPort(0, std::move(ext_input));

  // Set the coefficient of friction such that the contact forces are right
  // on the edge of the friction cone. Determine the predicted normal force
  // (this simple formula is dependent upon the upright rod configuration).
  const double mu_stick = f_x / (dut_->get_rod_mass() *
                                 -dut_->get_gravitational_acceleration() -
                                 f_y);
  dut_->set_mu_coulomb(mu_stick);
  contacts[0].mu = contacts[1].mu = dut_->get_mu_coulomb();

  // Calculate the derivatives.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // Verify that derivatives match what we expect: sticking should continue.
  const double tol = 10 * dut_->get_cfm();
  EXPECT_NEAR((*derivatives_)[0], xc[3], tol);
  EXPECT_NEAR((*derivatives_)[1], xc[4], tol);
  EXPECT_NEAR((*derivatives_)[2], xc[5], tol);
  EXPECT_NEAR((*derivatives_)[3], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[4], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[5], 0.0, tol);

  // Set the coefficient of friction to 99.9% of the sticking value and then
  // verify that the contact state transitions from a sticking one to a
  // non-sticking one.
  const double mu_slide = 0.999 * mu_stick;
  dut_->set_mu_coulomb(mu_slide);
  contacts[0].mu = contacts[1].mu = dut_->get_mu_coulomb();

  // First, redetermine the acceleration-level active set.
  dut_->DetermineAccelLevelActiveSet(*context_, context_->get_mutable_state());
  EXPECT_EQ(contacts[0].state, RigidContact::ContactState::
      kContactingAndSliding);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_GT((*derivatives_)[3], tol);  // horizontal accel. should be nonzero.

  // Set the coefficient of friction to zero and try again.
  dut_->set_mu_coulomb(0.0);
  contacts[0].mu = contacts[1].mu = dut_->get_mu_coulomb();
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR((*derivatives_)[0], xc[3], tol);
  EXPECT_NEAR((*derivatives_)[1], xc[4], tol);
  EXPECT_NEAR((*derivatives_)[2], xc[5], tol);
  // Rod should now accelerate in the direction of any external forces.
  EXPECT_NEAR((*derivatives_)[3], fext(0)/dut_->get_rod_mass(), tol);
  // There should still be no vertical acceleration.
  EXPECT_NEAR((*derivatives_)[4], 0.0, tol);
  // The moment caused by applying the force should result in a
  // counter-clockwise acceleration.
  EXPECT_NEAR((*derivatives_)[5],
              fext(2)/dut_->get_rod_moment_of_inertia(), tol);
}

// Verify the inconsistent (Painlevé Paradox) configuration occurs.
TEST_F(Rod2DDAETest, Inconsistent) {
  State<double>* state = context_->get_mutable_state();
  EXPECT_THROW(dut_->DetermineAccelLevelActiveSet(*context_, state),
               std::runtime_error);
}

// Verify the second inconsistent (Painlevé Paradox) configuration occurs.
TEST_F(Rod2DDAETest, Inconsistent2) {
  SetSecondInitialConfig();
  State<double>* state = context_->get_mutable_state();
  EXPECT_THROW(dut_->DetermineAccelLevelActiveSet(*context_, state),
               std::runtime_error);
}

// Verify that the (non-impacting) Painlevé configuration does not result in a
// state change.
TEST_F(Rod2DDAETest, ImpactNoChange) {
  // Set state.
  std::unique_ptr<State<double>> new_state = CloneState();
  EXPECT_FALSE(dut_->IsImpacting(context_->get_state()));
  dut_->ModelImpact(new_state.get());
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));

  // Verify that the mode is still sliding.
  const std::vector<RigidContact>& contacts =
      dut_->get_contacts(context_->get_state());
  EXPECT_EQ(contacts.size(), 2);
  EXPECT_TRUE(contacts.front().state ==
      RigidContact::ContactState::kContactingAndSliding ||
      contacts.back().state ==
          RigidContact::ContactState::kContactingAndSliding);
}

// Verify that applying the impact model to an impacting configuration results
// in a non-impacting configuration. This test exercises the model for the case
// where impulses that yield tangential sticking lie within the friction cone.
TEST_F(Rod2DDAETest, InfFrictionImpactThenNoImpact) {
  // Cause the initial state to be impacting.
  SetImpactingState();

  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

  // Verify that the state is in a sliding mode.
  std::vector<RigidContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  EXPECT_EQ(contacts.size(), 2);
  EXPECT_EQ(contacts[0].state,
            RigidContact::ContactState::kContactingAndSliding);
  EXPECT_EQ(contacts[1].state,
            RigidContact::ContactState::kNotContacting);

  // Set the coefficient of friction to infinite. This forces the rod code
  // to go through the first impact path (impulse within the friction cone).
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());
  contacts[0].mu = contacts[1].mu = dut_->get_mu_coulomb();

  // Handle the impact, redetermine the active set, and copy the result to the
  // context.
  VectorX<double> Nv, Fv;
  double zero_tol;
  dut_->ModelImpact(new_state.get(), &Nv, &Fv, &zero_tol);
  dut_->DetermineVelLevelActiveSet(new_state.get(), Nv, Fv, zero_tol);
  context_->get_mutable_state()->SetFrom(*new_state);
  EXPECT_FALSE(dut_->IsImpacting(context_->get_state()));

  // Verify that the state is no longer in a sliding mode.
  EXPECT_EQ(contacts[0].state, RigidContact::ContactState::
      kContactingWithoutSliding);
  EXPECT_EQ(contacts[1].state, RigidContact::ContactState::kNotContacting);

  // Do one more impact- there should now be no change.
  dut_->ModelImpact(new_state.get());
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Verify that applying an impact model to an impacting state results in a
// non-impacting state. This test exercises the model for the case
// where impulses that yield tangential sticking lie outside the friction cone.
TEST_F(Rod2DDAETest, NoFrictionImpactThenNoImpact) {
  // Set the initial state to be impacting.
  SetImpactingState();

  // Verify that the state is in a sliding mode.
  std::vector<RigidContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  EXPECT_EQ(contacts.size(), 2);
  EXPECT_TRUE(contacts.front().state ==
      RigidContact::ContactState::kContactingAndSliding);
  EXPECT_TRUE(contacts.back().state ==
          RigidContact::ContactState::kNotContacting);

  // Set the coefficient of friction to zero. This forces the rod code
  // to go through the second impact path (impulse corresponding to sticking
  // friction post-impact lies outside of the friction cone).
  dut_->set_mu_coulomb(0.0);
  contacts[0].mu = contacts[1].mu = dut_->get_mu_coulomb();

  // Handle the impact and copy the result to the context.
  std::unique_ptr<State<double>> new_state = CloneState();
  VectorX<double> Nv, Fv;
  double zero_tol;
  dut_->ModelImpact(new_state.get(), &Nv, &Fv, &zero_tol);
  dut_->DetermineVelLevelActiveSet(new_state.get(), Nv, Fv, zero_tol);
  context_->get_mutable_state()->SetFrom(*new_state);
  EXPECT_FALSE(dut_->IsImpacting(context_->get_state()));

  // Verify that the state is still in a sliding mode.
  EXPECT_EQ(contacts.size(), 2);
  EXPECT_TRUE(contacts.front().state ==
      RigidContact::ContactState::kContactingAndSliding);
  EXPECT_TRUE(contacts.back().state ==
      RigidContact::ContactState::kNotContacting);

  // Do one more impact- there should now be no change.
  // Verify that there is no further change from this second impact.
  dut_->ModelImpact(new_state.get(), &Nv, &Fv, &zero_tol);
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              10 * std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));

  // Redetermine the active set and verify that the state is still in a sliding
  // mode.
  dut_->DetermineVelLevelActiveSet(new_state.get(), Nv, Fv, zero_tol);
  EXPECT_EQ(contacts.size(), 2);
  EXPECT_TRUE(contacts.front().state ==
      RigidContact::ContactState::kContactingAndSliding);
  EXPECT_TRUE(contacts.back().state ==
      RigidContact::ContactState::kNotContacting);
}

// Verify that no exceptions thrown for a non-sliding configuration.
TEST_F(Rod2DDAETest, NoSliding) {
  const double half_len = dut_->get_rod_half_length();
  const double r22 = std::sqrt(2) / 2;
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();

  // Set the coefficient of friction to zero (triggering the case on the
  // edge of the friction cone).
  dut_->set_mu_coulomb(0.0);
  std::vector<RigidContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  contacts[0].mu = contacts[1].mu = dut_->get_mu_coulomb();
  
  // This configuration has no sliding velocity.
  xc[0] = -half_len * r22;
  xc[1] = half_len * r22;
  xc[2] = 3 * M_PI / 4.0;
  xc[3] = 0.0;
  xc[4] = 0.0;
  xc[5] = 0.0;
  contacts[0].state = RigidContact::ContactState::kContactingWithoutSliding;

  // Verify no impact.
  EXPECT_FALSE(dut_->IsImpacting(context_->get_state()));

  // No exceptions should be thrown.
  EXPECT_NO_THROW(dut_->CalcTimeDerivatives(*context_, derivatives_.get()));

  // Set the coefficient of friction to effective no-slip (triggering the
  // case strictly inside the friction cone).
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());
  contacts[0].mu = contacts[1].mu = dut_->get_mu_coulomb();

  // No exceptions should be thrown.
  EXPECT_NO_THROW(dut_->CalcTimeDerivatives(*context_, derivatives_.get()));
}

// Test multiple (two-point) contact configurations.
TEST_F(Rod2DDAETest, MultiPoint) {
  ContinuousState<double> &xc =
      *context_->get_mutable_continuous_state();
  std::vector<RigidContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());

  // Set the rod to a horizontal, two-contact configuration with sliding.
  xc[0] = 0;
  xc[1] = 0;
  xc[2] = 0;
  xc[3] = 1.0;
  EXPECT_EQ(contacts.size(), 2);
  contacts[0].state = contacts[1].state =
      RigidContact::ContactState::kContactingAndSliding;

  // Verify no impact.
  EXPECT_FALSE(dut_->IsImpacting(context_->get_state()));

  // Set the coefficient of friction to zero.
  dut_->set_mu_coulomb(0.0);
  contacts[0].mu = contacts[1].mu = dut_->get_mu_coulomb();

  // Compute the derivatives and verify that the linear and angular acceleration
  // are approximately zero.
  const double eps = 10 * dut_->get_cfm();
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR((*derivatives_)[3], 0, eps);
  EXPECT_NEAR((*derivatives_)[4], 0, eps);
  EXPECT_NEAR((*derivatives_)[5], 0, eps);

  // Set the coefficient of friction to "very large".
  const double large = 100.0;
  dut_->set_mu_coulomb(large);
  contacts[0].mu = contacts[1].mu = dut_->get_mu_coulomb();

  // The normal force on the rod will be equal to -mg (the mass times the
  // gravitational acceleration). Therefore, the frictional force will be equal
  // to μmg. The tangential acceleration should then be μg. Note that the
  // gravitational acceleration is signed,

  // Check derivatives.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR((*derivatives_)[3], large *
      dut_->get_gravitational_acceleration(), eps * large);
  EXPECT_NEAR((*derivatives_)[4], 0, eps);
  EXPECT_NEAR((*derivatives_)[5], 0, eps);

  // Set the rod velocity to zero.
  xc[3] = 0.0;
  contacts[0].state = contacts[1].state =
      RigidContact::ContactState::kContactingWithoutSliding;
  EXPECT_FALSE(dut_->IsImpacting(context_->get_state()));  // Verify no impact.

  // Set a constant force pushing the rod.
  const double fX = 1.0;
  std::unique_ptr<BasicVector<double>> ext_input =
      std::make_unique<BasicVector<double>>(3);
  ext_input->SetAtIndex(0, fX);
  ext_input->SetAtIndex(1, 0.0);
  ext_input->SetAtIndex(2, 0.0);
  context_->FixInputPort(0, std::move(ext_input));

  // Verify that the linear and angular acceleration are still zero.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR((*derivatives_)[3], 0, eps);
  EXPECT_NEAR((*derivatives_)[4], 0, eps);
  EXPECT_NEAR((*derivatives_)[5], 0, eps);

  // Set the coefficient of friction to zero, redetermine the active set,
  // and recompute the derivatives. The force should result
  // in the rod being pushed to the right.
  dut_->set_mu_coulomb(0.0);
  contacts[0].mu = contacts[1].mu = dut_->get_mu_coulomb();
  dut_->DetermineAccelLevelActiveSet(*context_, context_->get_mutable_state());
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR((*derivatives_)[3], fX/dut_->get_rod_mass(), eps);
  EXPECT_NEAR((*derivatives_)[4], 0, eps);
  EXPECT_NEAR((*derivatives_)[5], 0, eps);
}

// Verify that the Painlevé configuration does not correspond to an impacting
// state.
TEST_F(Rod2DDAETest, ImpactNoChange2) {
  SetSecondInitialConfig();

  // Verify no impact.
  EXPECT_FALSE(dut_->IsImpacting(context_->get_state()));

  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();
  dut_->ModelImpact(new_state.get());
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Verify that applying the impact model to an impacting state results
// in a non-impacting state.
TEST_F(Rod2DDAETest, InfFrictionImpactThenNoImpact2) {
  // Cause the initial state to be impacting.
  SetImpactingState();

  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

  // Set the coefficient of friction to infinite. This forces the rod code
  // to go through the first impact path.
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());
  std::vector<RigidContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  EXPECT_EQ(contacts.size(), 2);
  contacts[0].mu = contacts[1].mu = dut_->get_mu_coulomb();

  // Handle the impact and copy the result to the context.
  VectorX<double> Nv, Fv;
  double zero_tol;
  dut_->ModelImpact(new_state.get(), &Nv, &Fv, &zero_tol);
  dut_->DetermineVelLevelActiveSet(new_state.get(), Nv, Fv, zero_tol);
  context_->get_mutable_state()->SetFrom(*new_state);

  // Verify the state no longer corresponds to an impact.
  EXPECT_FALSE(dut_->IsImpacting(context_->get_state()));

  // Verify that the state is now in a sticking mode.
  EXPECT_EQ(contacts[0].state,
            RigidContact::ContactState::kContactingWithoutSliding);

  // Do one more impact- there should now be no change.
  dut_->ModelImpact(new_state.get());
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Verify that applying the impact model to an impacting state results in a
// non-impacting state.
TEST_F(Rod2DDAETest, NoFrictionImpactThenNoImpact2) {
  // Cause the initial state to be impacting.
  SetImpactingState();

  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

  // Verify that the state is still in a sliding configuration.
  std::vector<RigidContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  EXPECT_EQ(contacts.size(), 2);
  EXPECT_EQ(contacts[0].state,
            RigidContact::ContactState::kContactingAndSliding);
  EXPECT_EQ(contacts[1].state,
            RigidContact::ContactState::kNotContacting);

  // Set the coefficient of friction to zero. This forces the rod code
  // to go through the second impact path.
  dut_->set_mu_coulomb(0.0);
  contacts[0].mu = contacts[1].mu = dut_->get_mu_coulomb();

  // Verify that the state is still in a sliding configuration.
  EXPECT_EQ(contacts[0].state,
            RigidContact::ContactState::kContactingAndSliding);
  EXPECT_EQ(contacts[1].state,
            RigidContact::ContactState::kNotContacting);

  // Handle the impact and copy the result to the context.
  dut_->ModelImpact(new_state.get());
  context_->get_mutable_state()->SetFrom(*new_state);
  EXPECT_FALSE(dut_->IsImpacting(context_->get_state()));

  // Do one more impact- there should now be no change.
  dut_->ModelImpact(new_state.get());
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));

  // Verify that the state is still in a sliding configuration.
  EXPECT_EQ(contacts[0].state,
            RigidContact::ContactState::kContactingAndSliding);
  EXPECT_EQ(contacts[1].state,
            RigidContact::ContactState::kNotContacting);
}

// Verifies that rod in a ballistic state does not correspond to an impact.
TEST_F(Rod2DDAETest, BallisticNoImpact) {
  // Cause the initial state to be impacting.
  SetImpactingState();

  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

  // Move the rod upward vertically so that it is no longer impacting and
  // set the mode to ballistic motion.
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();
  xc[1] += 10.0;
  std::vector<RigidContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  EXPECT_EQ(contacts.size(), 2);
  contacts[0].state = contacts[1].state =
      RigidContact::ContactState::kNotContacting;

  // Verify that no impact occurs.
  EXPECT_FALSE(dut_->IsImpacting(context_->get_state()));
}

// Validates the set of witness functions is determined correctly.
TEST_F(Rod2DDAETest, NumWitnessFunctions) {
  // Get the set of contacts.
  std::vector<RigidContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());

  // Verify that the correct number of witness functions is reported for...
  // (a) Sliding single contact (default initial state).
  EXPECT_EQ(dut_->get_witness_functions(*context_).size(), 3);

  // (b) Ballistic motion.
  SetBallisticState();
  EXPECT_EQ(dut_->get_witness_functions(*context_).size(), 2);

  // (c) Sticking single contact.
  contacts.front().state =
      RigidContact::ContactState::kContactingWithoutSliding;
  EXPECT_EQ(dut_->get_witness_functions(*context_).size(), 3);

  // (d) Sticking two contacts.
  contacts.back().state = RigidContact::ContactState::kContactingWithoutSliding;
  EXPECT_EQ(dut_->get_witness_functions(*context_).size(), 4);

  // (e) Sliding two contacts.
  contacts.front().state = RigidContact::ContactState::kContactingAndSliding;
  contacts.back().state = contacts.front().state;
  EXPECT_EQ(dut_->get_witness_functions(*context_).size(), 4);
}

// Checks the witness function for calculating the signed distance.
TEST_F(Rod2DDAETest, SignedDistWitness) {
  // Rod initially touches the half-space in a kissing configuration and is
  // oriented at a 45 degree angle; check that the signed distance for the
  // first contact is zero and the second is sqrt(2)*rod_half_length().
  const double tol = 10*std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(dut_->signed_distance_witnesses_[0]->Evaluate(*context_),
              0.0, tol);
  EXPECT_NEAR(dut_->signed_distance_witnesses_[1]->Evaluate(*context_),
              std::sqrt(2)*dut_->get_rod_half_length(), tol);

  // Set the rod to a non-contacting configuration and check that the signed
  // distances are both positive.
  SetBallisticState();
  EXPECT_GT(dut_->signed_distance_witnesses_[0]->Evaluate(*context_), 0.0);
  EXPECT_GT(dut_->signed_distance_witnesses_[1]->Evaluate(*context_), 0.0);

  // Set the rod to an interpenetrating configuration and check that the
  // signed distance is -1 for both points.
  SetInterpenetratingConfig();
  EXPECT_NEAR(dut_->signed_distance_witnesses_[0]->Evaluate(*context_),
              -1.0, tol);
  EXPECT_NEAR(dut_->signed_distance_witnesses_[1]->Evaluate(*context_),
              -1.0, tol);

  // TODO(edrumwri): Verify that the witness triggers.
}

// Evaluates the witness function for when the rod should separate from the
// half-space.
TEST_F(Rod2DDAETest, SeparationWitness) {
  // Set the rod to an upward configuration so that accelerations are simple
  // to predict.
  ContinuousState<double>& xc = *context_->get_mutable_continuous_state();

  // Vertical velocity is still zero.
  xc[0] = 0.0;
  xc[1] = dut_->get_rod_half_length();
  xc[2] = M_PI_2;

  // Ensure that witness is positive.
  const double w0 = dut_->separating_accel_witnesses_[0]->Evaluate(*context_);
  EXPECT_GT(w0, 0);

  // Now add a large upward force and verify that the witness is negative.
  const double flarge_up = 100.0;
  std::unique_ptr<BasicVector<double>> ext_input =
      std::make_unique<BasicVector<double>>(3);
  ext_input->SetAtIndex(0, 0.0);
  ext_input->SetAtIndex(1, flarge_up);
  ext_input->SetAtIndex(2, 0.0);
  context_->FixInputPort(0, std::move(ext_input));
  const double wf = dut_->separating_accel_witnesses_[0]->Evaluate(*context_);
  EXPECT_LT(wf, 0);

  // Verify that the witness triggers.
  EXPECT_TRUE(dut_->separating_accel_witnesses_[0]->should_trigger(w0, wf));
}

// Evaluates the witness function for sliding velocity direction changes.
TEST_F(Rod2DDAETest, VelocityChangesWitness) {
  // Verify that the sliding velocity before the Painleve configuration is
  // negative.
  const double w0 = dut_->sliding_dot_witnesses_[0]->Evaluate(*context_);
  EXPECT_LT(w0, 0);

  // Make the rod slide to the right.
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();
  xc[3] = 10.0;

  // Verify that the sliding velocity before the second Painleve configuration
  // is positive.
  const double wf = dut_->sliding_dot_witnesses_[0]->Evaluate(*context_);
  EXPECT_GT(wf, 0);

  // Verify that the witness triggers.
  EXPECT_TRUE(dut_->sliding_dot_witnesses_[0]->should_trigger(w0, wf));
}

// Checks the witness for transition from sticking to sliding.
TEST_F(Rod2DDAETest, StickingSlidingWitness) {
  // Put the rod into an upright configuration with no tangent velocity and
  // some horizontal force.
  const double half_len = dut_->get_rod_half_length();
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();
  xc[0] = 0.0;       // com horizontal position
  xc[1] = half_len;  // com vertical position
  xc[2] = M_PI_2;    // rod rotation
  std::unique_ptr<BasicVector<double>> ext_input =
      std::make_unique<BasicVector<double>>(3);
  ext_input->SetAtIndex(0, 1.0);
  ext_input->SetAtIndex(1, 0.0);
  ext_input->SetAtIndex(2, 0.0);
  context_->FixInputPort(0, std::move(ext_input));

  // Mark the contact as sticking without sliding.
  std::vector<RigidContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  contacts[0].state = RigidContact::ContactState::kContactingWithoutSliding;

  // Verify that the "slack" is positive.
  const double inf = std::numeric_limits<double>::infinity();
  dut_->set_mu_coulomb(inf);
  const double w0 = dut_->sticking_friction_forces_slack_witnesses_[0]->
                            Evaluate(*context_);
  EXPECT_GT(w0, 0);

  // Set the coefficient of friction to zero.
  dut_->set_mu_coulomb(0.0);
  contacts[0].mu = dut_->get_mu_coulomb();

  // Verify that the "slack" is negative.
  const double wf = dut_->sticking_friction_forces_slack_witnesses_[0]->
                            Evaluate(*context_);
  EXPECT_LT(wf, 0);

  // Verify that the witness triggers.
  EXPECT_TRUE(dut_->sticking_friction_forces_slack_witnesses_[0]->
      should_trigger(w0, wf));

}

// Class for testing the Rod 2D example using a first order time
// stepping approach.
class Rod2DTimeSteppingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const double dt = 1e-2;
    dut_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SimulationType::kTimeStepping, dt);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);

    // Use a non-unit mass.
    dut_->set_rod_mass(2.0);

    // Set a zero input force (this is the default).
    std::unique_ptr<BasicVector<double>> ext_input =
        std::make_unique<BasicVector<double>>(3);
    ext_input->SetAtIndex(0, 0.0);
    ext_input->SetAtIndex(1, 0.0);
    ext_input->SetAtIndex(2, 0.0);
    context_->FixInputPort(0, std::move(ext_input));
  }

  BasicVector<double> *mutable_discrete_state() {
    return context_->get_mutable_discrete_state(0);
  }
// Sets a secondary initial Rod2D configuration.
  void SetSecondInitialConfig() {
    // Set the configuration to an inconsistent (Painlevé) type state with
    // the rod at a 135 degree counter-clockwise angle with respect to the
    // x-axis. The rod in [Stewart, 2000] is at a 45 degree counter-clockwise
    // angle with respect to the x-axis.
    // * [Stewart, 2000]  D. Stewart, "Rigid-Body Dynamics with Friction and
    //                    Impact". SIAM Rev., 42(1), 3-39, 2000.
    using std::sqrt;
    const double half_len = dut_->get_rod_half_length();
    const double r22 = std::sqrt(2) / 2;
    auto xd = mutable_discrete_state()->get_mutable_value();

    xd[0] = -half_len * r22;
    xd[1] = half_len * r22;
    xd[2] = 3 * M_PI / 4.0;
    xd[3] = 1.0;
    xd[4] = 0.0;
    xd[5] = 0.0;
  }

  std::unique_ptr<Rod2D<double>> dut_;  //< The device under test.
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
};

// Verify that Rod 2D system eventually goes to rest using the
// first-order time stepping approach (this tests expected meta behavior).
TEST_F(Rod2DTimeSteppingTest, RodGoesToRest) {
  // Set the initial state to an inconsistent configuration.
  SetSecondInitialConfig();

  // Init the simulator.
  Simulator<double> simulator(*dut_, std::move(context_));

  // Integrate forward to a point where the rod should be at rest.
  const double t_final = 10;
  simulator.StepTo(t_final);

  // Get angular orientation and velocity.
  const auto xd = simulator.get_context().get_discrete_state(0)->get_value();
  const double theta = xd(2);
  const double theta_dot = xd(5);

  // After sufficiently long, theta should be 0 or M_PI and the velocity
  // should be nearly zero.
  EXPECT_TRUE(std::fabs(theta) < 1e-6 || std::fabs(theta - M_PI) < 1e-6);
  EXPECT_NEAR(theta_dot, 0.0, 1e-6);
}

// Validates the number of witness functions is determined correctly.
TEST_F(Rod2DTimeSteppingTest, NumWitnessFunctions) {
  EXPECT_EQ(dut_->get_witness_functions(*context_).size(), 0);
}

// Class for cross-testing the Rod2D example using piecewise DAE and time
// stepping approaches.
class Rod2DCrossValidationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create both test devices.
    pdae_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SimulationType::kPiecewiseDAE, 0.0);
    ts_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SimulationType::kTimeStepping, dt_);

    // Use non-unit masses for the rods.
    pdae_->set_rod_mass(2.0);
    ts_->set_rod_mass(2.0);

    // Create contexts for them.
    std::unique_ptr<Context<double>> context_pdae = pdae_->
        CreateDefaultContext();
    std::unique_ptr<Context<double>> context_ts = ts_->CreateDefaultContext();

    // Set CFM for both approaches.
    const double cfm = std::numeric_limits<double>::epsilon();
    pdae_->set_cfm(cfm);
    ts_->set_cfm(cfm);

    // Set Baumgarte stabilization such that all constraint errors would be
    // eliminated at the end of the time step.
    ts_->set_erp(1.0);

    // Set input forces for both devices.
    const Vector3<double> fext(get_horizontal_external_force(), 0, 0);
    std::unique_ptr<BasicVector<double>> ext_input =
        std::make_unique<BasicVector<double>>(fext);
    context_ts->FixInputPort(0, std::move(ext_input));
    ext_input = std::make_unique<BasicVector<double>>(fext);
    context_pdae->FixInputPort(0, std::move(ext_input));

    // Set the friction coefficients.
    ts_->set_mu_coulomb(get_mu_coulomb());
    pdae_->set_mu_coulomb(get_mu_coulomb());
    std::vector<RigidContact>& contacts = pdae_->get_contacts(
        context_pdae->get_mutable_state());
    for (size_t i = 0; i < contacts.size(); ++i)
      contacts[i].mu = get_mu_coulomb();

    // Init the simulators.
    simulator_ts_ = std::make_unique<Simulator<double>>(*ts_,
                                                        std::move(context_ts));
    simulator_pdae_ = std::make_unique<Simulator<double>>(
        *pdae_,std::move(context_pdae));

    // We want the integrator in the piecewise DAE approach to take only a
    // single semi-implicit step in order to line the solutions up with time
    // stepping as much as possible.
    simulator_pdae_->reset_integrator<SemiExplicitEulerIntegrator<double>>(
        *pdae_, dt_, simulator_pdae_->get_mutable_context());
  }

  // Sets both rods to an impacting horizontal configuration with zero
  // horizontal velocity.
  void set_two_point_impacting_configuration() {
    // The vertical location of the rod should be such that the initial
    // signed distance is ever so slightly positive.
    const double y0 = 1.5e-8;

    Context<double>* context_ts = simulator_ts_->get_mutable_context();
    auto xd = context_ts->get_mutable_discrete_state(0)->get_mutable_value();
    xd[0] = 0;   // Place horizontal com location at zero.
    xd[1] = y0;  // Vertical location of COM.
    xd[2] = 0;   // Theta = zero rotation.
    xd[3] = 0;   // Zero horizontal velocity.
    xd[4] = -1;  // Impacting velocity.

    Context<double>* context_pdae = simulator_pdae_->get_mutable_context();
    auto xc = context_pdae->get_mutable_continuous_state_vector();
    xc->SetAtIndex(0, 0);   // Horizontal com location.
    xc->SetAtIndex(1, y0);  // Vertical com location.
    xc->SetAtIndex(2, 0);   // Angular rotation.
    xc->SetAtIndex(3, 0);   // Zero horizontal velocity.
    xc->SetAtIndex(4, -1);  // Impacting velocity.

    // piecewise DAE requires mode variables to be set too.
    std::vector<RigidContact>& contacts = pdae_->get_contacts(
        context_pdae->get_mutable_state());
    for (size_t i = 0; i < contacts.size(); ++i)
      contacts[i].state = RigidContact::ContactState::kNotContacting;
  }

  // Sets both rods to a resting horizontal configuration.
  void set_horizontal_motionless_configuration() {
    Context<double>* context_ts = simulator_ts_->get_mutable_context();
    auto xd = context_ts->get_mutable_discrete_state(0)->get_mutable_value();
    xd[0] = 0;  //   Place horizontal com location at zero.
    xd[1] = 0;  //   Zero rotation implies vertical location of COM is zero.
    xd[2] = 0;  //   Theta = zero rotation.
    xd[3] = 0;  //   Zero horizontal velocity.

    Context<double>* context_pdae = simulator_pdae_->get_mutable_context();
    auto xc = context_pdae->get_mutable_continuous_state_vector();
    xc->SetAtIndex(0, 0);  // Horizontal com location.
    xc->SetAtIndex(1, 0);  // Vertical com location.
    xc->SetAtIndex(2, 0);  // Angular rotation.
    xc->SetAtIndex(3, 0);  // Zero horizontal velocity.

    // piecewise DAE requires mode variables to be set too.
    std::vector<RigidContact>& contacts = pdae_->get_contacts(
        context_pdae->get_mutable_state());
    for (size_t i = 0; i < contacts.size(); ++i)
      contacts[i].state = RigidContact::ContactState::kContactingWithoutSliding;
  }

  // Sets both rods to a horizontal sliding configuration.
  void set_horizontal_sliding_configuration() {
    Context<double>* context_ts = simulator_ts_->get_mutable_context();
    auto xd = context_ts->get_mutable_discrete_state(0)->get_mutable_value();
    xd[0] = 0;  //   Place horizontal com location at zero.
    xd[1] = 0;  //   Zero rotation implies vertical location of COM is zero.
    xd[2] = 0;  //   Theta = zero rotation.
    xd[3] = 1;  //   Horizontal velocity.

    Context<double>* context_pdae = simulator_pdae_->get_mutable_context();
    auto xc = context_pdae->get_mutable_continuous_state_vector();
    xc->SetAtIndex(0, 0);  // Horizontal com location.
    xc->SetAtIndex(1, 0);  // Vertical com location.
    xc->SetAtIndex(2, 0);  // Angular rotation.
    xc->SetAtIndex(3, 1);  // Horizontal velocity.

    // piecewise DAE requires mode variables to be set too.
    std::vector<RigidContact>& contacts = pdae_->get_contacts(
        context_pdae->get_mutable_state());
    for (size_t i = 0; i < contacts.size(); ++i)
      contacts[i].state = RigidContact::ContactState::kContactingAndSliding;
  }

  // Gets the horizontal applied forces for both devices.
  virtual double get_horizontal_external_force() const = 0;

  // Gets the coefficient of friction.
  virtual double get_mu_coulomb() const = 0;

 protected:
  // The integration step size (should be large to allow for event transitions).
  const double dt_{0.1};

  std::unique_ptr<Rod2D<double>> pdae_;
  std::unique_ptr<Rod2D<double>> ts_;
  std::unique_ptr<Simulator<double>> simulator_pdae_;
  std::unique_ptr<Simulator<double>> simulator_ts_;
};

class Rod2DCrossValidationImpactingTest : public Rod2DCrossValidationTest {
 protected:
  double get_horizontal_external_force() const override { return 0; }
  double get_mu_coulomb() const override { return 0.0; }
};

// This test checks to see whether the time stepping and piecewise DAE
// solutions are identical for the situation where an impact occurs at the
// beginning of the interval.
TEST_F(Rod2DCrossValidationImpactingTest, ImpactingWithoutHorizontalVelocity) {
  // If the rod were to impact at only a single point, the velocity would be
  // altered at the beginning of the interval, and we should expect the
  // piecewise DAE and time stepping solutions to differ. In this case, they
  // should be nearly identical.
  set_two_point_impacting_configuration();

  // Integrate forward by a single *large* dt. Note that the update rate
  // is set by the time stepping system, so stepping to dt should yield
  // exactly one step.
  simulator_ts_->StepTo(dt_);
  simulator_pdae_->StepTo(dt_);
  EXPECT_EQ(simulator_ts_->get_num_discrete_updates(), 1);

  // Two unrestricted updates necessary (one for each contact point impacting).
  EXPECT_EQ(simulator_pdae_->get_num_unrestricted_updates(), 2);
  EXPECT_EQ(simulator_pdae_->get_num_steps_taken(), 2);

  // See whether the states are equal.
  const Context<double>& context_ts = simulator_ts_->get_context();
  const Context<double>& context_pdae = simulator_pdae_->get_context();
  const auto& xd = context_ts.get_discrete_state(0)->get_value();
  const auto& xc = context_pdae.get_continuous_state_vector();

  // Check that the solution is nearly identical.
  const double tol = std::numeric_limits<double>::epsilon() * 10;
  EXPECT_NEAR(xc[0], xd[0], tol);
  EXPECT_NEAR(xc[1], xd[1], tol);
  EXPECT_NEAR(xc[2], xd[2], tol);
  EXPECT_NEAR(xc[3], xd[3], tol);
  EXPECT_NEAR(xc[4], xd[4], tol);
  EXPECT_NEAR(xc[5], xd[5], tol);
}

class Rod2DCrossValidationSlidingTest : public Rod2DCrossValidationTest {
 protected:
  double get_horizontal_external_force() const override { return 0; }
  double get_mu_coulomb() const override { return 0.01; }
};

// This test checks to see whether a simulation step of the piecewise
// DAE based Rod2D system is equivalent to a single step of the semi-explicit
// time stepping based system *without any transitions from sliding to
// not-sliding*.
TEST_F(Rod2DCrossValidationSlidingTest, OneStepSolutionSliding) {
  // Integrate forward by a single *large* dt. Note that the update rate
  // is set by the time stepping system, so stepping to dt should yield
  // exactly one step.
  simulator_ts_->StepTo(dt_);
  simulator_pdae_->StepTo(dt_);
  EXPECT_EQ(simulator_ts_->get_num_discrete_updates(), 1);

  // See whether the states are equal.
  const Context<double>& context_ts = simulator_ts_->get_context();
  const Context<double>& context_pdae = simulator_pdae_->get_context();
  const auto& xd = context_ts.get_discrete_state(0)->get_value();
  const auto& xc = context_pdae.get_continuous_state_vector();

  // Check that the solution is nearly identical.
  const double tol = std::numeric_limits<double>::epsilon() * 10;
  EXPECT_NEAR(xc[0], xd[0], tol);
  EXPECT_NEAR(xc[1], xd[1], tol);
  EXPECT_NEAR(xc[2], xd[2], tol);
  EXPECT_NEAR(xc[3], xd[3], tol);
  EXPECT_NEAR(xc[4], xd[4], tol);
  EXPECT_NEAR(xc[5], xd[5], tol);
}

// This test checks to see whether a simulation step of the piecewise
// DAE based Rod2D system is equivalent to a single step of the semi-explicit
// time stepping based system for two sliding contacts *without any transitions
// from sliding to not-sliding*.
TEST_F(Rod2DCrossValidationSlidingTest, OneStepSolutionTwoSliding) {
  // Set the rods to a sidewise configuration.
  set_horizontal_sliding_configuration();

  // Integrate forward by a single *large* dt. Note that the update rate
  // is set by the time stepping system, so stepping to dt should yield
  // exactly one step.
  simulator_ts_->StepTo(dt_);
  simulator_pdae_->StepTo(dt_);
  EXPECT_EQ(simulator_ts_->get_num_discrete_updates(), 1);
  EXPECT_EQ(simulator_pdae_->get_num_unrestricted_updates(), 0);
  EXPECT_EQ(simulator_pdae_->get_num_steps_taken(), 1);

  // See whether the states are equal.
  const Context<double>& context_ts = simulator_ts_->get_context();
  const Context<double>& context_pdae = simulator_pdae_->get_context();
  const auto& xd = context_ts.get_discrete_state(0)->get_value();
  const auto& xc = context_pdae.get_continuous_state_vector();

  // Check that the solution is nearly identical.
  const double tol = std::numeric_limits<double>::epsilon() * 10;
  EXPECT_NEAR(xc[0], xd[0], tol);
  EXPECT_NEAR(xc[1], xd[1], tol);
  EXPECT_NEAR(xc[2], xd[2], tol);
  EXPECT_NEAR(xc[3], xd[3], tol);
  EXPECT_NEAR(xc[4], xd[4], tol);
  EXPECT_NEAR(xc[5], xd[5], tol);
}

class Rod2DCrossValidationStickingTest : public Rod2DCrossValidationTest {
 protected:
  virtual void SetUp() {
    // Do the majority of the setup as normal.
    Rod2DCrossValidationTest::SetUp();

    // Set horizontal velocity to zero for time stepping system.
    Context<double>* context_ts = simulator_ts_->get_mutable_context();
    auto xd = context_ts->get_mutable_discrete_state(0)->get_mutable_value();
    xd[3] = 0;  //   zero horizontal velocity.

    // Setting horizontal velocity to zero for piecewise DAE system also
    // requires a mode variable change.
    Context<double>* context_pdae = simulator_pdae_->get_mutable_context();
    auto xc = context_pdae->get_mutable_continuous_state_vector();
    xc->SetAtIndex(3, 0);  // zero horizontal velocity.
    std::vector<RigidContact>& contacts = pdae_->get_contacts(
        context_pdae->get_mutable_state());
    for (size_t i = 0; i < contacts.size(); ++i) {
      if (contacts[i].state ==
          RigidContact::ContactState::kContactingAndSliding) {
        contacts[i].state =
            RigidContact::ContactState::kContactingWithoutSliding;
      }
    }
  }

  double get_horizontal_external_force() const override { return 1.0; }
  double get_mu_coulomb() const override { return 100.0; }
};

// This test checks to see whether a simulation step of the piecewise
// DAE based Rod2D system is equivalent to a single step of the semi-explicit
// time stepping based system.
TEST_F(Rod2DCrossValidationStickingTest, OneStepSolutionSticking) {
  // Mode variable for point of contact must initially be set to not sliding.

  // Integrate forward by a single *large* dt. Note that the update rate
  // is set by the time stepping system, so stepping to dt should yield
  // exactly one step.
  simulator_ts_->StepTo(dt_);
  simulator_pdae_->StepTo(dt_);
  EXPECT_EQ(simulator_ts_->get_num_discrete_updates(), 1);

  // See whether the states are equal.
  const Context<double>& context_ts = simulator_ts_->get_context();
  const Context<double>& context_pdae = simulator_pdae_->get_context();
  const auto& xd = context_ts.get_discrete_state(0)->get_value();
  const auto& xc = context_pdae.get_continuous_state_vector();

  // Check that the solution is nearly identical.
  const double tol = std::numeric_limits<double>::epsilon() * 10;
  EXPECT_NEAR(xc[0], xd[0], tol);
  EXPECT_NEAR(xc[1], xd[1], tol);
  EXPECT_NEAR(xc[2], xd[2], tol);
  EXPECT_NEAR(xc[3], xd[3], tol);
  EXPECT_NEAR(xc[4], xd[4], tol);
  EXPECT_NEAR(xc[5], xd[5], tol);
}

// This test checks to see whether a simulation step of the piecewise
// DAE based Rod2D system is equivalent to a single step of the semi-explicit
// time stepping based system for two sticking contacts.
TEST_F(Rod2DCrossValidationStickingTest, OneStepSolutionTwoSticking) {
  // Set the rods to a sidewise configuration.
  set_horizontal_motionless_configuration();

  // Integrate forward by a single *large* dt. Note that the update rate
  // is set by the time stepping system, so stepping to dt should yield
  // exactly one step.
  simulator_ts_->StepTo(dt_);
  simulator_pdae_->StepTo(dt_);
  EXPECT_EQ(simulator_ts_->get_num_discrete_updates(), 1);

  // See whether the states are equal.
  const Context<double>& context_ts = simulator_ts_->get_context();
  const Context<double>& context_pdae = simulator_pdae_->get_context();
  const auto& xd = context_ts.get_discrete_state(0)->get_value();
  const auto& xc = context_pdae.get_continuous_state_vector();

  // Check that the solution is nearly identical.
  const double tol = std::numeric_limits<double>::epsilon() * 10;
  EXPECT_NEAR(xc[0], xd[0], tol);
  EXPECT_NEAR(xc[1], xd[1], tol);
  EXPECT_NEAR(xc[2], xd[2], tol);
  EXPECT_NEAR(xc[3], xd[3], tol);
  EXPECT_NEAR(xc[4], xd[4], tol);
  EXPECT_NEAR(xc[5], xd[5], tol);
}

/*
// This test checks to see whether a single semi-explicit step of the piecewise
// DAE based Rod2D system is equivalent to a single step of the semi-explicit
// time stepping based system for a sticking contact scenario.
GTEST_TEST(Rod2DCrossValidationTest, OneStepSolutionSticking) {
  // Create two Rod2D systems.
  const double dt = 1e-1;
  Rod2D<double> ts(Rod2D<double>::SimulationType::kTimeStepping, dt);
  Rod2D<double> pdae(Rod2D<double>::SimulationType::kPiecewiseDAE, 0.0);

  // Set the coefficient of friction to a large value for both.
  const double mu = 100.0;
  ts.set_mu_coulomb(mu);
  pdae.set_mu_coulomb(mu);

  // Set "one step" constraint stabilization (not generally recommended, but
  // works for a single step) and small regularization.
  ts.set_cfm(std::numeric_limits<double>::epsilon());
  ts.set_erp(1.0);

  // Create contexts for both.
  std::unique_ptr<Context<double>> context_ts = ts.CreateDefaultContext();
  std::unique_ptr<Context<double>> context_pdae = pdae.CreateDefaultContext();

  // This configuration has no sliding velocity.
  const double half_len = pdae.get_rod_half_length();
  ContinuousState<double>& xc =
      *context_pdae->get_mutable_continuous_state();
  auto xd = context_ts->get_mutable_discrete_state(0)->get_mutable_value();
  xc[0] = xd[0] = 0.0;
  xc[1] = xd[1] = half_len;
  xc[2] = xd[2] = M_PI_2;
  xc[3] = xd[3] = 0.0;
  xc[4] = xd[4] = 0.0;
  xc[5] = xd[5] = 0.0;

  // Prepare to set contact states for the piecewise DAE system.
  std::vector<RigidContact>& contacts =
      pdae.get_contacts(context_pdae->get_mutable_state());
  EXPECT_EQ(contacts.size(), 2);

  // First point is Rl in Rod Frame (see class documentation); in this
  // new configuration, it contacts the half-space without sliding.
  contacts[0].state = RigidContact::ContactState::kContactingWithoutSliding;
  contacts[0].mu = pdae.get_mu_coulomb();
  contacts[0].u = Eigen::Vector3d(-pdae.get_rod_half_length(), 0, 0);

  // Second point is Rr in Rod Frame. In this new configuration; in this
  // new configuration, it does not contact the half-space..
  contacts[1].state = RigidContact::ContactState::kNotContacting;
  contacts[1].mu = pdae.get_mu_coulomb();
  contacts[1].u = Eigen::Vector3d(pdae.get_rod_half_length(), 0, 0);

  // Set constant input forces for both.
  const double x = 1.0;
  Vector3<double> fext(x, 0, x * ts.get_rod_half_length());
  std::unique_ptr<BasicVector<double>> ext_input =
    std::make_unique<BasicVector<double>>(fext);
  context_ts->FixInputPort(0, std::move(ext_input));
  ext_input = std::make_unique<BasicVector<double>>(fext);
  context_pdae->FixInputPort(0, std::move(ext_input));

  // Init the simulator for the time stepping system.
  Simulator<double> simulator_ts(ts, std::move(context_ts));

  // Integrate forward by a single *large* dt. Note that the update rate
  // is set by the time stepping system, so stepping to dt should yield
  // exactly one step.
  simulator_ts.StepTo(dt);
  EXPECT_EQ(simulator_ts.get_num_discrete_updates(), 1);

  // Manually integrate the continuous state forward for the piecewise DAE
  // based approach.
  std::unique_ptr<ContinuousState<double>> f = pdae.AllocateTimeDerivatives();
  pdae.CalcTimeDerivatives(*context_pdae, f.get());
  xc[3] += + dt * ((*f)[3]);
  xc[4] += + dt * ((*f)[4]);
  xc[5] += + dt * ((*f)[5]);
  xc[0] += + dt * xc[3];
  xc[1] += + dt * xc[4];
  xc[2] += + dt * xc[5];

  // Check that the solution is nearly identical.
  const double tol = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(xc[0], xd[0], tol);
  EXPECT_NEAR(xc[1], xd[1], tol);
  EXPECT_NEAR(xc[2], xd[2], tol);
  EXPECT_NEAR(xc[3], xd[3], tol);
  EXPECT_NEAR(xc[4], xd[4], tol);
  EXPECT_NEAR(xc[5], xd[5], tol);
}
*/

// Class for testing the Rod 2D example using compliant contact
// thus permitting integration as an ODE.
class Rod2DCompliantTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SimulationType::kCompliant, 0.0);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();

    // Use a non-unit mass.
    dut_->set_rod_mass(2.0);

    // Using default compliant contact parameters.

    // Set a zero input force (this is the default).
    std::unique_ptr<BasicVector<double>> ext_input =
        std::make_unique<BasicVector<double>>(3);
    ext_input->SetAtIndex(0, 0.0);
    ext_input->SetAtIndex(1, 0.0);
    ext_input->SetAtIndex(2, 0.0);
    context_->FixInputPort(0, std::move(ext_input));
  }

  // Calculate time derivatives using the context member and writing to
  // the derivatives member.
  void CalcTimeDerivatives() {
    dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  }

  std::unique_ptr<State<double>> CloneState() const {
    return context_->CloneState();
  }

  // Return the state x,y,θ,xdot,ydot,θdot as a Vector6.
  Vector6d get_state() const {
    const ContinuousState<double>& xc = *context_->get_continuous_state();
    return Vector6d(xc.CopyToVector());
  }

  // Return d/dt state xdot,ydot,θdot,xddot,yddot,θddot as a Vector6.
  Vector6d get_state_dot() const {
    const ContinuousState<double>& xcd = *derivatives_;
    return Vector6d(xcd.CopyToVector());
  }

  // Sets the planar pose in the context.
  void set_pose(double x, double y, double theta) {
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    xc[0] = x; xc[1] = y; xc[2] = theta;
  }

  // Sets the planar velocity in the context.
  void set_velocity(double xdot, double ydot, double thetadot) {
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    xc[3] = xdot; xc[4] = ydot; xc[5] = thetadot;
  }

  // Returns planar pose derivative (should be planar velocities xdot,
  // ydot, thetadot).
  Vector3d get_pose_dot() const {
    const ContinuousState<double>& xcd = *derivatives_;
    return Vector3d(xcd[0], xcd[1], xcd[2]);
  }

  // Returns planar acceleration (xddot, yddot, thetaddot).
  Vector3d get_accelerations() const {
    const ContinuousState<double>& xcd = *derivatives_;
    return Vector3d(xcd[3], xcd[4], xcd[5]);
  }

  // Sets the rod to a state that corresponds to ballistic motion.
  void SetBallisticState() {
    const double half_len = dut_->get_rod_half_length();
    set_pose(0, 10*half_len, M_PI_2);
    set_velocity(1, 2, 3);
  }

  // Sets the rod to a perfectly vertical position in which either the left or
  // right endpoint is contacting, or a perfectly horizontal position in which
  // both endpoints are contacting. In all cases the penetration depth into the
  // halfplane is set to 1 cm, and the rod velocity is set to zero.
  // k=-1,0,1 -> left, both, right.
  void SetContactingState(int k) {
    DRAKE_DEMAND(-1 <= k && k <= 1);
    const double half_len = dut_->get_rod_half_length();
    const double penetration = 0.01;  // 1 cm
    set_pose(0, std::abs(k)*half_len - penetration, -k*M_PI_2);
    set_velocity(0, 0, 0);
  }

  std::unique_ptr<Rod2D<double>> dut_;  //< The device under test.
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
};

// Verify that the compliant contact resists penetration.
TEST_F(Rod2DCompliantTest, ForcesHaveRightSign) {
  // We expect only roundoff errors, scaled by force magnitude (~1e-14).
  const double kTightTol = 50 * std::numeric_limits<double>::epsilon();

  SetContactingState(-1);  // left

  Vector3d F_Ro_W_left = dut_->CalcCompliantContactForces(*context_);

  CalcTimeDerivatives();
  Vector6d xcd = get_state_dot();

  EXPECT_EQ(xcd[0], 0);  // xdot, ydot, thetadot
  EXPECT_EQ(xcd[1], 0);
  EXPECT_EQ(xcd[2], 0);

  // Total acceleration is gravity plus acceleration due to contact forces;
  // extract just the contact contribution. It should point up!
  const double a_contact = xcd[4] - dut_->get_gravitational_acceleration();

  // Vertical acceleration is >> 1; just checking for correct sign. Rod is
  // vertical so horizontal and angular accelerations are zero.
  EXPECT_NEAR(xcd[3], 0, kTightTol);  // no x acceleration
  EXPECT_GT(a_contact, 1.);           // + y acceleration
  EXPECT_NEAR(xcd[5], 0, kTightTol);  // no angular acceleration

  // Now add some downward velocity; that should *increase* the force we
  // calculated above from just penetration. We're checking that the sign is
  // correct by making the penetration rate large enough to at least double
  // the overall force.
  set_velocity(0, -10, 0);
  Vector3d F_Ro_W_ldown = dut_->CalcCompliantContactForces(*context_);
  EXPECT_GT(F_Ro_W_ldown[1], 2*F_Ro_W_left[1]);  // Did it double?

  // An extreme upwards velocity should be a "pull out" situation resulting
  // in (exactly) zero force rather than a negative force.
  set_velocity(0, 1000, 0);
  Vector3d F_Ro_W_lup = dut_->CalcCompliantContactForces(*context_);
  EXPECT_TRUE(F_Ro_W_lup == Vector3d::Zero());

  // Sliding -x should produce a +x friction force and a positive torque;
  // no effect on y force.
  set_velocity(-10, 0, 0);
  Vector3d F_Ro_W_nx = dut_->CalcCompliantContactForces(*context_);
  EXPECT_GT(F_Ro_W_nx[0], 1.);
  EXPECT_NEAR(F_Ro_W_nx[1], F_Ro_W_left[1], kTightTol);
  EXPECT_GT(F_Ro_W_nx[2], 1.);

  // Sliding +x should produce a -x friction force and a negative torque;
  // no effect on y force.
  set_velocity(10, 0, 0);
  Vector3d F_Ro_W_px = dut_->CalcCompliantContactForces(*context_);
  EXPECT_LT(F_Ro_W_px[0], -1.);
  EXPECT_NEAR(F_Ro_W_px[1], F_Ro_W_left[1], kTightTol);
  EXPECT_LT(F_Ro_W_px[2], -1.);


  SetContactingState(1);  // Right should behave same as left.
  Vector3d F_Ro_W_right = dut_->CalcCompliantContactForces(*context_);
  EXPECT_TRUE(F_Ro_W_right.isApprox(F_Ro_W_left, kTightTol));

  // With both ends in contact the force should double and there should
  // be zero moment.
  SetContactingState(0);
  Vector3d F_Ro_W_both = dut_->CalcCompliantContactForces(*context_);
  EXPECT_TRUE(F_Ro_W_both.isApprox(F_Ro_W_left+F_Ro_W_right, kTightTol));
  EXPECT_NEAR(F_Ro_W_both[2], 0., kTightTol);
}

// Validates the number of witness functions is determined correctly.
TEST_F(Rod2DCompliantTest, NumWitnessFunctions) {
  EXPECT_EQ(dut_->get_witness_functions(*context_).size(), 0);
}

}  // namespace rod2d
}  // namespace examples
}  // namespace drake
