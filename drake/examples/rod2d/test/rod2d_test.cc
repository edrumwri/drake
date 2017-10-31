#include "drake/examples/rod2d/rod2d.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/constraint/constraint_problem_data.h"
#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"

using drake::multibody::constraint::ConstraintAccelProblemData;
using drake::multibody::constraint::ConstraintVelProblemData;
using drake::systems::VectorBase;
using drake::systems::BasicVector;
using drake::systems::ContinuousState;
using drake::systems::State;
using drake::systems::SystemOutput;
using drake::systems::AbstractValues;
using drake::systems::Simulator;
using drake::systems::SemiExplicitEulerIntegrator;
using drake::systems::Context;
using drake::systems::WitnessFunction;
using drake::systems::DiscreteValues;
using drake::systems::rendering::PoseVector;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Vector6d = Eigen::Matrix<double, 6, 1>;

namespace drake {
namespace examples {
namespace rod2d {
namespace {

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
    std::vector<multibody::constraint::PointContact>& contacts =
        dut_->get_contacts(context_->get_mutable_state());
    contacts.resize(1);

    // First point is Rl in Rod Frame (see class documentation); in this
    // configuration, it contacts the half-space without sliding.
    contacts.front().sliding = false;
    contacts.front().id = 0;
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

    // Determine the point of contact.
    std::vector<multibody::constraint::PointContact>& contacts =
        dut_->get_contacts(context_->get_mutable_state());
    contacts.resize(1);
    contacts[0].id = reinterpret_cast<void*>(1L);           // Point is Rr
    contacts[0].sliding = true;
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
    std::vector<multibody::constraint::PointContact>& contacts =
        dut_->get_contacts(context_->get_mutable_state());
    contacts.clear(); 
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
    xc[3] = xc[4] = xc[5] = 0.0;   // velocity variables
  }

  // Sets the rod to a resting vertical configuration without modifying the
  // mode variables.
  void SetRestingVerticalConfig() {
    ContinuousState<double>& xc =
        *context_->get_mutable_continuous_state();
    xc[0] = 0.0;                             // com horizontal position
    xc[1] = dut_->get_rod_half_length();     // com vertical position
    xc[2] = M_PI_2;                          // rod rotation
    xc[3] = xc[4] = xc[5] = 0.0;             // velocity variables
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

  // Computes rigid impact data.
  void CalcRigidImpactVelProblemData(
      ConstraintVelProblemData<double>* data) {
    // Get the points of contact.
    std::vector<Vector2d> contacts;
    dut_->GetContactPoints(*context_, &contacts);

    // Compute the problem data.
    dut_->CalcImpactProblemData(*context_, contacts, data);
  }

  // Computes rigid contact data.
  void CalcConstraintAccelProblemData(
      ConstraintAccelProblemData<double>* data) {
    // Get the points of contact and contact tangent velocities.
    std::vector<Vector2d> contacts;
    std::vector<double> tangent_vels;
    dut_->GetContactPoints(*context_, &contacts);
    dut_->GetContactPointsTangentVelocities(*context_, contacts, &tangent_vels);

    // Compute the problem data.
    dut_->CalcConstraintProblemData(
        *context_, contacts, tangent_vels, data);
  }

  // Models an impact.
  void ModelImpact() {
    ConstraintVelProblemData<double> data(3 /* ngc */);
    CalcRigidImpactVelProblemData(&data);
    VectorX<double> cf;
    contact_solver_.SolveImpactProblem(data, &cf);

    // Get the update to the generalized velocity.
    VectorX<double> delta_v;
    contact_solver_.ComputeGeneralizedVelocityChange(data, cf, &delta_v);

    // Update the velocity part of the state.
    context_->get_mutable_continuous_state()->get_mutable_generalized_velocity()
        ->SetFromVector(data.solve_inertia(data.Mv) + delta_v);
  }

  // Gets the number of generalized coordinates for the rod.
  int get_rod_num_coordinates() { return 3; }

  // Gets the output dimension of a Jacobian multiplication operator.
  int GetOperatorDim(std::function<VectorX<double>(const VectorX<double>&)> J) {
    return J(VectorX<double>(get_rod_num_coordinates())).size();
  }

  // Checks the consistency of a transpose operator.
  void CheckTransOperatorDim(
      std::function<VectorX<double>(const VectorX<double>&)> JT,
  int num_constraints) {
    EXPECT_EQ(JT(VectorX<double>(num_constraints)).size(),
              get_rod_num_coordinates());
  }

  // Checks consistency of rigid contact problem data.
  void CheckProblemConsistency(
      const ConstraintAccelProblemData<double>& data,
      int num_contacts) {
    EXPECT_EQ(num_contacts, data.sliding_contacts.size() +
        data.non_sliding_contacts.size());
    EXPECT_EQ(GetOperatorDim(data.N_mult), num_contacts);
    CheckTransOperatorDim(data.N_minus_muQ_transpose_mult, num_contacts);
    EXPECT_EQ(GetOperatorDim(data.L_mult), data.kL.size());
    CheckTransOperatorDim(data.L_transpose_mult, data.kL.size());
    EXPECT_EQ(data.tau.size(), get_rod_num_coordinates());
    EXPECT_EQ(data.kN.size(), num_contacts);
    EXPECT_EQ(data.kF.size(), data.non_sliding_contacts.size());
    EXPECT_EQ(data.mu_non_sliding.size(), data.non_sliding_contacts.size());
    EXPECT_EQ(data.mu_sliding.size(), data.sliding_contacts.size());
    EXPECT_EQ(data.r.size(), data.non_sliding_contacts.size());
    EXPECT_TRUE(data.solve_inertia);
    EXPECT_TRUE(std::is_sorted(data.sliding_contacts.begin(),
                               data.sliding_contacts.end()));
    EXPECT_TRUE(std::is_sorted(data.non_sliding_contacts.begin(),
                               data.non_sliding_contacts.end()));

    // Only true because this problem is 2D.
    EXPECT_EQ(GetOperatorDim(data.F_mult), data.non_sliding_contacts.size());
    CheckTransOperatorDim(data.F_transpose_mult,
                          data.non_sliding_contacts.size());
  }

  // Checks consistency of rigid impact problem data.
  void CheckProblemConsistency(
      const ConstraintVelProblemData<double>& data,
      int num_contacts) {
    EXPECT_EQ(num_contacts, data.mu.size());
    EXPECT_EQ(num_contacts, data.r.size());
    EXPECT_EQ(GetOperatorDim(data.N_mult), num_contacts);
    CheckTransOperatorDim(data.N_transpose_mult, num_contacts);
    EXPECT_EQ(data.kN.size(), num_contacts);
    EXPECT_EQ(data.Mv.size(), get_rod_num_coordinates());
    EXPECT_TRUE(data.solve_inertia);
    EXPECT_EQ(GetOperatorDim(data.F_mult), num_contacts);
    CheckTransOperatorDim(data.F_transpose_mult, num_contacts);
    EXPECT_EQ(data.kF.size(), num_contacts);
    EXPECT_EQ(GetOperatorDim(data.L_mult), data.kL.size());
    CheckTransOperatorDim(data.L_transpose_mult, data.kL.size());
  }

  std::unique_ptr<Rod2D<double>> dut_;  //< The device under test.
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
  drake::multibody::constraint::ConstraintSolver<double>
      contact_solver_;
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
  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

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

  // Prepare to set contact states.
  std::vector<multibody::constraint::PointContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  contacts.resize(1);

  // Contact point is Rl in Rod Frame (see class documentation); in this
  // new configuration, it contacts the half-space without sliding.
  contacts.front().sliding = false;
  contacts.front().id = reinterpret_cast<void*>(0L); 

  // Rod should be impacting.
  EXPECT_TRUE(dut_->IsImpacting(context_->get_state()));

  // Handle the impact.
  dut_->HandleImpact(*context_, new_state.get());
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

/*
// Verify that derivatives match what we expect from a non-inconsistent
// contacting configuration.
TEST_F(Rod2DDAETest, ConsistentDerivativesContacting) {
  // Calculate the derivatives.
  SetUprightRestingState();
  std::vector<multibody::constraint::PointContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  EXPECT_EQ(contacts.size(), 1);
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
  contacts[0].sliding = true;
  dut_->set_mu_coulomb(0.0);
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
  std::vector<multibody::constraint::PointContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  contacts.resize(1);

  // Point is Rl in Rod Frame (see class documentation); in this
  // new configuration, it contacts the half-space without sliding.
  contacts.front().sliding = false;
  contacts.front().id = reinterpret_cast<void*>(0L); 

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

  // First, redetermine the acceleration-level active set.
  dut_->DetermineAccelLevelActiveSet(*context_, context_->get_mutable_state());
  EXPECT_TRUE(contacts[0].sliding);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_GT((*derivatives_)[3], tol);  // horizontal accel. should be nonzero.

  // Set the coefficient of friction to zero and try again.
  dut_->set_mu_coulomb(0.0);
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
*/

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
  const std::vector<multibody::constraint::PointContact>& contacts =
      dut_->get_contacts(context_->get_state());
  EXPECT_EQ(contacts.size(), 2);
  EXPECT_TRUE(contacts.front().sliding || contacts.back().sliding);
}

/*
// Verify that applying the impact model to an impacting configuration results
// in a non-impacting configuration. This test exercises the model for the case
// where impulses that yield tangential sticking lie within the friction cone.
TEST_F(Rod2DDAETest, InfFrictionImpactThenNoImpact) {
  // Cause the initial state to be impacting.
  SetImpactingState();

  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

  // Verify that the state is in a single-contact sliding mode.
  std::vector<multibody::constraint::PointContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  EXPECT_EQ(contacts.size(), 1);
  EXPECT_TRUE(contacts[0].sliding);

  // Set the coefficient of friction to infinite. This forces the rod code
  // to go through the first impact path (impulse within the friction cone).
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());

  // Handle the impact, redetermine the active set, and copy the result to the
  // context.
  double zero_tol;
  dut_->ModelImpact(new_state.get(), &zero_tol);
  dut_->DetermineVelLevelActiveSet(new_state.get(), zero_tol);
  context_->get_mutable_state()->SetFrom(*new_state);
  EXPECT_FALSE(dut_->IsImpacting(context_->get_state()));

  // Verify that the state is no longer in a sliding mode.
  EXPECT_FALSE(contacts[0].sliding);

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
  std::vector<multibody::constraint::PointContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  EXPECT_EQ(contacts.size(), 1);
  EXPECT_TRUE(contacts.front().sliding);

  // Set the coefficient of friction to zero. This forces the rod code
  // to go through the second impact path (impulse corresponding to sticking
  // friction post-impact lies outside of the friction cone).
  dut_->set_mu_coulomb(0.0);

  // Handle the impact and copy the result to the context.
  std::unique_ptr<State<double>> new_state = CloneState();
  double zero_tol;
  dut_->ModelImpact(new_state.get(), &zero_tol);
  dut_->DetermineVelLevelActiveSet(new_state.get(), zero_tol);
  context_->get_mutable_state()->SetFrom(*new_state);
  EXPECT_FALSE(dut_->IsImpacting(context_->get_state()));

  // Verify that the state is still in a sliding mode.
  EXPECT_EQ(contacts.size(), 1);
  EXPECT_TRUE(contacts.front().sliding);

  // Do one more impact- there should now be no change.
  // Verify that there is no further change from this second impact.
  dut_->ModelImpact(new_state.get(), &zero_tol);
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              10 * std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));

  // Redetermine the active set and verify that the state is still in a sliding
  // mode.
  dut_->DetermineVelLevelActiveSet(new_state.get(), zero_tol);
  EXPECT_EQ(contacts.size(), 1);
  EXPECT_TRUE(contacts.front().sliding);
}
*/

// Verify that no exceptions thrown for a non-sliding configuration.
TEST_F(Rod2DDAETest, NoSliding) {
  const double half_len = dut_->get_rod_half_length();
  const double r22 = std::sqrt(2) / 2;
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();

  // Set the coefficient of friction to zero (triggering the case on the
  // edge of the friction cone).
  dut_->set_mu_coulomb(0.0);
  std::vector<multibody::constraint::PointContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  EXPECT_EQ(contacts.size(), 1);  

  // This configuration has no sliding velocity.
  xc[0] = -half_len * r22;
  xc[1] = half_len * r22;
  xc[2] = 3 * M_PI / 4.0;
  xc[3] = 0.0;
  xc[4] = 0.0;
  xc[5] = 0.0;
  contacts.front().sliding = false;

  // Verify no impact.
  EXPECT_FALSE(dut_->IsImpacting(context_->get_state()));

  // No exceptions should be thrown.
  EXPECT_NO_THROW(dut_->CalcTimeDerivatives(*context_, derivatives_.get()));

  // Set the coefficient of friction to effective no-slip (triggering the
  // case strictly inside the friction cone).
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());

  // No exceptions should be thrown.
  EXPECT_NO_THROW(dut_->CalcTimeDerivatives(*context_, derivatives_.get()));
}

/*
// Test multiple (two-point) contact configurations.
TEST_F(Rod2DDAETest, MultiPoint) {
  ContinuousState<double> &xc =
      *context_->get_mutable_continuous_state();
  std::vector<multibody::constraint::PointContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());

  // Set the rod to a horizontal, two-contact configuration with sliding.
  xc[0] = 0;
  xc[1] = 0;
  xc[2] = 0;

  // Set the velocity on the rod such that it is moving horizontally.
  xc[3] = 1.0;
  EXPECT_EQ(contacts.size(), 2);
  contacts[0].sliding = contacts[1].sliding = true;

  // Verify no impact.
  EXPECT_FALSE(dut_->IsImpacting(context_->get_state()));

  // Set the coefficient of friction to zero.
  dut_->set_mu_coulomb(0.0);

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
  contacts[0].sliding = contacts[1].sliding = false;
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
  dut_->DetermineAccelLevelActiveSet(*context_, context_->get_mutable_state());
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR((*derivatives_)[3], fX/dut_->get_rod_mass(), eps);
  EXPECT_NEAR((*derivatives_)[4], 0, eps);
  EXPECT_NEAR((*derivatives_)[5], 0, eps);
}
*/

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

/*
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
  std::vector<multibody::constraint::PointContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  EXPECT_EQ(contacts.size(), 2);

  // Handle the impact and copy the result to the context.
  double zero_tol;
  dut_->ModelImpact(new_state.get(), &zero_tol);
  dut_->DetermineVelLevelActiveSet(new_state.get(), zero_tol);
  context_->get_mutable_state()->SetFrom(*new_state);

  // Verify the state no longer corresponds to an impact.
  EXPECT_FALSE(dut_->IsImpacting(context_->get_state()));

  // Verify that the state is now in a sticking mode.
  EXPECT_FALSE(contacts[0].sliding);

  // Do one more impact- there should now be no change.
  dut_->ModelImpact(new_state.get());
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state()->get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}
*/

// Verify that applying the impact model to an impacting state results in a
// non-impacting state.
TEST_F(Rod2DDAETest, NoFrictionImpactThenNoImpact2) {
  // Cause the initial state to be impacting.
  SetImpactingState();

  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

  // Cause the initial state to be impacting.
  SetImpactingState();

  // Verify that the state is still in a sliding configuration.
  std::vector<multibody::constraint::PointContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  EXPECT_EQ(contacts.size(), 1);
  EXPECT_TRUE(contacts.front().sliding);

  // Set the coefficient of friction to zero. This forces the rod code
  // to go through the second impact path.
  dut_->set_mu_coulomb(0.0);

  // Verify that the state is still in a sliding configuration.
  EXPECT_TRUE(contacts.front().sliding);

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
  EXPECT_TRUE(contacts.front().sliding);
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
  std::vector<multibody::constraint::PointContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  EXPECT_TRUE(contacts.empty());

  // Verify that no impact occurs.
  EXPECT_FALSE(dut_->IsImpacting(context_->get_state()));
}

// Validates the set of witness functions is determined correctly.
TEST_F(Rod2DDAETest, NumWitnessFunctions) {
  // Get the set of contacts.
  std::vector<multibody::constraint::PointContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());

  // Verify that the correct number of witness functions is reported for...
  // (a) Sliding single contact (default initial state).
  std::vector<const WitnessFunction<double>*> witnesses;
  dut_->GetWitnessFunctions(*context_, &witnesses);
  EXPECT_EQ(witnesses.size(), 3);

  // (b) Ballistic motion.
  SetBallisticState();
  witnesses.clear();
  dut_->GetWitnessFunctions(*context_, &witnesses);
  EXPECT_EQ(witnesses.size(), 2);

  // (c) Sticking single contact.
  contacts.resize(1);
  contacts.front().sliding = false;
  witnesses.clear();
  dut_->GetWitnessFunctions(*context_, &witnesses);
  EXPECT_EQ(witnesses.size(), 3);

  // (d) Sticking two contacts.
  contacts.resize(2);
  contacts.back().sliding = false;
  witnesses.clear();
  dut_->GetWitnessFunctions(*context_, &witnesses);
  EXPECT_EQ(witnesses.size(), 4);

  // (e) Sliding two contacts.
  contacts.front().sliding = contacts.back().sliding = true;
  witnesses.clear();
  dut_->GetWitnessFunctions(*context_, &witnesses);
  EXPECT_EQ(witnesses.size(), 4);
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
  std::vector<multibody::constraint::PointContact>& contacts =
      dut_->get_contacts(context_->get_mutable_state());
  ASSERT_EQ(contacts.size(), 1);
  contacts[0].sliding = false;

  // Verify that the "slack" is positive.
  const double inf = std::numeric_limits<double>::infinity();
  dut_->set_mu_coulomb(inf);
  const double w0 = dut_->sticking_friction_forces_slack_witnesses_[0]->
                            Evaluate(*context_);
  EXPECT_GT(w0, 0);

  // Set the coefficient of friction to zero.
  dut_->set_mu_coulomb(0.0);

  // Verify that the "slack" is negative.
  const double wf = dut_->sticking_friction_forces_slack_witnesses_[0]->
                            Evaluate(*context_);
  EXPECT_LT(wf, 0);

  // Verify that the witness triggers.
  EXPECT_TRUE(dut_->sticking_friction_forces_slack_witnesses_[0]->
      should_trigger(w0, wf));

}

// Verifies that the rigid contact problem data has reasonable values when the
// rod is in a ballistic state.
TEST_F(Rod2DDAETest, RigidContactProblemDataBallistic) {
  SetBallisticState();

  // Compute the problem data.
  ConstraintAccelProblemData<double> data(get_rod_num_coordinates());
  CalcConstraintAccelProblemData(&data);

  // Verify that the data has reasonable values.
  const int num_contacts = 0;
  CheckProblemConsistency(data, num_contacts);
}

// Verifies that the rigid contact problem data has reasonable values when the
// rod is in a two-contact, at-rest configuration.
TEST_F(Rod2DDAETest, RigidContactProblemDataHorizontalResting) {
  // Set the rod to a resting horizontal configuration.
  SetRestingHorizontalConfig();

  // Compute the problem data.
  ConstraintAccelProblemData<double> data(3 /* gen. vel. dim */);
  CalcConstraintAccelProblemData(&data);
  const int num_contacts = 2;
  CheckProblemConsistency(data, num_contacts);

  // Verify that both contacts are not sliding.
  EXPECT_EQ(data.non_sliding_contacts.size(), num_contacts);
}

// Verifies that the rigid contact problem data has reasonable values when the
// rod is in a two-contact, sliding configuration.
TEST_F(Rod2DDAETest, RigidContactProblemDataHorizontalSliding) {
  // Set the rod to a sliding horizontal configuration.
  SetRestingHorizontalConfig();
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();
  xc[3] = 1.0;  // horizontal velocity of the rod center-of-mass.

  // Compute the problem data.
  ConstraintAccelProblemData<double> data(3 /* gen. vel. dim */);
  CalcConstraintAccelProblemData(&data);
  const int num_contacts = 2;
  CheckProblemConsistency(data, num_contacts);

  // Verify that both contacts are sliding.
  EXPECT_EQ(data.sliding_contacts.size(), num_contacts);
}

// Verifies that the rigid contact problem data has reasonable values when the
// rod is in a single-contact, at-rest configuration.
TEST_F(Rod2DDAETest, RigidContactProblemDataVerticalResting) {
  // Set the rod to a resting vertical configuration.
  SetRestingVerticalConfig();

  // Compute the problem data.
  ConstraintAccelProblemData<double> data(3 /* gen. vel. dim */);
  CalcConstraintAccelProblemData(&data);
  const int num_contacts = 1;
  CheckProblemConsistency(data, num_contacts);

  // Verify that contact is not sliding.
  EXPECT_EQ(data.non_sliding_contacts.size(), num_contacts);

  // TODO(edrumwri): Most tests are necessary to better stress the validity of
  // N, N - μQ, F, dN/dt⋅v, and dF/dt⋅v.
  // Verify that N has no angular component.
  const double eps = 100 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(data.N_mult(Eigen::Vector3d::UnitZ())(0), 0, eps);
}

// Verifies that the rigid contact problem data has reasonable values when the
// rod is in a two-contact, sliding configuration.
TEST_F(Rod2DDAETest, RigidContactProblemDataVerticalSliding) {
  // Set the rod to a sliding vertical configuration.
  SetRestingVerticalConfig();
  ContinuousState<double>& xc =
      *context_->get_mutable_continuous_state();
  xc[3] = 1.0;

  // Compute the problem data.
  ConstraintAccelProblemData<double> data(3 /* gen. vel. dim */);
  CalcConstraintAccelProblemData(&data);
  const int num_contacts = 1;
  CheckProblemConsistency(data, num_contacts);

  // Verify that contact is sliding.
  EXPECT_EQ(data.sliding_contacts.size(), num_contacts);
}

/// Class for testing the Rod 2D example using a first order time
/// stepping approach.
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

/// Verify that Rod 2D system eventually goes to rest using the
/// first-order time stepping approach (this tests expected meta behavior).
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
  std::vector<const WitnessFunction<double>*> witnesses;
  dut_->GetWitnessFunctions(*context_, &witnesses);
  EXPECT_EQ(witnesses.size(), 0);
}

// This test checks to see whether a single semi-explicit step of the piecewise
// DAE based Rod2D system is equivalent to a single step of the semi-explicit
// time stepping based system.
GTEST_TEST(Rod2DCrossValidationTest, OneStepSolutionSliding) {
  // Create two Rod2D systems.
  const double dt = 1e-1;
  Rod2D<double> ts(Rod2D<double>::SimulationType::kTimeStepping, dt);
  Rod2D<double> pdae(Rod2D<double>::SimulationType::kPiecewiseDAE, 0.0);

  // Set the coefficient of friction to a small value for both.
  const double mu = 0.01;
  ts.set_mu_coulomb(mu);
  pdae.set_mu_coulomb(mu);

  // Set "one step" constraint stabilization (not generally recommended, but
  // works for a single step) and small regularization.
  ts.set_cfm(std::numeric_limits<double>::epsilon());
  ts.set_erp(1.0);

  // Create contexts for both.
  std::unique_ptr<Context<double>> context_ts = ts.CreateDefaultContext();
  std::unique_ptr<Context<double>> context_pdae = pdae.CreateDefaultContext();

  // Set zero input forces for both.
  Vector3<double> fext(0, 0, 0);
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
  auto xc = context_pdae->get_mutable_continuous_state_vector();
  xc->SetAtIndex(3, xc->GetAtIndex(3) + dt * ((*f)[3]));
  xc->SetAtIndex(4, xc->GetAtIndex(4) + dt * ((*f)[4]));
  xc->SetAtIndex(5, xc->GetAtIndex(5) + dt * ((*f)[5]));
  xc->SetAtIndex(0, xc->GetAtIndex(0) + dt * xc->GetAtIndex(3));
  xc->SetAtIndex(1, xc->GetAtIndex(1) + dt * xc->GetAtIndex(4));
  xc->SetAtIndex(2, xc->GetAtIndex(2) + dt * xc->GetAtIndex(5));

  // See whether the states are equal.
  const Context<double>& context_ts_new = simulator_ts.get_context();
  const auto& xd = context_ts_new.get_discrete_state(0)->get_value();

  // Check that the solution is nearly identical.
  const double tol = std::numeric_limits<double>::epsilon() * 10;
  EXPECT_NEAR(xc->GetAtIndex(0), xd[0], tol);
  EXPECT_NEAR(xc->GetAtIndex(1), xd[1], tol);
  EXPECT_NEAR(xc->GetAtIndex(2), xd[2], tol);
  EXPECT_NEAR(xc->GetAtIndex(3), xd[3], tol);
  EXPECT_NEAR(xc->GetAtIndex(4), xd[4], tol);
  EXPECT_NEAR(xc->GetAtIndex(5), xd[5], tol);

  // TODO(edrumwri): Introduce more extensive tests that cross-validate the
  // time-stepping based approach against the piecewise DAE-based approach for
  // the case of sliding contacts at multiple points.
}

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

// Class that adds publishing to the Rod 2D system and uses it to determine
// intervals of continuous time.
class Rod2DPlus : public Rod2D<double> {
 public:
  Rod2DPlus(Rod2D<double>::SimulationType simulation_type, double dt) :
      Rod2D<double>(simulation_type, dt) {}

  // Gets the time steps that the simulator was called upon.
  std::vector<double> get_dts(const Context<double>& context) {
    std::vector<double> dts;
    for (size_t i = 1; i < times_.size(); ++i) {
      if (times_[i] - times_[i-1] > 0)
        dts.push_back(times_[i] - times_[i-1]);
    }

    // See whether to add the time to the end.
    if (context.get_time() > times_.back())
      dts.push_back(context.get_time() - times_.back());

    return dts;
  }

 protected:
  void DoPublish(const systems::Context<double>& context) const override {
    times_.push_back(context.get_time());
  }

 private:
  // Times at which the publisher was called; assumes publishing is called
  // at the endpoints of continuous interval.
  mutable std::vector<double> times_;
};

// Steps a rod forward with a non-uniform step using time stepping. Used
// for cross validation tests to ensure that time stepping and piecewise DAE
// are integrated from the same starting points.
void StepNonuniform(const Rod2D<double>& rod_ts, Context<double>* context,
                    double dt) {
  // Get the existing context.
  DiscreteValues<double>* xd = context->get_mutable_discrete_state();

  // Create a new time stepping system.
  Rod2D<double> rod(Rod2D<double>::SimulationType::kTimeStepping, dt);

  // Set pertiennt data.
  rod.set_rod_half_length(rod_ts.get_rod_half_length());
  rod.set_rod_mass(rod_ts.get_rod_mass());
  rod.set_rod_moment_of_inertia(rod_ts.get_rod_moment_of_inertia());
  rod.set_cfm(rod_ts.get_cfm());
  rod.set_erp(rod_ts.get_erp());
  rod.set_mu_coulomb(rod_ts.get_mu_coulomb());

  // Allocate variables.
  std::unique_ptr<DiscreteValues<double>> discrete_updates = rod.
      AllocateDiscreteVariables();

  // Create the event.
  systems::DiscreteEvent<double> event;
  event.action = systems::DiscreteEvent<double>::kDiscreteUpdateAction;

  // Compute the update.
  rod.CalcDiscreteVariableUpdates(*context, event,
                                  discrete_updates.get());

  // Then, write them back into the context.
  xd->CopyFrom(*discrete_updates);

  // Update the context.
  context->set_time(context->get_time() + dt);
}

// Class for cross-testing the Rod2D example using piecewise DAE and time
// stepping approaches.
class Rod2DCrossValidationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create both test devices.
    pdae_ = std::make_unique<Rod2DPlus>(
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
    std::vector<multibody::constraint::PointContact>& contacts = pdae_->get_contacts(
        context_pdae->get_mutable_state());

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
    std::vector<multibody::constraint::PointContact>& contacts = pdae_->get_contacts(
        context_pdae->get_mutable_state());
    for (size_t i = 0; i < contacts.size(); ++i)
      contacts[i].state = multibody::constraint::PointContact::ContactState::kNotContacting;
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
    std::vector<multibody::constraint::PointContact>& contacts = pdae_->get_contacts(
        context_pdae->get_mutable_state());
    for (size_t i = 0; i < contacts.size(); ++i)
      contacts[i].state = multibody::constraint::PointContact::ContactState::kContactingWithoutSliding;
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
    std::vector<multibody::constraint::PointContact>& contacts = pdae_->get_contacts(
        context_pdae->get_mutable_state());
    for (size_t i = 0; i < contacts.size(); ++i)
      contacts[i].state = multibody::constraint::PointContact::ContactState::kContactingAndSliding;
  }

  // Gets the horizontal applied forces for both devices.
  virtual double get_horizontal_external_force() const = 0;

  // Gets the coefficient of friction.
  virtual double get_mu_coulomb() const = 0;

 protected:
  // The integration step size (should be large to allow for event transitions).
  const double dt_{0.1};

  std::unique_ptr<Rod2DPlus> pdae_;
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

// This test checks to see whether a piecewise DAE simulation is equal to a
// time stepping simulation over a given interval.
TEST_F(Rod2DCrossValidationSlidingTest, Interval) {
  // Integrate forward to one second.
//  simulator_pdae_->get_mutable_integrator()->set_maximum_step_size(1);
  simulator_pdae_->StepTo(0.100001);
  std::vector<double> dts = pdae_->get_dts(simulator_pdae_->get_context());
  for (size_t i = 0; i < dts.size(); ++i)
    StepNonuniform(*ts_, simulator_ts_->get_mutable_context(), dts[i]);
  std::cout << "Integration steps:";
  for (size_t i = 0; i < dts.size(); ++i)
    std::cout << " " << dts[i];
  std::cout << std::endl;

  // See whether the states are equal.
  const Context<double>& context_ts = simulator_ts_->get_context();
  const Context<double>& context_pdae = simulator_pdae_->get_context();
  const auto& xd = context_ts.get_discrete_state(0)->get_value();
  const auto& xc = context_pdae.get_continuous_state_vector();

  // Check that the solution is nearly identical.
//  const double tol = std::numeric_limits<double>::epsilon() * 10;
  const double tol = 0;
  EXPECT_NEAR(context_ts.get_time(), context_pdae.get_time(), tol);
  EXPECT_NEAR(xc[0], xd[0], tol);
  EXPECT_NEAR(xc[1], xd[1], tol);
  EXPECT_NEAR(xc[2], xd[2], tol);
  EXPECT_NEAR(xc[3], xd[3], tol);
  EXPECT_NEAR(xc[4], xd[4], tol);
  EXPECT_NEAR(xc[5], xd[5], tol);

  // Verify that the signed distance is correct for the piecewise DAE
  // approach.
  Rod2D<double>* rod = (Rod2D<double>*) pdae_.get();
  EXPECT_GT(rod->signed_distance_witnesses_[0]->Evaluate(context_pdae), -tol);
  EXPECT_GT(rod->signed_distance_witnesses_[1]->Evaluate(context_pdae), -tol);
}

// This test checks to see whether a simulation step of the piecewise
// DAE based Rod2D system is equivalent to a single step of the semi-explicit
// time stepping based system for two sliding contacts *without any transitions
// from sliding to not-sliding*.
TEST_F(Rod2DCrossValidationSlidingTest, OneStepSolutionTwoSliding) {
  // Set the rods to a sidewise configuration.
  set_horizontal_motionless_configuration();

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

class Rod2DCrossValidationLargeMuSlidingTest : public Rod2DCrossValidationTest {
 protected:
  double get_horizontal_external_force() const override { return 0; }
  double get_mu_coulomb() const override { return 1; }
};

// This test checks solution equality when the rod starts with sliding and
// transitions to non-sliding.
TEST_F(Rod2DCrossValidationLargeMuSlidingTest, OneStepSolutionSliding) {
  set_horizontal_sliding_configuration();

  // Integrate forward by a single *large* dt. Note that the update rate
  // is set by the time stepping system, so stepping to dt should yield
  // exactly one step.
  simulator_pdae_->StepTo(dt_ * 2);
  std::vector<double> dts = pdae_->get_dts(simulator_pdae_->get_context());
  for (size_t i = 0; i < dts.size(); ++i)
    StepNonuniform(*ts_, simulator_ts_->get_mutable_context(), dts[i]);

  // See whether the states are equal.
  const Context<double>& context_ts = simulator_ts_->get_context();
  const Context<double>& context_pdae = simulator_pdae_->get_context();
  const auto& xd = context_ts.get_discrete_state(0)->get_value();
  const auto& xc = context_pdae.get_continuous_state_vector();

  // Check that the solutions are nearly identical.
  const double tol = std::numeric_limits<double>::epsilon() * 10;
  EXPECT_NEAR(context_ts.get_time(), context_pdae.get_time(), tol);
  EXPECT_NEAR(xc[0], xd[0], tol);
  EXPECT_NEAR(xc[1], xd[1], tol);
  EXPECT_NEAR(xc[2], xd[2], tol);
  EXPECT_NEAR(xc[3], xd[3], tol);
  EXPECT_NEAR(xc[4], xd[4], tol);
  EXPECT_NEAR(xc[5], xd[5], tol);
}

class Rod2DCrossValidationFrictionlessSlidingTest : public
                                                    Rod2DCrossValidationTest {
 protected:
  double get_horizontal_external_force() const override { return 1; }
  double get_mu_coulomb() const override { return 0; }
};

// This test checks to see whether a simulation step of the piecewise
// DAE based Rod2D system is equivalent to a single step of the semi-explicit
// time stepping based system for two sliding contacts *including a transition
// from not-sliding to sliding*.
TEST_F(Rod2DCrossValidationFrictionlessSlidingTest, OneStepSolutionTwoSliding) {
  // Set the rods to a sidewise configuration.
  set_horizontal_motionless_configuration();

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
  void SetUp() override {
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
    std::vector<multibody::constraint::PointContact>& contacts = pdae_->get_contacts(
        context_pdae->get_mutable_state());
    for (size_t i = 0; i < contacts.size(); ++i) {
      if (contacts[i].state ==
          multibody::constraint::PointContact::ContactState::kContactingAndSliding) {
        contacts[i].state =
            multibody::constraint::PointContact::ContactState::kContactingWithoutSliding;
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
  std::vector<multibody::constraint::PointContact>& contacts =
      pdae.get_contacts(context_pdae->get_mutable_state());
  EXPECT_EQ(contacts.size(), 2);

  // First point is Rl in Rod Frame (see class documentation); in this
  // new configuration, it contacts the half-space without sliding.
  contacts[0].state = multibody::constraint::PointContact::ContactState::kContactingWithoutSliding;
  contacts[0].mu = pdae.get_mu_coulomb();
  contacts[0].u = Eigen::Vector3d(-pdae.get_rod_half_length(), 0, 0);

  // Second point is Rr in Rod Frame. In this new configuration; in this
  // new configuration, it does not contact the half-space..
  contacts[1].state = multibody::constraint::PointContact::ContactState::kNotContacting;
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
  std::vector<const WitnessFunction<double>*> witnesses;
  dut_->GetWitnessFunctions(*context_, &witnesses);
  EXPECT_EQ(witnesses.size(), 0);
}

// Verifies that output ports give expected values.
GTEST_TEST(Rod2DCrossValidationTest, Outputs) {
  // Create two Rod2D systems, one time stepping, one with continuous state.
  const double dt = 1e-1;
  Rod2D<double> ts(Rod2D<double>::SimulationType::kTimeStepping, dt);
  Rod2D<double> pdae(Rod2D<double>::SimulationType::kPiecewiseDAE, 0.0);

  // Create contexts for both.
  std::unique_ptr<Context<double>> context_ts = ts.CreateDefaultContext();
  std::unique_ptr<Context<double>> context_pdae = pdae.CreateDefaultContext();

  // Allocate outputs for both.
  auto output_ts = ts.AllocateOutput(*context_ts);
  auto output_pdae = pdae.AllocateOutput(*context_pdae);

  // Compute outputs.
  ts.CalcOutput(*context_ts, output_ts.get());
  pdae.CalcOutput(*context_pdae, output_pdae.get());

  // Set port indices.
  const int state_port = 0;
  const int pose_port = 1;

  // Verify that state outputs are identical.
  const double eq_tol = 10 * std::numeric_limits<double>::epsilon();
  const VectorXd x_ts = output_ts->get_vector_data(state_port)->CopyToVector();
  VectorXd x_pdae = output_pdae->get_vector_data(state_port)->CopyToVector();
  EXPECT_LT((x_ts - x_pdae).lpNorm<Eigen::Infinity>(), eq_tol);

  // Transform the rod and verify that pose output is as expected.
  x_pdae[0] = 0;
  x_pdae[1] = pdae.get_rod_half_length();
  x_pdae[2] = M_PI_2;
  context_pdae->get_mutable_continuous_state()->SetFromVector(x_pdae);
  pdae.CalcOutput(*context_pdae, output_pdae.get());

  // Rotation by theta is converted to rotation around +y by theta + π/2.
  const PoseVector<double>* const pose = dynamic_cast<
      PoseVector<double>*>(output_pdae->GetMutableVectorData(pose_port));
  const Eigen::Quaterniond quat = pose->get_rotation();
  EXPECT_NEAR(quat.y(), 1, eq_tol);

  // -- Translation along +y is converted to translation along +z.
  const auto translation = pose->get_translation();
  EXPECT_NEAR(translation.z(), pdae.get_rod_half_length(), eq_tol);
}

}  // namespace
}  // namespace rod2d
}  // namespace examples
}  // namespace drake
