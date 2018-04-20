#include "drake/examples/rod2d/rod2d.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/constraint/constraint_problem_data.h"
#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::multibody::constraint::ConstraintAccelProblemData;
using drake::multibody::constraint::ConstraintVelProblemData;
using drake::multibody::constraint::SlidingModeType;
using drake::systems::VectorBase;
using drake::systems::BasicVector;
using drake::systems::ContinuousState;
using drake::systems::State;
using drake::systems::SystemOutput;
using drake::systems::AbstractValues;
using drake::systems::Simulator;
using drake::systems::Context;
using drake::systems::rendering::PoseVector;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Vector6d = Eigen::Matrix<double, 6, 1>;

namespace drake {
namespace examples {
namespace rod2d {

// A system that outputs a constant vector between t_switch and t_switch_end.
class StepOutput : public systems::LeafSystem<double> {
 public:
  explicit StepOutput(double t_switch, double t_switch_end, const Vector3d& v) :
      t_switch_(t_switch), t_switch_end_(t_switch_end), v_(v) {
  this->DeclareVectorOutputPort(systems::BasicVector<double>(3),
      &StepOutput::CalcOutput);
 }

 private:
  void CalcOutput(
      const systems::Context<double>& context,
      systems::BasicVector<double>* output) const {
    Vector3<double> f;
    f.setZero();
    if (context.get_time() > t_switch_ && context.get_time() < t_switch_end_)
      f = v_;
    output->SetFromVector(f);
  }

  double t_switch_{0.0};
  double t_switch_end_{ std::numeric_limits<double>::infinity() };
  Vector3<double> v_;
};

// Creates a system diagram that applies an upward input force at t = 0.1s,
// for the ContactingAndAcceleratingUpward test.
std::unique_ptr<systems::Diagram<double>> CreateRodDiagramWithTimedInput(
  double t_switch, double t_switch_end, const Vector3d& f) {
  systems::DiagramBuilder<double> builder;

  // Add the rod to the system.
  const double dt = 0.0;  // Since system is piecewise DAE.
  Rod2D<double>* rod = builder.AddSystem<Rod2D<double>>(
      Rod2D<double>::SimulationType::kPiecewiseDAE, dt);
  rod->set_name("rod");

  // Adds the step input to the system.
  StepOutput* step = builder.AddSystem<StepOutput>(t_switch, t_switch_end, f);
  step->set_name("step_input");

  // Wire the systems together.
  builder.Connect(step->get_output_port(0), rod->get_input_port(0));

  // Return the diagram system.
  return builder.Build();
}

/// Class for testing the Rod2D example using a piecewise DAE
/// approach.
class Rod2DDAETest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SystemType::kPiecewiseDAE, 0.0);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();

    // Use a non-unit mass.
    dut_->set_rod_mass(2.0);

    // Set cfm to be very small, so that the complementarity problems are
    // well conditioned but the system is still nearly perfectly rigid. erp is
    // to be set to 0.2 (a reasonable default).
    const double cfm = 100 * std::numeric_limits<double>::epsilon();
    const double erp = 0.2;
    dut_->SetStiffnessAndDissipation(cfm, erp);

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

  VectorBase<double>& continuous_state() {
    return context_->get_mutable_continuous_state_vector();
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
        context_->get_mutable_continuous_state();
    xc[0] = 0;
    xc[1] = half_len * r22;
    xc[2] = -M_PI / 4.0;
    xc[3] = 1.0;
    xc[4] = 0.0;
    xc[5] = 0.0;

    // TODO: Set the abstract variables appropriately. 
  }

  // Sets the rod to a state that corresponds to ballistic motion.
  void SetBallisticState() {
    const double half_len = dut_->get_rod_half_length();
    ContinuousState<double>& xc =
        context_->get_mutable_continuous_state();
    xc[0] = 0.0;
    xc[1] = 10 * half_len;
    xc[2] = M_PI_2;
    xc[3] = 1.0;
    xc[4] = 2.0;
    xc[5] = 3.0;

    // Put the rod into ballistic mode. 
    dut_->SetBallisticMode(&context_->get_mutable_state());
  }

  // Sets the rod to an interpenetrating configuration without modifying the
  // velocity or any mode variables.
  void SetInterpenetratingConfig() {
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    // Configuration has the rod on its side.
    xc[0] = 0.0;    // com horizontal position
    xc[1] = -1.0;   // com vertical position
    xc[2] = 0.0;    // rod rotation
  }

  // Sets the rod to a resting horizontal configuration without modifying the
  // mode variables.
  void SetRestingHorizontalConfig() {
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    // Configuration has the rod on its side.
    xc[0] = 0.0;     // com horizontal position
    xc[1] = 0.0;     // com vertical position
    xc[2] = 0.0;     // rod rotation
    xc[3] = xc[4] = xc[5] = 0.0;   // velocity variables
  }

  // Sets the rod to a resting vertical configuration without modifying the
  // mode variables.
  void SetRestingVerticalConfig() {
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
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
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    xc[4] = -1.0;    // com horizontal velocity

    // Set the abstract variables appropriately by putting the rod into
    // ballistic mode (i.e., this will be the point right before an impact),
    // and then enabling the normal velocity witness. This is the mechanism
    // that Rod2D::ModelImpact() uses to determine its contact points.
    dut_->SetBallisticMode(&context_->get_mutable_state());
    const int right_endpoint_id = 1;
    dut_->GetNormalVelWitness(
        right_endpoint_id, context_->get_state())->set_enabled(true);
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
    dut_->GetContactPointsTangentVelocities(*context_, context_->get_state(),
        contacts, &tangent_vels);

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
    context_->get_mutable_continuous_state().get_mutable_generalized_velocity()
        .SetFromVector(data.solve_inertia(data.Mv) + delta_v);
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

// Verifies that the state vector functions throw no exceptions.
TEST_F(Rod2DDAETest, NamedStateVectorsNoThrow) {
  EXPECT_NO_THROW(Rod2D<double>::get_mutable_state(context_.get()));
  EXPECT_NO_THROW(Rod2D<double>::get_state(*context_));
  EXPECT_NO_THROW(Rod2D<double>::get_state(context_->get_continuous_state()));
  EXPECT_NO_THROW(Rod2D<double>::get_mutable_state(
      &context_->get_mutable_continuous_state()));
}

// Tests that named state vector components are at expected indices.
TEST_F(Rod2DDAETest, ExpectedIndices) {
  // Set the state.
  Rod2dStateVector<double>& state = Rod2D<double>::get_mutable_state(
      context_.get());
  state.set_x(1.0);
  state.set_y(2.0);
  state.set_theta(3.0);
  state.set_xdot(5.0);
  state.set_ydot(7.0);
  state.set_thetadot(11.0);

  // Check the indices.
  const VectorBase<double>& x = context_->get_continuous_state_vector();
  EXPECT_EQ(state.x(), x[0]);
  EXPECT_EQ(state.y(), x[1]);
  EXPECT_EQ(state.theta(), x[2]);
  EXPECT_EQ(state.xdot(), x[3]);
  EXPECT_EQ(state.ydot(), x[4]);
  EXPECT_EQ(state.thetadot(), x[5]);
}

// Checks that the output port represents the state.
TEST_F(Rod2DDAETest, Output) {
  const ContinuousState<double>& xc = context_->get_continuous_state();
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
  EXPECT_TRUE(dut_->IsImpacting(*context_));
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
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[0] = 0.0;
  xc[1] = half_len;
  xc[2] = M_PI_2;
  xc[3] = 0.0;
  xc[4] = -1.0;
  xc[5] = 0.0;

  // Rod should not be impacting.
  EXPECT_TRUE(dut_->IsImpacting(*context_));

  // Model the impact (as fully inelastic), updating the state.
  ModelImpact();

  // Verify that the state has been modified such that the body is no longer
  // in an impacting state and the configuration has not been modified.
  const double tol = 10 * std::numeric_limits<double>::epsilon();
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
  const ContinuousState<double>& xc = context_->get_continuous_state();
  EXPECT_NEAR((*derivatives_)[0], xc[3], tol);  // qdot = v ...
  EXPECT_NEAR((*derivatives_)[1], xc[4], tol);  // ... for this ...
  EXPECT_NEAR((*derivatives_)[2], xc[5], tol);  // ... system.
  EXPECT_NEAR((*derivatives_)[3], 0.0, tol);   // Zero horizontal acceleration.
  EXPECT_NEAR((*derivatives_)[4], g, tol);     // Gravitational acceleration.
  EXPECT_NEAR((*derivatives_)[5], 0.0, tol);   // Zero rotational acceleration.

  // TODO: Verify the abstract variables are still set correctly. 
}

// Verify that derivatives match what we expect from a non-inconsistent
// contacting configuration.
TEST_F(Rod2DDAETest, ConsistentDerivativesContacting) {
  // Set the initial state to sustained contact with zero tangential velocity
  // at the point of contact.
  const double half_len = dut_->get_rod_half_length();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[0] = 0.0;
  xc[1] = half_len;
  xc[2] = M_PI_2;
  xc[3] = 0.0;
  xc[4] = 0.0;
  xc[5] = 0.0;
  dut_->SetOneEndpointContacting(
      dut_->get_left_endpoint_index(), &context_->get_mutable_state(),
      SlidingModeType::kNotSliding);

  // Calculate the derivatives.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // Verify that derivatives match what we expect from a non-inconsistent
  // contacting configuration. In this case, there is no initial sliding,
  // velocity and the rod is oriented vertically, so we expect no sliding
  // to begin to occur.
  const double tol = std::numeric_limits<double>::epsilon() * 100;
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
  dut_->SetOneEndpointContacting(
      dut_->get_left_endpoint_index(), &context_->get_mutable_state(),
      SlidingModeType::kSliding);

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
  // at the point of contact and the rod being straight up.
  const double half_len = dut_->get_rod_half_length();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[0] = 0.0;
  xc[1] = half_len;
  xc[2] = M_PI_2;
  xc[3] = 0.0;
  xc[4] = 0.0;
  xc[5] = 0.0;
  dut_->SetOneEndpointContacting(
      dut_->get_left_endpoint_index(), &context_->get_mutable_state(),
      SlidingModeType::kNotSliding);

  // Coulomb friction will not be used.
  dut_->set_mu_coulomb(0.0);

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
  dut_->set_mu_static(mu_stick);

  // Calculate the derivatives.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // Verify that derivatives match what we expect: sticking should continue.
  const double tol = 100 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR((*derivatives_)[0], xc[3], tol);
  EXPECT_NEAR((*derivatives_)[1], xc[4], tol);
  EXPECT_NEAR((*derivatives_)[2], xc[5], tol);
  EXPECT_NEAR((*derivatives_)[3], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[4], 0.0, tol);
  EXPECT_NEAR((*derivatives_)[5], 0.0, tol);

  // Set the coefficient of friction to 99.9% of the sticking value and then
  // verify that the contact state transitions from a sticking one to a
  // non-sticking one. Note that we have to set the mode appropriately to
  // obtain this behavior.
  const double mu_slide = 0.999 * mu_stick;
  dut_->set_mu_static(mu_slide);
  auto& contacts = dut_->get_contacts_used_in_force_calculations(
      &context_->get_mutable_state());
  contacts.front().sliding_type = SlidingModeType::kTransitioning;  
  contacts.back().sliding_type = SlidingModeType::kTransitioning;  
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_GT((*derivatives_)[3], tol);  // horizontal accel. should be nonzero.

  // Set the coefficient of friction to zero and try again.
  dut_->set_mu_static(0.0);
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

// Verify that the (non-impacting) Painlevé configuration does not result in a
// state change.
TEST_F(Rod2DDAETest, ImpactNoChange) {
  // Verify not impacting at initial state.
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // Get the continuous state.
  const VectorX<double> xc_old = context_->get_continuous_state().
      get_vector().CopyToVector();

  // Model the impact and get the continuous state out.
  ModelImpact();
  const VectorX<double> xc = context_->get_continuous_state().
      get_vector().CopyToVector();

  // Verify the continuous state did not change.
  EXPECT_TRUE(CompareMatrices(xc_old, xc,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Verify that applying the impact model to an impacting configuration results
// in a non-impacting configuration. This test exercises the model for the case
// where impulses that yield tangential sticking lie within the friction cone.
TEST_F(Rod2DDAETest, InfFrictionImpactThenNoImpact) {
  // Cause the initial state to be impacting.
  SetImpactingState();

  // Set the coefficient of friction to "large".
  dut_->set_mu_coulomb(100.0);

  // Model the impact and copy the result to the context.
  std::unique_ptr<State<double>> new_state = CloneState();
  dut_->ModelImpact(*context_, new_state.get());
  context_->get_mutable_state().SetFrom(*new_state);
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // Do one more impact- there should now be no change.
  dut_->ModelImpact(*context_, new_state.get());
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state().get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state().get_vector().
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

  // Set the coefficient of friction to zero. This forces the rod code
  // to go through the second impact path (impulse corresponding to sticking
  // friction post-impact lies outside of the friction cone).
  dut_->set_mu_coulomb(0.0);

  // Handle the impact and copy the result to the context.
  std::unique_ptr<State<double>> new_state = CloneState();
  dut_->ModelImpact(*context_, new_state.get());
  context_->get_mutable_state().SetFrom(*new_state);
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // Do one more impact- there should now be no change.
  // Verify that there is no further change from this second impact.
  dut_->ModelImpact(*context_, new_state.get());
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state().get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state().get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Verify that no exceptions thrown for a non-sliding configuration.
TEST_F(Rod2DDAETest, NoSliding) {
  const double half_len = dut_->get_rod_half_length();
  const double r22 = std::sqrt(2) / 2;
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  State<double>* state = &context_->get_mutable_state();

  // Set the coefficient of friction to zero (triggering the case on the
  // edge of the friction cone).
  dut_->set_mu_coulomb(0.0);
  dut_->set_mu_static(0.0);

  // This configuration has no sliding velocity.
  xc[0] = -half_len * r22;
  xc[1] = half_len * r22;
  xc[2] = 3 * M_PI / 4.0;
  xc[3] = 0.0;
  xc[4] = 0.0;
  xc[5] = 0.0;
  dut_->SetOneEndpointContacting(
      dut_->get_left_endpoint_index(), state, SlidingModeType::kNotSliding);

  // Verify no impact.
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // No exceptions should be thrown.
  EXPECT_NO_THROW(dut_->CalcTimeDerivatives(*context_, derivatives_.get()));

  // Set the coefficient of friction to effective no-slip (triggering the
  // case strictly inside the friction cone).
  dut_->set_mu_static(std::numeric_limits<double>::infinity());
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());

  // No exceptions should be thrown.
  EXPECT_NO_THROW(dut_->CalcTimeDerivatives(*context_, derivatives_.get()));
}

// Test multiple (two-point) contact configurations.
TEST_F(Rod2DDAETest, MultiPoint) {
  State<double>& mutable_state = context_->get_mutable_state();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();

  // Set the rod to a horizontal, two-contact configuration.
  xc[0] = 0;
  xc[1] = 0;
  xc[2] = 0;

  // Set the velocity on the rod such that it is moving horizontally.
  xc[3] = 1.0;
  dut_->SetBothEndpointsContacting(&mutable_state, SlidingModeType::kSliding);
  EXPECT_FALSE(dut_->IsImpacting(*context_));  // Verify no impact.

  // Set the coefficient of friction to zero.
  dut_->set_mu_coulomb(0.0);

  // TODO(edrumwri): Investigate how to reduce these tolerances.
  // Compute the derivatives and verify that the linear and angular acceleration
  // are approximately zero.
  const double tol = 600 * std::numeric_limits<double>::epsilon();
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR((*derivatives_)[3], 0, tol);
  EXPECT_NEAR((*derivatives_)[4], 0, tol);
  EXPECT_NEAR((*derivatives_)[5], 0, tol);

  // Set the coefficient of friction to "very large".
  const double large = 100.0;
  dut_->set_mu_coulomb(large);

  // TODO(edrumwri): Check derivatives now.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR((*derivatives_)[3], -large *
      std::abs(dut_->get_gravitational_acceleration()), tol * large);
  EXPECT_NEAR((*derivatives_)[4], 0, tol);
  EXPECT_NEAR((*derivatives_)[5], 0, tol);

  // Set the rod velocity to zero.
  xc[3] = 0.0;
  EXPECT_FALSE(dut_->IsImpacting(*context_));  // Verify no impact.

  // Set a constant force pushing the rod.
  dut_->set_mu_static(large);
  const double fX = 1.0;
  std::unique_ptr<BasicVector<double>> ext_input =
      std::make_unique<BasicVector<double>>(3);
  ext_input->SetAtIndex(0, fX);
  ext_input->SetAtIndex(1, 0.0);
  ext_input->SetAtIndex(2, 0.0);
  context_->FixInputPort(0, std::move(ext_input));
  dut_->SetBothEndpointsContacting(
      &mutable_state, SlidingModeType::kNotSliding);

  // Verify that the linear and angular acceleration are still zero.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_NEAR((*derivatives_)[3], 0, tol);
  EXPECT_NEAR((*derivatives_)[4], 0, tol);
  EXPECT_NEAR((*derivatives_)[5], 0, tol);
}

// Verify that the Painlevé configuration does not correspond to an impacting
// state.
TEST_F(Rod2DDAETest, ImpactNoChange2) {
  SetSecondInitialConfig();

  // Get the continuous state.
  const VectorX<double> xc_old = context_->get_continuous_state().
    get_vector().CopyToVector();

  // Model the impact and get the continuous state out.
  ModelImpact();
  const VectorX<double> xc = context_->get_continuous_state().
    get_vector().CopyToVector();

  // Verify the continuous state did not change.
  EXPECT_TRUE(CompareMatrices(xc_old, xc,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Verify that applying the impact model to an impacting state results
// in a non-impacting state.
TEST_F(Rod2DDAETest, InfFrictionImpactThenNoImpact2) {
  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

  // Cause the initial state to be impacting.
  SetImpactingState();

  // Set the coefficient of friction to infinite. This forces the rod code
  // to go through the first impact path.
  dut_->set_mu_coulomb(std::numeric_limits<double>::infinity());

  // Model the impact and copy the result to the context.
  dut_->ModelImpact(*context_, new_state.get());
  context_->get_mutable_state().SetFrom(*new_state);

  // Verify the state no longer corresponds to an impact.
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // Do one more impact- there should now be no change.
  dut_->ModelImpact(*context_, new_state.get());
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state().get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state().get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Verify that applying the impact model to an impacting state results in a
// non-impacting state.
TEST_F(Rod2DDAETest, NoFrictionImpactThenNoImpact2) {
  // Set writable state.
  std::unique_ptr<State<double>> new_state = CloneState();

  // Cause the initial state to be impacting.
  SetImpactingState();

  // Set the coefficient of friction to zero. This forces the rod code
  // to go through the second impact path.
  dut_->set_mu_coulomb(0.0);

  // Model the impact and copy the result to the context.
  dut_->ModelImpact(*context_, new_state.get());
  context_->get_mutable_state().SetFrom(*new_state);
  EXPECT_FALSE(dut_->IsImpacting(*context_));

  // Do one more impact- there should now be no change.
  dut_->ModelImpact(*context_, new_state.get());
  EXPECT_TRUE(CompareMatrices(new_state->get_continuous_state().get_vector().
                                  CopyToVector(),
                              context_->get_continuous_state().get_vector().
                                  CopyToVector(),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Verifies that rod in a ballistic state does not correspond to an impact.
TEST_F(Rod2DDAETest, BallisticNoImpact) {
  // Cause the initial state to be impacting.
  SetImpactingState();

  // Move the rod upward vertically so that it is no longer impacting.
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[1] += 10.0;

  // Verify that no impact occurs.
  EXPECT_FALSE(dut_->IsImpacting(*context_));
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
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
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
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[3] = 1.0;

  // Compute the problem data.
  ConstraintAccelProblemData<double> data(3 /* gen. vel. dim */);
  CalcConstraintAccelProblemData(&data);
  const int num_contacts = 1;
  CheckProblemConsistency(data, num_contacts);

  // Verify that contact is sliding.
  EXPECT_EQ(data.sliding_contacts.size(), num_contacts);
}

// Verifies that the mode changes as expected when the rod is in sustained
// contact in a horizontal configuration and goes from sliding to not sliding.
TEST_F(Rod2DDAETest, SlidingToNotSliding) {
  // Get a pointer to the mutable state- it will be used repeatedly.
  systems::State<double>* state = &context_->get_mutable_state();

  // Set the state such that the velocity is horizontal
  // (and the contact mode is set appropriately).
  SetRestingHorizontalConfig();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[3] = 0.1;
  dut_->SetBothEndpointsContacting(state, SlidingModeType::kSliding);

  // Simulate forward.
  const double t_final = 1.0;
  Simulator<double> sim(*dut_, std::move(context_));
  sim.get_mutable_context().set_accuracy(1e-8);
  sim.StepTo(t_final);

  // Verify that exactly one unrestricted update (sliding direction change)
  // was handled.
  EXPECT_EQ(sim.get_num_unrestricted_updates(), 1);

  // Both contact points should be in the set of force calculations.
  EXPECT_EQ(dut_->get_contacts_used_in_force_calculations(state).size(), 2);

  // Verify that three witness functions are active for the each endpoint- one
  // for signed distance, one for normal force, and one for sticking friction
  // slack.
  const int left_endpoint_id = 0, right_endpoint_id = 1;
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(dut_->GetNormalForceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(dut_->GetStickingFrictionForceSlackWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(dut_->GetNormalForceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(dut_->GetStickingFrictionForceSlackWitness(
      right_endpoint_id, *state)->is_enabled());

  // All other witnesses should be disabled.
  EXPECT_FALSE(dut_->GetPosSlidingWitness(
      left_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetNegSlidingWitness(
      left_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetPosSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetNegSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
}

// Verifies that the mode changes as expected when the rod is in sustained
// contact in a horizontal configuration and goes from not sliding to sliding.
TEST_F(Rod2DDAETest, NotSlidingToSliding) {
  // Set the input force to slide to the right. 
  std::unique_ptr<BasicVector<double>> ext_input =
      std::make_unique<BasicVector<double>>(3);
  ext_input->SetAtIndex(0, 1.0);
  ext_input->SetAtIndex(1, 0.0);
  ext_input->SetAtIndex(2, 0.0);
  context_->FixInputPort(0, std::move(ext_input));

  // Get a pointer to the mutable state- it will be used repeatedly.
  systems::State<double>* state = &context_->get_mutable_state();

  // Set the coefficient of friction to very near zero.
  dut_->set_mu_coulomb(1e-6);
  dut_->set_mu_static(1e-6);

  // Set the state such that the velocity is zero 
  // (and the contact mode is set appropriately).
  SetRestingHorizontalConfig();
  dut_->SetBothEndpointsContacting(state, SlidingModeType::kTransitioning);
  dut_->DetermineContactModes(*context_, state);

  // Verify that the both contacts are still in the set of force calculations. 
  EXPECT_EQ(dut_->get_contacts_used_in_force_calculations(state).size(), 2);

  // Verify that both contacts are transitioning.
  const int left_endpoint_id = dut_->get_left_endpoint_index();
  const int right_endpoint_id = dut_->get_right_endpoint_index();
  EXPECT_TRUE(
      dut_->GetPosSlidingWitness(left_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(
      dut_->GetPosSlidingWitness(right_endpoint_id, *state)->is_enabled());

  // Simulate forward.
  const double t_final = 1.0;
  Simulator<double> sim(*dut_, std::move(context_));
  sim.get_mutable_context().set_accuracy(1e-8);
  sim.StepTo(t_final);

  // Verify that one unrestricted update occurs: transition to proper sliding.
  EXPECT_EQ(sim.get_num_unrestricted_updates(), 1);

  // Both contact points should be in the set of force calculations.
  EXPECT_EQ(dut_->get_contacts_used_in_force_calculations(state).size(), 2);

  // Verify that five witness functions are active for the each endpoint- one
  // for signed distance, one for normal force, one for sliding friction
  // direction change, and two for sliding velocity.
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(dut_->GetNormalForceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(dut_->GetNormalForceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(dut_->GetPosSlidingWitness(
      left_endpoint_id, *state)->is_enabled()); 
  EXPECT_TRUE(dut_->GetPosSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_TRUE(dut_->GetNegSlidingWitness(
      left_endpoint_id, *state)->is_enabled()); 
  EXPECT_TRUE(dut_->GetNegSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 

  // The sticking friction slack witnesses should be disabled. 
  EXPECT_FALSE(dut_->GetStickingFrictionForceSlackWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetStickingFrictionForceSlackWitness(
      right_endpoint_id, *state)->is_enabled());
}

// Verifies that the mode changes as expected when the rod is in sustained
// contact in a horizontal configuration and goes from not sliding to sliding
// after some time.
TEST_F(Rod2DDAETest, NotSlidingToSliding2) {
  // Create a diagram with an input that causes the rod to start accelerating
  // to the right at t = 0.1s.
  const double t_switch_on = 0.1;
  const double t_switch_off = std::numeric_limits<double>::infinity();
  auto diagram = CreateRodDiagramWithTimedInput(
      t_switch_on, t_switch_off, Vector3d(1, 0, 0));

  // Create a new context.
  context_ = diagram->CreateDefaultContext();

  // Get a pointer to the mutable state- it will be used repeatedly.
  systems::State<double>* state = &context_->get_mutable_state();

  // Set the coefficient of friction to very near zero.
  Rod2D<double>& rod = *const_cast<Rod2D<double>*>(
      dynamic_cast<const Rod2D<double>*>(diagram->GetSystems().front()));
  rod.set_mu_coulomb(1e-6);
  rod.set_mu_static(1e-6);

  // Set the state such that the velocity is zero 
  // (and the contact mode is set appropriately).
  SetRestingHorizontalConfig();
  rod.SetBothEndpointsContacting(state, SlidingModeType::kNotSliding);

  // TODO: Re-enable and debug this.
  /*
  rod.DetermineContactModes(rod, *context_), state);
  */

  // Verify that the both contacts are still in the set of force calculations. 
  EXPECT_EQ(rod.get_contacts_used_in_force_calculations(state).size(), 2);

  // Verify that contacts are not transitioning.
  const int left_endpoint_id = rod.get_left_endpoint_index();
  const int right_endpoint_id = rod.get_right_endpoint_index();
  EXPECT_FALSE(
      rod.GetPosSlidingWitness(left_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(
      rod.GetPosSlidingWitness(right_endpoint_id, *state)->is_enabled());

  // Simulate forward.
  const double t_final = 1.0;
  Simulator<double> sim(*diagram, std::move(context_));
  sim.get_mutable_context().set_accuracy(1e-8);
  sim.StepTo(t_final);

  // Both contact points should be in the set of force calculations.
  EXPECT_EQ(rod.get_contacts_used_in_force_calculations(state).size(), 2);

  // Verify that three witness functions are active for the each endpoint- one
  // for signed distance, one for normal force, and one for sliding friction
  // direction change. 
  EXPECT_TRUE(rod.GetSignedDistanceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(rod.GetNormalForceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(rod.GetSignedDistanceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(rod.GetNormalForceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(rod.GetPosSlidingWitness(
      left_endpoint_id, *state)->is_enabled()); 
  EXPECT_TRUE(rod.GetNegSlidingWitness(
      left_endpoint_id, *state)->is_enabled()); 
  EXPECT_TRUE(rod.GetPosSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_TRUE(rod.GetNegSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 

  // The stiction friction slack witnesses should be disabled. 
  EXPECT_FALSE(rod.GetStickingFrictionForceSlackWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(rod.GetStickingFrictionForceSlackWitness(
      right_endpoint_id, *state)->is_enabled());
}

// Verifies that the mode changes as expected when the rod is in contact but
// moving slightly upward (and remains in contact).
TEST_F(Rod2DDAETest, ContactingAndMovingSlightlyUpward) {
  // Get a pointer to the mutable state- it will be used repeatedly.
  systems::State<double>* state = &context_->get_mutable_state();

  // Set the state such that the vertical velocity is upward and not sliding
  // (and the contact mode is set appropriately).
  SetRestingVerticalConfig();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[1] = -0.1;
  xc[4] = 1e-5;
  dut_->SetOneEndpointContacting(
      dut_->get_left_endpoint_index(), state, SlidingModeType::kNotSliding);

  // Since the velocity is positive, the point should be removed from the
  // force calculations.
  dut_->get_contacts_used_in_force_calculations(state).clear();

  // Set contact candidate ids.
  const int left_endpoint_id = 0;
  const int right_endpoint_id = 1;

  // This also means that we need to disable relevant witnesses. 
  dut_->GetStickingFrictionForceSlackWitness(left_endpoint_id, *state)->
      set_enabled(false);
  dut_->GetNormalForceWitness(left_endpoint_id, *state)->set_enabled(false);

  // Get the normal velocity witness.
  auto normal_vel_witness = static_cast<NormalVelWitness<double>*>(
      dut_->GetNormalVelWitness(left_endpoint_id, *state));

  // Verify that the normal velocity witness returns a positive value.
  EXPECT_GT(normal_vel_witness->Evaluate(*context_), 0);

  // Activate the normal velocity witness for the left endpoint. 
  normal_vel_witness->set_enabled(true);

  // Simulate forward.
  const double t_final = 0.1;
  Simulator<double> sim(*dut_, std::move(context_));
  sim.get_mutable_context().set_accuracy(1e-8);
  sim.StepTo(t_final);

  // Verify that exactly one unrestricted update (the one we're searching for)
  // was handled.
  EXPECT_EQ(sim.get_num_unrestricted_updates(), 1);

  // Verify that the normal velocity witness returns a non-negative value.
  const double zero_tol = 1e-6;
  EXPECT_GT(normal_vel_witness->Evaluate(sim.get_context()), -zero_tol);

  // The left contact point should now be back in the set of force calculations.
  EXPECT_EQ(dut_->get_contacts_used_in_force_calculations(state).size(), 1);

  // Verify that three witness functions are active for the left endpoint- one
  // for signed distance, one for normal force, and one for sticking friction
  // slack.
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(dut_->GetNormalForceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(dut_->GetStickingFrictionForceSlackWitness(
      left_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetPosSlidingWitness(
      left_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetNegSlidingWitness(
      left_endpoint_id, *state)->is_enabled()); 

  // Only one witness function for the right endpoint should be enabled.
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetNormalForceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetStickingFrictionForceSlackWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetPosSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetNegSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
}

// Verifies that the mode changes as expected when the rod is in contact but
// moving upward (and remains in contact).
TEST_F(Rod2DDAETest, ContactingAndMovingUpward) {
  // Get a pointer to the mutable state- it will be used repeatedly.
  systems::State<double>* state = &context_->get_mutable_state();

  // Set the state such that the vertical velocity is upward and not sliding
  // (and the contact mode is set appropriately).
  SetRestingVerticalConfig();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[1] = -0.1;
  xc[4] = 10000;
  dut_->SetOneEndpointContacting(
      dut_->get_left_endpoint_index(), state, SlidingModeType::kNotSliding);

  // Since the velocity is positive, the point should be removed from the
  // force calculations.
  dut_->get_contacts_used_in_force_calculations(state).clear();

  // Set contact candidate ids.
  const int left_endpoint_id = 0;
  const int right_endpoint_id = 1;

  // This also means that we need to disable relevant witnesses. 
  dut_->GetStickingFrictionForceSlackWitness(left_endpoint_id, *state)->
      set_enabled(false);
  dut_->GetNormalForceWitness(left_endpoint_id, *state)->set_enabled(false);

  // Get the normal velocity witness.
  auto normal_vel_witness = static_cast<NormalVelWitness<double>*>(
      dut_->GetNormalVelWitness(left_endpoint_id, *state));

  // Activate the normal velocity witness for the left endpoint. 
  normal_vel_witness->set_enabled(true);

  // Simulate forward.
  const double t_final = 0.1;
  Simulator<double> sim(*dut_, std::move(context_));
  sim.get_mutable_context().set_accuracy(1e-8);
  sim.StepTo(t_final);

  // Verify that exactly one unrestricted update (the one we're searching for)
  // was handled.
  EXPECT_EQ(sim.get_num_unrestricted_updates(), 1);

  // Verify that the normal velocity witness returns a positive value.
  EXPECT_GT(normal_vel_witness->Evaluate(sim.get_context()), 0);

  // The set of contact points in force calculations should still be empty. 
  EXPECT_EQ(dut_->get_contacts_used_in_force_calculations(state).size(), 0);

  // Verify that only the signed distance witness function is active for the
  // left endpoint (i.e., the normal velocity witness should no longer be
  // active). 
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetNormalForceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetStickingFrictionForceSlackWitness(
      left_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetPosSlidingWitness(
      left_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetNegSlidingWitness(
      left_endpoint_id, *state)->is_enabled()); 

  // Only one witness function for the right endpoint should be enabled.
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetNormalForceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetStickingFrictionForceSlackWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetPosSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetNegSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
}

// Verifies that the mode changes as expected when the rod is in contact but
// is moving upward and the contact then breaks.
TEST_F(Rod2DDAETest, ContactingMovingUpwardAndSeparating) 
{
  // Get a pointer to the mutable state- it will be used repeatedly.
  systems::State<double>* state = &context_->get_mutable_state();

  // Set the state such that the velocity is zero (not separating, not sliding)
  // (and the contact mode is set appropriately.
  SetRestingVerticalConfig();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[1] = -0.1;
  xc[4] = 10000;
  dut_->SetOneEndpointContacting(
      dut_->get_left_endpoint_index(), state, SlidingModeType::kNotSliding);

  // Since the velocity is positive, the point should be removed from the
  // force calculations.
  dut_->get_contacts_used_in_force_calculations(state).clear();

  // Set contact candidate ids.
  const int left_endpoint_id = 0;
  const int right_endpoint_id = 1;

  // This also means that we need to disable relevant witnesses. 
  dut_->GetStickingFrictionForceSlackWitness(left_endpoint_id, *state)->
      set_enabled(false);
  dut_->GetNormalForceWitness(left_endpoint_id, *state)->set_enabled(false);

  // Get the normal velocity witness.
  auto normal_vel_witness = static_cast<NormalVelWitness<double>*>(
      dut_->GetNormalVelWitness(left_endpoint_id, *state));

  // Activate the normal velocity witness for the left endpoint. 
  normal_vel_witness->set_enabled(true);

  // Simulate forward.
  const double t_final = 0.1;
  Simulator<double> sim(*dut_, std::move(context_));
  sim.get_mutable_context().set_accuracy(1e-8);
  sim.StepTo(t_final);

  // Verify that exactly one unrestricted update was handled: the signed
  // distance witness.
  EXPECT_EQ(sim.get_num_unrestricted_updates(), 1);

  // The set of contact points in force calculations should still be empty. 
  EXPECT_EQ(dut_->get_contacts_used_in_force_calculations(state).size(), 0);

  // Verify that only the signed distance witness function is active for the
  // left endpoint. 
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetNormalForceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetStickingFrictionForceSlackWitness(
      left_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetPosSlidingWitness(
      left_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetNegSlidingWitness(
      left_endpoint_id, *state)->is_enabled()); 

  // Only one witness function for the right endpoint should be enabled.
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetNormalForceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetStickingFrictionForceSlackWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetPosSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetNegSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
}

// Verifies that the mode changes as expected when the rod is in contact but
// begins accelerating upward. 
TEST_F(Rod2DDAETest, ContactingAndAcceleratingUpward) {
  // Create a diagram with an input that causes the rod to start accelerating
  // upward at t = 0.1s.
  const double t_switch = 0.1;
  const double inf = std::numeric_limits<double>::infinity();
  auto diagram = CreateRodDiagramWithTimedInput(
      t_switch, inf, Vector3d(0, 10, 0));

  // Create a new context.
  context_ = diagram->CreateDefaultContext();

  // Get a pointer to the mutable state- it will be used repeatedly.
  systems::State<double>* state = &context_->get_mutable_state();

  // Set the state such that the velocity is not sliding
  // (and the contact mode is set appropriately).
  SetRestingVerticalConfig();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[1] = -0.1;
  dut_->SetOneEndpointContacting(
      dut_->get_left_endpoint_index(), state, SlidingModeType::kNotSliding);

  // The set of contact points in force calculations should have one element. 
  EXPECT_EQ(dut_->get_contacts_used_in_force_calculations(state).size(), 1);

  // Set contact candidate ids.
  const int left_endpoint_id = 0;
  const int right_endpoint_id = 1;

  // Simulate forward.
  const double t_final = t_switch + t_switch * 0.1;
  Simulator<double> sim(*diagram, std::move(context_));
  sim.get_mutable_context().set_accuracy(1e-8);
  sim.StepTo(t_final);

  // Verify that exactly one unrestricted update (the one we're searching for)
  // was handled.
  EXPECT_EQ(sim.get_num_unrestricted_updates(), 1);

  // The set of contact points in force calculations should now be empty. 
  EXPECT_TRUE(dut_->get_contacts_used_in_force_calculations(state).empty());

  // Verify that only the signed distance witness and normal force witness
  // functions are active for the left endpoint.
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(dut_->GetNormalAccelWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetNormalForceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetStickingFrictionForceSlackWitness(
      left_endpoint_id, *state)->is_enabled()); 

  // Only one witness function for the right endpoint should be enabled.
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetNormalForceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetStickingFrictionForceSlackWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetPosSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetNegSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
}

// Verifies that the mode changes as expected when the rod is in contact but
// accelerating upward *for only a moment*. 
TEST_F(Rod2DDAETest, ContactingAndAcceleratingUpwardMomentarily) {
  // Create a diagram with an input that causes the rod to start accelerating
  // upward at t = 0.1s.
  const double t_switch_on = 0.1, t_switch_off = 0.2;
  auto diagram = CreateRodDiagramWithTimedInput(
      t_switch_on, t_switch_off, Vector3d(0, 10, 0));

  // Create a new context.
  context_ = diagram->CreateDefaultContext();

  // Get a pointer to the mutable state- it will be used repeatedly.
  systems::State<double>* state = &context_->get_mutable_state();

  // Set the state such that the velocity is not sliding
  // (and the contact mode is set appropriately).
  SetRestingVerticalConfig();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[1] = -0.1;
  dut_->SetOneEndpointContacting(
      dut_->get_left_endpoint_index(), state, SlidingModeType::kNotSliding);

  // The set of contact points in force calculations should have one element. 
  EXPECT_EQ(dut_->get_contacts_used_in_force_calculations(state).size(), 1);

  // Set contact candidate ids.
  const int left_endpoint_id = 0;
  const int right_endpoint_id = 1;

  // Simulate forward.
  const double t_final = 2 * t_switch_off;
  Simulator<double> sim(*diagram, std::move(context_));
  sim.get_mutable_context().set_accuracy(1e-8);
  sim.StepTo(t_final);

  // Verify that exactly three unrestricted updates: one to remove the point
  // from the force set, one as the acceleration goes back to zero, and another
  // as the velocity goes back to zero (adding the point back to the force set).
  EXPECT_EQ(sim.get_num_unrestricted_updates(), 3);

  // The set of contact points in force calculations should still be of size
  // one.
  EXPECT_EQ(dut_->get_contacts_used_in_force_calculations(state).size(), 1);

  // Verify that the signed distance witness and normal force functions are
  // active for the left endpoint.
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(dut_->GetNormalForceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetNormalAccelWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(dut_->GetStickingFrictionForceSlackWitness(
      left_endpoint_id, *state)->is_enabled()); 

  // Only one witness function for the right endpoint should be enabled.
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetNormalForceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetStickingFrictionForceSlackWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetPosSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetNegSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
}

// Verifies that the mode changes as expected when the rod is in contact but
// begins accelerating upward. 
TEST_F(Rod2DDAETest, ContactingAndAcceleratingUpwardThenBreaks) {
  // Create a diagram with an input that causes the rod to start accelerating
  // upward at t = 0.1s.
  const double t_switch = 0.1;
  const double inf = std::numeric_limits<double>::infinity();
  auto diagram = CreateRodDiagramWithTimedInput(
      t_switch, inf, Vector3d(0, 10, 0));

  // Create a new context.
  context_ = diagram->CreateDefaultContext();

  // Get a pointer to the mutable state- it will be used repeatedly.
  systems::State<double>* state = &context_->get_mutable_state();

  // Set the state such that the velocity is not sliding
  // (and the contact mode is set appropriately).
  SetRestingVerticalConfig();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[1] = -0.1;
  dut_->SetOneEndpointContacting(dut_->get_left_endpoint_index(), state,
      SlidingModeType::kNotSliding);

  // The set of contact points in force calculations should have one element. 
  EXPECT_EQ(dut_->get_contacts_used_in_force_calculations(state).size(), 1);

  // Set contact candidate ids.
  const int left_endpoint_id = 0;
  const int right_endpoint_id = 1;

  // Simulate forward.
  const double t_final = t_switch * 100.0;
  Simulator<double> sim(*diagram, std::move(context_));
  sim.get_mutable_context().set_accuracy(1e-8);
  sim.StepTo(t_final);

  // Verify that exactly two unrestricted updates were handled: one as the
  // contact is removed from the force calculations and another as the
  // rod breaks contact.
  EXPECT_EQ(sim.get_num_unrestricted_updates(), 2);

  // The set of contact points in force calculations should now be empty. 
  EXPECT_TRUE(dut_->get_contacts_used_in_force_calculations(state).empty());

  // Verify that only the signed distance witness functions is active for the
  // left endpoint.
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetNormalAccelWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetNormalForceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetStickingFrictionForceSlackWitness(
      left_endpoint_id, *state)->is_enabled()); 

  // Only one witness function for the right endpoint should be enabled.
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetNormalForceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetStickingFrictionForceSlackWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetPosSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetNegSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
}

/// Verifies the correct behavior as the rod impacts the ground. The rod should
/// impact and then remain in sustained contact.
TEST_F(Rod2DDAETest, ImpactThenSustainedContact) {
  // Get a pointer to the mutable state- it will be used repeatedly.
  systems::State<double>* state = &context_->get_mutable_state();

  // Set the state such that the vertical velocity is downward and the rod
  // is not contacting.
  SetRestingVerticalConfig();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[1] = 1.5;
  xc[4] = -1;
  dut_->SetBallisticMode(state);

  // Set contact candidate ids.
  const int left_endpoint_id = 0;
  const int right_endpoint_id = 1;

  // Get the normal velocity witness.
  auto normal_vel_witness = static_cast<NormalVelWitness<double>*>(
      dut_->GetNormalVelWitness(left_endpoint_id, *state));

  // Verify that the normal velocity witness returns a negative value.
  EXPECT_LT(normal_vel_witness->Evaluate(*context_), 0);

  // Simulate forward.
  const double t_final = 1;
  Simulator<double> sim(*dut_, std::move(context_));
  sim.get_mutable_context().set_accuracy(1e-8);
  sim.StepTo(t_final);

  // Verify that exactly one unrestricted update (impact with an immediate
  // transition to sustained contact).
  EXPECT_EQ(sim.get_num_unrestricted_updates(), 1);

  // Verify that the normal velocity witness returns a non-negative value.
  const double zero_tol = 1e-6;
  EXPECT_GT(normal_vel_witness->Evaluate(sim.get_context()), -zero_tol);

  // The left contact point should now be in the set of force calculations.
  EXPECT_EQ(dut_->get_contacts_used_in_force_calculations(state).size(), 1);

  // Verify that three witness functions are active for the left endpoint- one
  // for signed distance, one for normal force, and one for sticking friction
  // slack.
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(dut_->GetNormalForceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_TRUE(dut_->GetStickingFrictionForceSlackWitness(
      left_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetPosSlidingWitness(
      left_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetNegSlidingWitness(
      left_endpoint_id, *state)->is_enabled()); 

  // Only one witness function for the right endpoint should be enabled.
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetNormalForceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetStickingFrictionForceSlackWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetPosSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetNegSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
}

/// Verifies the correct behavior as the rod impacts the ground while
/// accelerating upward. The rod should impact and then immediately separate.
TEST_F(Rod2DDAETest, AcceleratingUpwardImpactThenImmediateSeparation) {
  // Set an input force that overwhelms gravity. 
  std::unique_ptr<BasicVector<double>> ext_input =
      std::make_unique<BasicVector<double>>(3);
  ext_input->SetAtIndex(0, 0.0);
  ext_input->SetAtIndex(1, -1.5 * dut_->get_rod_mass() *
      dut_->get_gravitational_acceleration());
  ext_input->SetAtIndex(2, 0.0);
  context_->FixInputPort(0, std::move(ext_input));

  // Get a pointer to the mutable state- it will be used repeatedly.
  systems::State<double>* state = &context_->get_mutable_state();

  // Set the state such that the vertical velocity is downward and the rod
  // is not contacting.
  SetRestingVerticalConfig();
  ContinuousState<double>& xc = context_->get_mutable_continuous_state();
  xc[1] = 1.5;
  xc[4] = -10;
  dut_->SetBallisticMode(state);

  // Set contact candidate ids.
  const int left_endpoint_id = 0;
  const int right_endpoint_id = 1;

  // Get the normal velocity witness.
  auto normal_vel_witness = static_cast<NormalVelWitness<double>*>(
      dut_->GetNormalVelWitness(left_endpoint_id, *state));

  // Verify that the normal velocity witness returns a negative value.
  EXPECT_LT(normal_vel_witness->Evaluate(*context_), 0);

  // Simulate forward.
  const double t_final = 1;
  Simulator<double> sim(*dut_, std::move(context_));
  sim.get_mutable_context().set_accuracy(1e-8);
  sim.StepTo(t_final);

  // Verify that exactly two unrestricted updates occurred (impact with an
  // immediate transition to separating contact), followed by separation.
  EXPECT_EQ(sim.get_num_unrestricted_updates(), 2);

  // The set of force calculations should be empty.
  EXPECT_TRUE(dut_->get_contacts_used_in_force_calculations(state).empty());

  // Verify that one witness function is active for the left endpoint
  // (for signed distance).
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetNormalForceWitness(
      left_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetStickingFrictionForceSlackWitness(
      left_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetPosSlidingWitness(
      left_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetNegSlidingWitness(
      left_endpoint_id, *state)->is_enabled()); 

  // Only one witness function for the right endpoint should be enabled.
  EXPECT_TRUE(dut_->GetSignedDistanceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetNormalForceWitness(
      right_endpoint_id, *state)->is_enabled());
  EXPECT_FALSE(dut_->GetStickingFrictionForceSlackWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetPosSlidingWitness(
      right_endpoint_id, *state)->is_enabled()); 
  EXPECT_FALSE(dut_->GetNegSlidingWitness(
      right_endpoint_id, *state)->is_enabled());
}

/// Class for testing the Rod 2D example using a first order time
/// stepping approach.
class Rod2DDiscretizedTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const double dt = 1e-2;
    dut_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SystemType::kDiscretized, dt);
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

  BasicVector<double>& mutable_discrete_state() {
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
    auto xd = mutable_discrete_state().get_mutable_value();

    xd[0] = 0;
    xd[1] = half_len * r22;
    xd[2] = -M_PI / 4.0;
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
/// first-order discretization approach (this tests expected meta behavior).
TEST_F(Rod2DDiscretizedTest, RodGoesToRest) {
  // Set the initial state to an inconsistent configuration.
  SetSecondInitialConfig();

  // Init the simulator.
  Simulator<double> simulator(*dut_, std::move(context_));

  // Integrate forward to a point where the rod should be at rest.
  const double t_final = 10;
  simulator.StepTo(t_final);

  // Get angular orientation and velocity.
  const auto xd = simulator.get_context().get_discrete_state(0).get_value();
  const double theta = xd(2);
  const double theta_dot = xd(5);

  // After sufficiently long, theta should be 0 or M_PI and the velocity
  // should be nearly zero.
  EXPECT_TRUE(std::fabs(theta) < 1e-6 || std::fabs(theta - M_PI) < 1e-6);
  EXPECT_NEAR(theta_dot, 0.0, 1e-6);
}

// This test checks to see whether a single semi-explicit step of the piecewise
// DAE based Rod2D system is equivalent to a single step of the semi-explicit
// time stepping based system.
GTEST_TEST(Rod2DCrossValidationTest, OneStepSolutionSliding) {
  // Create two Rod2D systems.
  const double dt = 1e-1;
  Rod2D<double> ts(Rod2D<double>::SystemType::kDiscretized, dt);
  Rod2D<double> pdae(Rod2D<double>::SystemType::kPiecewiseDAE, 0.0);

  // Set the coefficient of friction to a small value for both.
  const double mu = 0.01;
  ts.set_mu_coulomb(mu);
  pdae.set_mu_coulomb(mu);

  // Set "one step" constraint stabilization (not generally recommended, but
  // works for a single step) and small regularization.
  const double cfm = std::numeric_limits<double>::epsilon();
  const double erp = 1.0;
  ts.SetStiffnessAndDissipation(cfm, erp);

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

  // Init the simulator for the discretized (time stepping) system.
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
  auto& xc = context_pdae->get_mutable_continuous_state_vector();
  xc.SetAtIndex(3, xc.GetAtIndex(3) + dt * ((*f)[3]));
  xc.SetAtIndex(4, xc.GetAtIndex(4) + dt * ((*f)[4]));
  xc.SetAtIndex(5, xc.GetAtIndex(5) + dt * ((*f)[5]));
  xc.SetAtIndex(0, xc.GetAtIndex(0) + dt * xc.GetAtIndex(3));
  xc.SetAtIndex(1, xc.GetAtIndex(1) + dt * xc.GetAtIndex(4));
  xc.SetAtIndex(2, xc.GetAtIndex(2) + dt * xc.GetAtIndex(5));

  // See whether the states are equal.
  const Context<double>& context_ts_new = simulator_ts.get_context();
  const auto& xd = context_ts_new.get_discrete_state(0).get_value();

  // Check that the solution is nearly identical.
  const double tol = std::numeric_limits<double>::epsilon() * 10;
  EXPECT_NEAR(xc.GetAtIndex(0), xd[0], tol);
  EXPECT_NEAR(xc.GetAtIndex(1), xd[1], tol);
  EXPECT_NEAR(xc.GetAtIndex(2), xd[2], tol);
  EXPECT_NEAR(xc.GetAtIndex(3), xd[3], tol);
  EXPECT_NEAR(xc.GetAtIndex(4), xd[4], tol);
  EXPECT_NEAR(xc.GetAtIndex(5), xd[5], tol);

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
  Rod2D<double> ts(Rod2D<double>::SystemType::kDiscretized, dt);
  Rod2D<double> pdae(Rod2D<double>::SystemType::kPiecewiseDAE, 0.0);

  // Set the coefficient of friction to a large value for both.
  const double mu = 100.0;
  ts.set_mu_coulomb(mu);
  pdae.set_mu_coulomb(mu);

  // Set "one step" constraint stabilization (not generally recommended, but
  // works for a single step) and small regularization.
  const double cfm = std::numeric_limits<double>::epsilon();
  const double erp = 1.0;
  ts.SetStiffnessAndDissipation(cfm, erp);

  // Create contexts for both.
  std::unique_ptr<Context<double>> context_ts = ts.CreateDefaultContext();
  std::unique_ptr<Context<double>> context_pdae = pdae.CreateDefaultContext();

  // This configuration has no sliding velocity.
  const double half_len = pdae.get_rod_half_length();
  ContinuousState<double>& xc = context_pdae->get_mutable_continuous_state();
  auto xd = context_ts->get_mutable_discrete_state(0).get_mutable_value();
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
  // is set by the discretized system, so stepping to dt should yield
  // exactly one step.
  simulator_ts.StepTo(dt);
  EXPECT_EQ(simulator_ts.get_num_discrete_updates(), 1);

  // TODO(edrumwri): Manually integrate the continuous state forward for the
  // piecewise DAE based approach.
  std::unique_ptr<ContinuousState<double>> f = pdae.AllocateTimeDerivatives();
  // TODO(edrumwri): Compute derivatives here.
  xc[3] += + dt * ((*f)[3]);
  xc[4] += + dt * ((*f)[4]);
  xc[5] += + dt * ((*f)[5]);
  xc[0] += + dt * xc[3];
  xc[1] += + dt * xc[4];
  xc[2] += + dt * xc[5];

  // TODO(edrumwri): Check that the solution is nearly identical.
}

/// Class for testing the Rod 2D example using compliant contact
/// thus permitting integration as an ODE.
class Rod2DContinuousTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_ = std::make_unique<Rod2D<double>>(
        Rod2D<double>::SystemType::kContinuous, 0.0);
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
    const ContinuousState<double>& xc = context_->get_continuous_state();
    return Vector6d(xc.CopyToVector());
  }

  // Return d/dt state xdot,ydot,θdot,xddot,yddot,θddot as a Vector6.
  Vector6d get_state_dot() const {
    const ContinuousState<double>& xcd = *derivatives_;
    return Vector6d(xcd.CopyToVector());
  }

  // Sets the planar pose in the context.
  void set_pose(double x, double y, double theta) {
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
    xc[0] = x; xc[1] = y; xc[2] = theta;
  }

  // Sets the planar velocity in the context.
  void set_velocity(double xdot, double ydot, double thetadot) {
    ContinuousState<double>& xc = context_->get_mutable_continuous_state();
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

/// Verify that the compliant contact resists penetration.
TEST_F(Rod2DContinuousTest, ForcesHaveRightSign) {
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

// Verifies that output ports give expected values.
GTEST_TEST(Rod2DCrossValidationTest, Outputs) {
  // Create two Rod2D systems, one time stepping (i.e., discretized),
  // one with continuous state.
  const double dt = 1e-1;
  Rod2D<double> ts(Rod2D<double>::SystemType::kDiscretized, dt);
  Rod2D<double> pdae(Rod2D<double>::SystemType::kPiecewiseDAE, 0.0);

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
  context_pdae->get_mutable_continuous_state().SetFromVector(x_pdae);
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

}  // namespace rod2d
}  // namespace examples
}  // namespace drake
