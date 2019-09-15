#include <DR/primitives/impedance_controller.h>

#include <gtest/gtest.h>

#include <drake/multibody/benchmarks/pendulum/make_pendulum_plant.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram.h>
#include "drake/systems/framework/diagram_builder.h"
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/primitives/adder.h>
#include <drake/systems/primitives/demultiplexer.h>

using drake::multibody::benchmarks::pendulum::MakePendulumPlant;
using drake::multibody::benchmarks::pendulum::PendulumParameters;
using drake::multibody::MultibodyPlant;
using drake::systems::Adder;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::ContinuousState;
using drake::systems::Demultiplexer;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::InputPort;
using drake::systems::LeafSystem;
using drake::MatrixX;
using drake::VectorX;
using Eigen::VectorBlock;

namespace DR {
namespace {

const double K = 1000;
const double C = 10;

/// This class implements an impedance controller for treating a pendulum as if...
class PendulumImpedanceController : public ImpedanceController<double> {
 public:
  explicit PendulumImpedanceController(MultibodyPlant<double> const* all_plant) :
      ImpedanceController<double>(all_plant) { }

  double lower_limit() const { return lower_limit_; }
  void set_lower_limit(double limit) { lower_limit_ = limit; }

  // Determines whether the limit is active (meaning that phi.size() > 0).
  bool IsLimitActive(const Context<double>& context) const {
    // Evaluate the robot's position.
    const VectorBlock<const VectorX<double>> q = this->all_q_estimated_input_port().Eval(context);
    return (q[0] <= lower_limit_);
  }

  // Phi can be interpreted as the signed distance from the limit. We only allow this distance to be non-positive; if
  // it does become positive, the constraint disappears.
  VectorX<double> DoCalcPhi(const Context<double>& context) const final {
    if (!IsLimitActive(context)) { return VectorX<double>(); }

    const VectorBlock<const VectorX<double>> q = this->all_q_estimated_input_port().Eval(context);
    VectorX<double> result(1);
    result << -lower_limit_;
    return result + q;
  }

  VectorX<double> DoCalcPhiDot(const Context<double>& context) const final {
    if (!IsLimitActive(context)) { return VectorX<double>(); }

    // Just return the velocity.
    const VectorBlock<const VectorX<double>> v = this->all_v_estimated_input_port().Eval(context);
    return v;
  }

  // G must always be returned. Note that G will either be an identity matrix (when the limit is violated) or an empty
  // matrix (otherwise); this virtual limit scenario, which uses the rotational axis of the pendulum as the (rotational)
  // impedance direction simplifies our test fixture and the calculation of G considerably.
  MatrixX<double> DoCalcG(const Context<double>& context) const final {
    int phi_dim = (IsLimitActive(context)) ? 1 : 0;
    return MatrixX<double>::Identity(phi_dim, 1);
  }

  // For consistency, K should only return a value when Phi does.
  MatrixX<double> DoCalcK(const Context<double>& context) const final {
    if (!IsLimitActive(context)) { return MatrixX<double>(); }
    MatrixX<double> K_matrix(1, 1);
    K_matrix << K;
    return K_matrix;
  }

  // For consistency, C should only return a value when Phi does.
  MatrixX<double> DoCalcC(const Context<double>& context) const final {
    if (!IsLimitActive(context)) { return MatrixX<double>(); }
    MatrixX<double> C_matrix(1, 1);
    C_matrix << C;
    return C_matrix;
  }

 private:
  // The robot's lower limit is zero by default.
  double lower_limit_{0};
};

// This class implements a virtual *lower* joint limit for a single DoF system using stiffness and damping parameters.
class OneDofSystemVirtualLimit : public LeafSystem<double> {
 public:
  OneDofSystemVirtualLimit() {
    // Declare the state input.
    this->DeclareVectorInputPort("state", BasicVector<double>(2 /* one DoF x (position + velocity = 2 */));

    // Declare the command output.
    this->DeclareVectorOutputPort(
        "force", BasicVector<double>(1 /* one actuator */), &OneDofSystemVirtualLimit::CalcOutput);
  }

  void set_lower_limit(double limit) { lower_limit_ = limit; }
  double lower_limit() const { return lower_limit_; }

 private:
  void CalcOutput(const Context<double>& context, BasicVector<double>* output) const {
    ASSERT_EQ(output->size(), 1);

    // Evaluate the input port to get the current state; if it's not connected, return now.
    const InputPort<double>& input_port = this->get_input_port(0);
    const VectorBlock<const VectorX<double>> estimated_state = input_port.Eval(context);

    // First dimension will be position; next will be velocity.
    const double position = estimated_state[0];
    const double velocity = estimated_state[1];

    // Get the position violation. If this value is negative, there is no violation and the virtual limit will produce a
    // zero output.
    const double e = lower_limit_ - position;
    if (e < 0) {
      output->SetZero();
      return;
    }

    // Get the time derivative of the position violation. A positive value means that the violation is increasing,
    // while a negative value means that it is decreasing.
    const double edot = -velocity;

    // Compute the virtual limit force.
    output->get_mutable_value()[0] = K*e + C*edot;
  }

  // Limit position is at zero by default.
  double lower_limit_{0};
};

class ImpedanceControllerTest : public ::testing::Test {
 public:
  void SetPendulumVelocity(double qdot) const {
    VectorX<double> qdot_vec(1);
    qdot_vec << qdot;
    pendulum_->SetVelocities(pendulum_context_, qdot_vec);
  }

  void SetPendulumPosition(double q) const {
    VectorX<double> q_vec(1);
    q_vec << q;
    pendulum_->SetPositions(pendulum_context_, q_vec);
  }

  double CalcPendulumAcceleration() const {
    std::unique_ptr<ContinuousState<double>> xc = context_->get_continuous_state().Clone();
    diagram_->CalcTimeDerivatives(*context_, xc.get());
    return (*xc)[1]; /* Acceleration will be the second component of the vector */
  }

  void SetDesiredPendulumAcceleration(double qdd) {
    impedance_controller_context_->FixInputPort(
        impedance_controller_->all_vdot_desired_input_port().get_index(), VectorX<double>::Ones(1) * qdd);
  }

  PendulumImpedanceController& impedance_controller() { return *impedance_controller_; }
  const Context<double>& impedance_controller_context() const { return *impedance_controller_context_; }
  OneDofSystemVirtualLimit& limiter() { return *limiter_; }
  const Context<double>& limiter_context() const { return *limiter_context_; }

 private:
  void SetUp() {
    DiagramBuilder<double> builder;

    // Construct the pendulum.
    PendulumParameters pendulum_parameters;
    pendulum_ = builder.AddSystem(MakePendulumPlant(pendulum_parameters));

    // Construct the impedance controller.
    impedance_controller_ = builder.AddSystem<PendulumImpedanceController>(pendulum_);

    // Construct the virtual limiter for the 1-DoF system.
    limiter_ = builder.AddSystem<OneDofSystemVirtualLimit>();
    builder.Connect(pendulum_->get_state_output_port(), limiter_->get_input_port(0));

    // Connect the virtual limiter and the impedance controller to the plant actuation input.
    const Adder<double>& adder = *builder.AddSystem<Adder<double>>(
        2 /* number of input ports */, 1 /* dimension of all ports */);
    builder.Connect(limiter_->get_output_port(0), adder.get_input_port(0));
    builder.Connect(impedance_controller_->generalized_effort_output_port(), adder.get_input_port(1));

    // Connect the "all" state output to the impedance controller inputs.
    const Demultiplexer<double>& demuxer = *builder.AddSystem<Demultiplexer<double>>(
        2 /* size */, 1 /* output ports size */);
    builder.Connect(pendulum_->get_state_output_port(), demuxer.get_input_port(0));
    builder.Connect(demuxer.get_output_port(0), impedance_controller_->all_q_estimated_input_port());
    builder.Connect(demuxer.get_output_port(1), impedance_controller_->all_v_estimated_input_port());

    // Connect the adder output to the plant input.
    builder.Connect(adder.get_output_port(), pendulum_->get_actuation_input_port());

    // Construct the diagram.
    diagram_ = builder.Build();

    // Create the context.
    context_ = diagram_->CreateDefaultContext();

    // Get relevant contexts.
    pendulum_context_ = &diagram_->GetMutableSubsystemContext(*pendulum_, context_.get());
    impedance_controller_context_ = &diagram_->GetMutableSubsystemContext(*impedance_controller_, context_.get());
    limiter_context_ = &diagram_->GetMutableSubsystemContext(*limiter_, context_.get());
  }

  PendulumImpedanceController* impedance_controller_{nullptr};
  MultibodyPlant<double>* pendulum_{nullptr};
  OneDofSystemVirtualLimit* limiter_{nullptr};
  Context<double>* pendulum_context_{nullptr};
  Context<double>* impedance_controller_context_{nullptr};
  Context<double>* limiter_context_{nullptr};
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> context_;
};

// Verifies that the output port can be queried.
TEST_F(ImpedanceControllerTest, Ports) {
  // Set the desired acceleration for the pendulum to zero.
  SetDesiredPendulumAcceleration(0.0);

  EXPECT_NO_THROW(impedance_controller().generalized_effort_output_port().Eval(impedance_controller_context()));
}

// Tests that, in the absence of any constraints (phi values), the controller acts as an inverse dynamics controller.
TEST_F(ImpedanceControllerTest, FreeSpace) {
  // Set the velocity of the pendulum to non-zero.
  SetPendulumVelocity(0.1234 /* a unique value */);

  // Set the pendulum position such that it is not violating the joint limit.
  const double y = 0.1;  // meters
  SetPendulumPosition(limiter().lower_limit() + y);

  // Set the desired acceleration for the pendulum to zero.
  SetDesiredPendulumAcceleration(0.0);

  // Calculate the time derivatives of the pendulum state and verify that the acceleration is zero.
  const double tol = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(CalcPendulumAcceleration(), 0.0, tol);
}

// Checks that the acceleration is what was requested when the limit is active.
TEST_F(ImpedanceControllerTest, PendulumJointLimit) {
  // Set the velocity of the pendulum to zero.
  SetPendulumVelocity(0);

  // Set the pendulum position such that it is violating our virtual joint limit by y units.
  const double y = 0.1;  // meters
  impedance_controller().set_lower_limit(limiter().lower_limit());
  SetPendulumPosition(limiter().lower_limit() - y);

  // Set the desired acceleration for the pendulum to zero.
  SetDesiredPendulumAcceleration(0.0);

  // Calculate the time derivatives of the pendulum state and verify that the acceleration is zero.
  const double tol = 200 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(CalcPendulumAcceleration(), 0.0, tol);

  // Set the pendulum velocity to nonzero.
  SetPendulumVelocity(1.0);

  // Again verify that the acceleration is zero.
  EXPECT_NEAR(CalcPendulumAcceleration(), 0.0, tol);

  // Change the position of the pendulum so that gravitational effects are zero.
  SetPendulumPosition(0.0);

  // Reset the pendulum velocity to zero.
  SetPendulumVelocity(0.0);

  // Change the limit position of the pendulum to its current position, modulo a small violation. This change must also
  // be reflected in the impedance controller.
  limiter().set_lower_limit(y);
  impedance_controller().set_lower_limit(y);

  // Verify that the force output by the limiter is equal and opposite to the force commanded by the controller.
  ASSERT_NEAR(CalcPendulumAcceleration(), 0.0, tol);
  const VectorBlock<const VectorX<double>> commanded_force =
      impedance_controller().generalized_effort_output_port().Eval(impedance_controller_context());
  const VectorBlock<const VectorX<double>> limit_force =
      limiter().get_output_port(0).Eval(limiter_context());
  ASSERT_EQ(commanded_force[0], -limit_force[0]);
}

}  // namespace
}  // namespace DR
