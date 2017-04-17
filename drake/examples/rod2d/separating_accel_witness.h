#pragma once

#include "drake/examples/rod2d/rod2d.h"
#include "drake/examples/rod2d/rigid_contact.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace examples {
namespace rod2d {

template <class T>
class Rod2D;

/// Witness function for determining whether a point of contact is accelerating
/// away from the half-space.
template <class T>
class SeparatingAccelWitness : public systems::WitnessFunction<T> {
 public:
  SeparatingAccelWitness(Rod2D<T>* rod, int contact_index) :
      rod_(rod), contact_index_(contact_index) {}

  /// This witness function indicates an unrestricted update needs to be taken.
  typename systems::DiscreteEvent<T>::ActionType get_action_type()
  const override {
    return systems::DiscreteEvent<T>::ActionType::kUnrestrictedUpdateAction;
  }

  /// This witness triggers only when the signed distance goes from strictly
  /// positive to zero/negative.
  typename systems::WitnessFunction<T>::TriggerType get_trigger_type()
  const override {
    return systems::WitnessFunction<T>::TriggerType::kPositiveThenNegative;
  }

  // Select the trigger time for this witness function to bisect the two
  // time values.
  T do_get_trigger_time(const std::pair<T, T>& time_and_witness_value0,
                        const std::pair<T, T>& time_and_witness_valuef)
  const override {
    return (time_and_witness_value0.first + time_and_witness_valuef.first) / 2;
  }

  /// The witness function itself.
  T Evaluate(const systems::Context<T>& context) override {
    using std::sin;

    // Verify the system is simulated using piecewise DAE.
    DRAKE_DEMAND(rod_->get_simulation_type() ==
        Rod2D<T>::SimulationType::kPiecewiseDAE);

    // Get the contact information.
    const RigidContact& contact =
        rod_->get_contacts(context.get_state())[contact_index_];

    // Verify rod is undergoing contact at the specified index.
    DRAKE_DEMAND(contact.state != RigidContact::ContactState::kNotContacting);

    // TODO(edrumwri): Only compute this once over the entire set
    //                 of witness functions generally.

    // Populate problem data.
    RigidContactAccelProblemData<T> problem_data;
    rod_->InitRigidContactAccelProblemData(context.get_state(), &problem_data);
    const std::vector<int>& sliding_contacts = problem_data.sliding_contacts;
    const std::vector<int>& non_sliding_contacts =
        problem_data.non_sliding_contacts;

    // Compute Jacobian matrices.
    rod_->FormRigidContactAccelJacobians(context.get_state(), &problem_data);

    // Alias vectors and Jacobian matrices.
    const MatrixX<T>& N = problem_data.N;
    const MatrixX<T>& Q = problem_data.Q;
    const MatrixX<T>& F = problem_data.F;
    const VectorX<T>& mu_sliding = problem_data.mu_sliding;

    // Construct the inverse generalized inertia matrix computed about the
    // center of mass of the rod and expressed in the world frame.
    const T mass = rod_->get_rod_mass();
    const T J = rod_->get_rod_moment_of_inertia();
    Matrix3<T> iM;
    iM << 1.0/mass, 0,         0,
        0,          1.0/mass,  0,
        0,          0,         1.0/J;

    // Alias matrices.
    MatrixX<T>& N_minus_mu_Q = problem_data.N_minus_mu_Q;
    MatrixX<T>& iM_x_FT = problem_data.iM_x_FT;

    // Precompute using indices.
    N_minus_mu_Q = rod_->
        AddScaledRightTerm(N, -mu_sliding, Q, sliding_contacts);
    iM_x_FT = rod_->MultTranspose(iM, F, non_sliding_contacts);

    // Formulate the system of linear equations.
    MatrixX<T> MM;
    VectorX<T> qq;
    rod_->FormSustainedContactLinearSystem(context, problem_data, &MM, &qq);

    // Solve the linear system. The LCP used in the active set method computed
    // an (invertible) basis to attain its solution, which means that MM should
    // be invertible. However, the LCP matrix was regularized (which may have
    // allowed an otherwise unsolvable LCP to be unsolvable in addition to its
    // other function, that of minimizing the effect of pivoting error on the
    // ability of the solver to find a solution on known solvable problems). For
    // this reason, we will also employ regularization here.
    const double cfm = rod_->get_cfm();
    MM += MatrixX<T>::Identity(MM.rows(), MM.rows()) * cfm;
    rod_->QR_.compute(MM);
    VectorX<T> zz = rod_->QR_.solve(-qq);

    // Return the normal force. A negative value means that the force has
    // become tensile, which violates the compressiveness constraint.
    return zz(contact_index_);
  }

  // Uninformed time tolerance.
  T get_time_isolation_tolerance() const override {
    using std::sqrt;
    return sqrt(std::numeric_limits<double>::epsilon());
  }

  // Uninformed dead-band values and time tolerance.
  T get_positive_dead_band() const override {
    using std::sqrt;
    return sqrt(std::numeric_limits<double>::epsilon());
  }

  // Uninformed dead-band values and time tolerance.
  T get_negative_dead_band() const override {
    using std::sqrt;
    return sqrt(std::numeric_limits<double>::epsilon());
  }

 private:
  /// Pointer to the rod system.
  Rod2D<T>* rod_;

  /// Index of the contact point that this witness function applies to.
  int contact_index_{-1};
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

