#pragma once

#include "drake/example/rod2d/rod2d.h"
#include "drake/example/rod2d/rigid_contact.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace examples {
namespace rod2d {

template <class T>
class StickingFrictionForcesSlackWitness : public systems::WitnessFunction<T> {
 public:
  StickingFrictionForcesSlackWitness(Rod2D<T>* rod, int contact_index) :
      rod_(rod), contact_index_(contact_index) {}

  /// This witness function indicates an unrestricted update needs to be taken.
  systems::ActionType<T> get_action_type() const override {
    return systems::ActionType<T>::kUnrestrictedUpdateAction;
  }

  /// This witness triggers whenever the slack in the force becomes negative.
  TriggerType get_trigger_type() const override {
    return systems::WitnessFunction::TriggerType::kPositiveThenNegative;
  }

  /// The witness function itself.
  T Evaluate(const systems::Context<T>& context) override {
    using std::sin;

    // Verify the system is simulated using piecewise DAE.
    DRAKE_DEMAND(rod->get_simulation_type() ==
        Rod2D<T>::SimulationType::kPiecewiseDAE);

    // Get the contact information.
    const RigidContact& contact = rod_->get_contacts(context)[contact_index_];

    // Verify rod is not undergoing sliding contact at the specified index.
    DRAKE_DEMAND(contact.state ==
        RigidContact::ContactState::kContactingWithoutSliding);

    // Compute the acceleration at the point of contact before contact
    // forces are applied.
    // Compute the external forces (expressed in the world frame).
    const Vector3<T> fext = rod_->ComputeExternalForces(context);

    // TODO(edrumwri): Only compute this once over the entire set
    //                 of witness functions generally.

    // Populate problem data.
    RigidContactAccelProblemData<T> problem_data;
    rod_->InitRigidContactAccelProblemData(context, &problem_data);
    const std::vector<int>& sliding_contacts = problem_data.sliding_contacts;
    const std::vector<int>& non_sliding_contacts =
        problem_data.non_sliding_contacts;

    // Get numbers of friction directions and types of contacts.
    const int num_sliding = sliding_contacts.size();
    const int num_non_sliding = non_sliding_contacts.size();
    const int nc = num_sliding + num_non_sliding;
    const int k = get_num_tangent_directions_per_contact();
    const int nk = k * num_non_sliding;
    const int half_nk = nk / 2;

    // Compute Jacobian matrices.
    FormRigidContactAccelJacobians(context.get_state(), &problem_data);

    // Alias vectors and Jacobian matrices.
    const MatrixX<T>& N = problem_data.N;
    const MatrixX<T>& Q = problem_data.Q;
    const MatrixX<T>& F = problem_data.F;
    const VectorX<T>& mu_sliding = problem_data.mu_sliding;

    // Construct the inverse generalized inertia matrix computed about the
    // center of mass of the rod and expressed in the world frame.
    Matrix3<T> iM;
    iM << 1.0/mass_, 0,         0,
        0,         1.0/mass_, 0,
        0,         0,         1.0/J_;

    // Alias matrices.
    MatrixX<T>& N_minus_mu_Q = problem_data.N_minus_mu_Q;
    MatrixX<T>& iM_x_FT = problem_data.iM_x_FT;

    // Precompute using indices.
    N_minus_mu_Q = AddScaledRightTerm(N, -mu_sliding, Q, sliding_contacts);
    iM_x_FT = MultTranspose(iM, F, non_sliding_contacts);

    // Formulate and solve the linear equation 0 = MM⋅zz + qq for zz.
    MatrixX<T> MM;
    VectorX<T> qq;
    FormSustainedContactLinearSystem(context, problem_data, &MM, &qq);

    // Formulate the system of linear equations.
    MatrixX<T> MM;
    VectorX<T> qq;
    FormSustainedContactLinearSystem(context, problem_data, &MM, &qq);

    // Solve the linear system. The LCP used in the active set method computed
    // an (invertible) basis to attain its solution, which means that MM should
    // be invertible. However, the LCP matrix was regularized (which may have
    // allowed an otherwise unsolvable LCP to be unsolvable in addition to its
    // other function, that of minimizing the effect of pivoting error on the
    // ability of the solver to find a solution on known solvable problems). For
    // this reason, we will also employ regularization here.
    const double cfm = get_cfm();
    MM += MatrixX<T>::Identity(MM.rows(), MM.rows()) * cfm;
    QR_.compute(MM);
    VectorX<T> zz = QR_.solve(-qq);

    // Get the normal force and the ℓ₁-norm of the frictional force.
    const auto fN = zz[contact_index_];
    const auto fF = zz.segment(nc + contact_index * k, k).template lpNorm<1>();

    // Determine the slack.
    return contact.mu*fN - fF;
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

