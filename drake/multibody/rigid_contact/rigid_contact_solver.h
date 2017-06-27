#pragma once

#include <memory>
#include <numeric>
#include <utility>

#include "drake/multibody/rigid_contact/rigid_contact_problem_data.h"
#include "drake/solvers/moby_lcp_solver.h"

namespace drake {
namespace multibody {
namespace rigid_contact {

/**
 * Rigid contact problem solver.
 */
template <typename T>
class RigidContactSolver {
 public:
  void SolveContactProblem(double cfm,
      const RigidContactAccelProblemData<T>& problem_data,
      VectorX<T>* cf) const;

 private:
  void FormSustainedContactLinearSystem(
      const RigidContactAccelProblemData<T>& problem_data,
      MatrixX<T>* MM, VectorX<T>* qq) const;
  void FormSustainedContactLCP(
      const RigidContactAccelProblemData<T>& problem_data,
      MatrixX<T>* MM,
      Eigen::Matrix<T, Eigen::Dynamic, 1>* qq) const;

  drake::solvers::MobyLCPSolver<T> lcp_;
  mutable Eigen::CompleteOrthogonalDecomposition<MatrixX<T>> QR_;
};

/// Solves the appropriate contact problem at the acceleration level.
/// @param cfm The regularization factor to apply to the contact problem,
///            also known as the "constraint force mixing" parameter.
/// @param problem_data The data used to compute the contact forces.
/// @param cf The computed contact forces, on return. Aborts if @p cf is null.
/// @pre Contact data has been computed.
/// @throws a std::runtime_error if the contact forces cannot be computed
///         (due to, e.g., an inconsistent configuration).
template <typename T>
void RigidContactSolver<T>::SolveContactProblem(double cfm,
    const RigidContactAccelProblemData<T>& problem_data,
    VectorX<T>* cf) const {
  using std::max;
  using std::abs;
  DRAKE_DEMAND(cf);

  // Alias problem data.
  const std::vector<int>& sliding_contacts = problem_data.sliding_contacts;
  const std::vector<int>& non_sliding_contacts =
      problem_data.non_sliding_contacts;

  // Get numbers of friction directions and types of contacts.
  const int num_sliding = sliding_contacts.size();
  const int num_non_sliding = non_sliding_contacts.size();
  const int nc = num_sliding + num_non_sliding;
  const int nr = std::accumulate(problem_data.r.begin(),
                                 problem_data.r.end(), 0);

  // Look for fast exit.
  if (nc == 0) {
    cf->resize(0);
    return;
  }
    
  // Initialize contact force vector.
  cf->resize(nc + nr);

  // Formulate and solve the linear equation 0 = MM⋅zz + qq for zz *unless*
  // there are contacts transitioning from not sliding to sliding.
  MatrixX<T> MM;
  VectorX<T> qq;
  if (problem_data.transitioning_contacts) {
    // Transitioning contacts requires formulating and solving an LCP.
    FormSustainedContactLCP(problem_data, &MM, &qq);

    // Regularize the LCP matrix as necessary.
    const int nvars = qq.size();
    MM += MatrixX<T>::Identity(nvars, nvars) * cfm;

    // Get the zero tolerance for solving the LCP.
    const T zero_tol = max(cfm, lcp_.ComputeZeroTolerance(MM));

    // Solve the LCP and compute the values of the slack variables.
    VectorX<T> zz;
    bool success = lcp_.SolveLcpLemke(MM, qq, &zz, -1, zero_tol);
    VectorX<T> ww = MM * zz + qq;

    // NOTE: This LCP might not be solvable due to inconsistent configurations.
    // Check the answer and throw a runtime error if it's no good.
    if (!success || (zz.size() > 0 && (zz.minCoeff() < -10*zero_tol ||
        ww.minCoeff() < -10*zero_tol ||
        abs(zz.dot(ww)) > nvars * 100 * zero_tol))) {
      throw std::runtime_error("Unable to solve LCP- it may be unsolvable.");
    }

    // Get the contact forces in the contact frame.
    cf->segment(0, nc) = zz.segment(0, nc);
    cf->segment(nc, nr) = zz.segment(nc, nr) -
    zz.segment(nc + nr, nr);
  } else {
    // No contacts transitioning means that an easier problem can be solved.
    FormSustainedContactLinearSystem(problem_data, &MM, &qq);

    // Solve the linear system. The LCP used in the active set method computed
    // an (invertible) basis to attain its solution, which means that MM
    // should be invertible. However, the LCP matrix was regularized (which
    // may have allowed an otherwise unsolvable LCP to be unsolvable in
    // addition to its other function, that of minimizing the effect of
    // pivoting error on the ability of the solver to find a solution on known
    // solvable problems). For this reason, we will also employ regularization
    // here.
    const int nvars = qq.size();
    MM += MatrixX<T>::Identity(nvars, nvars) * cfm;
    QR_.compute(MM);
    *cf = QR_.solve(-qq);
  }
}

// Forms the system of linear equations used to compute the accelerations during
// ODE evaluations. Specifically, this method forms the matrix @p MM and vector
// @p qq used to describe the linear complementarity problem MM*z + qq = w,
// where:
// (1) z ≥ 0
// (2) w ≥ 0
// (3) z ⋅ w = 0
// If the active set is correctly determined, the contact mode variables will
// provide the solution w = 0, implying that z = MM⁻¹⋅qq. This function only
// forms MM and qq. It does not attempt to solve the linear system (or, by
// extension, determine whether said solution for z results in a solution to
// the linear complementarity problem).
template <class T>
void RigidContactSolver<T>::FormSustainedContactLinearSystem(
    const RigidContactAccelProblemData<T>& problem_data,
    MatrixX<T>* MM, VectorX<T>* qq) const {
  using std::fabs;

  DRAKE_DEMAND(MM);
  DRAKE_DEMAND(qq);

  // Get numbers of types of contacts.
  const int num_sliding = problem_data.sliding_contacts.size();
  const int num_non_sliding = problem_data.non_sliding_contacts.size();
  const int nc = num_sliding + num_non_sliding;
  const int nr = std::accumulate(problem_data.r.begin(),
                                 problem_data.r.end(), 0);

  // Problem matrices and vectors are mildly adapted from:
  // M. Anitescu and F. Potra. Formulating Dynamic Multi-Rigid Body Contact
  // Problems as Solvable Linear Complementarity Problems. Nonlinear Dynamics,
  // 14, 1997.

  // Alias matrices / vectors to make accessing them more wieldy.
  const MatrixX<T>& N = problem_data.N;
  const MatrixX<T>& F = problem_data.F;
  const VectorX<T>& Ndot_x_v = problem_data.Ndot_x_v;
  const VectorX<T>& Fdot_x_v = problem_data.Fdot_x_v;

  // Construct the matrix:
  // N⋅M⁻¹⋅(Nᵀ - μQᵀ)  N⋅M⁻¹⋅Fᵀ
  // F⋅M⁻¹⋅Nᵀ          F⋅M⁻¹⋅Dᵀ
  const int nvars = nc + nr;
  const MatrixX<T> M_inv_x_FT = problem_data.solve_inertia(F.transpose());
  MM->resize(nvars, nvars);
  MM->block(0, 0, nc, nc) = N *
      problem_data.solve_inertia(problem_data.N_minus_mu_Q.transpose());
  MM->block(0, nc, nc, nr) = N * M_inv_x_FT;

  // Now construct the tangent contact direction rows for non-sliding contacts.
  MM->block(nc, 0, nr, nc) = MM->block(0, nc, nc, nr).transpose();
  MM->block(nc, nc, nr, nr) = F * M_inv_x_FT;

  // Construct the vector:
  // N⋅M⁻¹⋅fext + dN/dt⋅v
  // F⋅M⁻¹⋅fext + dF/dt⋅v
  const VectorX<T> M_inv_x_f = problem_data.solve_inertia(problem_data.f);
  qq->resize(nvars, 1);
  qq->segment(0, nc) = N * M_inv_x_f + Ndot_x_v;
  qq->segment(nc, nr) = F * M_inv_x_f + Fdot_x_v;
}

// Forms the LCP matrix and vector, which is used to determine the active
// set of constraints at the acceleration-level.
template <class T>
void RigidContactSolver<T>::FormSustainedContactLCP(
    const RigidContactAccelProblemData<T>& problem_data,
    MatrixX<T>* MM,
    Eigen::Matrix<T, Eigen::Dynamic, 1>* qq) const {
  using std::fabs;

  DRAKE_DEMAND(MM);
  DRAKE_DEMAND(qq);

  // Get numbers of types of contacts.
  const int num_sliding = problem_data.sliding_contacts.size();
  const int num_non_sliding = problem_data.non_sliding_contacts.size();
  const int nc = num_sliding + num_non_sliding;
  const int nr = std::accumulate(problem_data.r.begin(),
                                 problem_data.r.end(), 0);
  const int nk = nr * 2;

  // Problem matrices and vectors are mildly adapted from:
  // M. Anitescu and F. Potra. Formulating Dynamic Multi-Rigid Body Contact
  // Problems as Solvable Linear Complementarity Problems. Nonlinear Dynamics,
  // 14, 1997.

  // Alias matrices / vectors to make accessing them less clunky.
  const MatrixX<T>& N = problem_data.N;
  const MatrixX<T>& F = problem_data.F;
  const VectorX<T>& Ndot_x_v = problem_data.Ndot_x_v;
  const VectorX<T>& Fdot_x_v = problem_data.Fdot_x_v;
  const VectorX<T>& mu_non_sliding = problem_data.mu_non_sliding;

  // Construct a matrix similar to E in Anitscu and Potra 1997. This matrix
  // will be used to specify the constraints 0 ≤ μ⋅fN - E⋅fF ⊥ λ ≥ 0 and
  // 0 ≤ e⋅λ + F⋅dv/dt ⊥ fF ≥ 0.
  MatrixX<T> E = MatrixX<T>::Zero(nk, num_non_sliding);
  for (int i = 0, j = 0; i < num_non_sliding; ++i) {
    const int num_tangent_dirs = problem_data.r[i];
    E.col(i).segment(j, num_tangent_dirs).setOnes();
    j += num_tangent_dirs;
  }

  // Construct the LCP matrix. First do the "normal contact direction" rows:
  // N⋅M⁻¹⋅(Nᵀ - μQᵀ)  N⋅M⁻¹⋅Dᵀ  0
  // D⋅M⁻¹⋅Nᵀ          D⋅M⁻¹⋅Dᵀ  E
  // μ                 -Eᵀ       0
  // where D = [F -F]
  const int nvars = nc + nk + num_non_sliding;
  MatrixX<T> M_inv_x_FT = problem_data.solve_inertia(F.transpose());
  MM->resize(nvars, nvars);
  MM->block(0, 0, nc, nc) = N *
      problem_data.solve_inertia(problem_data.N_minus_mu_Q.transpose());
  MM->block(0, nc, nc, nr) = N * M_inv_x_FT;
  MM->block(0, nc + nr, nc, nr) = -MM->block(0, nc, nc, nr);
  MM->block(0, nc + nk, num_non_sliding, num_non_sliding).setZero();

  // Now construct the un-negated tangent contact direction rows (everything
  // but last block column).
  MM->block(nc, 0, nr, nc) = MM->block(0, nc, nc, nr).transpose();
  MM->block(nc, nc, nr, nr) = F * M_inv_x_FT;
  MM->block(nc, nc + nr, nr, nr) = -MM->block(nc, nc, nr, nr);

  // Now construct the negated tangent contact direction rows (everything but
  // last block column). These negated tangent contact directions allow the
  // LCP to compute forces applied along the negative x-axis.
  MM->block(nc + nr, 0, nr, nc + nk) = -MM->block(nc, 0, nr, nc + nk);

  // Construct the last block column for the last set of rows (see Anitescu and
  // Potra, 1997).
  MM->block(nc, nc + nk, nk, num_non_sliding) = E;

  // Construct the last two rows, which provide the friction "cone" constraint.
  MM->block(nc + nk, 0, num_non_sliding, num_non_sliding) =
      Eigen::DiagonalMatrix<T, Eigen::Dynamic>(mu_non_sliding);
  MM->block(nc + nk, nc, num_non_sliding, nk) = -E.transpose();
  MM->block(nc + nk, nc + nk, num_non_sliding, num_non_sliding).setZero();

  // Construct the LCP vector:
  // N⋅M⁻¹⋅fext + dN/dt⋅v
  // D⋅M⁻¹⋅fext + dD/dt⋅v
  // 0
  // where, as above, D is defined as [F -F]
  VectorX<T> M_inv_x_f = problem_data.solve_inertia(problem_data.f);
  qq->resize(nvars, 1);
  qq->segment(0, nc) = N * M_inv_x_f + Ndot_x_v;
  qq->segment(nc, nr) = F * M_inv_x_f + Fdot_x_v;
  qq->segment(nc + nr, nr) = -qq->segment(nc, nr);
  qq->segment(nc + nk, num_non_sliding).setZero();
}

}  // namespace rigid_contact
}  // namespace multibody
}  // namespace drake
