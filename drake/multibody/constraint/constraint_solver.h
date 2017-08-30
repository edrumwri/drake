#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

#include "drake/multibody/constraint/constraint_problem_data.h"
#include "drake/solvers/moby_lcp_solver.h"

namespace drake {
namespace multibody {
namespace constraint {

/// Solves constraint problems for constraint forces. Specifically, given
/// problem data corresponding to a rigid or multi-body system constrained
/// bilaterally and/or unilaterally and acted upon by friction, this class
/// computes the constraint forces.
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// They are already available to link against in the containing library.
template <typename T>
class ConstraintSolver {
 public:
  ConstraintSolver() = default;
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConstraintSolver)

  /// Solves the appropriate constraint problem at the acceleration level.
  /// @param cfm The non-negative regularization factor to apply to the
  ///            constraint problem (i.e., the underlying complementarity
  ///            problem or linear system- the latter for problems with only
  ///            bilateral constraints), also known as the "constraint force
  ///            mixing" parameter.
  /// @param problem_data The data used to compute the constraint forces.
  /// @param cf The computed constraint forces, on return, in a packed storage
  ///           format. The first `nc` elements of `cf` correspond to the
  ///           magnitudes of the contact forces applied along the normals of
  ///           the `nc` contact points. The next elements of `cf`
  ///           correspond to the frictional forces along the `r` spanning
  ///           directions at each non-sliding point of contact. The first `r`
  ///           values (after the initial `nc` elements) correspond to the first
  ///           non-sliding contact, the next `r` values correspond to the
  ///           second non-sliding contact, etc. The next `ℓ` values of `cf`
  ///           correspond to the forces applied to enforce generic unilateral
  ///           constraints. The final `b` values of `cf` correspond to the
  ///           forces applied to enforce generic bilateral constraints. Thi
  ///           packed storage format can be turned into more useful
  ///           representations through 
  ///           ComputeGeneralizedForceFromConstraintForces() and
  ///           CalcContactForcesInContactFrames(). `cf` will be resized as
  ///           necessary.
  /// @pre Constraint data has been computed.
  /// @throws a std::runtime_error if the constraint forces cannot be computed
  ///         (due to, e.g., an "inconsistent" rigid contact configuration).
  /// @throws a std::logic_error if `cf` is null or `cfm` is negative.
  void SolveConstraintProblem(double cfm,
      const ConstraintAccelProblemData<T>& problem_data,
      VectorX<T>* cf) const;

  /// Solves the appropriate impact problem at the velocity level.
  /// @param cfm The non-negative regularization factor to apply to the impact
  ///            problem (i.e., the underlying complementarity problem), also
  ///            known as the "constraint force mixing" parameter.
  /// @param problem_data The data used to compute the impulsive constraint
  ///            forces.
  /// @param cf The computed impulsive forces, on return, in a packed storage
  ///           format. The first `nc` elements of `cf` correspond to the
  ///           magnitudes of the contact impulses applied along the normals of
  ///           the `nc` contact points. The next elements of `cf`
  ///           correspond to the frictional impulses along the `r` spanning
  ///           directions at each point of contact. The first `r`
  ///           values (after the initial `nc` elements) correspond to the first
  ///           contact, the next `r` values correspond to the second contact,
  ///           etc. The next `ℓ` values of `cf` correspond to the impulsive
  ///           forces applied to enforce unilateral constraint functions. The
  ///           final `b` values of `cf` correspond to the forces applied to
  ///           enforce generic bilateral constraints. This packed storage
  ///           format can be turned into more useful representations through
  ///           ComputeGeneralizedImpulseFromConstraintImpulses() and
  ///           CalcImpactForcesInContactFrames(). `cf` will be resized as
  ///           necessary.
  /// @pre Constraint data has been computed.
  /// @throws a std::runtime_error if the constraint forces cannot be computed
  ///         (due to, e.g., the effects of roundoff error in attempting to
  ///         solve a complementarity problem); in such cases, it is
  ///         recommended to increase `cfm` and attempt again.
  /// @throws a std::logic_error if `cf` is null or `cfm` is negative.
  void SolveImpactProblem(double cfm,
                          const ConstraintVelProblemData<T>& problem_data,
                          VectorX<T>* cf) const;

  /// Computes the generalized force on the system from the constraint forces
  /// given in packed storage.
  /// @param problem_data The data used to compute the contact forces.
  /// @param cf The computed constraint forces, in the packed storage
  ///           format described in documentation for SolveConstraintProblem.
  /// @param[out] generalized_force The generalized force acting on the system
  ///             from the total constraint wrench is stored here, on return.
  ///             This method will resize `generalized_force` as necessary. The
  ///             indices of `generalized_force` will exactly match the indices
  ///             of `problem_data.f`.
  /// @throws std::logic_error if `generalized_force` is null or `cf`
  ///         vector is incorrectly sized.
  static void ComputeGeneralizedForceFromConstraintForces(
      const ConstraintAccelProblemData<T>& problem_data,
      const VectorX<T>& cf,
      VectorX<T>* generalized_force);

  /// Computes the generalized impulse on the system from the constraint
  /// impulses given in packed storage.
  /// @param problem_data The data used to compute the constraint impulses.
  /// @param cf The computed constraint impulses, in the packed storage
  ///           format described in documentation for SolveImpactProblem.
  /// @param[out] generalized_impulse The generalized impulse acting on the
  ///             system from the total constraint wrench is stored here, on
  ///             return. This method will resize `generalized_impulse` as
  ///             necessary. The indices of `generalized_impulse` will exactly
  ///             match the indices of `problem_data.v`.
  /// @throws std::logic_error if `generalized_impulse` is null or `cf`
  ///         vector is incorrectly sized.
  static void ComputeGeneralizedImpulseFromConstraintImpulses(
      const ConstraintVelProblemData<T>& problem_data,
      const VectorX<T>& cf,
      VectorX<T>* generalized_impulse);

  /// Computes the system generalized acceleration, given the external forces
  /// (stored in `problem_data`) and the constraint forces.
  /// @param cf The computed constraint forces, in the packed storage
  ///           format described in documentation for SolveConstraintProblem.
  /// @throws std::logic_error if `generalized_acceleration` is null or
  ///         `cf` vector is incorrectly sized.
  static void ComputeGeneralizedAcceleration(
      const ConstraintAccelProblemData<T>& problem_data,
      const VectorX<T>& cf,
      VectorX<T>* generalized_acceleration);

  /// Computes the change to the system generalized velocity from constraint
  /// impulses.
  /// @param cf The computed constraint impulses, in the packed storage
  ///           format described in documentation for SolveImpactProblem.
  /// @throws std::logic_error if `generalized_delta_v` is null or
  ///         `cf` vector is incorrectly sized.
  static void ComputeGeneralizedVelocityChange(
      const ConstraintVelProblemData<T>& problem_data,
      const VectorX<T>& cf,
      VectorX<T>* generalized_delta_v);

  /// Gets the contact forces expressed in each contact frame *for 2D contact
  /// problems* from the "packed" solution returned by SolveConstraintProblem().
  /// @param cf the output from SolveConstraintProblem()
  /// @param problem_data the problem data input to SolveConstraintProblem()
  /// @param contact_frames the contact frames corresponding to the contacts.
  ///        The first column of each matrix should give the contact normal,
  ///        while the second column gives a contact tangent. For sliding
  ///        contacts, the contact tangent should point along the direction of
  ///        sliding. For non-sliding contacts, the tangent direction should be
  ///        that used to determine `problem_data.F`. All vectors should be
  ///        expressed in the global frame.
  /// @param[out] contact_forces a non-null vector of a doublet of values, where
  ///             the iᵗʰ element represents the force along each basis
  ///             vector in the iᵗʰ contact frame.
  /// @throws std::logic_error if `contact_forces` is null, if
  ///         `contact_forces` is not empty, if `cf` is not the
  ///         proper size, if the number of tangent directions is not one per
  ///         non-sliding contact (indicating that the contact problem might not
  ///         be 2D), if the number of contact frames is not equal to the number
  ///         of contacts, or if a contact frame does not appear to be
  ///         orthonormal.
  /// @note On return, the contact force at the iᵗʰ contact point expressed
  ///       in the world frame is `contact_frames[i]` * `contact_forces[i]`.
  static void CalcContactForcesInContactFrames(
      const VectorX<T>& cf,
      const ConstraintAccelProblemData<T>& problem_data,
      const std::vector<Matrix2<T>>& contact_frames,
      std::vector<Vector2<T>>* contact_forces);

  /// Gets the contact impulses expressed in each contact frame *for 2D contact
  /// problems* from the "packed" solution returned by SolveImpactProblem().
  /// @param cf the output from SolveImpactProblem()
  /// @param problem_data the problem data input to SolveImpactProblem()
  /// @param contact_frames the contact frames corresponding to the contacts.
  ///        The first column of each matrix should give the contact normal,
  ///        while the second column gives a contact tangent (specifically, the
  ///        tangent direction used to determine `problem_data.F`). All
  ///        vectors should be expressed in the global frame.
  /// @param[out] contact_impulses a non-null vector of a doublet of values,
  ///             where the iᵗʰ element represents the impulsive force along
  ///             each basis vector in the iᵗʰ contact frame.
  /// @throws std::logic_error if `contact_impulses` is null, if
  ///         `contact_impulses` is not empty, if `cf` is not the
  ///         proper size, if the number of tangent directions is not one per
  ///         contact (indicating that the contact problem might not be 2D), if
  ///         the number of contact frames is not equal to the number
  ///         of contacts, or if a contact frame does not appear to be
  ///         orthonormal.
  /// @note On return, the contact impulse at the iᵗʰ contact point expressed
  ///       in the world frame is `contact_frames[i]` * `contact_impulses[i]`.
  static void CalcImpactForcesInContactFrames(
      const VectorX<T>& cf,
      const ConstraintVelProblemData<T>& problem_data,
      const std::vector<Matrix2<T>>& contact_frames,
      std::vector<Vector2<T>>* contact_impulses);

 private:
  // Computes a constraint space compliance matrix A⋅M⁻¹⋅Bᵀ, where A ∈ ℝᵃˣᵐ
  // (realized here using an operator) and B ∈ ℝᵇˣᵐ are both Jacobian matrices
  // and M⁻¹ ∈ ℝᵐˣᵐ is the inverse of the generalized inertia matrix. Note that
  // mixing types of constraints is explicitly allowed. Aborts if A_iM_BT is
  // not of size a × b.
  static void ComputeConstraintSpaceComplianceMatrix(
      std::function<VectorX<T>(const VectorX<T>&)> A_mult,
      int a,
      const MatrixX<T>& M_inv_BT,
      Eigen::Ref<MatrixX<T>>);

  // Computes the matrix M⁻¹⋅Gᵀ, G ∈ ℝᵐˣⁿ is a constraint Jacobian matrix
  // (realized here using an operator) and M⁻¹ ∈ ℝⁿˣⁿ is the inverse of the
  // generalized inertia matrix. Resizes iM_GT as necessary.
  static void ComputeInverseInertiaTimesGT(
      std::function<MatrixX<T>(const MatrixX<T>&)> M_inv_mult,
      std::function<VectorX<T>(const VectorX<T>&)> G_transpose_mult,
      int m,
      MatrixX<T>* iM_GT);

  void FormImpactingConstraintLCP(
      const ConstraintVelProblemData<T>& problem_data,
      MatrixX<T>* MM, VectorX<T>* qq) const;
  void FormSustainedConstraintLCP(
      const ConstraintAccelProblemData<T>& problem_data,
      MatrixX<T>* MM, VectorX<T>* qq) const;

  drake::solvers::MobyLCPSolver<T> lcp_;
};

template <typename T>
void ConstraintSolver<T>::SolveConstraintProblem(double cfm,
    const ConstraintAccelProblemData<T>& problem_data,
    VectorX<T>* cf) const {
  using std::max;
  using std::abs;

  if (!cf)
    throw std::logic_error("cf (output parameter) is null.");
  if (cfm < 0.0) {
    throw std::logic_error("Constraint force mixing (CFM) parameter is "
                               "negative.");
  }

  // Alias problem data.
  const std::vector<int>& sliding_contacts = problem_data.sliding_contacts;
  const std::vector<int>& non_sliding_contacts =
      problem_data.non_sliding_contacts;

  // Get numbers of friction directions and types of contacts.
  const int num_sliding = sliding_contacts.size();
  const int num_non_sliding = non_sliding_contacts.size();
  const int num_contacts = num_sliding + num_non_sliding;
  const int num_spanning_vectors = std::accumulate(problem_data.r.begin(),
                                                   problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();
  const int num_eq_constraints = problem_data.kG.size();

  // Look for fast exit.
  if (num_contacts == 0 && num_limits == 0 && num_eq_constraints == 0) {
    cf->resize(0);
    return;
  }

  // Initialize contact force vector.
  cf->resize(num_contacts + num_spanning_vectors + num_limits +
      num_eq_constraints);

  // From [Cottle 1992] (p. 29), the constraint problem is a mixed linear
  // complementarity problem of the form:
  //     Au + Cv + a = 0
  //     Du + Bv + b ≥ 0
  //               v ≥ 0
  // vᵀ(b + Du + Bv) = 0
  // where u are "free" variables. If the matrix A is nonsingular, u can be
  // solved for:
  //      u = A⁻¹ (a + Cv)
  // allowing the mixed LCP to be converted to a "pure" LCP (q, M) by:
  // q = b - DA⁻¹a
  // M = B - DA⁻¹C

  // Our mixed linear complementarity problem takes the form:
  // (1) | M  -Gᵀ  -Nᵀ  -Dᵀ  0  -Lᵀ | | v̇ | + | M f | = | 0 |
  //     | G   0    0    0   0   0  | | fG | + |   0 | = | 0 |
  //     | N   0    0    0   0   0  | | fN | + |   0 | = | α |
  //     | D   0    0    0   E   0  | | fD | + |   0 | = | β |
  //     | 0   0    μ   -Eᵀ  0   0  | |  λ | + |   0 | = | γ |
  //     | L   0    0    0   0   0  | | fL | + |   0 | = | δ |
  // (2) 0 ≤ fN  ⊥  α ≥ 0
  // (3) 0 ≤ fD  ⊥  β ≥ 0
  // (4) 0 ≤ λ   ⊥  γ ≥ 0
  // (5) 0 ≤ fL  ⊥  δ ≥ 0

  // G is not of full row rank, making | M  -Gᵀ | singular.
  //                                   | G   0  |
  //
  // Selecting the largest independent subset of rows of G, which we call Ĝ,
  // addresses this problem. First, note that linear dependence in G implies
  // Gx = 0 for any vector x that satisfies Ĝx = 0. Now assume that G is a
  // stacked matrix with independent rows (Ĝ) on top and dependent rows (G̅) on
  // bottom:
  // G ≡ | Ĝ  |
  //     | G̅ |

  // We will assign zero to the components of fG corresponding to the dependent
  // rows of G, which allows casting the MCLP into a slightly modified form:
  // (6)  | M  -Ĝᵀ  -Nᵀ  -Dᵀ  0  -Lᵀ | | v̇ | + | M f | = | 0 |
  //      | Ĝ   0    0    0   0   0  | | fĜ | + |   0 | = | 0 |
  //      | N   0    0    0   0   0  | | fN | + |   0 | = | α |
  //      | D   0    0    0   E   0  | | fD | + |   0 | = | β |
  //      | 0   0    μ   -Eᵀ  0   0  | |  λ | + |   0 | = | γ |
  //      | L   0    0    0   0   0  | | fL | + |   0 | = | δ |
  // (7)  0 ≤ fN  ⊥  α ≥ 0
  // (8)  0 ≤ fD  ⊥  β ≥ 0
  // (9)  0 ≤ λ   ⊥  γ ≥ 0
  // (10) 0 ≤ fL  ⊥  δ ≥ 0

  // It should be clear that any solution to this MLCP allows us to solve the
  // MLCP (1)-(5) by setting fG = | fĜ |
  //                              |  0 |.

  // Set the initial set of active constraints.
  std::vector<int> active_constraints;

  // Determine new G_mult using active constraints.
  auto G_mult = [&problem_data, &active_constraints](
      const VectorX<T>& v) -> VectorX<T> {
    VectorX<T> result_full = problem_data.G_mult(v);
    VectorX<T> result(active_constraints.size());
    for (int i = 0; i < static_cast<int>(active_constraints.size()); ++i)
      result[i] = result_full[active_constraints[i]];
    return result;
  };

  // Determine new G_transpose_mult using active constraints
  auto G_transpose_mult = [&problem_data, &active_constraints](
      const VectorX<T>& f) -> VectorX<T> {
    VectorX<T> lambda = VectorX<T>::Zero(problem_data.kG.size());
    for (int i = 0; i < static_cast<int>(active_constraints.size()); ++i)
      lambda[i] = f[active_constraints[i]];
    return problem_data.G_transpose_mult(lambda);
  };

  // Determine the set of active constraints.
  MatrixX<T> iM_GT;
  Eigen::LDLT<MatrixX<T>> Del;
  MatrixX<T> tentative_Del;
  for (int i = 0; i < problem_data.kG.size(); ++i)
  {
    // Tentatively add the constraint to the active set of constraints.
    active_constraints.push_back(i);

    // Form the tentative Delassus matrix.
    ComputeInverseInertiaTimesGT(problem_data.solve_inertia,
                                 G_transpose_mult,
                                 problem_data.kG.size(), &iM_GT);
    ComputeConstraintSpaceComplianceMatrix(G_mult,
                                           problem_data.kG.size(),
                                           iM_GT, tentative_Del);

    // Try to do a LDL' factorization.
    Del.compute(tentative_Del);
    if (Del.info() != Eigen::Success) {
      // Remove the constraint from the active constraint set.
      active_constraints.pop_back();
    } else {
      // If the problem is fully constrained, do not keep looping.
      if (tentative_Del.rows() == problem_data.tau.size())
        break;
    }
  }

  // From a block matrix inversion,
  // | M  -Gᵀ |⁻¹ | Y | = |  C  E || Y | = | CY + EZ   |
  // | G'  0  |   | Z |   | -Eᵀ F || Z |   | -EᵀY + FZ |
  // where C  ≡ M⁻¹ - M⁻¹Gᵀ(GM⁻¹Gᵀ)⁻¹GM⁻¹
  //       E  ≡ M⁻¹Gᵀ(GM⁻¹Gᵀ)⁻¹
  //      -Eᵀ ≡ -(GM⁻¹Gᵀ)⁻¹GM⁻¹
  //       F  ≡ (GM⁻¹Gᵀ)
  // Compute block inversion.
  auto block_solve = [&problem_data, &Del, G_mult, G_transpose_mult,
      &active_constraints](const MatrixX<T>& X) -> MatrixX<T> {
    // Set the result matrix.
    const int C_rows = problem_data.tau.size();
    const int E_cols = active_constraints.size();
    MatrixX<T> result(C_rows + E_cols, C_rows + E_cols);

    // Name the blocks of X and result.
    const auto Y = X.block(0, 0, C_rows, X.cols());
    const auto Z = X.block(C_rows, 0, E_cols, X.cols());
    auto result_top = result.block(0, 0, C_rows, result.cols());
    auto result_bot = result.block(C_rows, 0, E_cols, result.cols());

    // 1. Begin computation of components of C.
    // Compute M⁻¹ Y
    const MatrixX<T> iM_Y = problem_data.solve_inertia(Y);

    // Compute G M⁻¹ Y
    MatrixX<T> G_iM_Y(E_cols, Y.cols());
    for (int i = 0; i < Y.cols(); ++i)
      G_iM_Y.col(i) = G_mult(iM_Y.col(i));

    // Compute (GM⁻¹Gᵀ)⁻¹GM⁻¹Y
    const MatrixX<T> Del_G_iM_Y = Del.solve(G_iM_Y);

    // Compute Gᵀ(GM⁻¹Gᵀ)⁻¹GM⁻¹Y
    const MatrixX<T> GT_Del_G_iM_Y = G_transpose_mult(Del_G_iM_Y);

    // Compute M⁻¹Gᵀ(GM⁻¹Gᵀ)⁻¹GM⁻¹Y
    const MatrixX<T> iM_GT_Del_G_iM_Y = problem_data.solve_inertia(
        GT_Del_G_iM_Y);

    // 2. Begin computation of components of E
    // Compute (GM⁻¹Gᵀ)⁻¹Z
    const MatrixX<T> Del_Z = Del.solve(Z);

    // Compute Gᵀ(GM⁻¹Gᵀ)⁻¹Z
    const MatrixX<T> GT_Del_Z = G_transpose_mult(Del_Z);

    // Compute M⁻¹Gᵀ(GM⁻¹Gᵀ)⁻¹Z = EZ
    const MatrixX<T> iM_GT_Del_Z = problem_data.solve_inertia(GT_Del_Z);

    // Set the top block of the result.
    result_top = problem_data.solve_inertia(Y) - iM_GT_Del_G_iM_Y + iM_GT_Del_Z;

    // Set the bottom block of the result.
    result_bot = Del.solve(Z) - Del_G_iM_Y;

    return result;
  };

  // Set up the pure linear complementarity problem.
  MatrixX<T> MM;
  VectorX<T> qq;
  FormSustainedConstraintLCP(problem_data, &MM, &qq);

  // Regularize the LCP matrix as necessary.
  const int num_vars = qq.size();
  MM += MatrixX<T>::Identity(num_vars, num_vars) * cfm;

  // Get the zero tolerance for solving the LCP.
  const T zero_tol = max(cfm, lcp_.ComputeZeroTolerance(MM));

  // Solve the LCP and compute the values of the slack variables.
  VectorX<T> zz;
  bool success = lcp_.SolveLcpLemke(MM, qq, &zz, -1, zero_tol);
  VectorX<T> ww = MM * zz + qq;

  // NOTE: This LCP might not be solvable due to inconsistent configurations.
  // Check the answer and throw a runtime error if it's no good.
  // LCP constraints are zz ≥ 0, ww ≥ 0, zzᵀww = 0. Since the zero tolerance
  // is used to check a single element for zero (within a single pivoting
  // operation), we must compensate for the number of pivoting operations and
  // the problem size. zzᵀww must use a looser tolerance to account for the
  // num_vars multiplies.
  const int npivots = lcp_.get_num_pivots();
  if (!success || (zz.size() > 0 &&
      (zz.minCoeff() < -num_vars * npivots * zero_tol ||
      ww.minCoeff() < -num_vars * npivots * zero_tol ||
      abs(zz.dot(ww)) > num_vars * num_vars * npivots * zero_tol))) {
    throw std::runtime_error("Unable to solve LCP- it may be unsolvable.");
  }

  // Get the constraint forces in the specified packed storage format.
  cf->segment(0, num_contacts) = zz.segment(0, num_contacts);
  cf->segment(num_contacts, num_spanning_vectors) =
      zz.segment(num_contacts, num_spanning_vectors) -
      zz.segment(num_contacts + num_spanning_vectors, num_spanning_vectors);
  cf->segment(num_contacts + num_spanning_vectors, num_limits) =
      zz.segment(num_contacts + num_spanning_vectors * 2, num_limits);
}

template <typename T>
void ConstraintSolver<T>::SolveImpactProblem(
    double cfm,
    const ConstraintVelProblemData<T>& problem_data,
    VectorX<T>* cf) const {
  using std::max;
  using std::abs;

  if (!cf)
    throw std::logic_error("cf (output parameter) is null.");
  if (cfm < 0.0) {
    throw std::logic_error("Constraint force mixing (CFM) parameter is "
                               "negative.");
  }

  // Get number of contacts and limits.
  const int num_contacts = problem_data.mu.size();
  if (static_cast<size_t>(num_contacts) != problem_data.r.size()) {
    throw std::logic_error("Number of elements in 'r' does not match number"
                               "of elements in 'mu'");
  }
  const int num_limits = problem_data.kL.size();
  const int num_eq_constraints = problem_data.kG.size();

  // Look for fast exit.
  if (num_contacts == 0 && num_limits == 0 && num_eq_constraints == 0) {
    cf->resize(0);
    return;
  }

  // Get number of tangent spanning vectors.
  const int num_spanning_vectors = std::accumulate(problem_data.r.begin(),
                                                   problem_data.r.end(), 0);

  // TODO(edrumwri): Fix me.
  // If no impact, do not apply the impact model.
  if ((num_contacts == 0 ||
       problem_data.N_mult(problem_data.v).minCoeff() >= 0) &&
      (num_limits == 0 ||
       problem_data.L_mult(problem_data.v).minCoeff() >= 0)) {
    cf->setZero(num_contacts + num_spanning_vectors + num_limits);
    return;
  }

  // Initialize contact force vector.
  cf->resize(num_contacts + num_spanning_vectors + num_limits +
      num_eq_constraints);

  // Set up the linear complementarity problem.
  MatrixX<T> MM;
  VectorX<T> qq;
  FormImpactingConstraintLCP(problem_data, &MM, &qq);

  // Regularize the LCP matrix as necessary.
  const int num_vars = qq.size();
  MM += MatrixX<T>::Identity(num_vars, num_vars) * cfm;

  // Get the tolerance for zero used by the LCP solver.
  const T zero_tol = max(cfm, lcp_.ComputeZeroTolerance(MM));

  // Solve the LCP and compute the values of the slack variables.
  VectorX<T> zz;
  bool success = lcp_.SolveLcpLemke(MM, qq, &zz, -1, zero_tol);
  VectorX<T> ww = MM * zz + qq;

  // NOTE: This LCP should always be solvable.
  // Check the answer and throw a runtime error if it's no good.
  // LCP constraints are zz ≥ 0, ww ≥ 0, zzᵀww = 0. Since the zero tolerance
  // is used to check a single element for zero (within a single pivoting
  // operation), we must compensate for the number of pivoting operations and
  // the problem size. zzᵀww must use a looser tolerance to account for the
  // num_vars multiplies.
  const int npivots = lcp_.get_num_pivots();
  if (!success ||
      (zz.size() > 0 &&
       (zz.minCoeff() < -num_vars * npivots * zero_tol ||
        ww.minCoeff() < -num_vars * npivots * zero_tol ||
        abs(zz.dot(ww)) > num_vars * num_vars * npivots * zero_tol))) {
    throw std::runtime_error("Unable to solve LCP- more regularization might "
                                 "be necessary.");
  }

  // Get the constraint forces in the specified packed storage format.
  cf->segment(0, num_contacts) = zz.segment(0, num_contacts);
  cf->segment(num_contacts, num_spanning_vectors) =
      zz.segment(num_contacts, num_spanning_vectors) -
          zz.segment(num_contacts + num_spanning_vectors, num_spanning_vectors);
  cf->segment(num_contacts + num_spanning_vectors, num_limits) =
      zz.segment(num_contacts + num_spanning_vectors * 2, num_limits);
}

template <class T>
void ConstraintSolver<T>::ComputeConstraintSpaceComplianceMatrix(
    std::function<VectorX<T>(const VectorX<T>&)> A_mult,
    int a,
    const MatrixX<T>& iM_BT,
    Eigen::Ref<MatrixX<T>> A_iM_BT) {
  const int b = iM_BT.cols();
  DRAKE_DEMAND(A_iM_BT.rows() == a && A_iM_BT.cols() == b);

  // Look for fast exit.
  if (a == 0 || b == 0)
    return;

  VectorX<T> iM_bT;     // Intermediate result vector.

  for (int i = 0; i < b; ++i) {
    iM_bT = iM_BT.col(i);
    A_iM_BT.col(i) = A_mult(iM_bT);
  }
}

template <class T>
void ConstraintSolver<T>::ComputeInverseInertiaTimesGT(
    std::function<MatrixX<T>(const MatrixX<T>&)> M_inv_mult,
    std::function<VectorX<T>(const VectorX<T>&)> G_transpose_mult,
    int m,
    MatrixX<T>* iM_GT) {
  DRAKE_DEMAND(iM_GT);
  DRAKE_DEMAND(iM_GT->cols() == m);

  VectorX<T> basis(m);  // Basis vector.
  VectorX<T> gT;        // Intermediate result vector.

  // Look for fast exit.
  if (m == 0)
    return;

  for (int i = 0; i < m; ++i) {
    // Get the i'th column of G.
    basis.setZero();
    basis[i] = 1;
    gT = G_transpose_mult(basis);
    iM_GT->col(i) = M_inv_mult(gT);
  }
}

// Forms the LCP matrix and vector, which is used to determine the constraint
// forces (and can also be used to determine the active set of constraints at
// the acceleration-level).
template <class T>
void ConstraintSolver<T>::FormSustainedConstraintLCP(
    const ConstraintAccelProblemData<T>& problem_data,
    MatrixX<T>* MM, VectorX<T>* qq) const {
  DRAKE_DEMAND(MM);
  DRAKE_DEMAND(qq);

  // Get numbers of types of contacts.
  const int num_sliding = problem_data.sliding_contacts.size();
  const int num_non_sliding = problem_data.non_sliding_contacts.size();
  const int num_contacts = num_sliding + num_non_sliding;
  const int num_spanning_vectors = std::accumulate(problem_data.r.begin(),
                                 problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();

  // Problem matrices and vectors are mildly adapted from:
  // M. Anitescu and F. Potra. Formulating Dynamic Multi-Rigid Body Contact
  // Problems as Solvable Linear Complementarity Problems. Nonlinear Dynamics,
  // 14, 1997.

  // Alias operators and vectors to make accessing them less clunky.
  auto N = problem_data.N_mult;
  auto F = problem_data.F_mult;
  auto FT = problem_data.F_transpose_mult;
  auto L = problem_data.L_mult;
  auto LT = problem_data.L_transpose_mult;
  auto iM = problem_data.solve_inertia;
  const VectorX<T>& Ndot_times_v = problem_data.Ndot_times_v;
  const VectorX<T>& Fdot_times_v = problem_data.Fdot_times_v;
  const VectorX<T>& kL = problem_data.kL;
  const VectorX<T>& mu_non_sliding = problem_data.mu_non_sliding;

  // Construct a matrix similar to E in [Anitescu 1997]. This matrix will be
  // used to specify the constraints (adapted from [Anitescu 1997] Eqn 2.7):
  // 0 ≤  μₙₛ fNᵢ - eᵀ fF  ⊥  λᵢ ≥ 0 and
  // 0 ≤ e λᵢ + F dv/dt + dF/dt v ⊥ fF ≥ 0,
  // where scalar λᵢ can roughly be interpreted as the remaining tangential
  // acceleration at non-sliding contact i after frictional forces have been
  // applied and e is a vector of ones (i.e., a segment of the appropriate
  // column of E). Note that this matrix differs from the exact definition of
  // E in [Anitescu 1997] to reflect the different layout of the LCP matrix
  // from [Anitescu 1997] (the latter puts all spanning directions corresponding
  // to a contact in one block; we put half of the spanning directions
  // corresponding to a contact into one block and the other half into another
  // block).
  MatrixX<T> E = MatrixX<T>::Zero(num_spanning_vectors, num_non_sliding);
  for (int i = 0, j = 0; i < num_non_sliding; ++i) {
    E.col(i).segment(j, problem_data.r[i]).setOnes();
    j += problem_data.r[i];
  }

  // Alias these variables for more readable construction of MM and qq.
  const int ngv = problem_data.tau.size();  // generalized velocity dimension.
  const int nc = num_contacts;
  const int nr = num_spanning_vectors;
  const int nk = nr * 2;
  const int nl = num_limits;
  const int num_vars = nc + nk + num_non_sliding + nl;

  // Precompute some matrices that will be reused repeatedly.
  MatrixX<T> iM_NT_minus_muQT(ngv, nc), iM_FT(ngv, nr), iM_LT(ngv, nl);
  ComputeInverseInertiaTimesGT(
      iM, problem_data.N_minus_muQ_transpose_mult, nc, &iM_NT_minus_muQT);
  ComputeInverseInertiaTimesGT(iM, FT, nr, &iM_FT);
  ComputeInverseInertiaTimesGT(iM, LT, nl, &iM_LT);

  // Prepare blocks of the LCP matrix, which takes the form:
  // N⋅M⁻¹⋅(Nᵀ - μₛQᵀ)  N⋅M⁻¹⋅Dᵀ  0   N⋅M⁻¹⋅Lᵀ
  // D⋅M⁻¹⋅(Nᵀ - μₛQᵀ)  D⋅M⁻¹⋅Dᵀ  E   D⋅M⁻¹⋅Lᵀ
  // μ                 -Eᵀ        0   0
  // L⋅M⁻¹⋅(Nᵀ - μₛQᵀ)  L⋅M⁻¹⋅Dᵀ  0   L⋅M⁻¹⋅Lᵀ
  // where D = |  F |
  //           | -F |
  MM->resize(num_vars, num_vars);
  Eigen::Ref<MatrixX<T>> N_iM_NT_minus_muQT = MM->block(0, 0, nc, nc);
  Eigen::Ref<MatrixX<T>> N_iM_FT = MM->block(0, nc, nc, nr);
  Eigen::Ref<MatrixX<T>> N_iM_LT = MM->block(
      0, nc + nk + num_non_sliding, nc, nl);
  Eigen::Ref<MatrixX<T>> F_iM_NT_minus_muQT = MM->block(nc, 0, nr, nc);
  Eigen::Ref<MatrixX<T>> F_iM_FT = MM->block(nc, nc, nr, nr);
  Eigen::Ref<MatrixX<T>> F_iM_LT = MM->block(nc, nc + nk, nr, nl);
  Eigen::Ref<MatrixX<T>> L_iM_NT_minus_muQT = MM->block(
      nc + nk + num_non_sliding, 0, nl, nc);
  Eigen::Ref<MatrixX<T>> L_iM_LT = MM->block(
      nc + nk + num_non_sliding, nc + nk + num_non_sliding, nl, nl);
  ComputeConstraintSpaceComplianceMatrix(
      N, nc, iM_NT_minus_muQT, N_iM_NT_minus_muQT);
  ComputeConstraintSpaceComplianceMatrix(N, nc, iM_FT, N_iM_FT);
  ComputeConstraintSpaceComplianceMatrix(N, nc, iM_LT, N_iM_LT);
  ComputeConstraintSpaceComplianceMatrix(
      F, nr, iM_NT_minus_muQT, F_iM_NT_minus_muQT);
  ComputeConstraintSpaceComplianceMatrix(F, nr, iM_FT, F_iM_FT);
  ComputeConstraintSpaceComplianceMatrix(F, nr, iM_LT, F_iM_LT);
  ComputeConstraintSpaceComplianceMatrix(
      L, nl, iM_NT_minus_muQT, L_iM_NT_minus_muQT);
  ComputeConstraintSpaceComplianceMatrix(L, nl, iM_LT, L_iM_LT);

  // Construct the LCP matrix. First do the "normal contact direction" rows.
  MM->block(0, nc + nr, nc, nr) = -MM->block(0, nc, nc, nr);
  MM->block(0, nc + nk, nc, num_non_sliding).setZero();

  // Now construct the un-negated tangent contact direction rows.
  MM->block(nc, nc + nr, num_spanning_vectors, nr) =
      -MM->block(nc, nc, nr, num_spanning_vectors);
  MM->block(nc, nc + nk, num_spanning_vectors, num_non_sliding) = E;

  // Now construct the negated tangent contact direction rows. These negated
  // tangent contact directions allow the LCP to compute forces applied along
  // the negative x-axis. E will have to be reset to un-negate it.
  MM->block(nc + nr, 0, nr, nc + nk + nl) = -MM->block(nc, 0, nr, nc + nk + nl);
  MM->block(nc + nr, nc + nk, num_spanning_vectors, num_non_sliding) = E;

  // Construct the next two rows, which provide the friction "cone" constraint.
  const std::vector<int>& ns_contacts = problem_data.non_sliding_contacts;
  MM->block(nc + nk, 0, num_non_sliding, nc).setZero();
  for (int i = 0; static_cast<size_t>(i) < ns_contacts.size(); ++i)
    (*MM)(nc + nk + i, ns_contacts[i]) = mu_non_sliding[i];
  MM->block(nc + nk, nc, num_non_sliding, num_spanning_vectors) =
      -E.transpose();
  MM->block(nc + nk, nc + num_spanning_vectors, num_non_sliding,
            num_spanning_vectors) = -E.transpose();
  MM->block(nc + nk, nc + nk, num_non_sliding, num_non_sliding + nl).setZero();

  // Construct the last row block, which provides the configuration limit
  // constraint.
  MM->block(nc + nk + num_non_sliding, 0, nl, nc + nk) =
      MM->block(0, nc + nk + num_non_sliding, nc + nk, nl).transpose().eval();

  // Construct the LCP vector:
  // N⋅M⁻¹⋅fext + dN/dt⋅v
  // D⋅M⁻¹⋅fext + dD/dt⋅v
  // 0
  // L⋅M⁻¹⋅fext + dL/dt⋅v + kL
  // where, as above, D is defined as [F -F]
  VectorX<T> M_inv_x_f = problem_data.solve_inertia(problem_data.tau);
  qq->resize(num_vars, 1);
  qq->segment(0, nc) = N(M_inv_x_f) + Ndot_times_v;
  qq->segment(nc, nr) = F(M_inv_x_f) + Fdot_times_v;
  qq->segment(nc + nr, num_spanning_vectors) = -qq->segment(nc, nr);
  qq->segment(nc + nk, num_non_sliding).setZero();
  qq->segment(nc + nk + num_non_sliding, num_limits) = L(M_inv_x_f) + kL;
}

// Forms the LCP matrix and vector, which is used to determine the collisional
// impulses.
template <class T>
void ConstraintSolver<T>::FormImpactingConstraintLCP(
    const ConstraintVelProblemData<T>& problem_data,
    MatrixX<T>* MM, VectorX<T>* qq) const {
  DRAKE_DEMAND(MM);
  DRAKE_DEMAND(qq);

  // Get numbers of contacts.
  const int num_contacts = problem_data.mu.size();
  const int num_spanning_vectors = std::accumulate(problem_data.r.begin(),
                                                   problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();

  // Problem matrices and vectors are nearly identical to:
  // M. Anitescu and F. Potra. Formulating Dynamic Multi-Rigid Body Contact
  // Problems as Solvable Linear Complementarity Problems. Nonlinear Dynamics,
  // 14, 1997.

  // Alias operators and vectors to make accessing them less clunky.
  const auto N = problem_data.N_mult;
  const auto NT = problem_data.N_transpose_mult;
  const auto F = problem_data.F_mult;
  const auto FT = problem_data.F_transpose_mult;
  const auto L = problem_data.L_mult;
  const auto LT = problem_data.L_transpose_mult;
  auto iM = problem_data.solve_inertia;
  const VectorX<T>& mu = problem_data.mu;

  // Construct the matrix E in [Anitscu 1997]. This matrix will be used to
  // specify the constraints:
  // 0 ≤ μ⋅fN - E⋅fF ⊥ λ ≥ 0 and
  // 0 ≤ e⋅λ + F⋅v ⊥ fF ≥ 0,
  // where λ can roughly be interpreted as the remaining tangential velocity
  // at the impacting contacts after frictional impulses have been applied and
  // e is a vector of ones (i.e., a segment of the appropriate column of E).
  // Note that this matrix differs from the exact definition of E in
  // [Anitescu 1997] to reflect the different layout of the LCP matrix from
  // [Anitescu 1997] (the latter puts all spanning directions corresponding
  // to a contact in one block; we put half of the spanning directions
  // corresponding to a contact into one block and the other half into another
  // block).
  MatrixX<T> E = MatrixX<T>::Zero(num_spanning_vectors, num_contacts);
  for (int i = 0, j = 0; i < num_contacts; ++i) {
    E.col(i).segment(j, problem_data.r[i]).setOnes();
    j += problem_data.r[i];
  }

  // Alias these variables for more readable construction of MM and qq.
  const int ngv = problem_data.v.size();  // generalized velocity dimension.
  const int nc = num_contacts;
  const int nr = num_spanning_vectors;
  const int nk = nr * 2;
  const int nl = num_limits;

  // Precompute some matrices that will be reused repeatedly.
  MatrixX<T> iM_NT(ngv, nc), iM_FT(ngv, nr), iM_LT(ngv, nl);
  ComputeInverseInertiaTimesGT(iM, NT, nc, &iM_NT);
  ComputeInverseInertiaTimesGT(iM, FT, nr, &iM_FT);
  ComputeInverseInertiaTimesGT(iM, LT, nl, &iM_LT);

  // Prepare blocks of the LCP matrix, which takes the form:
  // N⋅M⁻¹⋅Nᵀ  N⋅M⁻¹⋅Dᵀ  0   N⋅M⁻¹⋅Lᵀ
  // D⋅M⁻¹⋅Nᵀ  D⋅M⁻¹⋅Dᵀ  E   D⋅M⁻¹⋅Lᵀ
  // μ         -Eᵀ       0   0
  // L⋅M⁻¹⋅Nᵀ  L⋅M⁻¹⋅Dᵀ  0   L⋅M⁻¹⋅Lᵀ
  // where D = |  F |
  //           | -F |
  const int num_vars = nc * 2 + nk + num_limits;
  MM->resize(num_vars, num_vars);
  Eigen::Ref<MatrixX<T>> N_iM_NT = MM->block(0, 0, nc, nc);
  Eigen::Ref<MatrixX<T>> N_iM_FT = MM->block(0, nc, nc, nr);
  Eigen::Ref<MatrixX<T>> N_iM_LT = MM->block(0, nc * 2 + nk, nc, nl);
  Eigen::Ref<MatrixX<T>> F_iM_FT = MM->block(nc, nc, nr, nr);
  Eigen::Ref<MatrixX<T>> F_iM_LT = MM->block(nc, nc + nk, nr, nl);
  Eigen::Ref<MatrixX<T>> L_iM_LT = MM->block(nc * 2 + nk, nc * 2 + nk, nl, nl);
  ComputeConstraintSpaceComplianceMatrix(N, nc, iM_NT, N_iM_NT);
  ComputeConstraintSpaceComplianceMatrix(N, nc, iM_FT, N_iM_FT);
  ComputeConstraintSpaceComplianceMatrix(N, nc, iM_LT, N_iM_LT);
  ComputeConstraintSpaceComplianceMatrix(F, nr, iM_FT, F_iM_FT);
  ComputeConstraintSpaceComplianceMatrix(F, nr, iM_LT, F_iM_LT);
  ComputeConstraintSpaceComplianceMatrix(L, nl, iM_LT, L_iM_LT);

  // Construct the LCP matrix. First do the "normal contact direction" rows:
  MM->block(0, nc + nr, nc, nr) = -MM->block(0, nc, nc, nr);
  MM->block(0, nc + nk, nc, nc).setZero();

  // Now construct the un-negated tangent contact direction rows.
  MM->block(nc, 0, nr, nc) = MM->block(0, nc, nc, nr).transpose().eval();
  MM->block(nc, nc + nr, nr, nr) = -MM->block(nc, nc, nr, nr);
  MM->block(nc, nc + nk, num_spanning_vectors, nc) = E;

  // Now construct the negated tangent contact direction rows. These negated
  // tangent contact directions allow the LCP to compute forces applied along
  // the negative x-axis. E will have to be reset to un-negate it.
  MM->block(nc + nr, 0, nr, nc + nk + nl) = -MM->block(nc, 0, nr, nc + nk + nl);
  MM->block(nc + nr, nc + nk, num_spanning_vectors, nc) = E;

  // Construct the next two row blocks, which provide the friction "cone"
  // constraint.
  MM->block(nc + nk, 0, nc, nc) = Eigen::DiagonalMatrix<T, Eigen::Dynamic>(mu);
  MM->block(nc + nk, nc, nc, num_spanning_vectors) = -E.transpose();
  MM->block(nc + nk, nc + num_spanning_vectors, nc, num_spanning_vectors) =
      -E.transpose();
  MM->block(nc + nk, nc + nk, nc, nc + nl).setZero();

  // Construct the last row block, which provides the generic unilateral
  // constraints.
  MM->block(nc*2 + nk, 0, nl, nc + nk) =
      MM->block(0, nc*2 + nk, nc + nk, nl).transpose().eval();

  // Construct the LCP vector:
  // N⋅v
  // D⋅v
  // 0
  // L⋅v
  // where, as above, D is defined as [F -F]
  qq->resize(num_vars, 1);
  qq->segment(0, nc) = N(problem_data.v);
  qq->segment(nc, nr) = F(problem_data.v);
  qq->segment(nc + nr, nc) = -qq->segment(nc, nr);
  qq->segment(nc + nk, nc).setZero();
  qq->segment(nc*2 + nk, num_limits) = L(problem_data.v) + problem_data.kL;
}

template <class T>
void ConstraintSolver<T>::ComputeGeneralizedForceFromConstraintForces(
    const ConstraintAccelProblemData<T>& problem_data,
    const VectorX<T>& cf,
    VectorX<T>* generalized_force) {
  if (!generalized_force)
    throw std::logic_error("generalized_force vector is null.");

  // Get numbers of types of contacts.
  const int num_sliding = problem_data.sliding_contacts.size();
  const int num_non_sliding = problem_data.non_sliding_contacts.size();
  const int num_contacts = num_sliding + num_non_sliding;
  const int num_spanning_vectors = std::accumulate(problem_data.r.begin(),
                                 problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();
  const int num_bilat_constraints = problem_data.kG.size();

  // Verify cf is the correct size.
  const int num_vars = num_contacts + num_spanning_vectors + num_limits +
      num_bilat_constraints;
  if (cf.size() != num_vars) {
    throw std::logic_error("cf (constraint force) parameter incorrectly"
                               "sized.");
  }

  /// Get the normal and non-sliding contact forces.
  const Eigen::Ref<const VectorX<T>> f_normal = cf.segment(0, num_contacts);
  const Eigen::Ref<const VectorX<T>> f_non_sliding_frictional = cf.segment(
      num_contacts, num_spanning_vectors);

  /// Get the limit forces.
  const Eigen::Ref<const VectorX<T>> f_limit = cf.segment(
      num_contacts + num_spanning_vectors, num_limits);

  // Get the bilateral constraint forces.
  const Eigen::Ref<const VectorX<T>> f_bilat = cf.segment(
      num_contacts + num_spanning_vectors + num_limits, num_bilat_constraints);

  /// Compute the generalized force.
  *generalized_force = problem_data.N_minus_muQ_transpose_mult(f_normal) +
                       problem_data.F_transpose_mult(f_non_sliding_frictional) +
                       problem_data.L_transpose_mult(f_limit) +
                       problem_data.G_transpose_mult(f_bilat);
}

template <class T>
void ConstraintSolver<T>::ComputeGeneralizedImpulseFromConstraintImpulses(
    const ConstraintVelProblemData<T>& problem_data,
    const VectorX<T>& cf,
    VectorX<T>* generalized_impulse) {
  if (!generalized_impulse)
    throw std::logic_error("generalized_impulse vector is null.");

  // Get number of contacts.
  const int num_contacts = problem_data.mu.size();
  const int num_spanning_vectors = std::accumulate(problem_data.r.begin(),
                                                   problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();
  const int num_bilat_constraints = problem_data.kG.size();

  // Verify cf is the correct size.
  const int num_vars = num_contacts + num_spanning_vectors + num_limits +
      num_bilat_constraints;
  if (num_vars != cf.size()) {
    throw std::logic_error("Unexpected packed constraint force vector"
                               " dimension.");
  }

  /// Get the normal and tangential contact impulses.
  const Eigen::Ref<const VectorX<T>> f_normal = cf.segment(0, num_contacts);
  const Eigen::Ref<const VectorX<T>> f_frictional = cf.segment(
      num_contacts, num_spanning_vectors);

  /// Get the limit forces.
  const Eigen::Ref<const VectorX<T>> f_limit = cf.segment(
      num_contacts + num_spanning_vectors, num_limits);

  // Get the bilateral constraint forces.
  const Eigen::Ref<const VectorX<T>> f_bilat = cf.segment(
      num_contacts + num_spanning_vectors + num_limits, num_bilat_constraints);

  /// Compute the generalized impules.
  *generalized_impulse = problem_data.N_transpose_mult(f_normal)  +
                         problem_data.F_transpose_mult(f_frictional) +
                         problem_data.L_transpose_mult(f_limit) +
                         problem_data.G_transpose_mult(f_bilat);
}

template <class T>
void ConstraintSolver<T>::ComputeGeneralizedAcceleration(
    const ConstraintAccelProblemData<T>& problem_data,
    const VectorX<T>& cf,
    VectorX<T>* generalized_acceleration) {
  if (!generalized_acceleration)
    throw std::logic_error("generalized_acceleration vector is null.");

  VectorX<T> generalized_force;
  ComputeGeneralizedForceFromConstraintForces(problem_data, cf,
                                              &generalized_force);
  *generalized_acceleration = problem_data.solve_inertia(problem_data.tau +
                                                         generalized_force);
}

template <class T>
void ConstraintSolver<T>::ComputeGeneralizedVelocityChange(
    const ConstraintVelProblemData<T>& problem_data,
    const VectorX<T>& cf,
    VectorX<T>* generalized_delta_v) {

  if (!generalized_delta_v)
    throw std::logic_error("generalized_delta_v vector is null.");

  VectorX<T> generalized_impulse;
  ComputeGeneralizedImpulseFromConstraintImpulses(problem_data, cf,
                                                  &generalized_impulse);
  *generalized_delta_v = problem_data.solve_inertia(generalized_impulse);
}

template <class T>
void ConstraintSolver<T>::CalcContactForcesInContactFrames(
    const VectorX<T>& cf,
    const ConstraintAccelProblemData<T>& problem_data,
    const std::vector<Matrix2<T>>& contact_frames,
    std::vector<Vector2<T>>* contact_forces) {
  using std::abs;

  // Loose tolerance for unit vectors and orthogonality.
  const double loose_eps = std::sqrt(std::numeric_limits<double>::epsilon());

  // Verify that contact_forces is non-null and is empty.
  if (!contact_forces)
    throw std::logic_error("Vector of contact forces is null.");
  if (!contact_forces->empty())
    throw std::logic_error("Vector of contact forces is not empty.");

  // Verify that cf is the correct size.
  const int num_non_sliding_contacts = problem_data.non_sliding_contacts.size();
  const int num_contacts = problem_data.sliding_contacts.size() +
      num_non_sliding_contacts;
  const int num_spanning_vectors = std::accumulate(problem_data.r.begin(),
                                                   problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();
  const int num_bilat_constraints = problem_data.kG.size();
  const int num_vars = num_contacts + num_spanning_vectors + num_limits +
      num_bilat_constraints;
  if (num_vars != cf.size()) {
    throw std::logic_error("Unexpected packed constraint force vector "
                               "dimension.");
  }

  // Verify that the problem is indeed two-dimensional.
  if (num_spanning_vectors != num_non_sliding_contacts) {
    throw std::logic_error("Problem data 'r' indicates contact problem is not "
                               "two-dimensional");
  }

  // Verify that the correct number of contact frames has been specified.
  if (contact_frames.size() != static_cast<size_t>(num_contacts)) {
    throw std::logic_error("Number of contact frames does not match number of "
                               "contacts.");
  }

  // Verify that sliding contact indices are sorted.
  DRAKE_ASSERT(std::is_sorted(problem_data.sliding_contacts.begin(),
                              problem_data.sliding_contacts.end()));

  // Resize the force vector.
  contact_forces->resize(contact_frames.size());

  // Set the forces.
  for (int i = 0, sliding_index = 0, non_sliding_index = 0; i < num_contacts;
       ++i) {
    // Alias the force.
    Vector2<T>& contact_force_i = (*contact_forces)[i];

    // Get the contact normal and tangent.
    const Vector2<T> contact_normal = contact_frames[i].col(0);
    const Vector2<T> contact_tangent = contact_frames[i].col(1);

    // Verify that each direction is of unit length.
    if (abs(contact_normal.norm() - 1) > loose_eps)
      throw std::runtime_error("Contact normal apparently not unit length.");
    if (abs(contact_tangent.norm() - 1) > loose_eps)
      throw std::runtime_error("Contact tangent apparently not unit length.");

    // Verify that the two directions are orthogonal.
    if (abs(contact_normal.dot(contact_tangent)) > loose_eps) {
      std::ostringstream oss;
      oss << "Contact normal (" << contact_normal.transpose() << ") and ";
      oss << "contact tangent (" << contact_tangent.transpose() << ") ";
      oss << "insufficiently orthogonal.";
      throw std::logic_error(oss.str());
    }

    // Initialize the contact force expressed in the global frame.
    Vector2<T> f0(0, 0);

    // Add in the contact normal.
    f0 += contact_normal * cf[i];

    // Determine whether the contact is sliding.
    const bool is_sliding = std::binary_search(
        problem_data.sliding_contacts.begin(),
        problem_data.sliding_contacts.end(), i);

    // Subtract/add the tangential force in the world frame.
    if (is_sliding) {
      f0 -= contact_tangent * cf[i] * problem_data.mu_sliding[sliding_index++];
    } else {
      f0 += contact_tangent * cf[num_contacts + non_sliding_index++];
    }

    // Compute the contact force in the contact frame.
    contact_force_i = contact_frames[i].transpose() * f0;
  }
}

template <class T>
void ConstraintSolver<T>::CalcImpactForcesInContactFrames(
    const VectorX<T>& cf,
    const ConstraintVelProblemData<T>& problem_data,
    const std::vector<Matrix2<T>>& contact_frames,
    std::vector<Vector2<T>>* contact_impulses) {
  using std::abs;

  // Loose tolerance for unit vectors and orthogonality.
  const double loose_eps = std::sqrt(std::numeric_limits<double>::epsilon());

  // Verify that contact_impulses is non-null and is empty.
  if (!contact_impulses)
    throw std::logic_error("Vector of contact impulses is null.");
  if (!contact_impulses->empty())
    throw std::logic_error("Vector of contact impulses is not empty.");

  // Verify that cf is the correct size.
  const int num_contacts = problem_data.mu.size();
  const int num_spanning_vectors = std::accumulate(problem_data.r.begin(),
                                                   problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();
  const int num_bilat_constraints = problem_data.kG.size();
  const int num_vars = num_contacts + num_spanning_vectors + num_limits +
      num_bilat_constraints;
  if (num_vars != cf.size()) {
    throw std::logic_error("Unexpected packed constraint force vector "
                               "dimension.");
  }

  // Verify that the problem is indeed two-dimensional.
  if (num_spanning_vectors != num_contacts) {
    throw std::logic_error("Problem data 'r' indicates contact problem is not "
                               "two-dimensional");
  }

  // Verify that the correct number of contact frames has been specified.
  if (contact_frames.size() != static_cast<size_t>(num_contacts)) {
    throw std::logic_error("Number of contact frames does not match number of "
                               "contacts.");
  }

  // Resize the impulse vector.
  contact_impulses->resize(contact_frames.size());

  // Set the impulses.
  for (int i = 0, tangent_index = 0; i < num_contacts; ++i) {
    // Alias the impulse.
    Vector2<T>& contact_impulse_i = (*contact_impulses)[i];

    // Get the contact normal and tangent.
    const Vector2<T> contact_normal = contact_frames[i].col(0);
    const Vector2<T> contact_tangent = contact_frames[i].col(1);

    // Verify that each direction is of unit length.
    if (abs(contact_normal.norm() - 1) > loose_eps)
      throw std::runtime_error("Contact normal apparently not unit length.");
    if (abs(contact_tangent.norm() - 1) > loose_eps)
      throw std::runtime_error("Contact tangent apparently not unit length.");

    // Verify that the two directions are orthogonal.
    if (abs(contact_normal.dot(contact_tangent)) > loose_eps) {
      std::ostringstream oss;
      oss << "Contact normal (" << contact_normal.transpose() << ") and ";
      oss << "contact tangent (" << contact_tangent.transpose() << ") ";
      oss << "insufficiently orthogonal.";
      throw std::logic_error(oss.str());
    }

    // Compute the contact impulse expressed in the global frame.
    Vector2<T> j0 = contact_normal * cf[i] + contact_tangent *
        cf[num_contacts + tangent_index++];

    // Compute the contact impulse in the contact frame.
    contact_impulse_i = contact_frames[i].transpose() * j0;
  }
}

}  // namespace constraint
}  // namespace multibody
}  // namespace drake
