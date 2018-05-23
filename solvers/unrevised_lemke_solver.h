#pragma once

#include <algorithm>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/SparseCore>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

/// Non-template class for UnrevisedLemkeSolver<T> constants.
class UnrevisedLemkeSolverId {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnrevisedLemkeSolverId);
  UnrevisedLemkeSolverId() = delete;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();
};

/// A class for the Unrevised Implementation of Lemke Algorithm's for solving
/// Linear Complementarity Problems (LCPs). See MobyLcpSolver for a description
/// of LCPs. This code makes extensive use of the following document:
/// [Dai 2018]  Dai, H. and Drumwright, E. Computing the Principal Pivoting
///     Transform for Solving Linear Complementarity Problems with Lemke's
///     Algorithm. (2018, located in doc/pivot_column.pdf).
template <class T>
class UnrevisedLemkeSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnrevisedLemkeSolver)

  UnrevisedLemkeSolver() = default;
  ~UnrevisedLemkeSolver() override = default;

  /// Structure for collecting LCP solver statistics.
  struct SolverStatistics {
    /// The zero tolerance used by the solver, specified by the user or computed
    /// (if the user did not specify a value). Default value indicates the
    /// tolerance is not set.
    T zero_tol{-1.0};

    /// The number of pivots used in the solver.
    int num_pivots{-1};

    /// The number of candidate solutions rejected.
    int num_rejected_solutions{-1};

    /// The number of failed linear solve operations.
    int num_failed_linear_solves{-1};

    /// Whether degeneracy was detected.
    bool degeneracy_detected{false};

    /// Whether warmstarting was successful.
    bool warmstarting_successful{false};
  };

  /// Calculates the zero tolerance that the solver would compute if the user
  /// does not specify a tolerance.
  template <class U>
  static U ComputeZeroTolerance(const MatrixX<U>& M, const VectorX<T>& q) {
    using std::max;
    return M.rows() * max(M.template lpNorm<Eigen::Infinity>(),
                          q.template lpNorm<Eigen::Infinity>()) *
        (10 * std::numeric_limits<double>::epsilon());
  }

  /// Checks whether a given candidate solution to the LCP Mz + q = w, z ≥ 0,
  /// w ≥ 0, zᵀw = 0 is satisfied to a given tolerance. If the tolerance is
  /// non-positive, this method computes a reasonable tolerance using M.
  static bool IsSolution(
      const MatrixX<T>& M, const VectorX<T>& q, const VectorX<T>& z,
      T zero_tol = -1);

  /// Lemke's Algorithm for solving LCPs in the matrix class E, which contains
  /// all strictly semimonotone matrices, all P-matrices, and all strictly
  /// copositive matrices. The solver can be applied with occasional success to
  /// problems outside of its guaranteed matrix classes. Lemke's Algorithm is
  /// described in [Cottle 1992], Section 4.4.
  ///
  /// The solver will denote failure on return if it exceeds a problem-size
  /// dependent number of iterations.
  /// @param[in] M the LCP matrix.
  /// @param[in] q the LCP vector.
  /// @param[in,out] z the solution to the LCP on return (if the solver
  ///                succeeds). If the length of z is equal to the length of q,
  ///                the solver will attempt to use the basis from the last
  ///                solution. This strategy can prove exceptionally
  ///                fast if solutions differ little between successive calls.
  ///                If the solver fails (returns `false`),
  ///                `z` will be set to the zero vector on return.
  /// @param[out] statistics the solver statistics, on return.
  /// @param[in] zero_tol The tolerance for testing against zero. If the
  ///            tolerance is negative (default) the solver will determine a
  ///            generally reasonable tolerance.
  /// @returns `true` if the solver computes a solution to floating point
  ///           tolerances (i.e., if IsSolution() returns `true` on the problem)
  ///           and `false` otherwise.
  /// @throws std::logic_error if M is not square or the dimensions of M do not
  ///         match the length of q.
  ///
  /// * [Cottle 1992]      R. Cottle, J.-S. Pang, and R. Stone. The Linear
  ///                      Complementarity Problem. Academic Press, 1992.
  bool SolveLcpLemke(const MatrixX<T>& M, const VectorX<T>& q,
                     VectorX<T>* z, SolverStatistics* statistics,
                     const T& zero_tol = T(-1)) const;

  bool available() const override { return true; }

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverId solver_id() const override;

 private:
  friend class UnrevisedLemkePrivateTests;
  friend class UnrevisedLemkePrivateTests_SelectSubMatrixWithCovering_Test;
  friend class UnrevisedLemkePrivateTests_SelectSubColumnWithCovering_Test;
  friend class UnrevisedLemkePrivateTests_SelectSubVector_Test;
  friend class UnrevisedLemkePrivateTests_SetSubVector_Test;
  friend class UnrevisedLemkePrivateTests_ValidateIndices_Test;
  friend class UnrevisedLemkePrivateTests_IsEachUnique_Test;
  friend class UnrevisedLemkePrivateTests_LemkePivot_Test;
  friend class UnrevisedLemkePrivateTests_ConstructLemkeSolution_Test;
  friend class UnrevisedLemkePrivateTests_DetermineIndexSets_Test;
  friend class UnrevisedLemkePrivateTests_FindBlockingIndex_Test;
  friend class UnrevisedLemkePrivateTests_FindBlockingIndexCycling_Test;
  friend class UnrevisedLemkePrivateTests_FindComplementIndex_Test;

  struct LemkeIndexSets {
    std::vector<int> alpha, alpha_prime;
    std::vector<int> alpha_bar, alpha_bar_prime;
    std::vector<int> beta, beta_prime;
    std::vector<int> beta_bar, beta_bar_prime;
  };

  // A structure for holding a linear complementarity problem variable.
  class LCPVariable {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LCPVariable)
    LCPVariable() {}
    LCPVariable(bool z, int index) : z_{z}, index_{index} {}

    bool is_z() const { return z_; }
    int index() const { return index_; }

    // Gets the complement of this variable.
    LCPVariable Complement() const {
      DRAKE_ASSERT(index_ >= 0);
      LCPVariable comp;
      comp.z_ = !z_;
      comp.index_ = index_;
      return comp;
    }

    // Compares two LCP variables for equality.
    bool operator==(const LCPVariable& v) const {
      DRAKE_ASSERT(index_ >= 0 && v.index_ >= 0);
      return (z_ == v.z_ && index_ == v.index_);
    }

    // Comparison operator for using LCPVariable as a key.
    bool operator<(const LCPVariable& v) const {
      DRAKE_ASSERT(index_ >= 0 && v.index_ >= 0);
      if (index_ < v.index_) {
        return true;
      } else {
        if (index_ > v.index_) {
          return false;
        } else {
          // If here, the indices are equal. We will arbitrarily order w before
          // z (alphabetical ordering).
          return (!z_ && v.z_);
        }
      }
    }

   private:
    bool z_{true};        // Is this a z variable or a w variable?
    int index_{-1};       // Index of the variable in the problem, 0...n. n
                          // indicates that the variable is artificial. -1
                          // indicates that the index is uninitialized.
  };

  static void SelectSubMatrixWithCovering(
      const MatrixX<T>& in,
      const std::vector<int>& rows,
      const std::vector<int>& cols, MatrixX<T>* out);
  static void SelectSubColumnWithCovering(const MatrixX<T>& in,
      const std::vector<int>& rows,
      int column, VectorX<T>* out);
  static void SelectSubVector(const VectorX<T>& in,
      const std::vector<int>& rows, VectorX<T>* out);
  static void SetSubVector(
      const VectorX<T>& v_sub, const std::vector<int>& indices, VectorX<T>* v);
  static bool ValidateIndices(
    const std::vector<int>& row_indices,
    const std::vector<int>& col_indices, int num_rows, int num_cols);
  static bool ValidateIndices(
    const std::vector<int>& row_indices, int vector_size);
  static bool IsEachUnique(const std::vector<LCPVariable>& vars);
  bool LemkePivot(const MatrixX<T>& M, const VectorX<T>& q, int driving_index,
                  T zero_tol, VectorX<T>* M_bar_col, VectorX<T>* q_bar) const;
  bool ConstructLemkeSolution(const MatrixX<T>& M, const VectorX<T>& q,
      int artificial_index, T zero_tol, VectorX<T>* z) const;
  int FindComplementIndex(const LCPVariable& query) const;
  void DetermineIndexSets() const;
  bool FindBlockingIndex(
      const T& zero_tol, const VectorX<T>& matrix_col, const VectorX<T>& ratios,
      int* blocking_index) const;
  bool IsArtificial(const LCPVariable& v) const;
  bool Restart() const;

  typedef std::vector<LCPVariable> LCPVariableVector;

  // The state of the solver, used for backtracking. The partitions of
  // independent and dependent variables (denoted z' and w', respectively, in
  // [Dai 2018].
  struct SolverState {
    LCPVariableVector indep_variables;
    LCPVariableVector dep_variables;
    int driving_index{-1};
  };

  // Structure for mapping a vector of independent variables to a selection
  // index.
  class SolverStateComparator {
   public:
    // This does a lexicographic comparison.
    bool operator()(
        const SolverState& v1, const SolverState& v2) const {
      DRAKE_DEMAND(v1.indep_variables.size() == v2.indep_variables.size());

      // Now do the lexicographic comparison.
      for (int i = 0; i < static_cast<int>(v1.indep_variables.size()); ++i) {
        if (v1.indep_variables[i] < v2.indep_variables[i]) {
          return true;
        } else {
          if (v1.indep_variables[i] < v2.indep_variables[i])
            return false;
        }
      }

      // If still here, they're equal.
      return false;
    }
  };

  // Note: the mutable variables below are used in place of local variables both
  // to minimize heap allocations during the LCP solution process and to
  // facilitate warmstarting.

  // Temporary variable for determining index sets (i.e., α, α', α̅, α̅', etc.
  // from [Dai 2018]). The first int of each pair stores the
  // variable's own "internal" index and the second stores the index of the
  // variable in the requisite array ("independent w", "dependent w",
  // "independent z", and "dependent z") in [Dai 2018].
  mutable std::vector<std::pair<int, int>> variable_and_array_indices_;

  // Mapping from an LCP variable to the index of that variable in
  // indep_variables.
  mutable std::map<LCPVariable, int> indep_variables_indices_;

  // Maps tuples of independent variables to the variable selected for pivoting
  // when multiple pivoting choices are possible. If the LCP algorithm pivots
  // such that a tuple of independent variables is detected that has been seen
  // before, we would call this "cycling". We eliminate cycling by never
  // selecting the same variable for pivoting twice *from a given pivot*.
  mutable std::map<SolverState, int, SolverStateComparator>
      selections_;

  // These temporary matrices and vectors are members to facilitate minimizing
  // memory allocations/deallocations. Changing their value between invocations
  // of the LCP solver will not change the resulting computation.
  mutable MatrixX<T> M_alpha_beta_, M_alpha_bar_beta_;
  mutable VectorX<T> q_alpha_, q_alpha_bar_, q_prime_beta_prime_,
      q_prime_alpha_bar_prime_, e_, M_prime_driving_beta_prime_,
      M_prime_driving_alpha_bar_prime_, g_alpha_, g_alpha_bar_;

  // The index sets for the Lemke Algorithm and is a member variable to
  // permit warmstarting. Changing the index set between invocations of the LCP
  // solver will not change the resulting computation.
  mutable LemkeIndexSets index_sets_;

  // These have been made member variables to permit warmstarting. Changing
  // these state between invocations of the LCP solver will not change the
  // resulting computation.
  mutable SolverState state_;
};

}  // end namespace solvers
}  // end namespace drake

