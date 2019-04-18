#pragma once

#include <limits>
#include <memory>
#include <utility>

#include <Eigen/LU>

#include "drake/common/drake_copyable.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/analysis/implicit_integrator.h"

namespace drake {
namespace systems {

/**
 * A third-order, fully implicit integrator with second order error estimation.
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 *
 * This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
 * this class, please refer to https://drake.mit.edu/cxx_inl.html.
 *
 * Instantiated templates for the following kinds of T's are provided:
 *
 * - double
 * - AutoDiffXd
 *
 * A two-stage Radau IIa (see [Hairer, 1996], Ch. 5) method is used for
 * propagating the state forward, while the implicit trapezoid rule is used
 * for estimating the local (truncation) error at every time.
 *
 * The Radau3 method is known to be L-Stable, meaning both that
 * applying it at a fixed integration step to the  "test" equation `y(t) = eᵏᵗ`
 * yields zero (for `k < 0` and `t → ∞`) *and* that it is also A-Stable.
 * A-Stability, in turn, means that the method can integrate the linear constant
 * coefficient system `dx/dt = Ax` at any step size without the solution
 * becoming unstable (growing without bound). The practical effect of
 * L-Stability is that the integrator tends to be stable for any given step size
 * on an arbitrary system of ordinary differential equations. See
 * [Lambert, 1991], Ch. 6 for an approachable discussion on stiff differential
 * equations and L- and A-Stability.
 *
 * This implementation uses Newton-Raphson (NR). General implementational
 * details were taken from [Hairer, 1996] Ch. 8.
 *
 * See ImplicitIntegrator class documentation for further details.
 *
 * - [Hairer, 1996]   E. Hairer and G. Wanner. Solving Ordinary Differential
 *                    Equations II (Stiff and Differential-Algebraic Problems).
 *                    Springer, 1996.
 * - [Lambert, 1991]  J. D. Lambert. Numerical Methods for Ordinary Differential
 *                    Equations. John Wiley & Sons, 1991.
 */
template <class T>
class Radau3Integrator final : public ImplicitIntegrator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Radau3Integrator)

  explicit Radau3Integrator(const System<T>& system,
      Context<T>* context = nullptr);
  ~Radau3Integrator() override = default;

  /// The integrator supports error estimation.
  bool supports_error_estimation() const override { return true; }

  /// This integrator provides second order error estimates.
  int get_error_estimate_order() const override { return 2; }

  /// @name Cumulative statistics functions.
  /// The functions return statistics specific to the Radau3 integration
  /// process. Builds on top of the cumulative statistics functions from
  /// ImplicitIntegrator.
  /// @{

  /// Gets the number of iterations used in the Newton-Raphson nonlinear systems
  /// of equation solving process since the last call to ResetStatistics(). This
  /// count includes those Newton-Raphson iterations used during error
  /// estimation processes.
  int64_t get_num_newton_raphson_iterations() const {
    return num_nr_iterations_;
  }

  /// Gets the number of factorizations of the iteration matrix since the last
  /// call to ResetStatistics(). This count includes those refactorizations
  /// necessary during error estimation processes.
  int64_t get_num_iteration_matrix_factorizations() const {
    return num_iter_factorizations_;
  }

  /// @}

  /// @name Error-estimation statistics functions.
  /// The functions return statistics specific to the error estimation
  /// process.
  /// @{
  /// Gets the number of ODE function evaluations (calls to
  /// CalcTimeDerivatives()) *used only for computing the Jacobian matrices
  /// needed by the error estimation process* since the last call to
  /// ResetStatistics().
  int64_t get_num_error_estimator_derivative_evaluations_for_jacobian() const {
    return num_err_est_jacobian_function_evaluations_;
  }

  /// Gets the number of ODE function evaluations
  /// (calls to CalcTimeDerivatives()) *used only for the error estimation
  /// process* since the last call to ResetStatistics(). This count
  /// includes *all* such calls including (1) those necessary to compute
  /// Jacobian matrices; and (2) calls that exhibit little
  /// cost (due to results being cached).
  int64_t get_num_error_estimator_derivative_evaluations() const {
    return num_err_est_function_evaluations_;
  }

  /// Gets the number of iterations *used in the Newton-Raphson nonlinear
  /// systems of equation solving process for the error estimation process*
  /// since the last call to ResetStatistics().
  int64_t get_num_error_estimator_newton_raphson_iterations() const { return
        num_err_est_nr_iterations_;
  }

  /// Gets the number of Jacobian matrix evaluations *used only during
  /// the error estimation process* since the last call to ResetStatistics().
  int64_t get_num_error_estimator_jacobian_evaluations() const {
    return num_err_est_jacobian_reforms_;
  }

  /// Gets the number of factorizations of the iteration matrix *used only
  /// during the error estimation process* since the last call to
  /// ResetStatistics().
  int64_t get_num_error_estimator_iteration_matrix_factorizations() const {
    return num_err_est_iter_factorizations_;
  }

  /// @}

 private:
  int64_t do_get_num_newton_raphson_iterations() const final {
    return num_nr_iterations_;
  }

  int64_t do_get_num_iteration_matrix_factorizations() const final {
    return num_iter_factorizations_;
  }

  int64_t do_get_num_error_estimator_derivative_evaluations() const final {
    return num_err_est_function_evaluations_;
  }

  int64_t do_get_num_error_estimator_derivative_evaluations_for_jacobian()
      const final {
    return num_err_est_jacobian_function_evaluations_;
  }

  int64_t do_get_num_error_estimator_newton_raphson_iterations()
      const final {
    return num_err_est_nr_iterations_;
  }

  int64_t do_get_num_error_estimator_jacobian_evaluations() const final {
    return num_err_est_jacobian_reforms_;
  }

  int64_t do_get_num_error_estimator_iteration_matrix_factorizations()
      const final {
    return num_err_est_iter_factorizations_;
  }

  bool AttemptStepPaired(const T& t0, const T& dt,
      const VectorX<T>& xt0, VectorX<T>* xtplus_radau3, VectorX<T>* xtplus_itr);
  const VectorX<T>& ComputeFofg(
      const T& t0, const T& dt, const VectorX<T>& xt0, const VectorX<T>& Z);
  void DoInitialize() override;
  void DoResetStatistics() override;
  bool DoStep(const T& dt) override;
  bool StepRadau3(const T& t0, const T& dt, const VectorX<T>& xt0,
      VectorX<T>* xtplus, int trial = 1);
  bool StepImplicitTrapezoid(const T& t0, const T& h, const VectorX<T>& xt0,
      const VectorX<T>& dx0, VectorX<T>* xtplus);
  static MatrixX<T> CalcTensorProduct(const MatrixX<T>& A, const MatrixX<T>& B);
  static void ComputeImplicitTrapezoidIterationMatrix(const MatrixX<T>& J,
      const T& dt,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);
  static void ComputeRadau3IterationMatrix(const MatrixX<T>& J, const T& dt,
      const MatrixX<double>& A,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix);
  bool CalcMatrices(const T& t, const VectorX<T>& xt, const T& dt,
      const std::function<void(const MatrixX<T>&, const T&,
          typename ImplicitIntegrator<T>::IterationMatrix*)>&
      recompute_iteration_matrix,
      typename ImplicitIntegrator<T>::IterationMatrix* iteration_matrix,
      int trial);
  bool StepImplicitTrapezoidDetail(const T& t0, const T& dt,
      const VectorX<T>& xt0, const std::function<VectorX<T>()>& g,
      VectorX<T>* xtplus, int trial = 1);

  // The number of stages used in this integrator. Set this to 1 for the
  // integrator to be implicit Euler and 2 for it to be Radau3.
  static constexpr int kNumStages = 1;

  // The time-scaling coefficients for this RK-type integrator.
  std::vector<double> c_;

  // The stage-scaling coefficients for this RK-type integrator.
  MatrixX<double> A_;

  // The iteration matrix and factorizations for the Implicit Trapezoid method.
  typename ImplicitIntegrator<T>::IterationMatrix
      iteration_matrix_implicit_trapezoid_;

  // The iteration matrix for the Radau3 method.
  typename ImplicitIntegrator<T>::IterationMatrix iteration_matrix_radau3_;

  // The tensor product between A and an identity matrix, which is computed
  // on initialization and then stored.
  MatrixX<T> A_tp_eye_;

  // The solution propagation coefficients (that also scales the stages).
  std::vector<double> b_;

  // A kNumStages * |xc|-dimensional vector of the current iterate for the
  // Newton-Raphson process.
  VectorX<T> Z_;

  // The derivative evaluations at t0 and every stage.
  VectorX<T> F_of_g_;

  // Vector used in error estimate calculations.
  VectorX<T> err_est_vec_;

  // The continuous state update vector used during Newton-Raphson.
  std::unique_ptr<ContinuousState<T>> dx_state_;

  // Various statistics.
  int64_t num_nr_iterations_{0};
  int64_t num_iter_factorizations_{0};

  // Implicit trapezoid specific statistics.
  int64_t num_err_est_jacobian_reforms_{0};
  int64_t num_err_est_iter_factorizations_{0};
  int64_t num_err_est_function_evaluations_{0};
  int64_t num_err_est_jacobian_function_evaluations_{0};
  int64_t num_err_est_nr_iterations_{0};
};
}  // namespace systems
}  // namespace drake
