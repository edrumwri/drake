#pragma once

/// @file
/// Template method implementations for radau3_integrator.h.
/// Most users should only include that file, not this one.
/// For background, see https://drake.mit.edu/cxx_inl.html.

/* clang-format off to disable clang-format-includes */
#include "drake/systems/analysis/radau3_integrator.h"
/* clang-format on */

#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include "drake/common/text_logging.h"
#include "drake/math/autodiff.h"

namespace drake {
namespace systems {

template <class T>
Radau3Integrator<T>::Radau3Integrator(const System<T>& system,
    Context<T>* context) : ImplicitIntegrator<T>(system, context) {
  // Set the time coefficients (from [Hairer, 1996] Table 5.5).
  c_ = { 1.0/3, 1.0 };

  // Set the matrix coefficients (from the same table).
  A_.resize(kNumStages, kNumStages);
  A_(0, 0) = 5.0/12;     A_(0, 1) = -1.0/12;
  A_(1, 0) = 3.0/4;      A_(1, 1) = 1.0/4;

  // Set the propagation constants (again, from the same table).
  b_ = { 3.0/4, 1.0/4 };
}

template <class T>
void Radau3Integrator<T>::DoResetStatistics() {
  ImplicitIntegrator<T>::DoResetStatistics();
  num_err_est_nr_iterations_ = 0;
  num_err_est_function_evaluations_ = 0;
  num_err_est_jacobian_function_evaluations_ = 0;
  num_err_est_jacobian_reforms_ = 0;
  num_err_est_iter_factorizations_ = 0;
}

template <class T>
void Radau3Integrator<T>::DoInitialize() {
  using std::isnan;

  // Compute the tensor product of A with the identity matrix. A is a
  // kNumStages x kNumStages matrix. We need the tensor product to be a
  // m x m-dimensional matrix, where m = kNumStages * state_dim. Thus the
  // number of rows/columns of the identity matrix is state_dim.

  const int state_dim =
      this->get_context().get_continuous_state_vector().size();
  A_tp_eye_ = CalcTensorProduct(A_, MatrixX<T>::Identity(state_dim, state_dim));

  // Allocate storage for changes to state variables during Newton-Raphson.
  dZ_state_ = this->get_system().AllocateTimeDerivatives();

  const double kDefaultAccuracy = 1e-3;  // Good for this particular integrator.
  const double kLoosestAccuracy = 1e-2;

  // Set an artificial step size target, if not set already.
  if (isnan(this->get_initial_step_size_target())) {
    // Verify that maximum step size has been set.
    if (isnan(this->get_maximum_step_size()))
      throw std::logic_error("Neither initial step size target nor maximum "
                                 "step size has been set!");

    this->request_initial_step_size_target(
        this->get_maximum_step_size());
  }

  // Sets the working accuracy to a good value.
  double working_accuracy = this->get_target_accuracy();

  // If the user asks for accuracy that is looser than the loosest this
  // integrator can provide, use the integrator's loosest accuracy setting
  // instead.
  if (isnan(working_accuracy))
    working_accuracy = kDefaultAccuracy;
  else if (working_accuracy > kLoosestAccuracy)
    working_accuracy = kLoosestAccuracy;
  this->set_accuracy_in_use(working_accuracy);

  // Reset the Jacobian matrix (so that recomputation is forced).
  J_.resize(0, 0);
}

// Computes F(Z) used in [Hairer, 1996], (8.4).
template <class T>
const VectorX<T>& Radau3Integrator<T>::ComputeFofZ(const T& h) {
  Context<T>* context = this->get_mutable_context();
  const T t0 = context->get_time();
  const int num_states = context->get_continuous_state_vector().size();
  FofZ_.resize(num_states * kNumStages);

  // Evaluate the derivative at each stage.
  const VectorX<T> xt0 = context->get_continuous_state_vector().CopyToVector();
  for (int i = 0, j = 0; i < kNumStages; ++i, j += num_states) {
    const auto Z_i = Z_.segment(j, num_states);
    context->SetTimeAndContinuousState(t0 + c_[i] * h, xt0 + Z_i);
    auto F_i = FofZ_.segment(j, num_states);
    F_i = this->EvalTimeDerivativesUsingContext();
  }

  return FofZ_;
}

// Performs the bulk of the stepping computation for both implicit Euler and
// implicit trapezoid method; all those methods need to do is provide a
// residual function (@p g) and a scale factor (@p scale) specific to the
// particular integrator scheme and this method does the rest.
// @param dt the integration step size to attempt.
// @param g the particular implicit function to zero.
// @param scale a scale factor- either 1 or 2- that allows this method to be
//        used by both implicit Euler and implicit trapezoid methods.
// @param [in,out] the starting guess for x(t+dt); the value for x(t+h) on
//        return (assuming that h > 0)
// @param trial the attempt for this approach (1-4). StepAbstract() uses more
//        computationally expensive methods as the trial numbers increase.
// @returns `true` if the method was successfully able to take an integration
//           step of size @p dt (or `false` otherwise).
// @pre The time and state of the system's context (stored by the integrator)
//      are t0 and x(t0) on entry.
// @post The time and state of the system's context (stored by the integrator)
//       will be set to t0+dt and x(t0+dt) on successful exit (indicated by
//       this function returning `true`) and will be indeterminate on
//       unsuccessful exit (indicated by this function returning `false`).
template <class T>
bool Radau3Integrator<T>::StepRadau3(const T& dt,
    VectorX<T>* xtplus, int trial) {
  using std::max;
  using std::min;

  // Verify the trial number is valid.
  DRAKE_ASSERT(trial >= 1 && trial <= 4);

  // Verify xtplus
  Context<T>* context = this->get_mutable_context();
  DRAKE_ASSERT(xtplus &&
               xtplus->size() == context->get_continuous_state_vector().size());

  SPDLOG_DEBUG(drake::log(), "StepRadau3() entered for t={}, h={}, trial={}",
               context->get_time(), dt, trial);


  // TODO(edrumwri) No longer do this.
  // Advance the context time; this means that all derivatives will be computed
  // at t+dt.
  const T tf = context->get_time() + dt;
  context->SetTimeAndContinuousState(tf, *xtplus);

  // Initialize the z iterate using (8.5) in [Hairer, 1996], p. 120.
  Z_.setZero(xtplus->size());  

  // Initialize the "last" norm of dZ; this will be used to detect convergence.
  T last_dZ_norm = std::numeric_limits<double>::infinity();

  // Set the iteration matrix construction method.
  auto construct_iteration_matrix = [this](const MatrixX<T>& J, const T& h,
      MatrixX<T>* iteration_matrix) {
    ComputeRadau3IterationMatrix(J, h, this->A_, iteration_matrix);
  };

  // Calculate Jacobian and iteration matrices (and factorizations), as needed.
  if (!CalcMatrices(tf, dt, &iteration_matrix_radau3_,
      construct_iteration_matrix, *xtplus, trial)) {
    this->set_last_call_succeeded(false);
    return false;
  }

  // The maximum number of Newton-Raphson iterations to take before declaring
  // failure. [Hairer, 1996] states, "It is our experience that the code becomes
  // more efficient when we allow a relatively high number of iterations (e.g.,
  // [7 or 10])", p. 121.
  // TODO(edrumwri): Consider making this a settable parameter. Not doing so
  //                 to avoid parameter overload.
  const int max_iterations = 10;

  // Do the Newton-Raphson iterations.
  for (int i = 0; i < max_iterations; ++i) {
    // Update the number of Newton-Raphson iterations.
    this->increment_nr_iterations();

    // Evaluate the derivatives using the current iterate.
    const VectorX<T>& FofZ = ComputeFofZ(dt);

    // TODO(edrumwri): Solve(.) must be updated to use the proper
    // factorization (implicit trapezoid, Radau3).
    // Compute the state update using (8.4) in [Hairer, 1996], p. 119.
    VectorX<T> dZ = this->Solve(iteration_matrix_radau3_,
        A_tp_eye_ * (dt * FofZ) - Z_);

    // TODO(edrumwri): Get the infinity norm of the update vector using
    // (3.1b) from [Hairer, 1996].
    dZ_state_->get_mutable_vector().SetFromVector(dZ);
    T dZ_norm = this->CalcStateChangeNorm(*dZ_state_);

    // Update the iterate.
    Z_ += dZ;

    // The check below looks for convergence using machine epsilon. Without
    // this check, the convergence criteria can be applied when
    // |dZ_norm| ~ 1e-22 (one example taken from practice), which does not
    // allow the norm to be reduced further. What happens: dx_norm will become
    // equivalent to last_dZ_norm, making theta = 1, and eta = infinity. Thus,
    // convergence would never be identified.
    if (dZ_norm < 10 * std::numeric_limits<double>::epsilon()) {
      // Compute the solution using (3.1b) from [Hairer, 1996].

      context->SetContinuousState(*xtplus);
      this->set_last_call_succeeded(true);
      return true;
    }

    // Compute the convergence rate and check convergence.
    // [Hairer, 1996] notes that this convergence strategy should only be
    // applied after *at least* two iterations (p. 121).
    if (i >= 1) {
      const T theta = dZ_norm / last_dZ_norm;
      const T eta = theta / (1 - theta);
      SPDLOG_DEBUG(drake::log(), "Newton-Raphson loop {} theta: {}, eta: {}",
                   i, theta, eta);

      // Look for divergence.
      if (theta > 1) {
        SPDLOG_DEBUG(drake::log(), "Newton-Raphson divergence detected for "
            "h={}", dt);
        break;
      }

      // Look for convergence using Equation 8.10 from [Hairer, 1996].
      // [Hairer, 1996] determined values of kappa in [0.01, 0.1] work most
      // efficiently on a number of test problems with *RADAU5* (a fifth order
      // implicit integrator), p. 121. We select a value halfway in-between.
      const double kappa = 0.05;
      const double k_dot_tol = kappa * this->get_accuracy_in_use();
      if (eta * dZ_norm < k_dot_tol) {
        SPDLOG_DEBUG(drake::log(), "Newton-Raphson converged; η = {}, h = {}",
                     eta, dt);
        // Compute the solution using (3.1b) from [Hairer, 1996].
        context->SetContinuousState(*xtplus);
        this->set_last_call_succeeded(true);
        return true;
      }
    }

    // Update the norm of the state update.
    last_dZ_norm = dZ_norm;
  }

  SPDLOG_DEBUG(drake::log(), "StepRadau3() convergence failed");

  // If Jacobian and iteration matrix factorizations are not reused, there
  // is nothing else we can try.
  if (!this->get_reuse()) {
    this->set_last_call_succeeded(false);
    return false;
  }

  // Try StepRadau3 again, freshening Jacobians and iteration matrix
  // factorizations as helpful.
  return StepRadau3(dt, xtplus, trial+1);
}

// Function for computing the iteration matrix for the Implicit Trapezoid
// method.
template <class T>
void Radau3Integrator<T>::ComputeImplicitTrapezoidIterationMatrix(
    const MatrixX<T>& J,
    const T& dt,
    MatrixX<T>* iteration_matrix) {
  const int n = J.rows();
  *iteration_matrix = J * (-dt / 2.0) + MatrixX<T>::Identity(n, n);
}

// Function for computing the iteration matrix for the Radau3 method. This
// is the matrix in [Hairer, 1996] (8.4) on p.119.
template <class T>
void Radau3Integrator<T>::ComputeRadau3IterationMatrix(
    const MatrixX<T>& J,
    const T& dt,
    const MatrixX<T>& A,
    MatrixX<T>* iteration_matrix) {
  const int n = J.rows() * kNumStages;
  *iteration_matrix = CalcTensorProduct(A * dt, J) -
      MatrixX<T>::Identity(n , n);
}

// Computes the tensor product between two matrices. Given 
// A = | a11 ... a1m |
//     | ...     ... |
//     | an1 ... anm |
// and some matrix B, the tensor product is is:
// A x B = | a11B ... a1mB |
//         | ...      ...  |
//         | an1B ... anmB |
template <class T>
MatrixX<T> Radau3Integrator<T>::CalcTensorProduct(
    const MatrixX<double>& A, const MatrixX<T>& B) {
  const int rows_A = A.rows();
  const int cols_A = A.cols();
  const int rows_B = B.rows();
  const int cols_B = B.cols();
  MatrixX<T> AB(rows_A * rows_B, cols_A * cols_B);
  for (int i = 0, ii = 0; i < rows_A; ++i, ii += rows_A) {
    for (int j = 0, jj = 0; j < cols_A; ++j, jj += cols_A) {
      AB.block(ii, jj, rows_B, cols_B) = A(i, j) * B;
    }
  }

  return AB;
}

// Computes necessary matrices for the Newton-Raphson iteration.
// TODO(edrumwri) Remove references to StepAbstract(). 
// Parameters are identical to those for StepAbstract;
// @see StepAbstract() for their documentation.
// @returns `false` if the calling StepAbstract method should indicate failure;
//          `true` otherwise.
template <class T>
bool Radau3Integrator<T>::CalcMatrices(
    const T& tf, const T& dt,
    MatrixX<T>* iteration_matrix,
    const std::function<void(const MatrixX<T>&, const T&, MatrixX<T>*)>&
        recompute_iteration_matrix,
    const VectorX<T>& xtplus, int trial) {
  // Compute the initial Jacobian and negated iteration matrices (see
  // rationale for the negation below) and factor them, if necessary.
  MatrixX<T>& J = this->get_mutable_jacobian();
  if (!this->get_reuse() || J.rows() == 0 || this->IsBadJacobian(J)) {
    // Note that the Jacobian can become bad through a divergent Newton-Raphson
    // iteration, which causes the state to overflow, which then causes the
    // Jacobian to overflow. If the state overflows, recomputing the Jacobian
    // using this bad state will result in another bad Jacobian, eventually
    // causing DoStep() to return indicating failure (but not before resetting
    // the continuous state to its previous, good value). DoStep() will then
    // be called again with a smaller step size and the good state; the
    // bad Jacobian will then be corrected.
    J = this->CalcJacobian(tf, xtplus);
    recompute_iteration_matrix(J, dt, iteration_matrix);
    this->Factor(iteration_matrix);
    return true;
  }

  switch (trial) {
    case 1:
      // For the first trial, we do nothing special.
      return true;

    case 2: {
      // For the second trial, re-construct and factor the iteration matrix.
      recompute_iteration_matrix(J, dt, iteration_matrix);
      this->Factor(iteration_matrix);
      return true;
    }

    case 3: {
      // If the last call to StepAbstract() ended in failure, we know that
      // the Jacobian matrix is fresh and the iteration matrix has been newly
      // formed and factored (on Trial #2), so there is nothing more to be
      // done.
      if (!this->last_call_succeeded()) {
        return false;
      } else {
        // Reform the Jacobian matrix and refactor the negation of
        // the iteration matrix. The idea of using the negation of this matrix
        // is that an O(n^2) subtraction is not necessary as would
        // be the case with MatrixX<T>::Identity(n, n) - J * (dt / scale).
        J = this->CalcJacobian(tf, xtplus);
        recompute_iteration_matrix(J, dt, iteration_matrix);
        Factor(iteration_matrix);
      }
      return true;

      case 4: {
        // Trial #4 indicates failure.
        return false;
      }

      default:
        throw std::domain_error("Unexpected trial number.");
    }
  }
}

// Steps forward by a single step of @p dt using the implicit trapezoid
// method, if possible.
// @param dt the maximum time increment to step forward.
// @param dx0 the time derivatives computed at the beginning of the integration
//        step.
// @param xtplus the state computed by the implicit Euler method.
// @returns `true` if the step was successful and `false` otherwise.
// @pre The time and state in the system's context (stored within the
//      integrator) are set to those at t0 (the beginning of the integration
//      step).
// @post The time and state in the system's context (stored within the
//       integrator) are set to those at t0+dt on successful return (i.e., when
//       the function returns `true`). State will be indeterminate on `false`
//       return.
/*
template <class T>
bool Radau3Integrator<T>::StepImplicitTrapezoid(const T& h,
                                                       const VectorX<T>& dx0,
                                                       VectorX<T>* xtplus) {
  using std::abs;

  // Get the current continuous state.
  Context<T>* context = this->get_mutable_context();
  const VectorX<T> xt0 = context->get_continuous_state_vector().CopyToVector();

  SPDLOG_DEBUG(drake::log(), "StepImplicitTrapezoid(h={}) t={}",
               h, context->get_time());

  // Set g for the implicit trapezoid method.
  // Define g(x(t+h)) ≡ x(t+h) - x(t) - h/2 (f(t,x(t)) + f(t+h,x(t+h)) and
  // evaluate it at the current x(t+h).
  std::function<VectorX<T>()> g =
      [&xt0, h, &dx0, context, this]() {
        return (context->get_continuous_state().CopyToVector() - xt0 -
            h/2 * (dx0 + this->EvalTimeDerivativesUsingContext().eval())).eval();
      };

  // Store statistics before calling StepAbstract(). The difference between
  // the modified statistics and the stored statistics will be used to compute
  // the trapezoid method-specific statistics.
  int stored_num_jacobian_evaluations = this->get_num_jacobian_evaluations();
  int stored_num_iter_factorizations =
      this->get_num_iteration_matrix_factorizations();
  int64_t stored_num_function_evaluations =
      this->get_num_derivative_evaluations();
  int64_t stored_num_jacobian_function_evaluations =
      this->get_num_derivative_evaluations_for_jacobian();
  int stored_num_nr_iterations = this->get_num_newton_raphson_iterations();

  // Step.
  bool success = StepAbstract(h, g, 2, xtplus);

  // Move statistics to implicit trapezoid-specific.
  num_err_est_jacobian_reforms_ +=
      this->get_num_jacobian_evaluations() - stored_num_jacobian_evaluations;
  num_err_est_iter_factorizations_ += 
      this->get_num_iteration_matrix_factorizations() -
      stored_num_iter_factorizations;
  num_err_est_function_evaluations_ +=
      this->get_num_derivative_evaluations() - stored_num_function_evaluations;
  num_err_est_jacobian_function_evaluations_ +=
      this->get_num_derivative_evaluations_for_jacobian() -
      stored_num_jacobian_function_evaluations;
  num_err_est_nr_iterations_ += this->get_num_newton_raphson_iterations() -
      stored_num_nr_iterations;

  return success;
}

// Steps both implicit Euler and implicit trapezoid forward by dt, if possible.
// @param dt the integration step size to attempt.
// @param [out] xtplus_ie contains the Euler integrator solution on return
// @param [out] xtplus_itr contains the implicit trapezoid solution on return
// @returns `true` if the integration was successful at the requested step size
//          and `false` otherwise.
// @pre The time and state in the system's context (stored by the integrator)
//      are set to {t0,x0} on entry (those at the beginning of the interval.
// @post The time and state of the system's context (stored by the integrator)
//       will be set to t0+dt and @p xtplus_ie on successful exit (indicated by
//       this function returning `true`) and will be indeterminate on
//       unsuccessful exit (indicated by this function returning `false`).
template <class T>
bool Radau3Integrator<T>::AttemptStepPaired(const T& dt,
                                                   VectorX<T>* xtplus_ie,
                                                   VectorX<T>* xtplus_itr) {
  using std::abs;
  DRAKE_ASSERT(xtplus_ie);
  DRAKE_ASSERT(xtplus_itr);

  // Save the time and state at the beginning of this interval.
  Context<T>* context = this->get_mutable_context();
  T t0 = context->get_time();
  const VectorX<T> xt0 = context->get_continuous_state_vector().CopyToVector();

  // Compute the derivative at xt0. NOTE: the derivative is calculated at this
  // point (early on in the integration process) in order to reuse the
  // derivative evaluation, via the cache, from the last integration step (if
  // possible).
  const VectorX<T> dx0 = EvalTimeDerivativesUsingContext();

  // Do the Euler step.
  if (!StepRadau3(dt)) {
    SPDLOG_DEBUG(drake::log(), "Implicit Euler approach did not converge for "
        "step size {}", dt);
    return false;
  }

  // Get the new state.
  *xtplus_ie = context->get_continuous_state_vector().CopyToVector();

  // Reset the state.
  context->SetTimeAndContinuousState(t0, xt0);

  // The error estimation process uses the implicit trapezoid method, which
  // is defined as:
  // x(t+h) = x(t) + h/2 (f(t, x(t) + f(t+h, x(t+h))
  // x(t+h) from the implicit Euler method is presumably a good starting point.

  // The error estimate is derived as follows (thanks to Michael Sherman):
  // x*(t+h) = xᵢₑ(t+h) + O(h²)      [implicit Euler]
  //         = xₜᵣ(t+h) + O(h³)      [implicit trapezoid]
  // where x*(t+h) is the true (generally unknown) answer that we seek.
  // This implies:
  // xᵢₑ(t+h) + O(h²) = xₜᵣ(t+h) + O(h³)
  // Given that the second order term subsumes the third order one:
  // xᵢₑ(t+h) - xₜᵣ(t+h) = O(h²)
  // Therefore the difference between the implicit trapezoid solution and the
  // implicit Euler solution gives a second-order error estimate for the
  // implicit Euler result xᵢₑ.

  // Attempt to compute the implicit trapezoid solution.
  *xtplus_itr = *xtplus_ie;
  if (StepImplicitTrapezoid(dt, dx0, xtplus_itr)) {
    // Reset the state to that computed by implicit Euler.
    // TODO(edrumwri): Explore using the implicit trapezoid method solution
    //                 instead as *the* solution, rather than the implicit
    //                 Euler. Refer to [Lambert, 1991], Ch 6.
    context->SetTimeAndContinuousState(t0 + dt, *xtplus_ie);
    return true;
  } else {
    SPDLOG_DEBUG(drake::log(), "Implicit trapezoid approach FAILED with a step"
        "size that succeeded on Radau3.");
    return false;
  }
}

/// Takes a given step of the requested size, if possible.
/// @returns `true` if successful and `false` otherwise.
/// @post the time and continuous state will be advanced only if `true` is
///       returned.
template <class T>
bool Radau3Integrator<T>::DoStep(const T& h) {
  Context<T>* context = this->get_mutable_context();

  // Save the current time and state.
  const T t0 = context->get_time();
  SPDLOG_DEBUG(drake::log(), "IE DoStep(h={}) t={}", h, t0);

  // TODO(sherm1) Heap allocation here; consider mutable temporaries instead.
  const VectorX<T> xt0 = context->get_continuous_state().CopyToVector();
  VectorX<T> xtplus_ie(xt0.size()), xtplus_itr(xt0.size());

  // If the requested h is less than the minimum step size, we'll advance time
  // using two explicit Euler steps of size h/2. For error estimation, we also
  // take a single step of size h. We can estimate the error in the larger
  // step that way, but note that we propagate the two half-steps on the
  // assumption the result will be better despite not having an error estimate
  // for them (that's called "local extrapolation").
  if (h < this->get_working_minimum_step_size()) {
    SPDLOG_DEBUG(drake::log(), "-- requested step too small, taking explicit "
        "step instead");

    // TODO(edrumwri): Investigate replacing this with an explicit trapezoid
    //                 step, which would be expected to give better accuracy.
    //                 The mitigating factor is that h is already small, so a
    //                 test of, e.g., a square wave function, should quantify
    //                 the improvement (if any).

    // The error estimation process for explicit Euler uses two half-steps
    // of explicit Euler (for a total of two derivative evaluations). The error
    // estimation process is derived as follows:
    // (1) x*(t+h) = xₑ(t+h) + h²/2 df/dt + ...                [full step]
    // (2) x*(t+h) = ̅xₑ(t+h) + h²/8 (df/dt + df₊/dt) + ...    [two 1/2-steps]
    //
    // where x*(t+h) is the true (generally unknown) answer that we seek,
    // f() is the ordinary differential equation evaluated at x(t), and
    // f₊() is the derivative evaluated at x(t + h/2). Subtracting (1) from
    // (2), the above equations are rewritten as:
    // 0 = x̅ₑ(t+h) - xₑ(t+h) + h²/8 (-3df/dt + df₊/dt) + ...
    // The sum of all but the first two terms on the right hand side
    // of the above equation is less in magnitude than ch², for some
    // sufficiently large c. Or, written using Big-Oh notation:
    // x̅ₑ(t+h) - xₑ(t+h) = O(h²)
    // Thus, subtracting the two solutions yields a second order error estimate
    // (we compute norms on the error estimate, so the apparent sign error
    // in the algebra when arriving at the final equation is inconsequential).

    // Compute the Euler step.
    // TODO(sherm1) Heap allocation here; consider mutable temporary instead.
    VectorX<T> xdot = this->EvalTimeDerivatives(*context).CopyToVector();
    xtplus_ie = xt0 + h * xdot;

    // Do one half step.
    // TODO(sherm1) Heap allocation here; consider mutable temporary instead.
    const VectorX<T> xtpoint5 = xt0 + h/2 * xdot;
    context->SetTimeAndContinuousState(t0 + h/2, xtpoint5);

    // Do another half step, then set the "trapezoid" state to be the result of
    // taking two explicit Euler half steps. The code below the if/then/else
    // block simply subtracts the two results to obtain the error estimate.
    xdot = this->EvalTimeDerivatives(*context).CopyToVector();  // xdot(t + h/2)
    xtplus_itr = xtpoint5 + h/2 * xdot;
    context->SetTimeAndContinuousState(t0 + h, xtplus_itr);

    // Update the error estimation ODE counts.
    num_err_est_function_evaluations_ += 2;
  } else {
    // Try taking the requested step.
    bool success = AttemptStepPaired(h, &xtplus_ie, &xtplus_itr);

    // If the step was not successful, reset the time and state.
    if (!success) {
      context->SetTimeAndContinuousState(t0, xt0);
      return false;
    }
  }

  // Compute and update the error estimate.
  err_est_vec_ = xtplus_ie - xtplus_itr;

  // Update the caller-accessible error estimate.
  this->get_mutable_error_estimate()->get_mutable_vector().
      SetFromVector(err_est_vec_);

  return true;
}
*/

}  // namespace systems
}  // namespace drake
