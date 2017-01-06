#pragma once

#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A first-order, half-explicit DAE solver. This approach solves DAEs of the
 * form:
 * <pre>
 * dx/dt = f(x(t), λ)
 * g(x) = 0
 * </pre>
 * using the first order relationship:
 * <pre>
 * x(t+Δt) = x(t) + Δt⋅f(x(t), λ)
 * </pre>
 * to solve the following nonlinear system of equations for λ:
 * <pre>
 * g(x(t)+Δt⋅f(x(t), λ) = 0
 * </pre>
 * After this value of λ has been obtained *implicitly*, x(t+Δt) is computed
 * *explicitly* using the second to last equation above.
 */
template <class T>
class HalfExplicitDAE1Solver : public IntegratorBase<T> {
 public:
  ~HalfExplicitDAE1Solver() override = default;

  // Disable copy, assign, and move.
  HalfExplicitDAE1Solver(const HalfExplicitDAE1Solver<T>& other) = delete;
  HalfExplicitDAE1Solver& operator=(const HalfExplicitDAE1Solver<T>& other) =
      delete;

  /**
   * Constructs a fixed-step integrator for a given system using the given
   * context for initial conditions.
   * @param system A reference to the system to be simulated
   * @param max_step_size The maximum (fixed) step size; the integrator will
   *                      not take larger step sizes than this.
   * @param context Pointer to the context (nullptr is ok, but the caller
   *                must set a non-null context before Initialize()-ing the
   *                integrator).
   * @sa Initialize()
   */
  HalfExplicitDAE1Solver(const System<T>& system, const T& max_step_size,
                          Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    IntegratorBase<T>::set_maximum_step_size(max_step_size);
    derivs_ = system.AllocateTimeDerivatives();
  }

  /**
   * Solver does not currently support error estimation.
   */
  bool supports_error_estimation() const override { return false; }

  /// Solver does not provide an error estimate.
  int get_error_estimate_order() const override { return 0; }

 private:
  void DoStepOnceFixedSize(const T& dt) override;

  // These are pre-allocated temporaries for use by integration
  std::unique_ptr<ContinuousState<T>> derivs_;
};

/**
 * Integrates the system forward in time by dt. This value is determined
 * by IntegratorBase::Step().
 */
template <class T>
void HalfExplicitDAE1Solver<T>::DoStepOnceFixedSize(const T& dt) {
  // Find the continuous state xc within the Context, just once.
  auto context = IntegratorBase<T>::get_mutable_context();
  VectorBase<T>* xc = context->get_mutable_continuous_state_vector();

  // TODO(sherm1) This should be calculating into the cache so that
  // Publish() doesn't have to recalculate if it wants to output derivatives.
  IntegratorBase<T>::get_system().CalcTimeDerivatives(
      IntegratorBase<T>::get_context(), derivs_.get());

  // Compute derivative and update configuration and velocity.
  // xc(t+h) = xc(t) + dt * xcdot(t, xc(t), u(t))
  const auto& xcdot = derivs_->get_vector();
  xc->PlusEqScaled(dt, xcdot);  // xc += dt * xcdot
  context->set_time(context->get_time() + dt);

  IntegratorBase<T>::UpdateStatistics(dt);
}
}  // namespace systems
}  // namespace drake

