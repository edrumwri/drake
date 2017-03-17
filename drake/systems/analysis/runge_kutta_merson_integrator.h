#pragma once

#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * An error controlled, 3rd order accurate Runge-Kutta integrator with a
 * fourth order error estimate.
 * @tparam T A double or autodiff type.
 *
 * This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
 * this class, please refer to http://drake.mit.edu/cxx_inl.html.
 *
 * Instantiated templates for the following kinds of T's are provided:
 * - double
 *
 * For a discussion of the Runge-Kutta-Merson method, see Hairer,
 * Norsett & Wanner, Solving ODEs I, 2nd rev. ed. pp. 166-8, and table 4.1
 * on page 167. 
 *
 * The Butcher tableaux for this integrator follows:
 * <pre>
 *        |
 * 0      |
 * 1/3    | 1/3
 * 1/3    | 1/6         1/6
 * 1/2    | 1/8         0            3/8 
 * 1      | 1/2         0            -3/2         2 
 * ---------------------------------------------------------------------------
 *          1/6         0            0            2/3          1/6
 *          1/10        0            3/10         2/5          1/5
 * </pre>
 * Description follows from Simbody, from which this integrator was adapted:
 * "This is a 5-stage, first-same-as-last (FSAL) 4th order method which gives 
 * us an embedded 3rd order method as well, so we can extract a 4th-order 
 * error estimate for the 3rd-order result, which error estimate can then be 
 * used for step size control, since it will behave as h^4. We then propagate 
 * the 4th order result (whose error is unknown), which Hairer calls 'local 
 * extrapolation'. We call the initial state (t0,y0) and want (t0+h,y1). We 
 * are given the initial derivative f0=f(t0,y0), which most likely is left 
 * over from an evaluation at the end of the last step.
 * 
 * We will call the derivatives at stage f1,f2,f3,f4 but these are done with 
 * only two temporaries fa and fb. (What we're calling 'f' Hairer calls 'k'.)
 */
template <class T>
class RungeKuttaMersonIntegrator : public IntegratorBase<T> {
 public:
  virtual ~RungeKuttaMersonIntegrator() {}

  RungeKuttaMersonIntegrator(const System <T>& system,
                               Context <T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    derivs0_ = system.AllocateTimeDerivatives();
    derivs1_ = system.AllocateTimeDerivatives();
    derivs2_ = system.AllocateTimeDerivatives();
    derivs3_ = system.AllocateTimeDerivatives();
    derivs4_ = system.AllocateTimeDerivatives();
    derivs5_ = system.AllocateTimeDerivatives();
  }

  /**
 * Integrator supports accuracy estimation.
 */
  bool supports_accuracy_estimation() const override { return true; }

  /**
   * Integrator supports error control.
   */
  bool supports_error_control() const override { return true; }

  /// Runge-Kutta-Merson is a fourth order integrator.
  int64_t get_error_order() const override { return 4; }

 private:
  void DoInitialize();
  bool DoStep(const T& dt) override;
  void Integrate(const T& dt);

  // The last integration step size
  T last_step_size_;

  // The predicted next integration step size
  T current_step_size_{IntegratorBase<T>::nan()};

  // These are pre-allocated temporaries for use by integration.
  std::unique_ptr <ContinuousState<T>> derivs0_, derivs1_, derivs2_, derivs3_,
                                       derivs4_, derivs5_;

  // State and time derivative copies for reversion during integration.
  VectorX<T> xc0_save_, xcdot0_save_;

  // The error estimate computed during Integrate(.)
  std::unique_ptr<VectorBase<T>> err_est_;
};
}  // systems
}  // drake
