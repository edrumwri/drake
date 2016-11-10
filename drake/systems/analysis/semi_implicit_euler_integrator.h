#pragma once

#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/**
 * A first-order, semi-implicit Euler integrator. State is updated in the
 * following manner:
 * <pre>
 * v(t0+h) = v(t0) + dv/dt(t0) * h
 * qdot  = N(q(t0)) * v(t0+h)
 * q(t0+h) = q(t0) + qdot * h
 * </pre>
 * where `v = dx/dt` are the generalized velocity variables, `q` are generalized
 * coordinates, and `x` are known as quasi-coordinates. `h` is the integration
 * step size, and `N` is a matrix (dependent upon `q`) that maps velocities
 * to time derivatives of position coordinates. For rigid body systems in 2D,
 * for example, `N` will generally be an identity matrix. For rigid body systems
 * in 3D, `N` and its transpose are frequently used to transform between
 * time derivatives of Euler parameters (unit quaternions) and angular 
 * velocities (and vice versa). See [Nikravesh]
 * TODO(edrumwri): add reference for Nikravesh
 * 
 * Note that these equations imply that the velocity variables are updated
 * first and that these new velocities are then used to update the generalized
 * coordinates (compare to ExplicitEulerIntegrator, where the generalized
 * coordinates are updated using the previous velocity variables).
 *
 * When the system is Hamiltonian, the semi-implicit Euler integrator is a
 * symplectic (momentum conserving) integrator. Symplectic integrators promise
 * large, stable step sizes compared to non-symplectic integrators.
 *
 * If we expand `dv/dt(t0)` to `dv/dt(q(t0), v(t0), t0)` it can start to become
 * clear how forces- functions of q(t0) or v(t0)- can be added into this
 * picture. One simple example using a spring-mass-damper is:<pre>
 * `dv/dt(t0) =  (-kq(t0) - bv(t0))/m
 * </pre>
 * One interesting possibility occurs when dv/dt(t0) is not the "simple" outcome
 * of a force, but rather is determined simultaneously with the force. Such is
 * the case when modeling rigid contact, where the accelerations must be
 * computed simultaneously (through solution to a linear complementarity problem
 * or a nonlinear complemenetarity problem) with the contact forces. In such a
 * case we can rearrange the top equation above to yield:
 * <pre>
 * v(t0+h) - v(t0) = dv/dt(t0) * h
 * </pre>
 * This rearrangement indicates that any forces computed by `dv/dt(t0)` can be
 * made to be impulsive forces by scaling them by `1/h`.
 *
 * <pre>
 * v(t0+h) = v(t0) + dv/dt(t0) * h
 * qdot  = N(q(t0)) * v(t0+h)
 * q(t0+h) = q(t0) + qdot * h
 * \dot{phi}(v(t0+h), q(t0)) >= 0
 * </pre>
 *
 * If phi(.) is satisfied at q(t0), and \ddot{phi} = 0 over the whole interval:
 * phi(q(t+delta t)) \approx phi(q(t0)) + \delta t * \dot{phi}(q(t0), v(t0)) +
 *                           \delta t^2/2 \ddot{\phi}(q(t0), v(t0), a(t0))
 * 
 *
 * (How does this relate to phi(v(t0+h), q(t0+h)))? When does this constraint
 * apply over the whole interval?)
 *
 * The semi-implicit Euler integrator is able to provide a first-order solution
 * to the rigid body dynamics equations with rigid contact and Coulomb friction.
 * This ability is particularly important because of the possibility of
 * *inconsistent configurations* that may occur when these models are used
 * together. An inconsistent configuration occurs when no set of non-impulsive
 * forces satisfies all of the model constraints (note that the transformation
 * to impulsive forces above is responsible for solving this problem).
 * Inconsistent configurations are typified by the problem of Painleve's
 * Paradox.
 *
 * (how does this differ from solving equations for explicit Euler?)
 * - constraints can remain enforced at the end of every time step
 *
 * <h4>Association between time stepping and the semi-implicit Euler
 * integrator:</h4>
 * Though many time stepping approaches use the formulations above, these
 * equations do not represent a "time stepping scheme". These equations can
 * be applied from one point in state space to another, assuming smoothness
 * in between, just like any other integrator: a simulator integrates to
 * discontinuities, the state of the ODE/DAE is re-initialized, and integration
 * continues.
 *
 * On the other hand, time stepping schemes enforce all constraints at a single
 * time in the integration process: though a billiard break may consist of tens
 * of collisions occurring sequentially over a millisecond of time, a time
 * stepping method will treat all of these collisions as occurring
 * simultaneously. [Stewart 1998] provides proofs that such an approach
 * converges to the solution of the continuous time rigid body dynamics problem
 * as the integration step goes to zero. The practical effect is of losing
 * first order accuracy when unilateral constraints are present.
 */
template <class T>
class SemiImplicitEulerIntegrator : public IntegratorBase<T> {
 public:
  virtual ~SemiImplicitEulerIntegrator() {}

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
  SemiImplicitEulerIntegrator(const System<T>& system, const T& max_step_size,
                          Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    IntegratorBase<T>::set_maximum_step_size(max_step_size);
    derivs_ = system.AllocateTimeDerivatives();
  }

  /**
   * Integrator does not support accuracy estimation.
   */
  bool supports_error_estimation() const override { return false; }

 private:
  bool DoStep(const T& dt) override;

  // These are pre-allocated temporaries for use by integration
  std::unique_ptr<ContinuousState<T>> derivs_;
};

/**
 * Integrates the system forward in time by dt. This value is determined
 * by IntegratorBase::Step().
 */
template <class T>
void SemiImplicitEulerIntegrator<T>::DoStep(const T& dt) {
  // Find the continuous state xc within the Context, just once.
  auto context = IntegratorBase<T>::get_mutable_context();
  VectorBase<T>* xc = context->get_mutable_continuous_state_vector();

  // TODO(sherm1) This should be calculating into the cache so that
  // Publish() doesn't have to recalculate if it wants to output derivatives.
  IntegratorBase<T>::get_system().EvalTimeDerivatives(
      IntegratorBase<T>::get_context(), derivs_.get());

  // Update configuration.
  // q(t0+h) = q(t0) + dt * qdot
  const auto& xcdot = derivs_->get_vector();
  xc->PlusEqScaled(dt, xcdot);  // xc += dt * xcdot
  context->set_time(context->get_time() + dt);
}
}  // namespace systems
}  // namespace drake

