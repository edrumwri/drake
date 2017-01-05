#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/solvers/moby_lcp_solver.h"

namespace drake {
namespace painleve {

/// Dynamical system representation of the Painleve' Paradox problem, taken
/// from [Stewart 2000]. The Painleve Paradox consists of a rod contacting
/// a planar surface *without impact* and subject to sliding Coulomb friction.
/// The problem is well known to correspond to an *inconsistent rigid contact
/// configuration*, where non-impulsive forces are necessary to resolve the
/// problem.
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// - AutoDiffXd
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// They are already available to link against in drakePainleve.
///
/// Inputs: no inputs.
/// States: planar position (state indices 0 and 1) and orientation (state
///         index 2), and planar linear velocity (state indices 3 and 4) and
///         scalar angular velocity (state index 5) in units of m, radians,
///         m/s, and rad/s, respectively. Orientation is measured counter-
///         clockwise with respect to the x-axis. One abstract state variable
///         (of type Painleve::Modes) is used to identify which dynamic mode
///         the system is in (e.g., ballistic, contacting at one point and
///         sliding, etc.), one abstract state variable (of type int) is used
///         to determine which endpoint(s) of the rod contact the halfspace
///         (k=-1 indicates the bottom of the rod when theta = pi/2, k=+1
///         indicates the top of the rod when theta = pi/2, and k=0 indicates
///         both endpoints of the rod are contacting the halfspace), and one
///         abstract state variable is used to indicate the sliding velocity
///         at the beginning of an integration interval.
/// Outputs: planar position (state indices 0 and 1) and orientation (state
///          index 2), and planar linear velocity (state indices 3 and 4) and
///          scalar angular velocity (state index 5) in units of m, radians,
///          m/s, and rad/s, respectively.
template <typename T>
class Painleve : public systems::LeafSystem<T> {
 public:
  /// Possible dynamic modes for the Painleve Paradox rod.
  enum Mode {
    /// Rod is currently undergoing ballistic motion.
    kBallisticMotion,

    /// Rod is sliding while undergoing non-impacting contact at one contact
    /// point (a rod endpoint); the other rod endpoint is not in contact.
    kSlidingSingleContact,

    /// Rod is sticking while undergoing non-impacting contact at one contact
    /// point (a rod endpoint); the other rod endpoint is not in contact.
    kStickingSingleContact,

    /// Rod is sliding at two contact points without impact.
    kSlidingTwoContacts,

    /// Rod is sticking at two contact points without impact.
    kStickingTwoContacts
  };

  // Constructor for the Painleve' Paradox system using piecewise DAE based
  // approach.
  Painleve();

  // Constructor for the Painleve' Paradox system using time stepping approach.
  Painleve(T dt);

  void DoCalcOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  void HandleImpact(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* new_state) const;

  /// Gets the acceleration due to gravity.
  T get_gravitational_acceleration() const { return g_; }

  /// Gets the coefficient of dynamic (sliding) Coulomb friction.
  T get_mu_Coulomb() const { return mu_; }

  /// Sets the coefficient of dynamic (sliding) Coulomb friction.
  void set_mu_Coulomb(T mu) { mu_ = mu; }

  /// Gets the mass of the rod.
  T get_rod_mass() const { return mass_; }

  /// Gets the length of the rod.
  T get_rod_length() const { return rod_length_; }

  /// Gets the rod moment of inertia.
  T get_rod_moment_of_inertia() const { return J_; }

  /// Gets the integration step size for the time stepping system.
  T get_integration_step_size() const { return dt_; }

  int DetermineNumberWitnessFunctions(const
                                      systems::Context<T>& context) const;
  void DoCalcDiscreteVariableUpdates(const systems::Context<T>& context,
                                   systems::DiscreteState<T>* discrete_state)
    const override;

 protected:
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;

 private:
  solvers::MobyLCPSolver lcp_;
  static T EvaluateSlidingDot(const Painleve<T>& painleve,
                              const systems::Context<T>& context);
  static T CalcSignedDistance(const Painleve<T>& painleve,
                              const systems::Context<T>& context);
  static T CalcNormalAccelWithoutContactForces(const Painleve<T>& painleve,
                                               const systems::Context<T>&
                                                 context);
  static T CalcEndpointDistance(const Painleve<T>& painleve,
                                const systems::Context<T>& context);
  static T sqr(const T& x) { return x*x; }

  T dt_{0.0};          // Integration step-size for time stepping approach.
  T mass_{1.0};        // The mass of the rod.
  T rod_length_{1.0};  // The length of the rod.
  T mu_{1000.0};       // The coefficient of friction.
  T g_{-9.81};         // The acceleration due to gravity.
  T J_{1.0};           // The moment of the inertia of the rod.
};

}  // namespace painleve
}  // namespace drake
