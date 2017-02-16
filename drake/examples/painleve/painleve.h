#pragma once

#include <memory>
#include <utility>

#include "drake/solvers/moby_lcp_solver.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace painleve {

/// Dynamical system representation of a rod contacting a half-space in
/// two dimensions. This system can be modeling and simulated using one of
/// three models: a pseudo-rigid model (the rod is rigid, but contact between
/// the rod and the half-space is modeled as compliant), a fully rigid model
/// simulated with piecewise differential algebraic equations, and a fully
/// rigid model simulated using a first-order time stepping approach. The rod
/// state is initialized to the configuration that corresponds to the
/// Painlevé Paradox problem, described in [Stewart 2000]. The paradox consists
/// of a rod contacting a planar surface *without impact* and subject to sliding
/// Coulomb friction. The problem is well known to correspond to an
/// *inconsistent rigid contact configuration*, where impulsive forces are
/// necessary to resolve the problem.
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// Inputs: planar force (two-dimensional) and torque (scalar), which are
///         arbitrary "external" forces (expressed in the world frame) applied
///         at the center-of-mass of the rod.
///
/// States: planar position (state indices 0 and 1) and orientation (state
///         index 2), and planar linear velocity (state indices 3 and 4) and
///         scalar angular velocity (state index 5) in units of m, radians,
///         m/s, and rad/s, respectively. Orientation is measured counter-
///         clockwise with respect to the x-axis. For simulations using the
///         piecewise DAE formulation, one abstract state variable
///         (of type Painleve::Mode) is used to identify which dynamic mode
///         the system is in (e.g., ballistic, contacting at one point and
///         sliding, etc.) and one abstract state variable (of type int) is used
///         to determine which endpoint(s) of the rod contact the halfspace
///         (k=-1 indicates the bottom of the rod when theta = pi/2, k=+1
///         indicates the top of the rod when theta = pi/2, and k=0 indicates
///         both endpoints of the rod are contacting the halfspace).
///
/// Outputs: planar position (state indices 0 and 1) and orientation (state
///          index 2), and planar linear velocity (state indices 3 and 4) and
///          scalar angular velocity (state index 5) in units of m, radians,
///          m/s, and rad/s, respectively.
///
/// * [Stewart, 2000]  D. Stewart, "Rigid-Body Dynamics with Friction and
///                    Impact". SIAM Rev., 42(1), 3-39, 2000.
template <typename T>
class Painleve : public systems::LeafSystem<T> {
 public:
  /// Simulation model and approach for the system.
  enum SimulationType {
    /// For simulating the system using rigid contact, Coulomb friction, and
    /// piecewise differential algebraic equations.
    kPiecewiseDAE,

    /// For simulating the system using rigid contact, Coulomb friction, and
    /// a first-order time stepping approach.
    kTimeStepping,

    /// For simulating the system using compliant contact, Coulomb friction,
    /// and ordinary differential equations.
    kCompliant
  };

  /// Possible dynamic modes for the Painleve Paradox rod.
  enum Mode {
    /// Mode is invalid.
    kInvalid,

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

  /// Constructor for the Painlevé Paradox system using either the piecewise
  /// DAE (differential algebraic equation) based approach or the compliant
  /// ordinary differential equation based approach.
  explicit Painleve(SimulationType simulation_type);

  /// Constructor for the Painlevé Paradox system using a time stepping
  /// approach.
  /// @param dt The integration step size. This step size cannot be reset
  ///           after construction.
  /// @throws std::logic_error if @p dt is not positive.
  explicit Painleve(double dt);

  /// Gets the constraint force mixing parameter (CFM, used for time stepping
  /// systems only).
  double get_cfm() const { return cfm_; }

  /// Sets the constraint force mixing parameter (CFM, used for time stepping
  /// systems only). The default CFM value is 1e-8.
  /// @param cfm a floating point value in the range [0, infinity].
  /// @throws std::logic_error if this is not a time stepping system or if
  ///         cfm is set to a negative value.
  void set_cfm(double cfm) {
    if (simulation_type_ != kTimeStepping)
      throw std::logic_error("Attempt to set CFM for non-time stepping "
                             "system.");
    if (cfm < 0)
      throw std::logic_error("Negative CFM value specified.");
    cfm_ = cfm;
  }

  /// Gets the error reduction parameter (ERP, used for time stepping systems
  /// only).
  double get_erp() const { return erp_; }

  /// Sets the error reduction parameter (ERP, used for time stepping systems
  /// only). The default ERP value is 0.8.
  /// @param erp a floating point value in the range [0, 1].
  /// @throws std::logic_error if this is not a time stepping system or if
  ///         erp is set to a negative value.
  void set_erp(double erp) {
    if (simulation_type_ != kTimeStepping)
      throw std::logic_error("Attempt to set ERP for non-time stepping "
                             "system.");
    if (erp < 0 || erp > 1)
      throw std::logic_error("Invalid ERP value specified.");
    erp_ = erp;
  }

  /// Models impact using an inelastic impact model with friction.
  /// @p new_state is set to the output of the impact model on return.
  void HandleImpact(const systems::Context<T>& context,
                    systems::State<T>* new_state) const;

  /// Gets the acceleration (with respect to the positive y-axis) due to
  /// gravity (i.e., this number should generally be negative).
  double get_gravitational_acceleration() const { return g_; }

  /// Sets the acceleration (with respect to the positive y-axis) due to
  /// gravity (i.e., this number should generally be negative).
  void set_gravitational_acceleration(double g) { g_ = g; }

  /// Gets the coefficient of dynamic (sliding) Coulomb friction.
  double get_mu_coulomb() const { return mu_; }

  /// Sets the coefficient of dynamic (sliding) Coulomb friction.
  void set_mu_coulomb(double mu) { mu_ = mu; }

  /// Gets the mass of the rod.
  double get_rod_mass() const { return mass_; }

  /// Sets the mass of the rod.
  void set_rod_mass(double mass) { mass_ = mass; }

  /// Gets the length of the rod.
  double get_rod_length() const { return rod_length_; }

  /// Sets the length of the rod.
  void set_rod_length(double rod_length) { rod_length_ = rod_length; }

  /// Gets the rod moment of inertia.
  double get_rod_moment_of_inertia() const { return J_; }

  /// Sets the rod moment of inertia.
  void set_rod_moment_of_inertia(double J) { J_ = J; }

  /// Get compliant contact normal stiffness in N/m.
  double get_stiffness() const { return stiffness_; }

  /// Set compliant contact normal stiffness in N/m (>= 0).
  void set_stiffness(double stiffness) {
    DRAKE_DEMAND(stiffness >= 0);
    stiffness_ = stiffness;
  }

  /// Get compliant contact normal dissipation in 1/v (s/m).
  double get_dissipation() const { return dissipation_; }

  /// Set compliant contact normal dissipation in 1/v (s/m, >= 0).
  void set_dissipation(double dissipation) {
    DRAKE_DEMAND(dissipation >= 0);
    dissipation_ = dissipation;
  }

  /// Get compliant contact static friction (stiction) coefficient `μ_s`.
  double get_mu_static() const { return mu_s_; }

  /// Set compliant contact stiction coefficent (>= mu_coulomb).
  void set_mu_static(double mu_static) {
    DRAKE_DEMAND(mu_static >= mu_);
    mu_s_ = mu_static;
  }

  /// Get the stiction velocity tolerance (m/s).
  double get_stiction_velocity_tolerance() const {return v_stick_tol_;}

  /// Set the stiction velocity tolerance (m/s). This is the maximum slip
  /// velocity that we are willing to consider as sticking. For a given normal
  /// force N this is the velocity at which the friction force will be largest,
  /// at `μ_s*N` where `μ_s` is the static coefficient of friction.
  void set_stiction_velocity_tolerance(double v_stick_tol) {
    DRAKE_DEMAND(v_stick_tol > 0);
    v_stick_tol_ = v_stick_tol;
  }

  /// Checks whether the system is in an impacting state, meaning that the
  /// relative velocity along the contact normal between the rod and the
  /// halfspace is such that the rod will begin interpenetrating the halfspace
  /// at any time Δt in the future (i.e., Δt > 0). If the context does not
  /// correspond to a configuration where the rod and halfspace are contacting,
  /// this method returns `false`.
  bool IsImpacting(const systems::Context<T>& context) const;

  /// Gets the integration step size for the time stepping system.
  /// @returns 0 if this is a DAE-based system.
  double get_integration_step_size() const { return dt_; }

  /// Gets the model and simulation type for this system.
  SimulationType get_simulation_type() const { return simulation_type_; }

  /// Return net contact forces as F_Ro_W=(fx,fy,tau) where f_Ro_W=(fx,fy) is
  /// applied at the rod origin Ro, and m_R=tau is the moment due to the
  /// contact forces actually being applied elsewhere. This may result from
  /// one or more contact points.
  Vector3<T> CalcCompliantContactForces(
      const systems::Context<T>& context) const;

 protected:
  int get_k(const systems::Context<T>& context) const;
  std::unique_ptr<systems::AbstractState> AllocateAbstractState()
                                            const override;
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;
  void DoCalcTimeDerivatives(const systems::Context<T>& context,
                             systems::ContinuousState<T>* derivatives)
                               const override;
  void DoCalcDiscreteVariableUpdates(const systems::Context<T>& context,
                                     systems::DiscreteState<T>* discrete_state)
      const override;
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;

 private:
  Vector2<T> CalcStickingImpactImpulse(const systems::Context<T>& context)
    const;
  Vector2<T> CalcFConeImpactImpulse(const systems::Context<T>& context) const;
  void CalcAccelerationsCompliantContactAndBallistic(
                                  const systems::Context<T>& context,
                                  systems::ContinuousState<T>* derivatives)
                                    const;
  void CalcAccelerationsBallistic(const systems::Context<T>& context,
                                  systems::ContinuousState<T>* derivatives)
                                    const;
  void CalcAccelerationsTwoContact(const systems::Context<T>& context,
                                   systems::ContinuousState<T>* derivatives)
                                     const;
  void CalcAccelerationsOneContactNoSliding(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const;
  void CalcAccelerationsOneContactSliding(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const;
  void SetAccelerations(const systems::Context<T>& context,
                        systems::VectorBase<T>* const f,
                        const T& fN, const T& fF,
                        const T& xc, const T& yc) const;
  Vector2<T> CalcStickingContactForces(
      const systems::Context<T>& context) const;


  // Utility method for determining the World frame location of one of three
  // points on the rod whose origin is Ro. Let r be the half-length of the rod.
  // Define point P = Ro+k*r where k = { -1, 0, 1 }. This returns p_WP.
  static Vector2<T> CalcRodEndpoint(const T& x, const T& y, const int k,
                                    const T& ctheta, const T& stheta,
                                    const double half_rod_len);

  // Given a location p_WC of a point C in the World frame, define the point Rc
  // of the rod that is coincident with C, and report Rc's World frame velocity
  // v_WRc. We're given p_WRo=(x,y) and V_WRo=(v_WRo,w_WR)=(xdot,ydot,thetadot).
  static Vector2<T> CalcCoincidentRodPointVelocity(
      const Vector2<T>& p_WRo, const Vector2<T>& v_WRo,
      const T& w_WR,  // aka thetadot
      const Vector2<T>& p_WC);

  // Quintic step function approximation used by Stribeck friction model.
  static T step5(const T& x);

  // Friction model used in compliant contact.
  static T CalcMuStribeck(const T& us, const T& ud, const T& v);

  // Solves linear complementarity problems for time stepping.
  solvers::MobyLCPSolver lcp_;

  // The simulation type, currently unable to be changed after object
  // construction.
  SimulationType simulation_type_;

  double dt_{0.0};          // Integration step-size for time stepping approach.
  double mass_{1.0};        // The mass of the rod (kg).
  double rod_length_{1.0};  // The length of the rod (m).
  double mu_{1000.0};       // The (dynamic) coefficient of friction.
  double g_{-9.81};         // The acceleration due to gravity (in y direction).
  double J_{1.0};           // The moment of the inertia of the rod.
  double erp_{0.8};         // ERP for time stepping systems.
  double cfm_{1e-8};        // CFM for time stepping systems.

  // Compliant contact parameters.
  double stiffness_{10000};   // Normal stiffness of the ground plane (N/m).
  double dissipation_{1};     // Dissipation factor in 1/v (s/m).
  double mu_s_{mu_};          // Static coefficient of friction (>= mu).
  double v_stick_tol_{1e-3};  // Velocity below which we are in stiction.
};

}  // namespace painleve
}  // namespace drake
