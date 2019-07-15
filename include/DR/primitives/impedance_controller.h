#pragma once

#include <memory>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>

namespace DR {

/**
 @section impedance-controller-overview Overview
  
 An impedance controller determines actuation forces that regulate the accelerations (and, by extension, the positions
 and velocities) of the robot and any objects that it might be contacting. Given reference accelerations (typically
 computed by some plan that accounts for forces from contact interactions), an impedance controller will compute
 actuator forces that realize the accelerations for interacting multibodies (e.g., a robot manipulating a box) subject
 to mass-spring-damper models (see @ref impedance-control-physical-model). Concretely, this means that the controller
 should drive the robot to, e.g., keep the box in a secure grasp during pick-and-place, as the task execution inevitably
 diverges from the planned execution. 

 @subsection impedance-control-physical-model Physical model

 The underlying model of the contact interactions is that of a mass-spring-damper system:
   @verbatim
   M̂φ̈ + Bφ̇ + Kφ = f̂                          (1)
   @endverbatim

 where φ is some function of the generalized coordinates of the multibody system (i.e., the robot and any objects
 that it interacts with), M̂ is a mass matrix, C and K are damping and stiffness matrices, respectively, and f̂ is some
 force.
 
 That mass-spring-damper system is coupled to our multibody dynamics system by:
   @verbatim
   Mv̇ = Gᵀ(-Kφ - Cφ̇ - M̂Ġv) + Bu + f          (2)
   @endverbatim

 where M is multibody inertia matrix, v are the generalized velocities, v̇ are the generalized accelerations, G is
 defined as ∂φ/∂q, B is the actuation matrix, u are the actuator forces, and f are the "external" forces acting on the
 multibody system. For simplicity, we will assume that the time derivative of the generalized coordinates is the
 generalized velocity (i.e., q̇ = v); this assumption can be relaxed but it slightly complicates the definition of G
 and the incorporation of error feedback (in (3), below).

 It can be shown that the multibody system (2) acts like the mass-spring-damper system (1) for the following definitions
 of M̂ and f̂:
   @verbatim
   M̂ ≡ (GM⁻¹Gᵀ)⁻¹
   f̂ ≡ M̂GM⁻¹f
   @endverbatim
 
 The controller is able to compute M, C, K, B, f, φ, and φ̇. v̇ is passed to the controller- it will generally be
 expected that it is the combination of output from a planning process and some error feedback terms, e.g.:
   @verbatim
   v̇ ≡ v̇ᵈᵉˢ + kᵖ(qᵈᵉˢ - q) + kᵈ(vᵈᵉˢ - v)   (3)
   @endverbatim
 where kᵖ and kᵈ are proportional and derivative gains. 

 The resulting controller is to compute actuator forces u such that (1) is satisfied (for v̇). By rearranging (2) we
 arrive at:
   @verbatim
   Bu = Gᵀ(Kφ + Cφ̇ + M̂Ġv) - Mv̇ - f           (4)
   @endverbatim

 For the common case where B is a binary matrix, u is particularly easy to compute. **This class requires B to be
 a binary matrix.** 

 @subsection impedance-control-challenges Challenges

 Applying impedance control to typical manipulation tasks requires being knowledgable of the following challenges:

 @subsubsection impedance-control-challenges-KC Determining K and C

 Stiffness and damping matrices K and C use units of force/displacement (e.g., N/m) and force/speed (e.g., Ns/m). K and
 C are determined by the ways that a robot is expected to interact with its  environment.
 
 If φ and φ̇ correspond to an axial contact, then K can be derived using Young's Modulus (often denoted "E"); values of E
 for typical materials are listed in engineering tables. Young's Modulus has units of pressure (e.g., N/m²), meaning
 that E has to be instantaneously converted to K using the current shape of a material (i.e., K will be a function of
 the multibody system state). The axial stiffness of two materials pressed into one another (e.g., a small sphere
 pressed into a large block) is:
   @verbatim
   K = AE/L
   @endverbatim

 where A is the cross-sectional area of contact and L is the length along that axis. Using the example of the sphere of
 radius r pressed d meters into a cube of volume (s³), where r < s/2, A will be rd and L will be s. See
 https://en.wikipedia.org/wiki/Contact_mechanics#Contact_between_a_sphere_and_a_half-space. 

 C generally must be determined empirically or estimated: there are no principled ways of determining this value. 

 φ and φ̇ can correspond to many other applications including effecting virtual constraints. For example, φ can be used
 to represent the deflection along some axes, with different stiffnesses along the different axes. Put another way,
 the robot can be made to act stiff in some directions and compliant in others. [Ott 2010] claims that impedance
 control results in poor accuracy in free-space due to unmodeled dynamics, though we note that is the case because there
 is no term that corrects steady-state error; our formulation in (3) can incorporate such a term. 

 @subsubsection impedance-control-measuring Measuring φ and φ̇

 Measuring φ and φ̇ is typically challenging using current technology: general devices for measuring deformation between
 pairs of bodies do not exist. Recently, researchers have developed particular mechanisms that are capable of reporting
 how they deform. Until such mechanisms are commercially available, a substitute is to map the (undeformed) geometries
 of two bodies in an overlapping configuration to a deformed configuration. This technique is a quasistatic approach,
 since the deformation at any state does not leverage the deformation at a previous state. To make the quasistatic
 approach concrete, consider two spheres; φ could then map to `max(0, d)` where d is the signed distance between the two
 spheres (the spheres do not stick together without contact, so there is no tensile deformation).

 Note that small measurement errors can result in large forces when materials are modeled as nearly rigid.
 The approach in [Drumwright 2019] can be used to address this problem because it underestimates contact forces for
 relatively large integration steps (which corresponds to controller update rates of 1000Hz or lower). 

 @subsubsection impedance-control-modeling-friction Modeling friction 

 Although it is possible to model friction using a mass-spring-damper model, it can be technically challenging.
 Recalling that no sensing system currently exists that can directly sense deformation, we again consider model-based
 alternatives. The quasistatic approach described above does not easily map to tangential deformation. Alternatively,
 one could imagine a model-based approach that uses Bayesian filtering to estimate state variables that describe
 contacting bodies' deformations, though we are presently unaware of any work that has demonstrated this idea.
 
 Another way to effect friction is to ignore φ (by assuming that K is zero) and use a large value for C to model
 friction. The advantage of this idea is that φ̇ can be easy to measure in that case: it would just be the slip velocity
 at a point of contact. The disadvantage is that spurious φ̇ measurements could result in large forces (see @ref
 impedance-control-measuring) and that stiction is not modeled (no force is applied when φ̇ is zero). However, the
 approach in [Drumwright 2019] organically avoids both problems. 

  @section impedance-controller-refs References

 [Drumwright 2019] E. Drumwright. An Unconditionally Stable First-Order Constraint Solver for Multibody Systems.
                   https://arxiv.org/abs/1905.10828. 2019.
 [Ott 2010]        C. Ott, R. Mukherjee, and Y. Nakamura. Unified Impedance and Admittance Control. Proc. IEEE Intl.
                   Conf. on Robotics and Automation, 2010.
 */
template <typename T>
class ImpedanceController : public drake::systems::LeafSystem<T> {
 public:
  /// Constructs the impedance controller with a plant containing all multibodies in the environment (including the
  /// robot).
  /// @throws std::logic_error if the plant's actuation matrix is not a binary matrix.
  ImpedanceController(
      const drake::multibody::MultibodyPlant<T>* all_plant);

  /// Gets a reference to the plant containing all multibodies in the environment (including the robot).
  const drake::multibody::MultibodyPlant<T>& all_plant() const { return all_plant_; }

  /// Gets the input port for the estimated generalized positions of every multibody in the environment.
  /// The vector is ordered according to the generalized positions of all_plant().
  const drake::systems::InputPort<T>& all_q_estimated_input_port() const {
    return drake::systems::System<T>::get_input_port(all_q_estimated_input_port_index_);
  }

  /// Gets the input port for the estimated generalized velocities of every multibody in the environment.
  /// The vector is ordered according to the generalized velocities of all_plant().
  const drake::systems::InputPort<T>& all_v_estimated_input_port() const {
    return drake::systems::System<T>::get_input_port(all_v_estimated_input_port_index_);
  }

  /// Gets the input port for desired accelerations of every multibody in the environment. The
  /// vector is ordered according to the generalized velocities of `all_plant()`. Desired accelerations for
  /// fixed bodies in the all_plant() are ignored.
  const drake::systems::InputPort<T>& all_vdot_desired_input_port() const {
    return drake::systems::System<T>::get_input_port(all_vdot_desired_input_port_index_);
  }

  /// Gets the port that provides commands to the actuators.
  const drake::systems::OutputPort<T>& generalized_effort_output_port() const {
    return drake::systems::System<T>::get_output_port(generalized_effort_output_port_index_);
  }

  /// Returns the m x n-dimensional Jacobian matrix of partial derivatives of the m-dimensional vector φ taken with
  /// with respect to the n-dimensional vector of generalized coordinates q of the plant. This calculation is cached on
  /// the estimated states input port.
  /// @param context the Context of this controller
  const drake::MatrixX<T>& EvalG(const drake::systems::Context<T>& context) const {
    return this->get_cache_entry(G_cache_index_).template Eval<drake::MatrixX<T>>(context);
  }

  /// Returns the deformations (virtual or physical) as a function of the Context. The length of this vector as well as
  /// the significance of the individual elements in the vector are Context-dependent. If this vector is zero, (4)
  /// degenerates to pure inverse dynamics control. This calculation is cached on the estimated states input port.
  /// @param context the Context of this controller
  const drake::VectorX<T>& EvalPhi(const drake::systems::Context<T>& context) const {
    return this->get_cache_entry(phi_cache_index_).template Eval<drake::VectorX<T>>(context);
  }

  /// Returns the deformation rates as a function of the Context. The length of this vector as well as
  /// the significance of the individual elements in the vector are Context-dependent. If this vector is zero, (4)
  /// degenerates to pure inverse dynamics control. This calculation is cached on the estimated states input port.
  /// @param context the Context of this controller
  const drake::VectorX<T>& EvalPhiDot(const drake::systems::Context<T>& context) const {
    return this->get_cache_entry(phi_dot_cache_index_).template Eval<drake::VectorX<T>>(context);
  }

  /// Returns the stiffness matrix as a function of the Context.This calculation is cached on the estimated states
  /// input port.
  /// @param context the Context of this controller
  const drake::MatrixX<T>& EvalK(const drake::systems::Context<T>& context) const {
    return this->get_cache_entry(K_cache_index_).template Eval<drake::MatrixX<T>>(context);
  }

  /// Returns the damping matrix as a function of the Context. This calculation is cached on the estimated states input
  /// port.
  /// @param context the Context of this controller
  const drake::MatrixX<T>& EvalC(const drake::systems::Context<T>& context) const {
    return this->get_cache_entry(C_cache_index_).template Eval<drake::MatrixX<T>>(context);
  }

 protected:
  /// Implements (4) in the class documentation. This particular implementation requires the actuation selection matrix
  /// B to be the identity matrix. This method is made virtual so that more sophisticated implementations can override
  /// it.
  virtual void CalcControlOutput(
      const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const;

  /// Derived classes must implement this function for computing the deformations (virtual or physical) as a function of
  /// the Context.
  /// @see EvalPhi
  virtual drake::VectorX<T> DoCalcPhi(const drake::systems::Context<T>& context) const = 0;

  /// Derived classes must implement this function for computing the deformation rates as a function of the Context.
  /// @see EvalPhiDot
  virtual drake::VectorX<T> DoCalcPhiDot(const drake::systems::Context<T>& context) const = 0;

  /// Derived classes must implement this function for computing the Jacobian matrix G as a function of the Context.
  /// @see EvalG
  virtual drake::MatrixX<T> DoCalcG(const drake::systems::Context<T>& context) const = 0;

  /// Derived classes must implement this function for computing the matrix K as a function of the Context.
  /// @see EvalK
  virtual drake::MatrixX<T> DoCalcK(const drake::systems::Context<T>& context) const = 0;

  /// Derived classes must implement this function for computing the matrix B as a function of the Context.
  /// @see EvalC
  virtual drake::MatrixX<T> DoCalcC(const drake::systems::Context<T>& context) const = 0;

 private:
  void CalcG(const drake::systems::Context<T>& context, drake::MatrixX<T>* output) const {
    const drake::MatrixX<T> G = DoCalcG(context);
    DRAKE_DEMAND(G.cols() == all_plant_.num_velocities());
    DRAKE_ASSERT(G.rows() == EvalPhi(context).size());
    *output = G;
  }

  drake::VectorX<T> CalcPhi(const drake::systems::Context<T>& context) const { return DoCalcPhi(context); }

  drake::VectorX<T> CalcPhiDot(const drake::systems::Context<T>& context) const {
    const drake::VectorX<T> phi_dot = DoCalcPhiDot(context);
    DRAKE_ASSERT(phi_dot.size() == EvalPhi(context).size());
    return phi_dot;
  }

  drake::MatrixX<T> CalcK(const drake::systems::Context<T>& context) const {
    const drake::MatrixX<T> K = DoCalcK(context);
    DRAKE_DEMAND(K.rows() == K.cols());
    DRAKE_ASSERT(K.rows() == EvalPhi(context).size());
    return K;
  }

  drake::MatrixX<T> CalcC(const drake::systems::Context<T>& context) const {
    const drake::MatrixX<T> C = DoCalcC(context);
    DRAKE_DEMAND(C.rows() == C.cols());
    DRAKE_ASSERT(C.rows() == EvalPhi(context).size());
    return C;
  }

  const drake::multibody::MultibodyPlant<T>& all_plant_;
  std::unique_ptr<drake::systems::Context<T>> all_plant_context_;

  // The matrix mapping actuator coordinates to generalized velocities.
  drake::MatrixX<T> B_;

  // Cache indices.
  drake::systems::CacheIndex G_cache_index_{};
  drake::systems::CacheIndex K_cache_index_{};
  drake::systems::CacheIndex C_cache_index_{};
  drake::systems::CacheIndex phi_cache_index_{};
  drake::systems::CacheIndex phi_dot_cache_index_{};

  // Port indices.
  drake::systems::InputPortIndex all_q_estimated_input_port_index_{};
  drake::systems::InputPortIndex all_v_estimated_input_port_index_{};
  drake::systems::InputPortIndex all_vdot_desired_input_port_index_{};
  drake::systems::OutputPortIndex generalized_effort_output_port_index_{};
};

template <typename T>
ImpedanceController<T>::ImpedanceController(
    const drake::multibody::MultibodyPlant<T>* all_plant) : all_plant_(*all_plant) {

  // Create a context for the "all plant".
  all_plant_context_ = all_plant->CreateDefaultContext();

  // Create the actuation matrix.
  B_ = all_plant->MakeActuationMatrix();

  // Verify that B is a binary matrix.
  for (int i = 0; i < B_.rows(); ++i) {
    for (int j = 0; j < B_.cols(); ++j) {
      if (B_(i, j) != 0 && B_(i, j) != 1)
        throw std::logic_error("Actuation matrix is not binary!");
    }
  }

  // Declare ports.
  all_q_estimated_input_port_index_ = this->DeclareVectorInputPort("all_q_estimated",
      drake::systems::BasicVector<T>(all_plant->num_positions())).get_index();
  all_v_estimated_input_port_index_ = this->DeclareVectorInputPort("all_v_estimated",
      drake::systems::BasicVector<T>(all_plant->num_velocities())).get_index();
  all_vdot_desired_input_port_index_ = this->DeclareVectorInputPort("all_vdot",
      drake::systems::BasicVector<T>(all_plant->num_velocities())).get_index();

  // Declare the output port.
  generalized_effort_output_port_index_ = this->DeclareVectorOutputPort("generalized_effort",
      drake::systems::BasicVector<T>(all_plant->num_actuators()),
      &ImpedanceController::CalcControlOutput).get_index();

  // TODO(edrumwri): Declare tickets.
  // Declare cache entries.
  G_cache_index_ = this->DeclareCacheEntry("G", drake::MatrixX<T>(), &ImpedanceController::CalcG).cache_index();
  K_cache_index_ = this->DeclareCacheEntry("K", drake::MatrixX<T>(), &ImpedanceController::CalcK).cache_index();
  C_cache_index_ = this->DeclareCacheEntry("C", drake::MatrixX<T>(), &ImpedanceController::CalcC).cache_index();
  phi_cache_index_ = this->DeclareCacheEntry("phi", drake::VectorX<T>(), &ImpedanceController::CalcPhi).cache_index();
  phi_dot_cache_index_ = this->DeclareCacheEntry(
      "phi_dot", drake::VectorX<T>(), &ImpedanceController::CalcPhiDot).cache_index();
}

template <typename T>
void ImpedanceController<T>::CalcControlOutput(
    const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* u) const {
  // Get necessary quantities.
  const drake::VectorX<T>& phi = EvalPhi(context);
  const drake::VectorX<T>& phi_dot = EvalPhiDot(context);
  const drake::MatrixX<T>& K = EvalK(context);
  const drake::MatrixX<T>& C = EvalC(context);
  const drake::MatrixX<T>& G = EvalG(context);

  // Update the MultibodyPlant context with the state.
  const Eigen::VectorBlock<const drake::VectorX<T>> estimated_q = all_q_estimated_input_port().Eval(context);
  const Eigen::VectorBlock<const drake::VectorX<T>> estimated_v = all_v_estimated_input_port().Eval(context);
  all_plant_.SetPositions(all_plant_context_.get(), estimated_q);
  all_plant_.SetVelocities(all_plant_context_.get(), estimated_v);

  // Get the desired acceleration.
  const Eigen::VectorBlock<const drake::VectorX<T>> vdot_des = all_vdot_desired_input_port().Eval(context);

  // Compute M * vdot_des.
  drake::multibody::MultibodyForces<T> external_forces(all_plant_);
  all_plant_.CalcForceElementsContribution(*all_plant_context_, &external_forces);
  const drake::VectorX<T> M_vdot_des = all_plant_.CalcInverseDynamics(*all_plant_context_, vdot_des, external_forces);

  // TODO(edrumwri): Incorporate M̂Ġv term, which is only considerable when control rates are slow and velocities
  //                 are significant.
  // Compute (4). Note that we are able to use the transpose operation since B is a binary matrix.
  u->SetFromVector(B_.transpose() * (G.transpose() * (K * phi + C * phi_dot) + M_vdot_des));
}

}  // namespace DR

// Instantiate templates.
extern template class DR::ImpedanceController<double>;

