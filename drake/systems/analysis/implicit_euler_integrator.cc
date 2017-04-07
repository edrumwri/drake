#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/implicit_euler_integrator-inl.h"
#include <unsupported/Eigen/AutoDiff>

namespace drake {
namespace systems {
template class ImplicitEulerIntegrator<double>;
template class ImplicitEulerIntegrator<Eigen::AutoDiffScalar<Eigen::VectorXd>>;
}  // namespace systems
}  // namespace drake


