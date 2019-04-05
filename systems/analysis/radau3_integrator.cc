#include "drake/systems/analysis/radau3_integrator.h"
#include "drake/systems/analysis/radau3_integrator-inl.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace systems {
template class Radau3Integrator<double>;
template class Radau3Integrator<AutoDiffXd>;
}  // namespace systems
}  // namespace drake


