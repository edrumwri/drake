#include "drake/systems/analysis/bogacki_shampine3_integrator.h"
#include "drake/systems/analysis/bogacki_shampine3_integrator-inl.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace systems {
template class BogackiShampine3Integrator<double>;
template class BogackiShampine3Integrator<AutoDiffXd>;
}  // namespace systems
}  // namespace drake
