#include <DR/simulation/model_generator.h>

#include <drake/common/autodiff.h>

namespace DR {
template class ModelGenerator<double>;
template class ModelGenerator<drake::AutoDiffXd>;
}  // namespace DR
