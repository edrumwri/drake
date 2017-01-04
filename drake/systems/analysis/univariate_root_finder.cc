#include "drake/systems/analysis/univariate_root_finder.h"
#include "drake/systems/analysis/univariate_root_finder-inl.h"
//#include <unsupported/Eigen/AutoDiff>

namespace drake {
namespace systems {
template class UnivariateRootFinder<double>;
//template class UnivariateRootFinder<Eigen::AutoDiffScalar<drake::Vector1d>>;
}  // namespace systems
}  // namespace drake

