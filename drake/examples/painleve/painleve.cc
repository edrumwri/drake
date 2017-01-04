// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/examples/painleve/painleve-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace painleve {

template class Painleve<double>;

// TODO(edrumwri): Enable AutoDiff build when the LCP solver supports
// AutoDiff.

}  // namespace painleve
}  // namespace drake
