#include <DR/simulation/shape_to_unit_inertia.h>

#include <drake/common/autodiff.h>

namespace DR {
template class ShapeToUnitInertia<double>;
template class ShapeToUnitInertia<drake::AutoDiffXd>;
}  // namespace DR
