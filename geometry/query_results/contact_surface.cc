#include "drake/geometry/query_results/contact_surface.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace geometry {

// TODO: Get the line below working.
//DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
//    class ContactSurface);

class ContactSurface<double>;
class ContactSurface<AutoDiffXd>;

}  // namespace systems
}  // namespace drake

