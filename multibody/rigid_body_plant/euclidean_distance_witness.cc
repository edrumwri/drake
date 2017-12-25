#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_plant/euclidean_distance_witness.h"
#include "drake/multibody/rigid_body_plant/euclidean_distance_witness-inl.h"

namespace drake {
namespace multibody {

// Template instantiation.
template class EuclideanDistanceWitnessFunction<double>;

}  // namespace examples
}  // namespace drake
