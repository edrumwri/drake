#include "drake/examples/schunk_wsg/dev/simulated_schunk_wsg_system.h"

#include <utility>

#include "drake/common/find_resource.h"
#include "drake/multibody/dev/time_stepping_rigid_body_plant.h"
#include "drake/multibody/parsers/sdf_parser.h"

namespace drake {

using multibody::joints::kFixed;

namespace examples {
namespace schunk_wsg {

template<typename T>
std::unique_ptr<drake::systems::TimeSteppingRigidBodyPlant<T>>
CreateSimulatedSchunkWsgSystem(double timestep) {
  auto rigid_body_tree = std::make_unique<RigidBodyTree<T>>();
  drake::parsers::sdf::AddModelInstancesFromSdfFile(
      FindResourceOrThrow(
          "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf"),
      kFixed, nullptr /* weld to frame */, rigid_body_tree.get());
  return std::make_unique<systems::TimeSteppingRigidBodyPlant<T>>(
      std::move(rigid_body_tree), timestep);
}

template std::unique_ptr<drake::systems::TimeSteppingRigidBodyPlant<double>>
CreateSimulatedSchunkWsgSystem(double timestep);

}  // namespace schunk_wsg
}  // namespace examples
}  // namespace drake
