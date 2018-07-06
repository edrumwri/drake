#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::Context;
using systems::BasicVector;
using systems::LeafSystem;


class TargetShifter : public LeafSystem<double> {

 public:
  bool use_cartesian = false;
  TargetShifter();
 private:
  void DoControlCalc(const Context<double>& context, BasicVector<double>* output) const;



};




}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake}