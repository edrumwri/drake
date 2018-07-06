#pragma once

#include "drake/examples/iiwa_soccer/source_switcher.h"
#include "drake/examples/iiwa_soccer/nonconstant_abstract_source.h"
#include "drake/examples/iiwa_soccer/spline_trajectory.h"
#include "drake/examples/iiwa_soccer/nonconstant_vector_source.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace examples {
namespace iiwa_soccer {

// Twiddling knobs without compiling everything.
double direction_multiply_target = 0;
double direction_multiply_waypoint = 10;
double waypoint_time = 2;
double target_time = 4;
double target_offset_height = 0.09;


// TODO(mws): Rename to a camel-case, more descriptive name.
// TODO(mws): Document the purpose of this data structure.
struct switches {
  // TODO(mws): Briefly document the purpose of the variables below.
  SourceSwitcher<double> *torque_source_switcher;
  SourceSwitcher<double> *setpt_source_switcher;
  SourceSwitcher<double> *offset_source_switcher;
  NonconstantVectorSource *ball_target_offset;
  NonconstantVectorSource *waypoint_target_offset;

  SplineTrajectories *spline_maker;

  //NonconstantAbstractSource<trajectories::PiecewisePolynomial<double>> *full_dim_traj_receiver;
  RigidBodyTree<double> *tree;
  };

}
}
}