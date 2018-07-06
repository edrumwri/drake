#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/event.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/common/eigen_types.h"


namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::LeafSystem;
using systems::Context;
using trajectories::PiecewisePolynomial;
using systems::PublishEvent;
using Eigen::Vector3d;


class SplineTrajectories : public LeafSystem<double> {
 public:
  bool debug_draw = false;
  SplineTrajectories(double start_time, double duration, lcm::DrakeLcm& lcm);

  void set_start_pt(const Vector3d& new_start_pt, double time_to_waypt, double time_to_destination);

  void DoPublish(const Context<double>& context, const std::vector<const PublishEvent<double>*>& events) const;

 private:
  void CalcTrajectory(const Context<double>& context, PiecewisePolynomial<double>* const output) const;
  void PublishFrames(std::vector<Eigen::Isometry3d> poses, std::vector<std::string> name) const;

  mutable double current_time_;
  double start_time_;
  double waypoint_time_ = 100;
  double end_time_ = 200;

  //const bool lock_start_pos_;
  mutable systems::BasicVector<double> current_origin_;
  mutable lcm::DrakeLcm* lcm_;
  mutable PiecewisePolynomial<double> current_trajectory_;
  mutable std::vector<double> breaks_; // The "phase" or "time" along the polynomial.
  mutable std::vector<MatrixX<double>> knots_; // Points we want to go through.

  mutable Vector3d start_pt_; // A safe trajectory starting point until a new one is assigned.

};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake}
