#include <iomanip>
#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/orthonormal_basis.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/vector_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

/*
0. Apply pushes.
1. Add pushes to the vector.
2. Update the initial state. 
3. Fix compile error.
4. Fix Diagram.
5. Fix actuator. 
*/

namespace drake {
namespace systems {

using drake::lcm::DrakeLcm;
using drake::multibody::joints::kFixed;
using Eigen::VectorXd;
using Eigen::Vector3d;
using std::make_unique;

// The pushes that will be executed, in the body frame of the box. 
std::vector<Vector3d> pushes = { Vector3d(;

// Simulation parameters.
DEFINE_int32(num_elements, 100, "The number of elastic elements");
DEFINE_double(timestep, 1e-4, "The time step");
DEFINE_double(k, 1e8, "The spring force applied to each element");
DEFINE_double(b, 1e4, "The damping force applied to each element");
DEFINE_double(force_duration, 1, "The duration for which each force is applied to a part of the box");

namespace {
const char* kBlockUrdf =
    "drake/examples/elastic/block.urdf";
}  // namespace

class ElasticComputation : public VectorSystem<double> {
 public:
  ElasticComputation(RigidBodyTree<double>* tree) {
    tree_ = tree;
  }

 protected:
  void DoCalcVectorOutput(
    const systems::Context<double>& context,
    const Eigen::VectorBlock<const VectorX<double>>& input,
    const Eigen::VectorBlock<const VectorX<double>>&,
    Eigen::VectorBlock<VectorX<double>>* output) const override {

    // The input is the state of the rigid body system at the last iteration.
  }

  // Creates samples over the surface of the box, roughly uniformly.
  void CreateSamples(int samples_per_dim) {
    const double x_len = 2;
    const double y_len = 0.5;
    const double z_len = 1;
    const double x_start = -1;
    const double y_start = -0.25;
    const double z_start = 0.5;
    const double x_end = x_start + x_len;
    const double y_end = y_start + y_len;
    const double z_end = z_start + z_len;
    const double x_inc = x_len / (samples_per_dim - 1);
    const double y_inc = y_len / (samples_per_dim - 1);
    const double z_inc = z_len / (samples_per_dim - 1);

    // Iterate over the top and bottom faces.
    for (int i = 0; i < samples_per_dim; ++i) {
      for (int j = 0; j < samples_per_dim; ++j) {
        samples_.emplace_back(x_start + x_inc * i, y_start + y_inc * j, z_start);
        samples_.emplace_back(x_start + x_inc * i, y_start + y_inc * j, z_end);
      }
    }

    // Iterate over the front and back faces.
    for (int i = 0; i < samples_per_dim; ++i) {
      for (int j = 0; j < samples_per_dim; ++j) {
        samples_.emplace_back(x_start, y_start + y_inc * j, z_start + z_inc * i);
        samples_.emplace_back(x_end, y_start + y_inc * j, z_start + z_inc * i);
      }
    }

    // Iterate over the left and right faces.
    for (int i = 0; i < samples_per_dim; ++i) {
      for (int j = 0; j < samples_per_dim; ++j) {
        samples_.emplace_back(x_start + x_inc * i, y_start, z_start + z_inc * j);
        samples_.emplace_back(x_start + x_inc * i, y_end, z_start + z_inc * j);
      }
    }
  }

  VectorX<double> ComputeContactForce(
      const VectorX<double>& q, const VectorX<double>& v) const {
    // Get the kinematics cache.
    auto kcache = tree_->doKinematics(q, v);

    // Set the force accumulator to zero.
    VectorX<double> f = VectorX<double>::Zero(6);

    // Get the transform.
    const int body_index = 1;
    const auto wTx = kcache.get_element(body_index).transform_to_world;

    for (int i = 0; i < static_cast<int>(samples_.size()); ++i) {
      // Get the point in the world frame.
      const Vector3<double> pw = wTx * samples_[i];

      // If the vertical location is non-negative, proceed to the next sample.
      if (pw[2] >= 0)
        continue;

      // Get the Jacobian matrix for the contact normal at this point.
      const auto J = tree_->
          transformPointsJacobian(kcache, samples_[i], body_index, 0, false);

      // Transform vector from contact frame to world frame.
      const int z_axis = 2;
      const Vector3<double> normal(0, 0, 1);
      const Matrix3<double> R_WC = math::ComputeBasisFromAxis(z_axis, normal);
      const auto N = R_WC.transpose() * J;

      // Get the vertical velocity at this point.
      const Vector3<double> pw_dot = N * kcache.getV(); 

      // Update the force accumulator using force to be applied at this point.
      const double fmag = FLAGS_k*-pw[2] - FLAGS_b*pw_dot[2];
      if (fmag > 0) {
        // Set the relative frame.
        Eigen::Isometry3d xTsample;
        xTsample.translation() = samples_[i];

        // Compute the wrench at the point. 
        const auto Nw = tree_->CalcFrameSpatialVelocityJacobianInWorldFrame(
            kcache, tree_->get_body(body_index), xTsample, false);    

        // Update the force accumulator.
        f += Nw * fmag;  
      }
    }

    return f;
  }

 private:
  std::vector<Vector3<double>> samples_;
  RigidBodyTree<double>* tree_{nullptr};
};

// Simple scenario of two blocks being pushed across a plane.  The first block
// has zero initial velocity.  The second has a small initial velocity in the
// pushing direction.
int main() {
  DiagramBuilder<double> builder;

  // Create RigidBodyTree.
  auto tree_ptr = make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow(kBlockUrdf),
      kFixed, nullptr /* weld to frame */, tree_ptr.get());

  // Instantiate a RigidBodyPlant from the RigidBodyTree.
  auto& plant = *builder.AddSystem<RigidBodyPlant<double>>(move(tree_ptr));
  plant.set_name("plant");

  const auto& tree = plant.get_rigid_body_tree();

  // RigidBodyActuators.
  DRAKE_DEMAND(tree.actuators.size() == 6u);

  // LCM communication.
  DrakeLcm lcm;

/*
  // Pusher
  VectorXd push_value(1);      // Single actuator.
  push_value << FLAGS_push;
  ConstantVectorSource<double>& push_source =
      *builder.template AddSystem<ConstantVectorSource<double>>(push_value);
  push_source.set_name("push_source");

  // NOTE: This is *very* fragile.  It is not obvious that input port 0 is
  // the actuator on the first brick loaded and input port 1 is likewise the
  // actuator for the second brick.
  builder.Connect(push_source.get_output_port(), plant.get_input_port(0));
  builder.Connect(push_source.get_output_port(), plant.get_input_port(1));
*/
  // Visualizer.
  const auto visualizer_publisher =
      builder.template AddSystem<DrakeVisualizer>(tree, &lcm, true);
  visualizer_publisher->set_name("visualizer_publisher");

  // Raw state vector to visualizer.
  builder.Connect(plant.state_output_port(),
                  visualizer_publisher->get_input_port(0));

  auto diagram = builder.Build();

  // Create simulator.
  auto simulator = std::make_unique<Simulator<double>>(*diagram);
  Context<double>& context = simulator->get_mutable_context();
  simulator->reset_integrator<RungeKutta2Integrator<double>>(*diagram,
                                                             FLAGS_timestep,
                                                             &context);
  // Set initial state.
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, &context);
  // 6 1-dof joints = 6 * (x, ẋ)
  const int kStateSize = 12;
  VectorX<double> initial_state(kStateSize);
  initial_state << 0, 0, 0.5, 0, 0, 0,  // block position
                   0, 0, 0, 0, 0, 0,     // block velocity
  plant.set_state_vector(&plant_context, initial_state);

  // Simulate. 
  const double sim_duration = pushes.size() * 1.5 * FLAGS_force_duration;
  simulator->StepTo(sim_duration);

  return 0;
}
}  // namespace systems
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::systems::main();
}
