#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/rendering/drake_visualizer_client.h"
#include "drake/systems/rendering/pose_aggregator.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"
#include "drake/systems/rendering/pose_vector.h"

using drake::VectorX;
using drake::multibody::joints::kQuaternion;
using drake::lcm::DrakeLcm;
using drake::multibody::collision::Element;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::Serializer;
using drake::systems::RigidBodyPlant;
using drake::systems::DiagramBuilder;
using drake::systems::rendering::MakeGeometryData;
using drake::systems::rendering::PoseAggregator;
using drake::systems::rendering::PoseBundleToDrawMessage;
using drake::systems::Simulator;
using drake::FindResourceOrThrow;
using drake::parsers::sdf::AddModelInstancesFromSdfFile;

class SingleBodyRigidBodyPlant : public RigidBodyPlant<double> {
 public:
  SingleBodyRigidBodyPlant(std::unique_ptr<const RigidBodyTree<double>> tree,
      double timestep) : RigidBodyPlant<double>(std::move(tree), timestep) {
    // Verify there is only a single rigid body (plus the world).
    DRAKE_DEMAND(get_rigid_body_tree().get_num_bodies() == 2);

    pose_output_port_ =
        &this->DeclareVectorOutputPort(&SingleBodyRigidBodyPlant::CopyPoseOut);
  }

  const drake::systems::OutputPort<double>& pose_output() const {
    return *pose_output_port_;
  }

  private:
    const drake::systems::OutputPort<double>* pose_output_port_;
    void CopyPoseOut(
      const Context<double>& context,
      drake::systems::rendering::PoseVector<double>* pose) const {

      // Get the non-world body.
      const auto& tree = this->get_rigid_body_tree();
      const RigidBody<double>& sole_body = tree.get_body(1);

      // Build a kinematics cache.
      const int nq = this->get_num_positions();
      const int nv = this->get_num_velocities();
      auto x = context.get_discrete_state(0).get_value();
      VectorX<double> q = x.topRows(nq);
      VectorX<double> v = x.bottomRows(nv);
      auto kinematics_cache = tree.doKinematics(q, v);

      // Get the pose of the link.
      auto wTx = tree.CalcBodyPoseInWorldFrame(kinematics_cache, sole_body).
          matrix();
      pose->set_translation(Eigen::Translation<double, 3>(wTx(0,3), wTx(1,3), wTx(2, 3)));
      Eigen::Matrix<double, 3, 3> m = wTx.block<3, 3>(0, 0);
      pose->set_rotation(Eigen::Quaternion<double>(m));
    }
};

// Simulation parameters.
DEFINE_double(sim_duration, 1.0, "Duration of the simulation");
DEFINE_double(dt, 1e-2, "Integration step size");
DEFINE_double(iso_window_length, 0, "Witness isolation step size (0 = "
    "no isolation)");
DEFINE_string(dynamic_obj_fname,
    "drake/multibody/rigid_body_plant/test/box.sdf",
    "SDF filename for the dynamic object");
DEFINE_string(terrain_obj_fname,
    "drake/multibody/rigid_body_plant/test/plane.obj",
    "Filename for the terrain .obj file");

int main(int argc, char* argv[]) {
  // Parse any flags.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();

  // Emit a one-time load message.
  Serializer<drake::lcmt_viewer_load_robot> load_serializer;
  std::vector<uint8_t> message_bytes;

  // TODO(edrumwri): Remove the DRAKE_VIEWER_DRAW, DRAKE_VIEWER_LOAD_ROBOT
  //                 magic strings as soon as they are a named constant within
  //                 Drake (or, even better, remove as much of this
  //                 copy/pasted visualization code when possible).
  // Build the simulation diagram.
  DrakeLcm lcm;
  DiagramBuilder<double> builder;
  PoseAggregator<double>* aggregator =
      builder.template AddSystem<PoseAggregator>();
  PoseBundleToDrawMessage* converter =
      builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem* publisher =
      builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher->set_publish_period(0.01);

  // Read in the dynamic object.
  auto tree = std::make_unique<RigidBodyTree<double>> ();
  AddModelInstancesFromSdfFile(
      FindResourceOrThrow(FLAGS_dynamic_obj_fname),
      kQuaternion, nullptr /* weld to frame */, tree.get());

  // Defines a color called "desert sand" according to htmlcsscolor.com.
  Eigen::Vector4d color;
  color << 0.9297, 0.7930, 0.6758, 1;

  // Get the triangle terrain.
  Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
  T_element_to_link.translation() << 0, 0, 0;
  RigidBody<double>& world = tree->world();
  const DrakeShapes::Mesh ground_mesh_geom("",
      FindResourceOrThrow(FLAGS_terrain_obj_fname));
  world.AddVisualElement(
      DrakeShapes::VisualElement(ground_mesh_geom, T_element_to_link, color));
  tree->addCollisionElement(
      Element(ground_mesh_geom, T_element_to_link, &world),
      world, "terrain");
  tree->compile();

  // Get the sole rigid body.
  const RigidBody<double>& sole_body = tree->get_body(1);

  // Create the plant and add it to the diagram.
  auto rb_plant = builder.template AddSystem<SingleBodyRigidBodyPlant>(
      std::move(tree), FLAGS_dt);
  if (FLAGS_iso_window_length > 0)
    rb_plant->set_witness_time_isolation(FLAGS_iso_window_length);

  // Set the contact material properties.
  const double kYoungsModulus = 1e12;
  const double kDissipation = 0;
  const double kMuStatic = 0, kMuDynamic = 0;
  drake::systems::CompliantMaterial material;
  material.set_youngs_modulus(kYoungsModulus).set_dissipation(kDissipation).
      set_friction(kMuStatic, kMuDynamic);
  rb_plant->set_default_compliant_material(material);

  // Create the load message.
  const auto& visual_elements = sole_body.get_visual_elements();
  drake::lcmt_viewer_load_robot message;
  message.num_links = 1;
  message.link.resize(1);
  message.link.back().name = "rb_plant";
  message.link.back().robot_num = 0;
  message.link.back().num_geom = 1;
  message.link.back().geom.resize(visual_elements.size());
  for (int i = 0; i < visual_elements.size(); ++i)
    message.link.back().geom[i] = MakeGeometryData(visual_elements[i]);

  // Send a load mesage.
  const int message_length = message.getEncodedSize();
  message_bytes.resize(message_length);
  message.encode(message_bytes.data(), 0, message_length);
  lcm.Publish("DRAKE_VIEWER_LOAD_ROBOT", message_bytes.data(),
              message_bytes.size());

  // Set the names of the systems.
  rb_plant->set_name("rb_plant");
  aggregator->set_name("aggregator");
  converter->set_name("converter");

  builder.Connect(rb_plant->pose_output(),
                  aggregator->AddSingleInput("rb_plant", 0));
  builder.Connect(*aggregator, *converter);
  builder.Connect(*converter, *publisher);

  // Instantiate the diagram.
  auto diagram = builder.Build();

  // Make no external forces act on the rod.
  auto context = diagram->CreateDefaultContext();

  // Get the sub-context for the rigid body plant.
  auto& rb_plant_context = diagram->GetMutableSubsystemContext(*rb_plant,
                                                               context.get());

  // Set the initial state.
  VectorX<double> x = rb_plant->get_state_vector(rb_plant_context);
  /*
  x[2] = std::sqrt(2)/2*1.01;
  Eigen::AngleAxis<double> aa(M_PI_4, Eigen::Matrix<double, 3, 1>::UnitX());
  Eigen::Quaterniond q(aa);
  x[3] = q.w();
  x[4] = q.x();
  x[5] = q.y();
  x[6] = q.z();*/
  x[2] = 1e-2;
  rb_plant->set_state_vector(&rb_plant_context.get_mutable_state(), x);

  // Start simulating.
  Simulator<double> simulator(*diagram, std::move(context));
  while (simulator.get_context().get_time() < FLAGS_sim_duration) {
    const double t = simulator.get_context().get_time();
    simulator.StepTo(std::min(t + 1, FLAGS_sim_duration));
  }
}
