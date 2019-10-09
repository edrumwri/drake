#include <DR/common/actuator_demultiplexer.h>
#include <drake/geometry/geometry_visualization.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/demultiplexer.h>

#include <DR/drivers/chopstick/chopstick_config.h>
#include <DR/primitives/primitive_behavior.h>
#include <DR/simulation/model_generator.h>

#include <gtest/gtest.h>

using drake::Vector3;
using drake::VectorX;
using drake::math::RigidTransform;
using drake::math::RotationMatrix;

namespace DR {
namespace {

// Converts the signal from an inverse dynamics controller (which is in velocity space) to the actuation input
// for a MultibodyPlant (which is in actuation space).
class VelocityToActuationConverter : public drake::systems::LeafSystem<double> {
 public:
  explicit VelocityToActuationConverter(const drake::multibody::MultibodyPlant<double>& plant) {
    B_ = plant.MakeActuationMatrix();
    this->DeclareVectorInputPort("velocity_variables", drake::systems::BasicVector<double>(plant.num_velocities()));
    this->DeclareVectorOutputPort("actuation_variables", drake::systems::BasicVector<double>(plant.num_actuators()),
                                  &VelocityToActuationConverter::CalcOutput);
  }

 private:
  void CalcOutput(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* output) const {
    Eigen::VectorBlock<const drake::VectorX<double>> f =
        this->get_input_port(drake::systems::InputPortIndex(0)).Eval(context);
    output->SetFromVector(B_.transpose() * f);
  }

  drake::MatrixX<double> B_;
};

// Tests the actuator demultiplexer by demultiplexing the output of two inverse dynamics controllers into a plant
// containing two chopstick model instances.
GTEST_TEST(ActuatorDemultiplexer, InverseDynamics) {
  // Construct the robot plant.
  drake::systems::DiagramBuilder<double> builder;
  auto& robot_plant = *builder.AddSystem<drake::multibody::MultibodyPlant<double>>();

  // Add the robot models to the plant and finalize it.
  drake::multibody::ModelInstanceIndex robot_left_instance, robot_right_instance;
  std::tie(robot_left_instance, robot_right_instance) = AddChopsticksToMBP(&robot_plant);
  robot_plant.Finalize();

  // Add the inverse dynamics controller using gains around the order of unity.
  const auto kp = drake::VectorX<double>::Ones(robot_plant.num_positions());
  const auto ki = drake::VectorX<double>::Zero(robot_plant.num_positions());
  const auto kd = drake::VectorX<double>::Ones(robot_plant.num_positions()) * 0.1;
  const auto& idyn_controller = *builder.AddSystem<drake::systems::controllers::InverseDynamicsController<double>>(
      robot_plant, kp, ki, kd, true /* has reference acceleration */);

  const auto& demuxer = *builder.AddSystem<ActuatorDemultiplexer<double>>(&robot_plant);
  const auto& v_to_u_converter = *builder.AddSystem<VelocityToActuationConverter>(robot_plant);

  drake::systems::InputPortIndex vdot_desired_port =
      builder.ExportInput(idyn_controller.get_input_port_desired_acceleration());

  // Wire the diagram and build it.
  builder.Connect(robot_plant.get_state_output_port(), idyn_controller.get_input_port_estimated_state());
  builder.Connect(robot_plant.get_state_output_port(), idyn_controller.get_input_port_desired_state());
  builder.Connect(idyn_controller.get_output_port_control(), v_to_u_converter.get_input_port(0));
  builder.Connect(v_to_u_converter.get_output_port(0), demuxer.full_actuation_input_port());
  builder.Connect(demuxer.actuated_model_output_port(robot_left_instance),
                  robot_plant.get_actuation_input_port(robot_left_instance));
  builder.Connect(demuxer.actuated_model_output_port(robot_right_instance),
                  robot_plant.get_actuation_input_port(robot_right_instance));
  std::unique_ptr<drake::systems::Diagram<double>> diagram = builder.Build();

  // Set the robot state to something arbitrary.
  std::unique_ptr<drake::systems::Context<double>> context = diagram->CreateDefaultContext();
  drake::systems::Context<double>& mbp_context = diagram->GetMutableSubsystemContext(robot_plant, context.get());
  drake::VectorX<double> q_and_v(robot_plant.num_positions() + robot_plant.num_velocities());
  for (int i = 0; i < q_and_v.size(); ++i) q_and_v[i] = i + 1;
  robot_plant.SetPositionsAndVelocities(&mbp_context, q_and_v);

  // Set inverse dynamics to yield a zero acceleration.
  context->FixInputPort(vdot_desired_port, drake::VectorX<double>::Zero(robot_plant.num_velocities()));

  // Compute the acceleration and verify that it is indeed zero.
  const drake::systems::ContinuousState<double>& xc = diagram->EvalTimeDerivatives(*context);
  EXPECT_LT(xc.get_generalized_velocity().CopyToVector().norm(), std::numeric_limits<double>::epsilon());
}

}  // namespace
}  // namespace DR
