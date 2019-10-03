#pragma once

#include <string>
#include <vector>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>

#include <DR/common/exception.h>

namespace DR {
/**
 Demultiplexes the total actuation signal for a plant into actuation signals for each model instance.
 */
template <typename T>
class ActuatorDemultiplexer : public drake::systems::LeafSystem<T> {
 public:
  ActuatorDemultiplexer(const ActuatorDemultiplexer<T>&) = default;
  ActuatorDemultiplexer& operator=(const ActuatorDemultiplexer<T>&) = default;
  ActuatorDemultiplexer(ActuatorDemultiplexer<T>&&) = default;
  ActuatorDemultiplexer& operator=(ActuatorDemultiplexer<T>&&) = default;

  explicit ActuatorDemultiplexer(const drake::multibody::MultibodyPlant<T>* plant) {
    plant_actuator_input_port_index_ =
        this->DeclareVectorInputPort("plant_actuator_input", drake::systems::BasicVector<T>(plant->num_actuators()))
            .get_index();
    actuated_model_output_port_index_.resize(plant->num_model_instances());
    for (drake::multibody::ModelInstanceIndex i(0); i < plant->num_model_instances(); ++i) {
      if (plant->num_actuated_dofs(i) > 0) {
        auto port_calc_fn = [this, plant, i](const drake::systems::Context<T>& context,
                                             drake::systems::BasicVector<T>* output) {
          Eigen::VectorBlock<const drake::VectorX<T>> u =
              this->get_input_port(this->plant_actuator_input_port_index_).Eval(context);
          output->SetFromVector(plant->GetActuationFromArray(i, u));
        };
        const std::string port_name = plant->GetModelInstanceName(i) + "_actuator_output";
        actuated_model_output_port_index_[i] =
            this->DeclareVectorOutputPort(port_name, drake::systems::BasicVector<T>(plant->num_actuated_dofs(i)),
                                          port_calc_fn)
                .get_index();
      }
    }
  }

  const drake::systems::InputPort<T>& full_actuation_input_port() const {
    return this->get_input_port(plant_actuator_input_port_index_);
  }

  const drake::systems::OutputPort<T>& actuated_model_output_port(drake::multibody::ModelInstanceIndex model) const {
    // If it's not valid, the model instance has no actuated DOF.
    DR_DEMAND(actuated_model_output_port_index_[model].is_valid());
    return this->get_output_port(actuated_model_output_port_index_[model]);
  }

 private:
  drake::systems::InputPortIndex plant_actuator_input_port_index_{};
  std::vector<drake::systems::OutputPortIndex> actuated_model_output_port_index_{};
};
}  // namespace DR

// Instantiate templates.
extern template class DR::ActuatorDemultiplexer<double>;
