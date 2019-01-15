#include "drake/multibody/plant/contact_surfaces_to_lcm.h"

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/lcmt_contact_surfaces_for_viz.hpp"
#include "drake/systems/framework/value.h"

namespace drake {
namespace multibody {

using systems::Context;
using systems::Value;

template <typename T>
ContactSurfacesToLcmSystem<T>::ContactSurfacesToLcmSystem(
    const MultibodyPlant<T>& plant)
    : systems::LeafSystem<T>() {
  DRAKE_DEMAND(plant.is_finalized());
  const int body_count = plant.num_bodies();

  body_names_.reserve(body_count);
  using std::to_string;
  for (BodyIndex i{0}; i < body_count; ++i) {
    const Body<T>& body = plant.get_body(i);
    body_names_.push_back(body.name() + "(" + to_string(body.model_instance()) +
                          ")");
  }
  this->set_name("ContactSurfacesToLcmSystem");
  // Must be the first declared input port to be compatible with the constexpr
  // declaration of contact_surfaces_input_port_index_.
  this->DeclareAbstractInputPort(
      Value<std::vector<geometry::ContactSurface<T>>>());
  this->DeclareAbstractOutputPort(
      &ContactSurfacesToLcmSystem::CalcLcmContactOutput);
}

template <typename T>
const systems::InputPort<T>&
ContactSurfacesToLcmSystem<T>::get_contact_surfaces_input_port() const {
  return this->get_input_port(contact_surfaces_input_port_index_);
}

template <typename T>
const systems::OutputPort<T>&
ContactSurfacesToLcmSystem<T>::get_lcm_message_output_port() const {
  return this->get_output_port(message_output_port_index_);
}

template <typename T>
void ContactSurfacesToLcmSystem<T>::CalcLcmContactOutput(
    const Context<T>& context, lcmt_contact_surfaces_for_viz* output) const {
  // Get input / output.
  const auto& contact_surfaces =
      this->EvalAbstractInput(context, contact_surfaces_input_port_index_)
          ->template GetValue<std::vector<geometry::ContactSurface<T>>>();
  auto& msg = *output;

  // Time in microseconds.
  msg.timestamp = static_cast<int64_t>(
      ExtractDoubleOrThrow(context.get_time()) * 1e6);
  msg.num_surfaces = contact_surfaces.size();
  msg.contact_surfaces.resize(msg.num_surfaces);

  for (int i = 0; i < static_cast<int>(contact_surfaces.size()); ++i) {
    lcmt_contact_surface_for_viz& surface_msg = msg.contact_surfaces[i];
    surface_msg.timestamp = msg.timestamp;

    const geometry::ContactSurface<T>& contact_surface = contact_surfaces[i];

    // TODO: Fix this.
    /*
    surface_msg.body1_name = body_names_.at(contact_info.bodyA_index());
    surface_msg.body2_name = body_names_.at(contact_info.bodyB_index());
*/

    const auto& triangles = contact_surface.triangles();
    surface_msg.num_triangles = static_cast<int32_t>(triangles.size());
    surface_msg.triangles.resize(surface_msg.num_triangles);

    // Loop through each contact triangle on the contact surface.
    for (int j = 0; j < surface_msg.num_triangles; ++j) {
      lcmt_contact_surface_tri_for_viz& tri_msg = surface_msg.triangles[j];
      tri_msg.timestamp = msg.timestamp;

      // Get the three vertices.  
      const geometry::ContactSurfaceVertex<T>& vA = triangles[j].vertex_A();
      const geometry::ContactSurfaceVertex<T>& vB = triangles[j].vertex_B();
      const geometry::ContactSurfaceVertex<T>& vC = triangles[j].vertex_C();

      auto write_double3 = [](const Vector3<T>& src, double* dest) {
        dest[0] = ExtractDoubleOrThrow(src(0));
        dest[1] = ExtractDoubleOrThrow(src(1));
        dest[2] = ExtractDoubleOrThrow(src(2));
      };

      write_double3(vA.location_w, tri_msg.a);
      write_double3(vB.location_w, tri_msg.b);
      write_double3(vC.location_w, tri_msg.c);
    }
  }
}

systems::lcm::LcmPublisherSystem* ConnectContactSurfacesToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    lcm::DrakeLcmInterface* lcm) {
  return ConnectContactSurfacesToDrakeVisualizer(
      builder, multibody_plant,
      multibody_plant.get_contact_surfaces_output_port(), lcm);
}

systems::lcm::LcmPublisherSystem* ConnectContactSurfacesToDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& multibody_plant,
    const systems::OutputPort<double>& contact_surfaces_port,
    lcm::DrakeLcmInterface* lcm) {
  DRAKE_DEMAND(builder != nullptr);

  auto contact_to_lcm =
      builder->template AddSystem<ContactSurfacesToLcmSystem<double>>(
          multibody_plant);
  contact_to_lcm->set_name("contact_to_lcm");

  auto contact_surfaces_publisher = builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_surfaces_for_viz>(
          "CONTACT_SURFACES", lcm));
  contact_surfaces_publisher->set_name("contact_surfaces_publisher");

  builder->Connect(contact_surfaces_port, contact_to_lcm->get_input_port(0));
  builder->Connect(contact_to_lcm->get_output_port(0),
                   contact_surfaces_publisher->get_input_port());
  contact_surfaces_publisher->set_publish_period(1 / 60.0);

  return contact_surfaces_publisher;
}

}  // namespace multibody
}  // namespace drake

// This should be kept in sync with the scalars that MultibodyPlant supports.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::ContactSurfacesToLcmSystem)
