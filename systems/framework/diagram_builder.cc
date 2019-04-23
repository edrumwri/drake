#include "drake/systems/framework/diagram_builder.h"

#include "drake/common/drake_variant.h"

namespace drake {
namespace systems {
namespace internal {
namespace {

using EitherPortIndex = variant<InputPortIndex, OutputPortIndex>;
using PortIdentifier = std::pair<const SystemBase*, EitherPortIndex>;

bool is_input_port_index(const EitherPortIndex& either) {
  return either.index() == 0;
}

std::string to_string(const PortIdentifier& port_id) {
  const SystemBase& system = *port_id.first;
  const EitherPortIndex& index = port_id.second;
  return is_input_port_index(index) ?
      system.get_input_port_base(drake::get<0>(index)).GetFullDescription() :
      system.get_output_port_base(drake::get<1>(index)).GetFullDescription();
}

// Helper to do the algebraic loop test. It recursively performs the
// depth-first search on the graph to find cycles.
bool HasCycleRecurse(
    const PortIdentifier& n,
    const std::map<PortIdentifier, std::set<PortIdentifier>>& edges,
    std::set<PortIdentifier>* visited,
    std::vector<PortIdentifier>* stack) {
  DRAKE_ASSERT(visited->count(n) == 0);
  visited->insert(n);

  auto edge_iter = edges.find(n);
  if (edge_iter != edges.end()) {
    DRAKE_ASSERT(std::find(stack->begin(), stack->end(), n) == stack->end());
    stack->push_back(n);
    for (const auto& target : edge_iter->second) {
      if (visited->count(target) == 0 &&
          HasCycleRecurse(target, edges, visited, stack)) {
        return true;
      } else if (std::find(stack->begin(), stack->end(), target) !=
                 stack->end()) {
        return true;
      }
    }
    stack->pop_back();
  }
  return false;
}

}  // namespace

void DiagramBuilderImpl::ThrowIfAlgebraicLoopsExist(
    const std::unordered_set<const SystemBase*>& systems,
    const std::map<
        std::pair<const SystemBase*, InputPortIndex>,
        std::pair<const SystemBase*, OutputPortIndex>>& connection_map) {
  // To discover loops, we will construct a digraph and check it for cycles.

  // The nodes in the digraph are the input and output ports mentioned by the
  // diagram's internal connections.  Ports that are not internally connected
  // cannot participate in a cycle, so we don't include them in the nodes set.
  std::set<PortIdentifier> nodes;

  // The edges in the digraph are a directed "influences" relation: for each
  // `value` in `edges[key]`, the `key` influences `value`.  (This is the
  // opposite of the "depends-on" relation.)
  std::map<PortIdentifier, std::set<PortIdentifier>> edges;

  // Add the diagram's internal connections to the digraph nodes *and* edges.
  // The output port influences the input port.
  for (const auto& item : connection_map) {
    const PortIdentifier input{item.first};
    const PortIdentifier output{item.second};
    nodes.insert(input);
    nodes.insert(output);
    edges[output].insert(input);
  }

  // Add more edges (*not* nodes) based on each System's direct feedthrough.
  // An input port influences an output port iff there is direct feedthrough
  // from that input to that output.  If a feedthrough edge refers to a port
  // not in `nodes`, we omit it because ports that are not connected inside the
  // diagram cannot participate in a cycle.
  for (const auto& system : systems) {
    for (const auto& item : system->GetDirectFeedthroughs()) {
      const PortIdentifier input{system, InputPortIndex{item.first}};
      const PortIdentifier output{system, OutputPortIndex{item.second}};
      if (nodes.count(input) > 0 && nodes.count(output) > 0) {
        edges[input].insert(output);
      }
    }
  }

  static constexpr char kAdvice[] =
      "A System may have conservatively reported that one of its output ports "
      "depends on an input port, making one of the 'is direct-feedthrough to' "
      "lines above spurious.  If that is the case, remove the spurious "
      "dependency as shown in the API documentation for "
      "LeafSystem::DoHasDirectFeedthrough.";

  // Evaluate the graph for cycles.
  std::set<PortIdentifier> visited;
  std::vector<PortIdentifier> stack;
  for (const auto& node : nodes) {
    if (visited.count(node) > 0) {
      continue;
    }
    if (HasCycleRecurse(node, edges, &visited, &stack)) {
      std::stringstream message;
      message << "Reported algebraic loop detected in DiagramBuilder:\n";
      for (const auto& item : stack) {
        message << "  " << to_string(item);
        if (is_input_port_index(item.second)) {
          message << " is direct-feedthrough to\n";
        } else {
          message << " is connected to\n";
        }
      }
      message << "  " << to_string(stack.front()) << "\n";
      message << kAdvice;
      throw std::runtime_error(message.str());
    }
  }
}

}  // namespace internal
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramBuilder)
