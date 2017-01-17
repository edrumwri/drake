#include "drake/multibody/collision/model.h"

#include <iostream>

using Eigen::Isometry3d;
using std::move;
using std::unique_ptr;
using std::vector;

namespace DrakeCollision {

Element* Model::AddElement(std::unique_ptr<Element> element) {
  ElementId id = element->getId();
  const auto& itr = elements.find(id);
  if (itr == elements.end()) {
    elements.insert(make_pair(id, move(element)));
    Element* raw_element = elements[id].get();
    DoAddElement(*raw_element);
    return raw_element;
  }
  throw std::runtime_error(
      "Attempting to add an element with a duplicate"
      "id: " +
      std::to_string(id));
}

bool Model::removeElement(ElementId id) {
  return elements.erase(id) > 0;
}

const Element* Model::FindElement(ElementId id) const {
  auto element_iter = elements.find(id);
  if (element_iter != elements.end()) {
    return element_iter->second.get();
  } else {
    return nullptr;
  }
}

Element* Model::FindMutableElement(ElementId id) {
  auto element_iter = elements.find(id);
  if (element_iter != elements.end()) {
    return element_iter->second.get();
  } else {
    return nullptr;
  }
}

void Model::getTerrainContactPoints(ElementId id0,
                                    Eigen::Matrix3Xd& terrain_points) {
  auto element_iter = elements.find(id0);
  if (element_iter != elements.end()) {
    element_iter->second->getTerrainContactPoints(terrain_points);
  } else {
    terrain_points = Eigen::Matrix3Xd();
  }
}

bool Model::updateElementWorldTransform(ElementId id,
                                        const Isometry3d& T_elem_to_world) {
  auto elem_itr = elements.find(id);
  if (elem_itr != elements.end()) {
    elem_itr->second->updateWorldTransform(
        T_elem_to_world);  // fixme: this is taking T_local_to_world, not
                           // T_elem_to_world.  so this method name is wrong
    return true;
  } else {
    return false;
  }
}

bool Model::transformCollisionFrame(
    const DrakeCollision::ElementId& eid,
    const Eigen::Isometry3d& transform_body_to_joint) {
  auto element = elements.find(eid);
  if (element != elements.end()) {
    element->second->SetLocalTransform(transform_body_to_joint *
                                       element->second->getLocalTransform());
    return true;
  } else {
    return false;
  }
}

bool closestPointsAllToAll(
    const vector<ElementId>& ids_to_check,
    bool use_margins,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<PointPair>& closest_points) {
  return false;
}

bool collisionPointsAllToAll(
    bool use_margins,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<PointPair>& points) {
  return false;
}

bool closestPointsPairwise(
    const vector<ElementIdPair>& id_pairs,
    bool use_margins,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<PointPair>& closest_points) {
  return false;
}

/**
 * A toString for the collision model.
 */
std::ostream& operator<<(std::ostream& os, const Model& model) {
  if (model.elements.size() == 0) {
    os << "No collision elements.";
  } else {
    for (auto it = model.elements.begin(); it != model.elements.end(); ++it)
      os << " ElementID: " << it->first << ":\n"
         << "    - world transform:\n"
         << it->second->getWorldTransform().matrix() << "\n"
         << "    - local transform:\n"
         << it->second->getLocalTransform().matrix() << "\n"
         << "    - has geometry? " << (it->second->hasGeometry() ? "yes" : "no")
         << "\n";
  }
  return os;
}

}  // namespace DrakeCollision
