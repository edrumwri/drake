#pragma once

#include <string>

#include "drake/geometry/shape_specification.h"
#include "drake/multibody/tree/unit_inertia.h"

namespace DR {

/** Class that turns a Shape into a multibody::UnitInertia<T> representation.
 This reifier has a unit_inertia() member that gets updated for each shape
 reified. The expected workflow would be:

 ```c++
  ShapeToUnitInertia<double> reifier;
  Sphere s(1.25);
  const Shape& ref = static_cast<const Shape&>(s); // accepts any input shape
  ref.Reify(&reifier);
  const drake::multibody::UnitInertia<double>& G_Bcm = reifier.unit_inertia();
 ```

 This will write out a string representation of every geometry registered to
 SceneGraph.  */
template <typename T>
class ShapeToUnitInertia final : public drake::geometry::ShapeReifier {
 public:
  /** @name  Implementation of ShapeReifier interface  */
  //@{

  void ImplementGeometry(const drake::geometry::Sphere& sphere,
                         void* user_data) final {
    unit_inertia_ =
        drake::multibody::UnitInertia<T>::SolidSphere(sphere.get_radius());
  }
  void ImplementGeometry(const drake::geometry::Cylinder& cylinder,
                         void* user_data) final {
    unit_inertia_ = drake::multibody::UnitInertia<T>::SolidCylinder(
        cylinder.get_radius(), cylinder.get_length(),
        drake::Vector3<T>::UnitZ());
  }
  void ImplementGeometry(const drake::geometry::HalfSpace& half_space,
                         void* user_data) final {
    throw std::logic_error("Cannot calculate the inertia of a HalfSpace");
  }
  void ImplementGeometry(const drake::geometry::Box& box,
                         void* user_data) final {
    const drake::Vector3<T>& size = box.size();
    unit_inertia_ =
        drake::multibody::UnitInertia<T>::SolidBox(size[0], size[1], size[2]);
  }
  void ImplementGeometry(const drake::geometry::Mesh& mesh,
                         void* user_data) final {
    throw std::logic_error("Inertia calculation not yet supported for Mesh");
  }
  void ImplementGeometry(const drake::geometry::Convex& convex,
                         void* user_data) final {
    throw std::logic_error(
        "Inertia calculation not yet supported for Convex");
  }

  //@}
  const drake::multibody::UnitInertia<T>& unit_inertia() const {
    DRAKE_DEMAND(unit_inertia_.has_value());
    return unit_inertia_.value();
  }

 private:
  drake::optional<typename drake::multibody::UnitInertia<T>> unit_inertia_{};
};

}  // namespace DR
