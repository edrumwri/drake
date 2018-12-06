#pragma once

#include <string>
#include <vector>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/parsing/package_map.h"

namespace drake {
namespace multibody {
namespace parsing {

/// Parses SDF and URDF input files into a MultibodyPlant and (optionally) a
/// SceneGraph.
class Parser final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Parser)

  /// Creates a Parser that adds models to the given plant and (optionally)
  /// scene_graph.
  ///
  /// @param plant A pointer to a mutable MultibodyPlant object to which parsed
  ///   model(s) will be added; `plant->is_finalized()` must remain `false` for
  ///   as long as the @p plant is in used by `this`.
  /// @param scene_graph A pointer to a mutable SceneGraph object used for
  ///   geometry registration (either to model visual or contact geometry).
  ///   May be nullptr.
  explicit Parser(
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr);

  /// Gets a non-mutable reference to the PackageMap used by this parser.
  const PackageMap& package_map() const { return package_map_; }

  /// Gets a mutable reference to the PackageMap used by this parser.
  PackageMap& mutable_package_map() { return package_map_; }

  /// Parses the SDF or URDF file named in @p file_name and adds all of its
  /// model(s) to @p plant.
  ///
  /// SDF files may contain multiple `<model>` elements.  New model instances
  /// will be added to @p plant for each `<model>` tag in the file.
  ///
  /// URDF files contain a single `<robot>` element.  Only a single model
  /// instance will be added to @p plant.
  ///
  /// @param file_name The name of the SDF or URDF file to be parsed.  The file
  ///   type will be inferred from the extension.
  /// @returns The set of model instance indices for the newly added models.
  /// @throws std::exception in case of errors.
  std::vector<ModelInstanceIndex> AddAllModelsFromFile(
      const std::string& file_name);

  /// Parses the SDF or URDF file named in @p file_name and adds one model to
  /// @p plant.  It is an error to call this using an SDF file with more than
  /// one `<model>` element.
  ///
  /// @param file_name The name of the SDF or URDF file to be parsed.  The file
  ///   type will be inferred from the extension.
  /// @param model_name The name given to the newly created instance of this
  ///   model.  If empty, the "name" attribute from the `<model>` or `<robot>`
  ///   tag will be used.
  /// @returns The instance index for the newly added model.
  /// @throws std::exception in case of errors.
  ModelInstanceIndex AddModelFromFile(
      const std::string& file_name,
      const std::string& model_name = {});

 private:
  PackageMap package_map_;
  multibody_plant::MultibodyPlant<double>* const plant_;
  geometry::SceneGraph<double>* const scene_graph_;
};

}  // namespace parsing
}  // namespace multibody
}  // namespace drake

