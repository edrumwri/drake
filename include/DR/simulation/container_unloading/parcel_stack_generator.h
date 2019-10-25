#pragma once

#include <algorithm>
#include <map>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <drake/common/random.h>
#include <drake/common/text_logging.h>

#include <DR/common/logging.h>
#include <DR/simulation/config.h>
#include <DR/tools/output_stream_operators.h>
#include <DR/tools/uniform_vector_distribution.h>

namespace DR {
// Currently supported classes of parcels for the unloading task.
enum class ParcelGeometries {
  kBox = 0,
  kMailTote = 1,
  kCarTire = 2,
  kNumTypes = 3,
};

// Stream operator for logging enum type ParcelGeometries.
std::ostream& operator<<(std::ostream& os, ParcelGeometries type) {
  switch (type) {
    case ParcelGeometries::kBox:
      return os << std::string("kBox");
    case ParcelGeometries::kMailTote:
      return os << std::string("kMailTote");
    case ParcelGeometries::kCarTire:
      return os << std::string("kCarTire");
    default:
      return os << std::string("UNKNOWN");
  }
  DRAKE_UNREACHABLE();
}

/**
 A class for generating a non-self intersecting, dense stack of parcels.
 Random variables for parcels in the stack include:
 - Coulomb Friction (static and dynamic)
 - size {x,y,z}
 - relative position to the parcel below {x,y}
 - color {r,g,b}
 - geometries from ParcelGeometries
 - yaw rotation
 - mass in kg
 */
class ParcelStackGenerator {
 public:
  // Types of randomized attributes for parcels:
  enum class AttributeType {
    // Pose distribution parameters {x,y,z}, {yaw}.
    kPoseTranslation,
    kPoseYawRotation,
    // Size distribution parameters {x,y,z}.
    kSize,
    // Color variation parameters {R,G,B,A}. For Box geometries only.
    kColor,
    // Coulomb Friction {static, dynamic}
    kCoulombFriction,
    // Mass {kg}.
    kMass,
  };

  // No copy-construction, or copy-assignment.
  ParcelStackGenerator(const ParcelStackGenerator&) = delete;
  void operator=(const ParcelStackGenerator&) = delete;

  // No move-construction, or move-assignment.
  ParcelStackGenerator(ParcelStackGenerator&&) = delete;
  void operator=(ParcelStackGenerator&&) = delete;

  // Default destructor.
  ~ParcelStackGenerator() = default;

  ParcelStackGenerator() = delete;

  /**
   @param model_directory the absolute path to the directory of mesh models for objects.
   */
  explicit ParcelStackGenerator(const std::string& model_directory) : model_directory_(model_directory) {}

  /**
  Stack parcels vertically while guaranteeing:
  - parcels never intersect.
  - parcels have enough space between them to fit a gripper.
  NOTE: To save some work, if the parcels are not floating, the inside of the stack is not populated with boxes.  The
        boxes not added in this optimization are still counted in the return value.

  @param pose The front center point on the parcel stack.
  @param floating Are bodies in the stack floating or fixed to the environment.
  @param stack_max_size The stack will occupy a region bounded by the intersecting intervals:
         0 < x < stack_max_size[0] ⋂ -0.5*stack_max_size[1] < y < 0.5*stack_max_size[1]
         ⋂ 0 < z < stack_max_size[2]
  @param gripper_affordance The width to add between boxes to permit a gripper to fit between them.
  @param stack_name Name appended to all bodies in the stack.
  @param bodies A pointer to an unordered_set of body configurations.  Bodies generated in this function will be
  inserted into this set.
   */
  std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> FillRegionWithParcels(
      const drake::math::RigidTransform<double>& origin_pose, bool floating,
      const drake::VectorX<double>& stack_max_size, double gripper_affordance, const std::string& stack_name,
      drake::RandomGenerator* random_generator) {
    std::unordered_set<std::unique_ptr<SingleBodyInstanceConfig>> bodies;

    const auto translation_range = std::make_pair(parcel_attribute_param(AttributeType::kPoseTranslation).lb(),
                                                  parcel_attribute_param(AttributeType::kPoseTranslation).ub());
    const auto yaw_range = std::make_pair(parcel_attribute_param(AttributeType::kPoseYawRotation).lb(),
                                          parcel_attribute_param(AttributeType::kPoseYawRotation).ub());
    const auto size_range = std::make_pair(parcel_attribute_param(AttributeType::kSize).lb(),
                                           parcel_attribute_param(AttributeType::kSize).ub());

    // Check pose limitations for this particular stacking algorithm.
    // No Z-translation variation.
    DR_DEMAND(translation_range.first[2] == 0.0 && translation_range.second[2] == 0.0);
    // At least one max-sized package fits in the region.
    if ((stack_max_size - size_range.second).minCoeff() < 0.0) {
      drake::log()->critical(
          "FillRegionWithParcels: One maximum-sized package will not fit in the designated stack volume!");
    }

    drake::log()->debug("FillRegionWithParcels: pose:{}, floating:{}, stack_max_size:{}", StreamToString(origin_pose),
                        floating, stack_max_size.transpose());

    // Generate a bounding box for the maximum reach of the displaced and rotated parcels.  This is the conservative
    // distance from the parcel, a point outside of this area will never intersect with the parcel.
    drake::Vector3<double> parcel_max_size = size_range.second;
    drake::Vector3<double> parcel_max_translation = translation_range.second - translation_range.first;
    double parcel_max_yaw = std::max(yaw_range.first[0], yaw_range.second[0]);
    drake::Vector3<double> parcel_max_coord = parcel_max_size * 0.5;

    // Get the maximum width and length added to the box from rotation.  pi/4 requires the most padding.
    double parcel_max_radius =
        sqrt(parcel_max_coord[0] * parcel_max_coord[0] + parcel_max_coord[1] * parcel_max_coord[1]);
    double max_rotation_padding = 2.0 * std::sin(std::min(parcel_max_yaw, M_PI_4)) * parcel_max_radius;

    // Determine the maximum area that a parcel might occupy---used to guarantee that parcels never intersect.
    // NOTE: *Special Case* gaps added between boxes for gripper "gripper_affordance".
    // TODO(samzapo) adding gripper_affordance should not be necessary, fix as a special case for planner.
    drake::Vector3<double> padded_parcel_max_size{
        2.0 * parcel_max_translation[0] + parcel_max_size[0] + max_rotation_padding,
        2.0 * parcel_max_translation[1] + parcel_max_size[1] + max_rotation_padding + 0.5 * gripper_affordance,
        parcel_max_size[2]};

    // Check at least one max-sized and padded package fits in the region.
    if ((stack_max_size - padded_parcel_max_size).minCoeff() <= 0.0) {
      std::string error_string = fmt::format(
          "FillRegionWithParcels: No packages will fit in the specified region: stack_max_size=[{}] is smaller than "
          "padded_parcel_max_size=[{}]",
          stack_max_size.transpose(), padded_parcel_max_size.transpose());
      drake::log()->critical(error_string);
      throw std::logic_error(error_string);
    }

    // Determine the number of parcel stacks in the x,y footprint.
    int num_parcel_wide = stack_max_size[1] / padded_parcel_max_size[1];
    int num_parcel_deep = stack_max_size[0] / padded_parcel_max_size[0];

    double parcel_fill_width = padded_parcel_max_size[1] * num_parcel_wide;

    for (int deep = 0; deep < num_parcel_deep; ++deep) {
      for (int wide = 0; wide < num_parcel_wide; ++wide) {
        double top_of_stack = 0.0;
        int stack_height = 0;
        while (top_of_stack < stack_max_size[2]) {
          auto body = std::make_unique<SingleBodyInstanceConfig>();
          // Set body name.
          std::string body_type_name = (floating) ? std::string("floating") : std::string("static");
          std::string body_name(body_type_name + "_L" + std::to_string(deep) + "_W" + std::to_string(wide) + "_H" +
                                std::to_string(stack_height++) + stack_name);
          body->set_name(body_name);

          // Evaluate what type of parcel this is.
          ParcelGeometries parcel_geometry_type =
              static_cast<ParcelGeometries>(parcel_type_distribution_(*random_generator));
          drake::Vector3<double> parcel_size = drake::Vector3<double>::Zero();

          // Determine parcel-type-specific params.
          switch (parcel_geometry_type) {
            case ParcelGeometries::kCarTire: {
              // TODO(support mesh bounding boxes): This bounding box is calculated for a `scale = 1` of this mesh.
              body->set_color(0.3, 0.3, 0.3, 1.0);
              parcel_size = drake::Vector3<double>{0.46, 0.46, 0.17};
              std::string absolute_filename(model_directory_ + "/objects/car_tire.obj");
              body->SetMeshGeometry(absolute_filename, 1.0);
              // Meshes need a bounding box for their collision geometry.
              body->set_collision_geometry(drake::geometry::Box(parcel_size[0], parcel_size[1], parcel_size[2]));
              break;
            }
            case ParcelGeometries::kMailTote: {
              // TODO(support mesh bounding boxes): This bounding box is calculated for a `scale = 1` of this mesh.
              body->set_color(0.3, 0.3, 0.3, 1.0);
              parcel_size = drake::Vector3<double>{0.67, 0.39, 0.31};
              std::string absolute_filename(model_directory_ + "/objects/mail_tote.obj");
              body->SetMeshGeometry(absolute_filename, 1.0);
              // Meshes need a bounding box for their collision geometry.
              body->set_collision_geometry(drake::geometry::Box(parcel_size[0], parcel_size[1], parcel_size[2]));
              break;
            }
            case ParcelGeometries::kBox: {
              parcel_size = EvalParcelAttribute(AttributeType::kSize, random_generator);
              body->SetBoxGeometry(parcel_size[0], parcel_size[1], parcel_size[2]);
              drake::Vector4<double> parcel_color = EvalParcelAttribute(AttributeType::kColor, random_generator);
              body->set_color(parcel_color[0], parcel_color[1], parcel_color[2], parcel_color[3]);
              break;
            }
            default: {
              throw std::runtime_error("Unknown ParcelGeometries for body '" + body_name +
                                       "': " + std::to_string(static_cast<int>(parcel_geometry_type)));
            }
          }

          drake::Vector2<double> parcel_friction =
              EvalParcelAttribute(AttributeType::kCoulombFriction, random_generator);
          body->SetCoulombFriction(parcel_friction[0], parcel_friction[1]);

          if (floating) {
            double parcel_mass = EvalParcelAttribute(AttributeType::kMass, random_generator)[0];
            body->set_mass_and_possibly_make_static(parcel_mass);
          } else {
            body->set_mass_and_possibly_make_static(0.0 /* static */);
          }

          // Set parcel orientation & translation.
          // z translation of parcel equal to top of box below it + half height.
          drake::Vector3<double> parcel_pose_translation =
              drake::Vector3<double>{padded_parcel_max_size[0] * (deep + 0.5),
                                     padded_parcel_max_size[1] * (wide + 0.5) - (parcel_fill_width * 0.5),
                                     top_of_stack + (parcel_size[2] * 0.5)} +
              EvalParcelAttribute(AttributeType::kPoseTranslation, random_generator);
          drake::Vector1<double> yaw = EvalParcelAttribute(AttributeType::kPoseYawRotation, random_generator);

          drake::math::RotationMatrix<double> body_rotation =
              drake::math::RollPitchYaw<double>(0.0, 0.0, yaw[0]).ToRotationMatrix();

          drake::math::RigidTransform<double> parcel_pose =
              origin_pose * drake::math::RigidTransform<double>(body_rotation, parcel_pose_translation);
          body->set_pose(parcel_pose);

          // If stack is at max height move on to next stack.
          if (top_of_stack + parcel_size[2] > stack_max_size[2]) {
            drake::log()->debug("name: {}, size: {}, pose: {}, NOT added because it is too tall.", body->name(),
                                parcel_size.transpose(), StreamToString(body->pose()));
            break;
          }

          // Add parcel to stack.
          drake::log()->debug("name: {}, size: {}, pose: {}", body->name(), parcel_size.transpose(),
                              StreamToString(body->pose()));
          bodies.insert(std::move(body));

          // Add height of new parcel to stack.
          top_of_stack += parcel_size[2];
        }
      }
    }
    return bodies;
  }

  void set_parcel_attribute_range(AttributeType attribute_type, const drake::VectorX<double>& lower_bound,
                                  const drake::VectorX<double>& upper_bound) {
    drake::log()->debug("{} has range=[[{}] to [{}]] interval_size=[{}]", attribute_type, lower_bound.transpose(),
                        upper_bound.transpose(), (upper_bound - lower_bound).transpose());
    parcel_attribute_distribution_.at(attribute_type)
        .param(UniformVectorDistribution<double>::param_type(lower_bound, upper_bound));
  }

  UniformVectorDistribution<double>::param_type parcel_attribute_param(AttributeType attribute_type) {
    return parcel_attribute_distribution_.at(attribute_type).param();
  }

  void set_parcel_type_distribution_weights(const std::vector<double>& weights) {
    parcel_type_distribution_.param(std::discrete_distribution<int>::param_type(weights.begin(), weights.end()));
  }

 private:
  drake::VectorX<double> EvalParcelAttribute(AttributeType attribute_type, drake::RandomGenerator* random_generator) {
    return parcel_attribute_distribution_.at(attribute_type)(*random_generator);
  }

  const std::string model_directory_{""};

  // Attribute type -> Distribution
  std::map<AttributeType, UniformVectorDistribution<double>> parcel_attribute_distribution_{
      {AttributeType::kPoseTranslation, UniformVectorDistribution<double>(3)},
      {AttributeType::kPoseYawRotation, UniformVectorDistribution<double>(1)},
      {AttributeType::kSize, UniformVectorDistribution<double>(3)},
      {AttributeType::kColor, UniformVectorDistribution<double>(4)},
      {AttributeType::kCoulombFriction, UniformVectorDistribution<double>(2)},
      {AttributeType::kMass, UniformVectorDistribution<double>(1)},
  };

  // Type
  std::discrete_distribution<int> parcel_type_distribution_;
};  // namespace DR

std::ostream& operator<<(std::ostream& os, ParcelStackGenerator::AttributeType attribute_type) {
  switch (attribute_type) {
    case ParcelStackGenerator::AttributeType::kPoseTranslation:
      os << "kPoseTranslation";
      break;
    case ParcelStackGenerator::AttributeType::kPoseYawRotation:
      os << "kPoseYawRotation";
      break;
    case ParcelStackGenerator::AttributeType::kSize:
      os << "kSize";
      break;
    case ParcelStackGenerator::AttributeType::kColor:
      os << "kColor";
      break;
    case ParcelStackGenerator::AttributeType::kCoulombFriction:
      os << "kCoulombFriction";
      break;
    case ParcelStackGenerator::AttributeType::kMass:
      os << "kMass";
      break;
    default:
      break;
  }
  return os;
}

}  // namespace DR
