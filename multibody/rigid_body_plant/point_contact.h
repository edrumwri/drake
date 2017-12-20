#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

enum class FeatureType {
  kVertex,
  kEdge,
  kFace
};

template <class T>
struct PointContact {
  void* idA;          // The identifier for geometry A.
  void* idB;          // The identifier for geometry B.
  FeatureType typeA;  // The feature in contact on geometry A.
  FeatureType typeB;  // The feature in contact on geometry B.
  void* feature_A_id; // The identifier of the feature on geometry A.
  void* feature_B_id; // The identifier of the feature on geometry B.
};

}  // multibody
}  // drake
