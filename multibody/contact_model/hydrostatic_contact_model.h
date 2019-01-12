#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

template <class T>
class MultibodyPlant;

template <class T>
class HydrostaticContactModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HydrostaticContactModel)
  HydrostaticContactModel(const multibody::MultibodyPlant<T>* plant);

 private:
  multibody::MultibodyPlant<T>* plant_{nullptr};
};

}  // namespace multibody
}  // namespace drake
