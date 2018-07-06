#include "drake/examples/iiwa_soccer/target_shifter.h"


namespace drake {
namespace examples {
namespace iiwa_soccer {

using systems::Context;
using systems::BasicVector;
using systems::LeafSystem;


TargetShifter::TargetShifter() {
  this->DeclareVectorInputPort(BasicVector<double>(3)); // Offset in cylindrical coords.
  this->DeclareVectorInputPort(BasicVector<double>(3)); // Ball position.
  this->DeclareVectorOutputPort(BasicVector<double>(3), &TargetShifter::DoControlCalc); // Output 0.
}

void TargetShifter::DoControlCalc(const Context<double>& context, BasicVector<double>* output) const {
  const systems::BasicVector<double>* offset = this->EvalVectorInput(context, 0);

  if (!use_cartesian) {

    const systems::BasicVector<double> *ball_pos = this->EvalVectorInput(context, 1);
    Vector3<double> ball_vec = ball_pos->CopyToVector();
    Vector3<double> offset_vec = offset->CopyToVector();

    double xy_length =
        ball_vec.cwiseProduct(Vector3<double>(1, 1, 0)).norm(); // Norm of projected vector in the xy plane.
    double cos_th = ball_vec[0] / xy_length;
    double sin_th = ball_vec[1] / xy_length;

    Matrix3<double> rotMat;
    rotMat << cos_th, sin_th, 0,
        -sin_th, cos_th, 0,
        0, 0, 1;

    Vector3<double> t_formed_offset = rotMat * offset_vec; //Now just the offset and not ball pos + offset.

    output->SetFromVector(t_formed_offset); // Rotate the given offset into cylindrical coords.

  }else {
    output->SetFromVector(offset->CopyToVector());
  }
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake}