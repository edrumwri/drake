#include "drake/multibody/constraint/constraint_solver.h"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/solvers/unrevised_lemke_solver.h"

using Vector2d = Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace constraint {
namespace {

Eigen::LLT<MatrixXd> lltM;
MatrixXd M, N, F;
VectorXd v, rhs, phi;

// Gets a random double number in the interval [0, 1].
double GetRandomDouble() {
  return static_cast<double>(rand()) / RAND_MAX;
}

void ConstructGeneralizedInertiaMatrix(int nv, double eigenvalue_range) {
  // Construct a random symmetric, positive definite matrix.
  MatrixXd J(nv, nv);
  J.setZero();

  // Compute nv rank-1 updates.
  VectorX<double> x(nv), y(nv);
  for (int i = 0; i < nv; ++i) {
    for (int j = 0; j < nv; ++j) {
      x[i] = GetRandomDouble() * 2.0 - 1.0;
      y[i] = GetRandomDouble() * 2.0 - 1.0;
    }
    J += x * y.transpose();
  }

  // Compute the singular value decomposition.
  Eigen::JacobiSVD<MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Space the eigenvalues so that they lie within the specified range.
}

void ConstructGeneralizedVelocityAndRhs(int nv) {
  v.resize(nv);
  rhs.resize(nv);
  for (int i = 0; i < v.size(); ++i) {
    v[i] = GetRandomDouble() * 2.0 - 1.0;
    rhs[i] = GetRandomDouble() * 2.0 - 1.0;
  }
}

void ConstructJacobians(int nc, int nr, int nv) {
  N.resize(nc, nv);
  F.resize(nc * nr, nv);
  for (int i = 0; i < nv; ++i) {
    for (int j = 0; j< nc; ++j)
      N(j, i) = GetRandomDouble() * 2.0 - 1.0;
    for (int j = 0; j< nc * nr; ++j)
      F(j, i) = GetRandomDouble() * 2.0 - 1.0;
  }
}

void ConstructPhi(int num_contacts) {
  // Construct the constraint evaluations.
  phi.resize(num_contacts);
  for (int i = 0; i < num_contacts; ++i)
    phi[i] = GetRandomDouble() * 2.0 - 1.0;
}

void ConstructTimeStepDependentData(
    ConstraintVelProblemData<double>* data, double dt) {
  // Set stiffness and damping to arbitrary values.
  const double k = 1e10;
  const double b = 1e5;

  // Construct gammaN.
  const int num_contacts = data->mu.size();
  data->gammaN.resize(num_contacts);
  const double denom = dt * k + b;
  const double cfm = 1.0 / denom;
  const double erp = (dt * k) / denom;
  data->gammaN.setOnes() *= cfm;

  // Construct kN.
  data->kN.resize(num_contacts);
  for (int i = 0; i < num_contacts; ++i)
   data->kN[i] = erp * phi[i] / dt;

  // Update Mv.
  data->Mv = M * v + rhs * dt;

}

void ConstructBaseProblemData(ConstraintVelProblemData<double>* data) {
  const int num_contacts = N.rows();
  const int total_friction_directions = F.rows();

  // Construct mu.
  data->mu.resize(num_contacts);
  for (int i = 0; i < num_contacts; ++i)
    data->mu[i] = GetRandomDouble() * 1.0;

  // NOTE: We keep the bilateral constraints and generic unilateral constraints
  // empty, consistent with [Drumwright and Shell, 2011].

  // Construct normal contact Jacobian operators.
  data->N_mult = [&](const MatrixX<double>& m) -> MatrixX<double> {
    return N * m;
  };
  data->N_transpose_mult = [&](const MatrixX<double>& m) -> MatrixX<double> {
    return N.transpose() * m;
  };

  // Construct tangential contact Jacobian operators.
  data->F_mult = [&](const MatrixX<double>& m) -> MatrixX<double> {
    return F * m;
  };
  data->F_transpose_mult = [&](const MatrixXd& m) -> MatrixX<double> {
    return F.transpose() * m;
  };

  // Construct kF.
  data->kF.setZero(total_friction_directions);

  // Note: gammaF is not zero in RigidBodyPlant!
  // Construct gammaF and gammaN.
  data->gammaF.setZero(total_friction_directions);
  data->gammaE.setZero(num_contacts);

  // Construct Mv.
  data->Mv = M * v;

  // Construct the inertia solve operator.
  lltM = Eigen::LLT<MatrixXd>(M);
  data->solve_inertia = [&](const MatrixXd& m) -> MatrixXd {
    return lltM.solve(m);
  };
}

}  // namespace
}  // namespace constraint
}  // namespace multibody
}  // namespace drake
