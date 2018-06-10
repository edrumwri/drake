#include "drake/multibody/constraint/constraint_solver.h"

#include <gflags/gflags.h>

#include <cmath>
#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/solvers/unrevised_lemke_solver.h"
#include "drake/solvers/moby_lcp_solver.h"

using Vector2d = Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using drake::multibody::constraint::ConstraintSolver;
using drake::multibody::constraint::ConstraintVelProblemData;

Eigen::LLT<MatrixXd> lltM;
MatrixXd M, N, F;
VectorXd v, rhs, phi;
drake::solvers::UnrevisedLemkeSolver<double> lemke;
drake::solvers::MobyLCPSolver<double> moby;

// Gets a random double number in the interval [0, 1].
double GetRandomDouble() {
  return static_cast<double>(rand()) / RAND_MAX;
}

void ConstructGeneralizedInertiaMatrix(int nv, double eigenvalue_range) {
  // Construct a random symmetric, positive definite matrix.
  MatrixXd J(nv, nv);
  J.setZero();

  // Compute nv rank-1 updates.
  VectorXd x(nv), y(nv);
  for (int i = 0; i < nv; ++i) {
    for (int j = 0; j < nv; ++j) {
      x[i] = GetRandomDouble() * 2.0 - 1.0;
      y[i] = GetRandomDouble() * 2.0 - 1.0;
    }
    J += x * y.transpose();
  }

  // Compute the singular value decomposition.
  Eigen::JacobiSVD<MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Get the singular values.
  VectorXd sigma = svd.singularValues();

  // Space the eigenvalues so that they lie within the specified range.
  M = svd.matrixU() * 
      Eigen::DiagonalMatrix<double, Eigen::Dynamic, Eigen::Dynamic>(sigma) *
      svd.matrixV().transpose();

  M.setIdentity();
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

void ConstructBaseProblemData(ConstraintVelProblemData<double>* data) {
  const int num_contacts = N.rows();
  const int total_friction_directions = F.rows();

  // Construct mu and r.
  data->mu.resize(num_contacts);
  data->r.resize(num_contacts);
  for (int i = 0; i < num_contacts; ++i) {
    data->mu[i] = GetRandomDouble();
    data->r[i] = 2;
  }

  // NOTE: We keep the bilateral constraints and generic unilateral constraints
  // empty, consistent with [Drumwright and Shell, 2011].

  // Construct normal contact Jacobian operators.
  data->N_mult = [&](const MatrixXd& m) -> MatrixXd {
    return N * m;
  };
  data->N_transpose_mult = [&](const MatrixXd& m) -> MatrixXd {
    return N.transpose() * m;
  };

  // Construct tangential contact Jacobian operators.
  data->F_mult = [&](const MatrixXd& m) -> MatrixXd {
    return F * m;
  };
  data->F_transpose_mult = [&](const MatrixXd& m) -> MatrixXd {
    return F.transpose() * m;
  };

  // Construct kF.
  data->kF.setZero(total_friction_directions);

  // Note: gammaF is not zero in RigidBodyPlant!
  // Construct gammaF and gammaE.
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

bool ConstructAndSolveProblem(double eigenvalue_range) {
  const int num_contacts = rand() % 19 + 2;
  const int nv = rand() % 10 + 6;  // Uniform from [6, 15].
  const int num_friction_dirs_per_contact = 2;
  ConstraintVelProblemData<double> data(nv);

  ConstructGeneralizedInertiaMatrix(nv, eigenvalue_range);
  ConstructGeneralizedVelocityAndRhs(nv);
  ConstructJacobians(num_contacts, num_friction_dirs_per_contact, nv);
  ConstructPhi(num_contacts);
  ConstructBaseProblemData(&data);
  ConstructTimeStepDependentData(&data, 1.0);

  // Construct the base MM and qq.
  MatrixXd MM_base, MM;
  VectorXd qq_base, qq;
  ConstraintSolver<double>::MlcpToLcpData mlcp_to_lcp_data;
  ConstraintSolver<double>::ConstructBaseDiscretizedTimeLCP(
      data, &mlcp_to_lcp_data, &MM_base, &qq_base);

  double dt = 1.0;
  VectorXd a;
  while (dt > std::numeric_limits<double>::epsilon()) {
    // Construct the time-step dependent data.
    ConstructTimeStepDependentData(&data, dt);
    MM = MM_base;
    qq = qq_base;
    ConstraintSolver<double>::UpdateDiscretizedTimeLCP(
        data, dt, &mlcp_to_lcp_data, &a, &MM, &qq);

    // Attempt to solve the linear complementarity problem.
    drake::solvers::UnrevisedLemkeSolver<double>::SolverStatistics stats;
    VectorXd zz;
//    if (lemke.SolveLcpLemke(MM, qq, &zz, &stats))
    if (moby.SolveLcpLemke(MM, qq, &zz))
      return true;
    dt *= 0.125;
  }

  return false;
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();

  // Get the number of trials to run and the eigenvalue range.
  DRAKE_DEMAND(argc == 3);
  const int num_trials = std::atoi(argv[1]);
  const double eigenvalue_range = std::atof(argv[2]);

  // Randomize.
//  srand(time(NULL));

  // Reset the number of successes.
  int num_successes = 0;

  // Run the specified number of trials.
  for (int i = 0; i < num_trials; ++i) {
    // Construct and solve the problem.
    if (ConstructAndSolveProblem(eigenvalue_range)) {
      ++num_successes;
      std::cout << "+" << std::flush;
    } else {
      std::cout << "-" << std::flush;
    }
  }

  std::cout << num_successes << " / " << num_trials << std::endl;
  return 0;
}

