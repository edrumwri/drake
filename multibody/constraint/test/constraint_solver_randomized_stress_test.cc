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

struct SolverStats {
  double dt{-1};                        // The dt the problem was solved with.
  double constraint_violation{-1};
  int num_pivots{-1};                   // The number of pivots required.
};
std::vector<SolverStats> moby_stats;
std::vector<SolverStats> unrevised_stats;

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
      x[j] = GetRandomDouble() * 2.0 - 1.0;
      y[j] = GetRandomDouble() * 2.0 - 1.0;
    }
    J += x * y.transpose();
  }

  // Compute the singular value decomposition.
  Eigen::JacobiSVD<MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Get the singular values.
  VectorXd sigma = svd.singularValues();

  // Space the eigenvalues so that they lie within the specified range.
  const int n = sigma.size();
  const double alpha = std::exp(std::log(eigenvalue_range) / n);
  sigma[n - 1] = 1.0;
  for (int i = n - 2; i >= 0; --i)
    sigma[i] = sigma[i + 1] / alpha;

  // Form the generalized inertia matrix.
  M = svd.matrixU() * 
      Eigen::DiagonalMatrix<double, Eigen::Dynamic, Eigen::Dynamic>(sigma) *
      svd.matrixV().transpose();
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

  // Construct redundant rows of N.
  const int N_redundant_rows = rand() % nc;
  for (int i = nc - N_redundant_rows; i < nc; ++i) {
      N.row(i).setZero();
      for (int j = 0; j < nc - N_redundant_rows; ++j)
          N.row(i) += N.row(j) * (GetRandomDouble() * 2.0 - 1.0);
  }

  // Construct redundant rows of F.
  const int F_redundant_rows = rand() % nr;
  for (int i = nr - F_redundant_rows; i < nr; ++i) {
        F.row(i).setZero();
        for (int j = 0; j < nr - F_redundant_rows; ++j)
            F.row(i) += F.row(j) * (GetRandomDouble() * 2.0 - 1.0);
    }
}

void ConstructPhi(int num_contacts) {
  // Construct the constraint evaluations.
  phi.resize(num_contacts);
  for (int i = 0; i < num_contacts; ++i)
    phi[i] = GetRandomDouble() * 2.0 - 1.0;
  phi.setZero();
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

void UpdateStats(const MatrixXd& MM, const VectorXd& qq, const VectorXd& zz, double dt, int num_pivots, SolverStats* stats) {
  if (qq.rows() == 0)
    return;
  const VectorXd ww = MM * zz + qq;
  stats->constraint_violation = std::min(0.0, std::min(ww.minCoeff(), std::min(zz.maxCoeff(), zz.dot(ww))));
  stats->dt = dt;
  stats->num_pivots = num_pivots;
}

void Output(int num_successes) {
  switch (num_successes) {
    case 2:
      std::cout << "+";
      break;

    case 1:
    std::cout << "1";
      break;

    case 0:
    std::cout << "0";
      break;

    default:
      DRAKE_ABORT();
  }

  std::cout << std::flush;
}

void OutputStats(const std::vector<SolverStats>& stats) {
  std::cout << "dt:";
  for (size_t i = 0; i < stats.size(); ++i)
    std::cout << " " << stats[i].dt;
  std::cout << std::endl;

  std::cout << "constraint violation:";
  for (size_t i = 0; i < stats.size(); ++i)
    std::cout << " " << stats[i].constraint_violation;
  std::cout << std::endl;

  std::cout << "pivots:";
  for (size_t i = 0; i < stats.size(); ++i)
    std::cout << " " << stats[i].num_pivots;
  std::cout << std::endl;
}

int ConstructAndSolveProblem(double eigenvalue_range) {
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

  // Initialize the number of successes.
  int num_successes = 0;

  // Try solving with Moby first.
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
    VectorXd zz;
    if (moby.SolveLcpLemke(MM, qq, &zz)) {
      moby_stats.push_back(SolverStats());
      UpdateStats(MM, qq, zz, dt, moby.get_num_pivots(), &moby_stats.back());
      ++num_successes;
      break;
    }
    dt *= 0.125;
  }

  // Try solving with unrevised Lemke.
  dt = 1.0;
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
    if (lemke.SolveLcpLemke(MM, qq, &zz, &stats)) {
      unrevised_stats.push_back(SolverStats());
      UpdateStats(MM, qq, zz, dt, stats.num_pivots, &unrevised_stats.back());
      ++num_successes;
      break;
    }
    dt *= 0.125;
  }

  return num_successes;
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

  // Run the specified number of trials.
  std::vector<int> outputs;
  for (int i = 0; i < num_trials; ++i) {
    // Construct and solve the problem.
    outputs.push_back(ConstructAndSolveProblem(eigenvalue_range));
  }

  // We do this separately so that logging does not get in the way.
  std::cout << "Test outputs: ";
  for (int i = 0; i < num_trials; ++i)
    Output(outputs[i]);
  std::cout << std::endl;

  std::cout << "Moby stats:" << std::endl;
  OutputStats(moby_stats);
  std::cout << "-------";
  std::cout << "Lemke stats:" << std::endl;
  OutputStats(unrevised_stats);
  return 0;
}

