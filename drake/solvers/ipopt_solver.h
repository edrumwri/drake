#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

class IpoptSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IpoptSolver)

  IpoptSolver() = default;
  ~IpoptSolver() override = default;

  // This solver is implemented in various pieces depending on if
  // Ipopt was available during compilation.
  bool available() const override;

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverType solver_type() const override { return SolverType::kIpopt; }

  std::string SolverName() const override { return "IPOPT"; }
};

}  // namespace solvers
}  // namespace drake
