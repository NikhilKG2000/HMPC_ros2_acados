#ifndef MPC_SOLVER_HOVERGAMES_SOLVER_INTERFACE_H
#define MPC_SOLVER_HOVERGAMES_SOLVER_INTERFACE_H

#include <memory>
#include <vector>

#include <mpc_base/solver_base.h>

std::unique_ptr<SolverBase> returnSolverPointer(int solver_number);

std::vector<std::unique_ptr<SolverBase>> returnSolverPointersVector();

#endif  // MPC_SOLVER_HOVERGAMES_SOLVER_INTERFACE_H
