#include <cmath>
#include <iostream>
#include <memory>

#include <mpc_solver/hovergames/hovergames_solver_interface.h>

int main()
{
  std::unique_ptr<SolverBase> solver = returnSolverPointer(0);
  if (!solver)
  {
    std::cerr << "Could not create TMPC solver" << std::endl;
    return 1;
  }

  const int status = solver->runSolverStep();
  const Eigen::VectorXd first_input = solver->stage_vars_.block(0, 0, 4, 1);
  const Eigen::Vector3d terminal_position =
    solver->stage_vars_.block(4, solver->n_, 3, 1);

  std::cout << "framework status: " << status << " (1 means success)\n";
  std::cout << "u0: " << first_input.transpose() << '\n';
  std::cout << "terminal position: " << terminal_position.transpose() << '\n';

  if (status != 1)
  {
    std::cerr << "C++ TMPC adapter smoke test failed" << std::endl;
    return 1;
  }
  if (!first_input.allFinite() || !terminal_position.allFinite())
  {
    std::cerr << "C++ TMPC adapter returned non-finite values" << std::endl;
    return 1;
  }
  if (std::abs(first_input[3] - 9.81) > 0.1)
  {
    std::cerr << "Unexpected hover thrust" << std::endl;
    return 1;
  }

  std::cout << "C++ TMPC adapter smoke test passed" << std::endl;
  return 0;
}
