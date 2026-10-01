#include <cmath>
#include <iostream>
#include <memory>

#include <Eigen/Core>

#include <mpc_solver/hovergames/hovergames_solver_interface.h>

int main()
{
  std::unique_ptr<SolverBase> solver = returnSolverPointer(1);
  if (!solver)
  {
    std::cerr << "PMPC adapter test failed: solver index 1 is null\n";
    return 1;
  }

  if (solver->n_ != 4 || solver->nu_ != 4 || solver->nx_ != 13 ||
      solver->npar_ != 88)
  {
    std::cerr << "PMPC adapter test failed: unexpected dimensions\n";
    return 1;
  }

  if (!solver->getConstantScalarBool("use_input_rates") ||
      !solver->getConstantScalarBool("use_tightened_obstacle_constraints") ||
      solver->getConstantScalarInt("n_states_to_constrain") != 2 ||
      solver->getConstantMatrixDouble("P_delta").rows() != 10 ||
      solver->getConstantMatrixDouble("P_delta").cols() != 10)
  {
    std::cerr << "PMPC adapter test failed: unexpected metadata\n";
    return 1;
  }

  const int status = solver->runSolverStep();
  const Eigen::Vector4d first_input = solver->stage_vars_.block<4, 1>(0, 0);
  const Eigen::Vector3d terminal_position =
    solver->stage_vars_.block<3, 1>(solver->nu_, solver->n_);
  const Eigen::Vector3d terminal_velocity =
    solver->stage_vars_.block<3, 1>(solver->nu_ + 3, solver->n_);

  std::cout << "status: " << status << " (1 means success)\n";
  std::cout << "horizon: N=" << solver->n_ << ", dt=" << solver->dt_
            << ", duration=" << solver->n_ * solver->dt_ << " s\n";
  std::cout << "u0: " << first_input.transpose() << '\n';
  std::cout << "terminal position: " << terminal_position.transpose() << '\n';
  std::cout << "terminal velocity norm: " << terminal_velocity.norm() << '\n';

  if (status != 1 || !first_input.allFinite() ||
      !terminal_position.allFinite() || !terminal_velocity.allFinite())
  {
    std::cerr << "PMPC adapter test failed: solve was unsuccessful\n";
    return 1;
  }
  if (terminal_velocity.norm() > 1.0e-5 ||
      std::abs(solver->stage_vars_(solver->nu_ + 9, solver->n_) - 9.81) >
        1.0e-5)
  {
    std::cerr << "PMPC adapter test failed: terminal state is not steady\n";
    return 1;
  }

  std::cout << "PMPC C++ adapter smoke test passed\n";
  return 0;
}
