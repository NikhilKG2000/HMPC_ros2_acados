#ifndef MPC_SOLVER_ACADOS_ACADOS_SOLVER_H
#define MPC_SOLVER_ACADOS_ACADOS_SOLVER_H

#include <string>
#include <vector>

#include <mpc_base/solver_base.h>

extern "C"
{
#include "acados_solver_hovergames_pmpc.h"
#include "acados_solver_hovergames_tmpc.h"
}

namespace mpc_solver
{

class HovergamesTmpcAcadosSolver final : public SolverBase
{
public:
  HovergamesTmpcAcadosSolver();
  ~HovergamesTmpcAcadosSolver() override;

  bool getConstantScalarBool(std::string constant_name) override;
  int getConstantScalarInt(std::string constant_name) override;
  double getConstantScalarDouble(std::string constant_name) override;
  Eigen::VectorXd getConstantVectorDouble(std::string constant_name) override;
  Eigen::MatrixXd getConstantMatrixDouble(std::string constant_name) override;

  std::vector<int> returnDataPositionsState(
    std::vector<std::string> find_names) override;
  std::vector<int> returnDataPositionsObjective(
    std::vector<std::string> find_names) override;
  std::vector<int> returnDataPositionsConstrain(
    std::vector<std::string> find_names) override;

  void loadStatesMatrixInSolver() override;
  void updateMeasuredState() override;
  void updateMeasuredStateInHorizon() override;
  void shiftHorizonInMatrix(int time_shift) override;
  void loadObjectiveParams() override;
  void loadWeightsParams() override;
  void loadConstrainParams() override;
  void loadAllParameters() override;
  void resetSolver() override;
  void insertPredictedTrajectoryInMatrix() override;
  int runSolverStep() override;
  void printSolverData() override;

private:
  void initializeSafeHover();
  void updateAcadosParameters();
  int constantIndexOrThrow(const std::string& constant_name) const;
  int mapAcadosStatus(int acados_status) const;

  hovergames_tmpc_solver_capsule* capsule_{nullptr};
  ocp_nlp_config* nlp_config_{nullptr};
  ocp_nlp_dims* nlp_dims_{nullptr};
  ocp_nlp_in* nlp_in_{nullptr};
  ocp_nlp_out* nlp_out_{nullptr};
  ocp_nlp_solver* nlp_solver_{nullptr};
  int last_acados_status_{0};
};

class HovergamesPmpcAcadosSolver final : public SolverBase
{
public:
  HovergamesPmpcAcadosSolver();
  ~HovergamesPmpcAcadosSolver() override;

  bool getConstantScalarBool(std::string constant_name) override;
  int getConstantScalarInt(std::string constant_name) override;
  double getConstantScalarDouble(std::string constant_name) override;
  Eigen::VectorXd getConstantVectorDouble(std::string constant_name) override;
  Eigen::MatrixXd getConstantMatrixDouble(std::string constant_name) override;

  std::vector<int> returnDataPositionsState(
    std::vector<std::string> find_names) override;
  std::vector<int> returnDataPositionsObjective(
    std::vector<std::string> find_names) override;
  std::vector<int> returnDataPositionsConstrain(
    std::vector<std::string> find_names) override;

  void loadStatesMatrixInSolver() override;
  void updateMeasuredState() override;
  void updateMeasuredStateInHorizon() override;
  void shiftHorizonInMatrix(int time_shift) override;
  void loadObjectiveParams() override;
  void loadWeightsParams() override;
  void loadConstrainParams() override;
  void loadAllParameters() override;
  void resetSolver() override;
  void insertPredictedTrajectoryInMatrix() override;
  int runSolverStep() override;
  void printSolverData() override;

private:
  void initializeSafeHover();
  void updateAcadosParameters();
  int constantIndexOrThrow(const std::string& constant_name) const;
  int mapAcadosStatus(int acados_status) const;

  hovergames_pmpc_solver_capsule* capsule_{nullptr};
  ocp_nlp_config* nlp_config_{nullptr};
  ocp_nlp_dims* nlp_dims_{nullptr};
  ocp_nlp_in* nlp_in_{nullptr};
  ocp_nlp_out* nlp_out_{nullptr};
  ocp_nlp_solver* nlp_solver_{nullptr};
  int last_acados_status_{0};
};

}  // namespace mpc_solver

#endif  // MPC_SOLVER_ACADOS_ACADOS_SOLVER_H
