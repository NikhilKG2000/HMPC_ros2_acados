#include "acados/acados_solver.h"
#include "hmpc_tmpc_settings_metadata.h"
#include "hmpc_pmpc_settings_metadata.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Eigen>

#include <acados/utils/types.h>

#include <mpc_solver/core/helper_functions.h>

namespace
{

constexpr int kN = hmpc_tmpc_settings::kN;
constexpr int kNu = hmpc_tmpc_settings::kNu;
constexpr int kNx = hmpc_tmpc_settings::kNx;

static_assert(kN == HOVERGAMES_TMPC_N,
              "TMPC horizon metadata differs from solver");
static_assert(kNu == HOVERGAMES_TMPC_NU,
              "TMPC input dimensions differ from solver");
static_assert(kNx == HOVERGAMES_TMPC_NX,
              "TMPC state dimensions differ from solver");
constexpr int kNumberOfObjectiveParameters =
  hmpc_tmpc_settings::kNumberOfObjectiveParameters;
constexpr int kNumberOfWeightParameters =
  hmpc_tmpc_settings::kNumberOfWeightParameters;
constexpr int kNumberOfConstraintParameters =
  hmpc_tmpc_settings::kNumberOfConstraintParameters;
constexpr int kNumberOfAcadosParameters =
  hmpc_tmpc_settings::kNumberOfAcadosParameters;


namespace pmpc_data
{

constexpr int kN = hmpc_pmpc_settings::kN;
constexpr int kNu = hmpc_pmpc_settings::kNu;
constexpr int kNx = hmpc_pmpc_settings::kNx;

static_assert(kN == HOVERGAMES_PMPC_N,
              "PMPC horizon metadata differs from solver");
static_assert(kNu == HOVERGAMES_PMPC_NU,
              "PMPC input dimensions differ from solver");
static_assert(kNx == HOVERGAMES_PMPC_NX,
              "PMPC state dimensions differ from solver");
constexpr int kNumberOfObjectiveParameters =
  hmpc_pmpc_settings::kNumberOfObjectiveParameters;
constexpr int kNumberOfWeightParameters =
  hmpc_pmpc_settings::kNumberOfWeightParameters;
constexpr int kNumberOfConstraintParameters =
  hmpc_pmpc_settings::kNumberOfConstraintParameters;
constexpr int kNumberOfAcadosParameters =
  hmpc_pmpc_settings::kNumberOfAcadosParameters;


}  // namespace pmpc_data

}  // namespace

namespace mpc_solver
{

HovergamesTmpcAcadosSolver::HovergamesTmpcAcadosSolver()
  : SolverBase(
      hmpc_tmpc_settings::kDt,
      kN,
      kNx,
      kNu,
      kNx,
      hmpc_tmpc_settings::stageVariableNames(),
      hmpc_tmpc_settings::constantNames(),
      hmpc_tmpc_settings::constantDimensions(),
      hmpc_tmpc_settings::constantSizes(),
      hmpc_tmpc_settings::constantStartIndices(),
      hmpc_tmpc_settings::constantValues(),
      kNumberOfObjectiveParameters + kNumberOfWeightParameters +
        kNumberOfConstraintParameters,
      hmpc_tmpc_settings::allParameterNames(),
      kNumberOfObjectiveParameters,
      hmpc_tmpc_settings::objectiveParameterNames(),
      kNumberOfWeightParameters,
      hmpc_tmpc_settings::weightParameterNames(),
      hmpc_tmpc_settings::kNDiscs,
      kNumberOfConstraintParameters,
      hmpc_tmpc_settings::constraintParameterNames(),
      {"ReferenceTrajectory", "StaticPolyhedronConstraints"})
{
  capsule_ = hovergames_tmpc_acados_create_capsule();
  if (capsule_ == nullptr)
  {
    throw std::runtime_error("Could not allocate HoverGames TMPC acados capsule");
  }

  const int create_status = hovergames_tmpc_acados_create(capsule_);
  if (create_status != 0)
  {
    hovergames_tmpc_acados_free_capsule(capsule_);
    capsule_ = nullptr;
    throw std::runtime_error(
      "Could not create HoverGames TMPC acados solver, status " +
      std::to_string(create_status));
  }

  nlp_config_ = hovergames_tmpc_acados_get_nlp_config(capsule_);
  nlp_dims_ = hovergames_tmpc_acados_get_nlp_dims(capsule_);
  nlp_in_ = hovergames_tmpc_acados_get_nlp_in(capsule_);
  nlp_out_ = hovergames_tmpc_acados_get_nlp_out(capsule_);
  nlp_solver_ = hovergames_tmpc_acados_get_nlp_solver(capsule_);

  initializeSafeHover();
  loadAllParameters();
  loadStatesMatrixInSolver();
}

HovergamesTmpcAcadosSolver::~HovergamesTmpcAcadosSolver()
{
  if (capsule_ != nullptr)
  {
    hovergames_tmpc_acados_free(capsule_);
    hovergames_tmpc_acados_free_capsule(capsule_);
  }
}

int HovergamesTmpcAcadosSolver::constantIndexOrThrow(
  const std::string& constant_name) const
{
  const int index = returnDataPosition(constant_name, constants_names_);
  if (index < 0)
  {
    throw std::out_of_range("Unknown solver constant: " + constant_name);
  }
  return index;
}

bool HovergamesTmpcAcadosSolver::getConstantScalarBool(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  return constants_[constants_start_idc_[index]] != 0.0;
}

int HovergamesTmpcAcadosSolver::getConstantScalarInt(std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  return static_cast<int>(constants_[constants_start_idc_[index]]);
}

double HovergamesTmpcAcadosSolver::getConstantScalarDouble(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  return constants_[constants_start_idc_[index]];
}

Eigen::VectorXd HovergamesTmpcAcadosSolver::getConstantVectorDouble(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  int size_index = 0;
  for (int item = 0; item < index; ++item)
  {
    size_index += constants_ndims_[item] <= 1 ? 1 : 2;
  }
  return constants_.segment(
    constants_start_idc_[index], constants_sizes_[size_index]);
}

Eigen::MatrixXd HovergamesTmpcAcadosSolver::getConstantMatrixDouble(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  int size_index = 0;
  for (int item = 0; item < index; ++item)
  {
    size_index += constants_ndims_[item] <= 1 ? 1 : 2;
  }
  const int rows = constants_sizes_[size_index];
  const int columns = constants_sizes_[size_index + 1];
  return Eigen::Map<const Eigen::MatrixXd>(
    constants_.data() + constants_start_idc_[index], rows, columns);
}

std::vector<int> HovergamesTmpcAcadosSolver::returnDataPositionsState(
  std::vector<std::string> find_names)
{
  return returnDataPositions(std::move(find_names), stage_vars_names_);
}

std::vector<int> HovergamesTmpcAcadosSolver::returnDataPositionsObjective(
  std::vector<std::string> find_names)
{
  return returnDataPositions(std::move(find_names), par_objectives_names_);
}

std::vector<int> HovergamesTmpcAcadosSolver::returnDataPositionsConstrain(
  std::vector<std::string> find_names)
{
  return returnDataPositions(std::move(find_names), par_constraints_names_);
}

void HovergamesTmpcAcadosSolver::initializeSafeHover()
{
  stage_vars_.setZero();
  stage_vars_.row(3).setConstant(9.81);
  stage_vars_.row(kNu + 2).setConstant(1.0);
  stage_vars_.row(kNu + 9).setConstant(9.81);
  last_updated_stage_vars_ = stage_vars_.col(0);

  par_objectives_.setZero();
  par_objectives_.row(3).setConstant(9.81);
  par_objectives_.row(kNu + 2).setConstant(1.0);
  par_objectives_.row(kNu + 9).setConstant(9.81);

  const auto default_weights =
    hmpc_tmpc_settings::defaultWeights();

  par_weights_ = Eigen::Map<const Eigen::VectorXd>(
    default_weights.data(),
    static_cast<Eigen::Index>(default_weights.size()));

  par_constraints_.setZero();
  for (int constraint = 0; constraint < 24; ++constraint)
  {
    par_constraints_.row(3 * constraint).setConstant(1.0);
    par_constraints_.row(3 * constraint + 2).setConstant(100.0);
  }
}

void HovergamesTmpcAcadosSolver::loadStatesMatrixInSolver()
{
  for (int stage = 0; stage <= n_; ++stage)
  {
    Eigen::VectorXd x = stage_vars_.block(kNu, stage, kNx, 1);
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, stage, "x", x.data());

    if (stage < n_)
    {
      Eigen::VectorXd u = stage_vars_.block(0, stage, kNu, 1);
      ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, stage, "u", u.data());
    }
  }

  Eigen::VectorXd initial_state = stage_vars_.block(kNu, 0, kNx, 1);
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", initial_state.data());
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", initial_state.data());
}

void HovergamesTmpcAcadosSolver::updateMeasuredState()
{
  stage_vars_.col(0) = last_updated_stage_vars_;
}

void HovergamesTmpcAcadosSolver::updateMeasuredStateInHorizon()
{
  for (int stage = 0; stage < nbar_; ++stage)
  {
    stage_vars_.col(stage) = last_updated_stage_vars_;
  }
}

void HovergamesTmpcAcadosSolver::shiftHorizonInMatrix(int time_shift)
{
  const int shift = std::max(0, std::min(time_shift, n_));
  const Eigen::MatrixXd previous = stage_vars_;
  for (int stage = 0; stage < nbar_; ++stage)
  {
    stage_vars_.col(stage) = previous.col(std::min(stage + shift, n_));
  }
}

void HovergamesTmpcAcadosSolver::updateAcadosParameters()
{
  std::vector<double> parameters(kNumberOfAcadosParameters, 0.0);
  for (int stage = 0; stage <= n_; ++stage)
  {
    for (int index = 0; index < kNumberOfObjectiveParameters; ++index)
    {
      parameters[index] = par_objectives_(index, stage);
    }
    for (int index = 0; index < kNumberOfConstraintParameters; ++index)
    {
      parameters[kNumberOfObjectiveParameters + index] =
        par_constraints_(index, stage);
    }

    const int status = hovergames_tmpc_acados_update_params(
      capsule_, stage, parameters.data(), kNumberOfAcadosParameters);
    if (status != 0)
    {
      throw std::runtime_error(
        "Updating acados parameters failed at stage " +
        std::to_string(stage) + " with status " + std::to_string(status));
    }
  }
}

void HovergamesTmpcAcadosSolver::loadObjectiveParams()
{
  updateAcadosParameters();
}

void HovergamesTmpcAcadosSolver::loadWeightsParams()
{
  // TMPC Q and R are compiled into the generated acados solver.
}

void HovergamesTmpcAcadosSolver::loadConstrainParams()
{
  updateAcadosParameters();
}

void HovergamesTmpcAcadosSolver::loadAllParameters()
{
  updateAcadosParameters();
}

void HovergamesTmpcAcadosSolver::resetSolver()
{
  if (hovergames_tmpc_acados_reset(capsule_, 1) != 0)
  {
    throw std::runtime_error("Resetting the HoverGames TMPC solver failed");
  }
  const Eigen::VectorXd current_weights = par_weights_;
  initializeSafeHover();
  par_weights_ = current_weights;
  loadAllParameters();
  loadStatesMatrixInSolver();
}

void HovergamesTmpcAcadosSolver::insertPredictedTrajectoryInMatrix()
{
  for (int stage = 0; stage <= n_; ++stage)
  {
    Eigen::VectorXd x(kNx);
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "x", x.data());
    stage_vars_.block(kNu, stage, kNx, 1) = x;

    if (stage < n_)
    {
      Eigen::VectorXd u(kNu);
      ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "u", u.data());
      stage_vars_.block(0, stage, kNu, 1) = u;
    }
  }

  // acados has no control variable at the terminal node. Reuse the final
  // optimized input to keep the legacy [u;x] warm-start matrix complete.
  stage_vars_.block(0, n_, kNu, 1) = stage_vars_.block(0, n_ - 1, kNu, 1);
}

int HovergamesTmpcAcadosSolver::mapAcadosStatus(int acados_status) const
{
  // Translate acados statuses to the original framework's exit codes.
  // Controller::controlLoop retries 0, -6 and -7, but not QP failure (-8).
  switch (acados_status)
  {
    case ACADOS_SUCCESS:
      return 1;
    case ACADOS_NAN_DETECTED:
      return -6;
    case ACADOS_MAXITER:
      return 0;
    case ACADOS_MINSTEP:
      return -7;
    case ACADOS_QP_FAILURE:
      return -8;
    default:
      return -7;
  }
}

int HovergamesTmpcAcadosSolver::runSolverStep()
{
  loadAllParameters();
  loadStatesMatrixInSolver();

  last_acados_status_ = hovergames_tmpc_acados_solve(capsule_);
  if (last_acados_status_ != 0)
  {
    std::cerr << "[MPC solver] acados returned status "
              << last_acados_status_ << std::endl;
    return mapAcadosStatus(last_acados_status_);
  }

  insertPredictedTrajectoryInMatrix();
  if (print_solver_data_)
  {
    printSolverData();
  }
  return 1;
}

void HovergamesTmpcAcadosSolver::printSolverData()
{
  double solve_time = std::numeric_limits<double>::quiet_NaN();
  int sqp_iterations = -1;
  ocp_nlp_get(nlp_solver_, "time_tot", &solve_time);
  ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iterations);

  std::cout << "[MPC solver] acados status: " << last_acados_status_ << '\n'
            << "[MPC solver] SQP iterations: " << sqp_iterations << '\n'
            << "[MPC solver] solve time [ms]: " << 1000.0 * solve_time << '\n';
  if (print_solver_output_)
  {
    std::cout << "[MPC solver] [u;x] trajectory:\n" << stage_vars_ << std::endl;
  }
}

HovergamesPmpcAcadosSolver::HovergamesPmpcAcadosSolver()
  : SolverBase(
      hmpc_pmpc_settings::kDt,
      pmpc_data::kN,
      pmpc_data::kNx,
      pmpc_data::kNu,
      pmpc_data::kNx,
      hmpc_pmpc_settings::stageVariableNames(),
      hmpc_pmpc_settings::constantNames(),
      hmpc_pmpc_settings::constantDimensions(),
      hmpc_pmpc_settings::constantSizes(),
      hmpc_pmpc_settings::constantStartIndices(),
      hmpc_pmpc_settings::constantValues(),
      pmpc_data::kNumberOfAcadosParameters,
      hmpc_pmpc_settings::allParameterNames(),
      pmpc_data::kNumberOfObjectiveParameters,
      hmpc_pmpc_settings::objectiveParameterNames(),
      pmpc_data::kNumberOfWeightParameters,
      hmpc_pmpc_settings::weightParameterNames(),
      hmpc_pmpc_settings::kNDiscs,
      pmpc_data::kNumberOfConstraintParameters,
      hmpc_pmpc_settings::constraintParameterNames(),
      {"GoalOriented", "StaticPolyhedronConstraints"})
{
  capsule_ = hovergames_pmpc_acados_create_capsule();
  if (capsule_ == nullptr)
  {
    throw std::runtime_error("Could not allocate HoverGames PMPC acados capsule");
  }

  const int create_status = hovergames_pmpc_acados_create(capsule_);
  if (create_status != 0)
  {
    hovergames_pmpc_acados_free_capsule(capsule_);
    capsule_ = nullptr;
    throw std::runtime_error(
      "Could not create HoverGames PMPC acados solver, status " +
      std::to_string(create_status));
  }

  nlp_config_ = hovergames_pmpc_acados_get_nlp_config(capsule_);
  nlp_dims_ = hovergames_pmpc_acados_get_nlp_dims(capsule_);
  nlp_in_ = hovergames_pmpc_acados_get_nlp_in(capsule_);
  nlp_out_ = hovergames_pmpc_acados_get_nlp_out(capsule_);
  nlp_solver_ = hovergames_pmpc_acados_get_nlp_solver(capsule_);

  initializeSafeHover();
  loadAllParameters();
  loadStatesMatrixInSolver();
}

HovergamesPmpcAcadosSolver::~HovergamesPmpcAcadosSolver()
{
  if (capsule_ != nullptr)
  {
    hovergames_pmpc_acados_free(capsule_);
    hovergames_pmpc_acados_free_capsule(capsule_);
  }
}

int HovergamesPmpcAcadosSolver::constantIndexOrThrow(
  const std::string& constant_name) const
{
  const int index = returnDataPosition(constant_name, constants_names_);
  if (index < 0)
  {
    throw std::out_of_range("Unknown solver constant: " + constant_name);
  }
  return index;
}

bool HovergamesPmpcAcadosSolver::getConstantScalarBool(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  return constants_[constants_start_idc_[index]] != 0.0;
}

int HovergamesPmpcAcadosSolver::getConstantScalarInt(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  return static_cast<int>(constants_[constants_start_idc_[index]]);
}

double HovergamesPmpcAcadosSolver::getConstantScalarDouble(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  return constants_[constants_start_idc_[index]];
}

Eigen::VectorXd HovergamesPmpcAcadosSolver::getConstantVectorDouble(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  int size_index = 0;
  for (int item = 0; item < index; ++item)
  {
    size_index += constants_ndims_[item] <= 1 ? 1 : 2;
  }
  return constants_.segment(
    constants_start_idc_[index], constants_sizes_[size_index]);
}

Eigen::MatrixXd HovergamesPmpcAcadosSolver::getConstantMatrixDouble(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  int size_index = 0;
  for (int item = 0; item < index; ++item)
  {
    size_index += constants_ndims_[item] <= 1 ? 1 : 2;
  }
  const int rows = constants_sizes_[size_index];
  const int columns = constants_sizes_[size_index + 1];
  return Eigen::Map<const Eigen::MatrixXd>(
    constants_.data() + constants_start_idc_[index], rows, columns);
}

std::vector<int> HovergamesPmpcAcadosSolver::returnDataPositionsState(
  std::vector<std::string> find_names)
{
  return returnDataPositions(std::move(find_names), stage_vars_names_);
}

std::vector<int> HovergamesPmpcAcadosSolver::returnDataPositionsObjective(
  std::vector<std::string> find_names)
{
  return returnDataPositions(std::move(find_names), par_objectives_names_);
}

std::vector<int> HovergamesPmpcAcadosSolver::returnDataPositionsConstrain(
  std::vector<std::string> find_names)
{
  return returnDataPositions(std::move(find_names), par_constraints_names_);
}

void HovergamesPmpcAcadosSolver::initializeSafeHover()
{
  stage_vars_.setZero();
  stage_vars_.row(3).setConstant(9.81);
  stage_vars_.row(pmpc_data::kNu + 2).setConstant(1.0);
  stage_vars_.row(pmpc_data::kNu + 9).setConstant(9.81);
  last_updated_stage_vars_ = stage_vars_.col(0);

  par_objectives_.setZero();
  par_objectives_.row(2).setConstant(1.0);

  const auto default_weights =
    hmpc_pmpc_settings::defaultWeights();

  par_weights_ = Eigen::Map<const Eigen::VectorXd>(
    default_weights.data(),
    static_cast<Eigen::Index>(default_weights.size()));

  par_constraints_.setZero();
  for (int constraint = 0; constraint < 24; ++constraint)
  {
    par_constraints_.row(3 * constraint).setConstant(1.0);
    par_constraints_.row(3 * constraint + 2).setConstant(100.0);
  }
}

void HovergamesPmpcAcadosSolver::loadStatesMatrixInSolver()
{
  for (int stage = 0; stage <= n_; ++stage)
  {
    Eigen::VectorXd x =
      stage_vars_.block(pmpc_data::kNu, stage, pmpc_data::kNx, 1);
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, stage, "x", x.data());

    if (stage < n_)
    {
      Eigen::VectorXd u =
        stage_vars_.block(0, stage, pmpc_data::kNu, 1);
      ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, stage, "u", u.data());
    }
  }

  Eigen::VectorXd initial_state =
    stage_vars_.block(pmpc_data::kNu, 0, pmpc_data::kNx, 1);
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", initial_state.data());
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", initial_state.data());
}

void HovergamesPmpcAcadosSolver::updateMeasuredState()
{
  stage_vars_.col(0) = last_updated_stage_vars_;
}

void HovergamesPmpcAcadosSolver::updateMeasuredStateInHorizon()
{
  for (int stage = 0; stage < nbar_; ++stage)
  {
    stage_vars_.col(stage) = last_updated_stage_vars_;
  }
}

void HovergamesPmpcAcadosSolver::shiftHorizonInMatrix(int time_shift)
{
  const int shift = std::max(0, std::min(time_shift, n_));
  const Eigen::MatrixXd previous = stage_vars_;
  for (int stage = 0; stage < nbar_; ++stage)
  {
    stage_vars_.col(stage) = previous.col(std::min(stage + shift, n_));
  }
}

void HovergamesPmpcAcadosSolver::updateAcadosParameters()
{
  std::vector<double> parameters(
    pmpc_data::kNumberOfAcadosParameters, 0.0);
  for (int stage = 0; stage <= n_; ++stage)
  {
    for (int index = 0;
         index < pmpc_data::kNumberOfObjectiveParameters; ++index)
    {
      parameters[index] = par_objectives_(index, stage);
    }
    for (int index = 0; index < pmpc_data::kNumberOfWeightParameters; ++index)
    {
      parameters[pmpc_data::kNumberOfObjectiveParameters + index] =
        par_weights_(index);
    }
    for (int index = 0;
         index < pmpc_data::kNumberOfConstraintParameters; ++index)
    {
      parameters[
        pmpc_data::kNumberOfObjectiveParameters +
        pmpc_data::kNumberOfWeightParameters + index] =
        par_constraints_(index, stage);
    }

    const int status = hovergames_pmpc_acados_update_params(
      capsule_, stage, parameters.data(),
      pmpc_data::kNumberOfAcadosParameters);
    if (status != 0)
    {
      throw std::runtime_error(
        "Updating PMPC acados parameters failed at stage " +
        std::to_string(stage) + " with status " + std::to_string(status));
    }
  }
}

void HovergamesPmpcAcadosSolver::loadObjectiveParams()
{
  updateAcadosParameters();
}

void HovergamesPmpcAcadosSolver::loadWeightsParams()
{
  updateAcadosParameters();
}

void HovergamesPmpcAcadosSolver::loadConstrainParams()
{
  updateAcadosParameters();
}

void HovergamesPmpcAcadosSolver::loadAllParameters()
{
  updateAcadosParameters();
}

void HovergamesPmpcAcadosSolver::resetSolver()
{
  if (hovergames_pmpc_acados_reset(capsule_, 1) != 0)
  {
    throw std::runtime_error("Resetting the HoverGames PMPC solver failed");
  }
  const Eigen::VectorXd current_weights = par_weights_;
  initializeSafeHover();
  par_weights_ = current_weights;
  loadAllParameters();
  loadStatesMatrixInSolver();
}

void HovergamesPmpcAcadosSolver::insertPredictedTrajectoryInMatrix()
{
  for (int stage = 0; stage <= n_; ++stage)
  {
    Eigen::VectorXd x(pmpc_data::kNx);
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "x", x.data());
    stage_vars_.block(pmpc_data::kNu, stage, pmpc_data::kNx, 1) = x;

    if (stage < n_)
    {
      Eigen::VectorXd u(pmpc_data::kNu);
      ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "u", u.data());
      stage_vars_.block(0, stage, pmpc_data::kNu, 1) = u;
    }
  }

  // FORCESPRO had a terminal control variable while acados does not. Its
  // independent optimum is zero command increments and hover thrust.
  stage_vars_.block(0, n_, pmpc_data::kNu, 1).setZero();
  stage_vars_(3, n_) = 9.81;
}

int HovergamesPmpcAcadosSolver::mapAcadosStatus(int acados_status) const
{
  // Translate acados statuses to the original framework's exit codes.
  // Controller::controlLoop retries 0, -6 and -7, but not QP failure (-8).
  switch (acados_status)
  {
    case ACADOS_SUCCESS:
      return 1;
    case ACADOS_NAN_DETECTED:
      return -6;
    case ACADOS_MAXITER:
      return 0;
    case ACADOS_MINSTEP:
      return -7;
    case ACADOS_QP_FAILURE:
      return -8;
    default:
      return -7;
  }
}

int HovergamesPmpcAcadosSolver::runSolverStep()
{
  loadAllParameters();
  loadStatesMatrixInSolver();

  last_acados_status_ = hovergames_pmpc_acados_solve(capsule_);
  if (last_acados_status_ != 0)
  {
    std::cerr << "[PMPC solver] acados returned status "
              << last_acados_status_ << std::endl;
    return mapAcadosStatus(last_acados_status_);
  }

  insertPredictedTrajectoryInMatrix();
  if (print_solver_data_)
  {
    printSolverData();
  }
  return 1;
}

void HovergamesPmpcAcadosSolver::printSolverData()
{
  double solve_time = std::numeric_limits<double>::quiet_NaN();
  int sqp_iterations = -1;
  ocp_nlp_get(nlp_solver_, "time_tot", &solve_time);
  ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iterations);

  std::cout << "[PMPC solver] acados status: " << last_acados_status_ << '\n'
            << "[PMPC solver] SQP iterations: " << sqp_iterations << '\n'
            << "[PMPC solver] solve time [ms]: " << 1000.0 * solve_time
            << '\n';
  if (print_solver_output_)
  {
    std::cout << "[PMPC solver] [u;x] trajectory:\n"
              << stage_vars_ << std::endl;
  }
}

}  // namespace mpc_solver
