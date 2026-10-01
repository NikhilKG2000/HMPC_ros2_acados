#include <mpc_core/functions/error_function.h>

#include <rclcpp/rclcpp.hpp>

void errorPrintingSolver(int exit_code)
{
  const auto logger = rclcpp::get_logger("mpc_solver");

  switch (exit_code)
  {
    case 0:
      RCLCPP_WARN(
          logger,
          "[Solver] Max iterations reached in solver. "
          "Check the original HMPC objective documentation for more information "
          "(exit_code = 0)");
      break;

    case 2:
      RCLCPP_WARN(
          logger,
          "[Solver] Specified timeout is reached (exit_code = 2)");
      break;

    case -4:
      RCLCPP_WARN(
          logger,
          "[Solver] Wrong number of inequalities input to solver "
          "(exit_code = -4)");
      break;

    case -5:
      RCLCPP_WARN(
          logger,
          "[Solver] Error occurred during matrix factorization "
          "(exit_code = -5)");
      break;

    case -6:
      RCLCPP_WARN(
          logger,
          "[Solver] NaN or INF occurred during functions evaluations "
          "(exit_code = -6)");
      break;

    case -7:
      RCLCPP_WARN(
          logger,
          "[Solver] The solver could not proceed. "
          "Check the original HMPC objective documentation for more information "
          "(exit_code = -7)");
      break;

    case -8:
      RCLCPP_WARN(
          logger,
          "[Solver] The internal QP solver could not proceed. "
          "Check the original HMPC objective documentation for more information "
          "(exit_code = -8)");
      break;

    case -10:
      RCLCPP_WARN(
          logger,
          "[Solver] NaN or INF occurred during evaluation of functions "
          "and derivatives. Check the original HMPC documentation for more "
          "details (exit_code = -10)");
      break;

    case -11:
      RCLCPP_WARN(
          logger,
          "[Solver] Invalid values in problem parameters "
          "(exit_code = -11)");
      break;

    case -100:
      RCLCPP_WARN(
          logger,
          "[Solver] License error (exit_code = -100)");
      break;

    default:
      RCLCPP_WARN_STREAM(
          logger,
          "[Solver] Exit code is not known (exit_code = "
              << exit_code << ")");
      break;
  }
}