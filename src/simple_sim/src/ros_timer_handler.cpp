#include "ros_timer_handler.h"

#include <cstdint>
#include <stdexcept>

#include <rcl/timer.h>

namespace Helpers
{

RosTimerHandler::RosTimerHandler(
    rclcpp::TimerBase::SharedPtr timer)
    : timer_(std::move(timer)),
      is_running_(false),
      n_runs_since_reset_(0),
      n_runs_since_start_(0),
      n_runs_total_(0),
      ignore_next_count_up_(false),
      max_control_loop_calls_(-1),
      max_control_loop_calls_flag_(false)
{
    timer_->cancel();
}

void RosTimerHandler::start(int max_control_loop_calls)
{
    max_control_loop_calls_ = max_control_loop_calls;
    n_runs_since_start_ = 0;

    timer_->reset();
    is_running_ = true;
}

void RosTimerHandler::stop()
{
    timer_->cancel();
    is_running_ = false;
}

bool RosTimerHandler::isRunning()
{
    return is_running_;
}

void RosTimerHandler::countUp()
{
    if (ignore_next_count_up_)
    {
        ignore_next_count_up_ = false;
        return;
    }

    ++n_runs_since_reset_;
    ++n_runs_since_start_;
    ++n_runs_total_;

    if (
        max_control_loop_calls_ != -1 &&
        n_runs_since_start_ >= max_control_loop_calls_)
    {
        stop();
        max_control_loop_calls_flag_ = true;
    }
}

void RosTimerHandler::resetCount(bool ignore_next_count_up)
{
    n_runs_since_reset_ = 0;

    if (ignore_next_count_up)
    {
        ignore_next_count_up_ = true;
    }
}

void RosTimerHandler::resetFlag()
{
    max_control_loop_calls_flag_ = false;
}

int RosTimerHandler::getCountSinceReset()
{
    return n_runs_since_reset_;
}

int RosTimerHandler::getCountSinceStart()
{
    return n_runs_since_start_;
}

int RosTimerHandler::getCountTotal()
{
    return n_runs_total_;
}

bool RosTimerHandler::getFlag()
{
    return max_control_loop_calls_flag_;
}

void RosTimerHandler::setPeriod(
    const rclcpp::Duration& duration,
    bool reset)
{
    int64_t old_period = 0;

    const rcl_ret_t result =
        rcl_timer_exchange_period(
            timer_->get_timer_handle().get(),
            duration.nanoseconds(),
            &old_period);

    if (result != RCL_RET_OK)
    {
        throw std::runtime_error(
            "Failed to update the simple_sim timer period");
    }

    if (reset)
    {
        timer_->reset();
    }
}

}  // namespace Helpers