#ifndef MPC_TOOLS_ROS_TIMER_HANDLER_H
#define MPC_TOOLS_ROS_TIMER_HANDLER_H

#include <rclcpp/timer.hpp>

namespace Helpers {

class RosTimerHandler
{
public:
    explicit RosTimerHandler(
        const rclcpp::TimerBase::SharedPtr &timer);

    ~RosTimerHandler() = default;

    void start(int max_control_loop_calls = -1);

    void stop();

    bool isRunning();

    void countUp();

    void countUpNotCompleted();

    void resetCount(bool ignore_next_count_up = false);

    void resetFlag();

    int getCountSinceReset();

    int getCountSinceStart();

    int getCountTotal();

    bool getFlag();

private:
    // Timer related
    rclcpp::TimerBase::SharedPtr timer_;
    bool is_running_;

    // Control loop related
    int n_runs_since_reset_;
    int n_runs_since_start_;
    int n_runs_total_;
    bool ignore_next_count_up_;
    int max_control_loop_calls_;
    bool max_control_loop_calls_flag_;
};

}  // namespace Helpers

#endif
