#ifndef MPC_CORE_CONTROLLER
#define MPC_CORE_CONTROLLER

#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <mpc_msgs/msg/mpc_header.hpp>

#include <mpc_base/controller_base.h>
#include <mpc_base/configuration_mpc.h>
#include <mpc_base/solver_base.h>
#include <mpc_base/interface_base.h>
#include <mpc_base/module_handler_base.h>

#include <mpc_tools/data_saver_json.h>
#include <mpc_tools/ROS_timer_handler.h>
#include <mpc_tools/benchmarker.h>

class Controller: public ControllerBase
{
public:
    Controller(
        rclcpp::Node& nh,
        ConfigurationMPC* ptr_config,
        SolverBase* ptr_solver,
        InterfaceBase* ptr_interface,
        ModuleHandlerBase* ptr_module_handler,
        DataSaverJson* ptr_data_saver_json);

    ~Controller();

    void runNode();
    void controlLoopStart();
    void controlLoopEnd();
    void controlLoopEndNotCompleted();
    void controlLoop() override;
    void printInfoAtNoSuccess(int& exit_code);

    void setInterface(InterfaceBase* ptr_interface) override;
    void startTimer(int max_control_loop_calls = -1) override;
    void stopTimer() override;
    bool timerIsRunning() override;
    int getCountSinceReset() override;
    int getCountTotal() override;
    void setMpcHeader(mpc_msgs::msg::MpcHeader& mpc_header) override;
    void resetTimerFlag() override;
    void OnReset() override;
    void OnObjectiveReached() override;
    void OnStateReceived();

    // ROS 2 node
    rclcpp::Node& nh_;

    ConfigurationMPC *ptr_config_;
    SolverBase *ptr_solver_;
    InterfaceBase *ptr_interface_;
    ModuleHandlerBase *ptr_module_handler_;
    DataSaverJson *ptr_data_saver_json_;

    std::string control_loop_profiling_string_;

    // ROS messages
    mpc_msgs::msg::MpcHeader mpc_header_;

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr
        pub_computation_times_;

    sensor_msgs::msg::Temperature computation_time_msg_;

    // Timer wrapper
    Helpers::RosTimerHandler timer_;

    Helpers::Benchmarker optimization_benchmarker_;
    Helpers::Benchmarker control_loop_benchmarker_;
    Helpers::Benchmarker module_benchmarkers_;

    std::mutex mutex_;

    int x_data_position_;
    int y_data_position_;

    bool objective_reached_ = false;
    bool updated_state_received_ = false;
    bool state_received_ = false;

    double prev_x_ = 0.0;
    double prev_y_ = 0.0;
    int exit_code_ = 0;
    int previous_exit_code_ = 0;

    bool first_loop_ = true;
};

#endif