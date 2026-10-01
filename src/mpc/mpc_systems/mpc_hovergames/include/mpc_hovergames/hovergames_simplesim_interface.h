#ifndef HOVERGAMES_SIMPLESIM_INTERFACE_H
#define HOVERGAMES_SIMPLESIM_INTERFACE_H

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <simple_sim/msg/drone_hovergames_control.hpp>

#include <mpc_hovergames/hovergames_drone_interface.h>

class HovergamesSimpleSimInterface
    : public HovergamesDroneInterface
{
public:
    HovergamesSimpleSimInterface(
        rclcpp::Node& node,
        ControllerBase* ptr_controller,
        ConfigurationMPC* ptr_config,
        SolverBase* ptr_solver,
        ModuleHandlerBase* ptr_module_handler,
        RobotRegion* ptr_robot_region,
        DataSaverJson* ptr_data_saver_json,
        std::mutex* mutex);

    ~HovergamesSimpleSimInterface() override = default;

    void OnStateCallback(
        const nav_msgs::msg::Odometry& msg) override;

    void OnComputeActuation() override;
    void OnActuate() override;
    void OnDeployObjectiveReachedStrategy() override;
    void OnDeployEmergencyStrategy() override;
    void OnActuateBrake() override;

private:
    rclcpp::Publisher<
        simple_sim::msg::DroneHovergamesControl>::SharedPtr
        command_pub_0_;

    rclcpp::TimerBase::SharedPtr bootstrap_timer_;

    simple_sim::msg::DroneHovergamesControl control_msg_;
};

#endif  // HOVERGAMES_SIMPLESIM_INTERFACE_H
