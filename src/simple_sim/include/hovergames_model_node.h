#ifndef HOVERGAMES_MODEL_NODE_H
#define HOVERGAMES_MODEL_NODE_H

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <simple_sim/msg/drone_hovergames_control.hpp>

#include "rk4.h"
#include "ros_timer_handler.h"

using rk4_state_type_ = std::vector<double>;

template<class StateType>
class HovergamesRK4System : public RK4::System<StateType>
{
public:
    HovergamesRK4System()
    {
        u.resize(4);
        u[0] = 0.0;
        u[1] = 0.0;
        u[2] = 0.0;
        u[3] = 0.0;
    }

    ~HovergamesRK4System() = default;

    std::vector<double> u;

    // x = [x; y; z; vx; vy; vz; phi; theta; psi; thrust]
    void operator()(
        const StateType& x,
        StateType& dxdt,
        double /* t */ )
    {
        const double A = -5.55;
        const double B = 5.55;

        const double A_yaw = -1.773;
        const double B_yaw = 1.773;

        const double A_thrust = -20.0;
        const double B_thrust = 20.0;

        // These equations must match the corresponding model in
        // mpc_solver/scripts/include/dynamics.py.
        dxdt[0] = x[3];
        dxdt[1] = x[4];
        dxdt[2] = x[5];
        dxdt[3] =
            x[9] *
            (
                std::sin(x[6]) * std::sin(x[8]) +
                std::cos(x[6]) * std::sin(x[7]) * std::cos(x[8])
            );
        dxdt[4] =
            x[9] *
            (
                -std::sin(x[6]) * std::cos(x[8]) +
                std::cos(x[6]) * std::sin(x[7]) * std::sin(x[8])
            );
        dxdt[5] =
            x[9] * std::cos(x[6]) * std::cos(x[7]) - g;
        dxdt[6] = A * x[6] + B * u[0];
        dxdt[7] = A * x[7] + B * u[1];
        dxdt[8] = A_yaw * x[8] + B_yaw * u[2];
        dxdt[9] = A_thrust * x[9] + B_thrust * u[3];
    }

private:
    const double g = 9.81;
};

class DroneHovergamesModel
{
public:
    DroneHovergamesModel(
        rclcpp::Node& node,
        std::vector<double> pos_init,
        bool run_event_based,
        double model_dt,
        int n_steps,
        double model_rate,
        bool add_timing_variance);

    ~DroneHovergamesModel();

    void startup();

    void controlCallback(
        simple_sim::msg::DroneHovergamesControl::ConstSharedPtr msg);

    void integrate();
    void publishState();
    void publishStateTimer();
    void publishTransform();

private:
    rclcpp::Node& node_;

    // Model properties
    std::vector<double> pos_init_{0.0, 0.0, 1.0};
    bool run_event_based_ = true;
    double model_dt_ = 0.05;
    int n_steps_ = 1;
    double model_rate_ = 100.0;
    bool add_timing_variance_ = false;

    // Timing
    Helpers::RosTimerHandler timer_;
    std::mt19937 random_generator_{};
    double mean_ = 0.0;
    double stddev_ = 0.0001;
    std::normal_distribution<double> normal_dist_;

    // ROS publishers and subscribers
    rclcpp::Subscription<
        simple_sim::msg::DroneHovergamesControl>::SharedPtr control_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;

    // ROS transform publisher
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // RK4 integration
    const double t_cur_ = 0.0;
    double t_final_ = n_steps_ * model_dt_;
    rk4_state_type_ x_cur_;
    HovergamesRK4System<rk4_state_type_> rk4_sys_;
    RK4::Integrator<
        rk4_state_type_,
        HovergamesRK4System<rk4_state_type_>> rk4_integrator_;
    tf2::Quaternion q_;

    // Messages
    nav_msgs::msg::Odometry state_msg_;
    geometry_msgs::msg::TransformStamped transform_msg_;
};

#endif  // HOVERGAMES_MODEL_NODE_H