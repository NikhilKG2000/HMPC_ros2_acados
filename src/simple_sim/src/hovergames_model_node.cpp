#include "hovergames_model_node.h"

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/create_timer.hpp>

#include "instrumentation_timer.h"

DroneHovergamesModel::DroneHovergamesModel(
    rclcpp::Node& node,
    std::vector<double> pos_init,
    bool run_event_based,
    double model_dt,
    int n_steps,
    double model_rate,
    bool add_timing_variance)
    : node_(node),
      pos_init_(pos_init),
      run_event_based_(run_event_based),
      model_dt_(model_dt),
      n_steps_(n_steps),
      model_rate_(model_rate),
      add_timing_variance_(add_timing_variance),
      timer_(
          rclcpp::create_timer(
              node.get_node_base_interface(),
              node.get_node_timers_interface(),
              node.get_clock(),
              std::chrono::duration<double>(1.0 / model_rate),
              std::bind(
                  &DroneHovergamesModel::publishStateTimer,
                  this))),
      tf_broadcaster_(
          std::make_unique<tf2_ros::TransformBroadcaster>(
              node.shared_from_this()))
{
    RCLCPP_WARN(
        node_.get_logger(),
        "[HMN] Initializing hovergames model node");

    timer_.stop();

    if (!run_event_based_ && add_timing_variance_)
    {
        normal_dist_.param(
            std::normal_distribution<double>::param_type(
                mean_,
                stddev_));
    }

    Helpers::Instrumentor::Get().BeginSession(
        "simple_sim",
        node_.get_name(),
        "simple_sim_hovergames_profiler.json");

    // Data initialization
    rk4_sys_.u.resize(4);
    rk4_sys_.u[0] = 0.0;
    rk4_sys_.u[1] = 0.0;
    rk4_sys_.u[2] = 0.0;
    rk4_sys_.u[3] = 9.81;

    x_cur_.resize(10);
    x_cur_[0] = pos_init_[0];
    x_cur_[1] = pos_init_[1];
    x_cur_[2] = pos_init_[2];
    x_cur_[3] = 0.0;
    x_cur_[4] = 0.0;
    x_cur_[5] = 0.0;
    x_cur_[6] = 0.0;
    x_cur_[7] = 0.0;
    x_cur_[8] = 0.0;
    x_cur_[9] = 9.81;

    state_msg_.header.stamp = node_.now();
    state_msg_.header.frame_id = "map";
    state_msg_.child_frame_id = "base_link";
    state_msg_.pose.pose.position.x = pos_init_[0];
    state_msg_.pose.pose.position.y = pos_init_[1];
    state_msg_.pose.pose.position.z = pos_init_[2];
    state_msg_.pose.pose.orientation.x = 0.0;
    state_msg_.pose.pose.orientation.y = 0.0;
    state_msg_.pose.pose.orientation.z = 0.0;
    state_msg_.pose.pose.orientation.w = 1.0;
    state_msg_.twist.twist.linear.x = 0.0;
    state_msg_.twist.twist.linear.y = 0.0;
    state_msg_.twist.twist.linear.z = 0.0;
    state_msg_.twist.twist.angular.x = 0.0;
    state_msg_.twist.twist.angular.y = 0.0;
    state_msg_.twist.twist.angular.z = 0.0;

    transform_msg_.header.stamp = state_msg_.header.stamp;
    transform_msg_.header.frame_id = state_msg_.header.frame_id;
    transform_msg_.child_frame_id = state_msg_.child_frame_id;
    transform_msg_.transform.translation.x =
        state_msg_.pose.pose.position.x;
    transform_msg_.transform.translation.y =
        state_msg_.pose.pose.position.y;
    transform_msg_.transform.translation.z =
        state_msg_.pose.pose.position.z;
    transform_msg_.transform.rotation =
        state_msg_.pose.pose.orientation;

    control_sub_ =
        node_.create_subscription<
            simple_sim::msg::DroneHovergamesControl>(
            "/drone_hovergames/control",
            1,
            std::bind(
                &DroneHovergamesModel::controlCallback,
                this,
                std::placeholders::_1));

    state_pub_ =
        node_.create_publisher<nav_msgs::msg::Odometry>(
            "/drone_hovergames/state",
            1);

    RCLCPP_INFO_STREAM(
        node_.get_logger(),
        "[HMN] Initial position: ["
            << pos_init_[0] << ", "
            << pos_init_[1] << ", "
            << pos_init_[2] << "]");

    RCLCPP_INFO_STREAM(
        node_.get_logger(),
        "[HMN] Run event based: " << run_event_based_);

    RCLCPP_INFO_STREAM(
        node_.get_logger(),
        "[HMN] Model dt: " << model_dt_);

    RCLCPP_INFO_STREAM(
        node_.get_logger(),
        "[HMN] Model n_steps: " << n_steps_);

    RCLCPP_INFO_STREAM(
        node_.get_logger(),
        "[HMN] Model rate: " << model_rate_);

    RCLCPP_INFO_STREAM(
        node_.get_logger(),
        "[HMN] Add timing variance: "
            << add_timing_variance_);

    t_final_ = n_steps_ * model_dt_;

    startup();

    RCLCPP_WARN(
        node_.get_logger(),
        "[HMN] Initialized hovergames model node");
}

DroneHovergamesModel::~DroneHovergamesModel()
{
    Helpers::Instrumentor::Get().EndSession();
}

void DroneHovergamesModel::startup()
{
    PROFILE_FUNCTION();

    // Let the occupancy grid publish first.
    rclcpp::sleep_for(std::chrono::milliseconds(1100));

    if (run_event_based_)
    {
        publishState();
    }
    else
    {
        timer_.start();
    }
}

void DroneHovergamesModel::controlCallback(
    simple_sim::msg::DroneHovergamesControl::ConstSharedPtr msg)
{
    PROFILE_FUNCTION();

    // Preserve the original HMPC simulator behavior.
    rk4_sys_.u.insert(
        rk4_sys_.u.begin(),
        msg->u.begin(),
        msg->u.end());

    integrate();

    if (run_event_based_)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(2));
        publishState();
    }
}

void DroneHovergamesModel::integrate()
{
    PROFILE_FUNCTION();

    rk4_integrator_.integrate_const(
        rk4_sys_,
        x_cur_,
        t_cur_,
        t_final_,
        model_dt_);

    for (int i = 0; i < 10; ++i)
    {
        x_cur_[i] =
            rk4_integrator_.observer_.x[n_steps_][i];
    }

    q_.setRPY(
        x_cur_[6],
        x_cur_[7],
        x_cur_[8]);

    state_msg_.pose.pose.position.x = x_cur_[0];
    state_msg_.pose.pose.position.y = x_cur_[1];
    state_msg_.pose.pose.position.z = x_cur_[2];

    state_msg_.twist.twist.linear.x = x_cur_[3];
    state_msg_.twist.twist.linear.y = x_cur_[4];
    state_msg_.twist.twist.linear.z = x_cur_[5];

    state_msg_.pose.pose.orientation.x = q_.x();
    state_msg_.pose.pose.orientation.y = q_.y();
    state_msg_.pose.pose.orientation.z = q_.z();
    state_msg_.pose.pose.orientation.w = q_.w();

    transform_msg_.transform.translation.x =
        state_msg_.pose.pose.position.x;
    transform_msg_.transform.translation.y =
        state_msg_.pose.pose.position.y;
    transform_msg_.transform.translation.z =
        state_msg_.pose.pose.position.z;
    transform_msg_.transform.rotation =
        state_msg_.pose.pose.orientation;
}

void DroneHovergamesModel::publishState()
{
    PROFILE_FUNCTION();

    state_msg_.header.stamp = node_.now();
    state_pub_->publish(state_msg_);

    publishTransform();
}

void DroneHovergamesModel::publishStateTimer()
{
    PROFILE_FUNCTION();

    if (add_timing_variance_)
    {
        const double random_dt =
            normal_dist_(random_generator_);

        const double duration =
            1.0 / model_rate_ + random_dt;

        timer_.setPeriod(
            rclcpp::Duration::from_seconds(duration),
            false);
    }

    publishState();
}

void DroneHovergamesModel::publishTransform()
{
    PROFILE_FUNCTION();

    transform_msg_.header.stamp =
        state_msg_.header.stamp;

    tf_broadcaster_->sendTransform(transform_msg_);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<rclcpp::Node>(
            "hovergames_model_sim_node");

    const std::vector<std::string> arguments =
        rclcpp::remove_ros_arguments(argc, argv);

    if (arguments.size() < 9)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Expected 8 arguments: "
            "x_init y_init z_init run_event_based "
            "model_dt n_steps model_rate "
            "add_timing_variance");

        rclcpp::shutdown();
        return -1;
    }

    double x_init = 0.0;
    double y_init = 0.0;
    double z_init = 1.0;

    std::sscanf(
        arguments[1].c_str(),
        "%lf",
        &x_init);

    std::sscanf(
        arguments[2].c_str(),
        "%lf",
        &y_init);

    std::sscanf(
        arguments[3].c_str(),
        "%lf",
        &z_init);

    std::vector<double> pos_init{
        x_init,
        y_init,
        z_init};

    bool run_event_based = true;

    const std::string run_event_based_string =
        arguments[4];

    if (run_event_based_string == "true")
    {
        run_event_based = true;
    }
    else if (run_event_based_string == "false")
    {
        run_event_based = false;
    }
    else
    {
        RCLCPP_ERROR_STREAM(
            node->get_logger(),
            "Invalid argument for run_event_based: "
                << run_event_based_string);

        rclcpp::shutdown();
        return -1;
    }

    double model_dt = 0.05;
    int n_steps = 1;
    double model_rate = 20.0;

    std::sscanf(
        arguments[5].c_str(),
        "%lf",
        &model_dt);

    std::sscanf(
        arguments[6].c_str(),
        "%d",
        &n_steps);

    std::sscanf(
        arguments[7].c_str(),
        "%lf",
        &model_rate);

    bool add_timing_variance = false;

    const std::string add_timing_variance_string =
        arguments[8];

    if (add_timing_variance_string == "true")
    {
        add_timing_variance = true;
    }
    else if (add_timing_variance_string == "false")
    {
        add_timing_variance = false;
    }
    else
    {
        RCLCPP_ERROR_STREAM(
            node->get_logger(),
            "Invalid argument for add_timing_variance: "
                << add_timing_variance_string);

        rclcpp::shutdown();
        return -1;
    }

    // Preserve the original two-second startup delay.
    rclcpp::sleep_for(std::chrono::seconds(2));

    {
        DroneHovergamesModel hovergames_model(
            *node,
            pos_init,
            run_event_based,
            model_dt,
            n_steps,
            model_rate,
            add_timing_variance);

        rclcpp::spin(node);
    }

    rclcpp::shutdown();
    return 0;
}