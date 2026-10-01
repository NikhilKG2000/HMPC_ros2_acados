#ifndef MPC_MODULES_OBJECTIVES_GOAL_ORIENTED
#define MPC_MODULES_OBJECTIVES_GOAL_ORIENTED

#include <deque>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <mpc_msgs/msg/goal_pose.hpp>
#include <mpc_msgs/msg/mpc_header.hpp>

#include <mpc_base/configuration_mpc.h>
#include <mpc_base/solver_base.h>

#include <mpc_tools/data_saver_json.h>
#include <mpc_tools/ros_visuals.h>

#include <mpc_modules/types/module.h>
#include <mpc_modules/types/realtime_data.h>

class GoalOriented : public ControllerModule
{
public:
    GoalOriented(
        std::string name,
        rclcpp::Node& nh,
        ConfigurationMPC* ptr_config,
        SolverBase* ptr_solver,
        DataSaverJson* ptr_data_saver_json);

    bool ObjectiveReached(const RealTimeData& data) override;
    void OnDataReceived(
        RealTimeData& data,
        std::string data_name) override;
    bool ReadyForControl(const RealTimeData& data) override;
    void Update(RealTimeData& data) override;
    void SetParameters(const RealTimeData& data) override;

    void PublishData(
        const mpc_msgs::msg::MpcHeader& mpc_header) override;

    void ExportDataJson(
        const int count_since_start,
        const int count_total) override;

    void CreateVisualizations() override;
    void PublishVisualizations() override;
    void OnReset() override;

    void GoalCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr goal);

private:
    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    geometry_msgs::msg::PoseStamped stored_goal_;

    rclcpp::Publisher<
        mpc_msgs::msg::GoalPose>::SharedPtr goal_position_rec_pub_;

    mpc_msgs::msg::GoalPose goal_position_msg_;

    int x_position_;
    int y_position_;
    int z_position_;
    int yaw_position_;

    int goal_x_position_;
    int goal_y_position_;
    int goal_z_position_;
    int goal_yaw_position_;

    bool received_goal_ = false;
    bool first_check_ = true;

    std::unique_ptr<ROSMarkerPublisher>
        goal_position_marker_pub_;

    std::unique_ptr<ROSMarkerPublisher>
        goal_position_ground_marker_pub_;
};

#endif