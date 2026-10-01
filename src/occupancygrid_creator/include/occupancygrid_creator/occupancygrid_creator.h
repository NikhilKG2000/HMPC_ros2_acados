#ifndef OCCUPANCYGRID_CREATOR_OCCUPANCYGRID_CREATOR_H
#define OCCUPANCYGRID_CREATOR_OCCUPANCYGRID_CREATOR_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <rclcpp/rclcpp.hpp>

#include <gazebo_msgs/msg/model_states.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <mpc_tools/ros_visuals.h>

class OccupancygridCreator
{
public:
    explicit OccupancygridCreator(rclcpp::Node& node);

    bool loadConfig();

    bool errorFunc(const std::string name);
    void defaultFunc(const std::string name);

    void createMap();

    void createStaticObstacles(
        std::vector<double> x,
        std::vector<double> y,
        std::vector<double> radius);

    void placeObstacleInGrid(
        nav_msgs::msg::OccupancyGrid& gridmap,
        double x_cur,
        double y_cur,
        double radius_cur);

    void placeSquareInImage(
        nav_msgs::msg::OccupancyGrid& gridmap,
        cv::Mat& occupancy_image,
        double x_cur,
        double y_cur,
        double length,
        double width,
        double orientation,
        double line_thickness);

    void placeInflatedSquareInImage(
        nav_msgs::msg::OccupancyGrid& gridmap,
        cv::Mat& occupancy_image,
        double x_cur,
        double y_cur,
        double length,
        double width,
        double orientation,
        double inflation_thickness);

    void drawRoundedRect(
        cv::Mat& image,
        cv::Point center,
        int width,
        int height,
        int cornerRadius,
        double angle,
        cv::Scalar color,
        int thickness);

    void placeLineInImage(
        nav_msgs::msg::OccupancyGrid& gridmap,
        cv::Mat& occupancy_image,
        double x_cur,
        double y_cur,
        double length,
        double orientation,
        double line_thickness);

    void callbackPositionObstacleSquares(
        geometry_msgs::msg::TransformStamped::ConstSharedPtr msg,
        long unsigned int i);

    void callbackPositionObstacleLines(
        geometry_msgs::msg::TransformStamped::ConstSharedPtr msg,
        long unsigned int i);

    void callbackPositionObstacleGazebo(
        gazebo_msgs::msg::ModelStates::ConstSharedPtr msg);

    void publishStaticObstacles();
    void createStaticObstacleVisualizations();
    void publishStaticObstacleVisualizations();

    rclcpp::Node& nh_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid gridmap_;
    std::vector<geometry_msgs::msg::TransformStamped> state_msgs_stored_;
    std::vector<geometry_msgs::msg::TransformStamped> state_msgs_lines_stored_;

    cv::Mat occupancy_image_;

    // Static obstacles
    bool static_obstacle_use;
    std::vector<double> static_circles_x_;
    std::vector<double> static_circles_y_;
    std::vector<double> static_circles_radius_;
    std::vector<double> static_squares_x_;
    std::vector<double> static_squares_y_;
    std::vector<double> static_squares_length_;
    std::vector<double> static_squares_width_;
    std::vector<double> static_squares_orientation_;
    std::vector<double> static_squares_line_thickness_;
    bool inflate_;
    double inflation_thickness_;

    // Dynamic obstacles
    std::vector<double> receiving_obstacle_squares_length_;
    std::vector<double> receiving_obstacle_squares_width_;
    std::vector<double> receiving_obstacle_squares_line_thickness_;
    std::vector<double> receiving_obstacle_squares_orientation_;
    std::vector<double> receiving_obstacle_lines_length_;
    std::vector<double> receiving_obstacle_lines_line_thickness_;

    // Publishers and subscribers
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;

    std::vector<
        rclcpp::Subscription<
            geometry_msgs::msg::TransformStamped>::SharedPtr> subs_vector_;

    std::vector<
        rclcpp::Subscription<
            geometry_msgs::msg::TransformStamped>::SharedPtr>
        subs_vector_lines_;

    rclcpp::Subscription<
        gazebo_msgs::msg::ModelStates>::SharedPtr sub_gazebo_;

    // Received obstacles
    bool receiving_obstacle_position_use_;
    std::vector<double> receiving_obstacle_radius_;
    std::vector<bool> state_received_;
    std::vector<bool> state_received_lines_;

    // Gazebo obstacles
    bool receiving_obstacle_position_gazebo_use_;
    double receiving_obstacle_gazebo_radius_;
    bool state_gazebo_received_;
    gazebo_msgs::msg::ModelStates state_msgs_gazebo_stored_;

    // Recording
    std::string recording_topic_base_ = "/grid/obs/rec/";
    std::string target_frame_ = "map";

    std::vector<
        rclcpp::Publisher<
            geometry_msgs::msg::PoseStamped>::SharedPtr>
        obs_rec_circle_pubs_;

    std::vector<
        rclcpp::Publisher<
            geometry_msgs::msg::PoseStamped>::SharedPtr>
        obs_rec_square_pubs_;

    std::vector<int64_t> static_circle_indices_to_record_;
    std::vector<int64_t> static_square_indices_to_record_;
    std::vector<std::string> static_circle_names_;
    std::vector<std::string> static_square_names_;

    // Visualization
    std::unique_ptr<ROSMarkerPublisher> static_obstacles_marker_pub_;
    std::vector<int64_t> static_circle_indices_to_visualize_;
    std::vector<int64_t> static_square_indices_to_visualize_;
    std::vector<double> marker_color_;
    double marker_z_;
};

#endif