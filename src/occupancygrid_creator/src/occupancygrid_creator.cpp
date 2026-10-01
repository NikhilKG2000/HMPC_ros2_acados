#include <occupancygrid_creator/occupancygrid_creator.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{

std::string ros2ParameterName(const std::string& ros1_name)
{
    std::string name = ros1_name;

    while (!name.empty() && name.front() == '/')
    {
        name.erase(name.begin());
    }

    std::replace(name.begin(), name.end(), '/', '.');
    return name;
}

bool hasParameterOverride(
    rclcpp::Node& node,
    const std::string& name)
{
    const auto& overrides =
        node.get_node_parameters_interface()->get_parameter_overrides();

    return overrides.find(name) != overrides.end();
}

template<typename T>
bool getRequiredParameter(
    rclcpp::Node& node,
    const std::string& ros1_name,
    T& value)
{
    const std::string name = ros2ParameterName(ros1_name);

    if (!node.has_parameter(name))
    {
        if (!hasParameterOverride(node, name))
        {
            return false;
        }

        value = node.declare_parameter<T>(name, T{});
        return true;
    }

    return node.get_parameter(name, value);
}

template<typename T>
bool getParameterOrDefault(
    rclcpp::Node& node,
    const std::string& ros1_name,
    T& value,
    const T& default_value)
{
    const std::string name = ros2ParameterName(ros1_name);

    const bool parameter_was_provided =
        node.has_parameter(name) ||
        hasParameterOverride(node, name);

    if (!node.has_parameter(name))
    {
        value = node.declare_parameter<T>(name, default_value);
    }
    else
    {
        node.get_parameter(name, value);
    }

    return parameter_was_provided;
}

}  // namespace

OccupancygridCreator::OccupancygridCreator(rclcpp::Node& node)
    : nh_(node),
      state_gazebo_received_(false)
{
    if (!loadConfig())
    {
        rclcpp::shutdown();
    }
}
bool OccupancygridCreator::loadConfig()
{
    double frequency;
    double map_size_x;
    double map_size_y;
    double map_resolution;
    double map_center_x;
    double map_center_y;
    std::string map_id;
    std::string publish_topic;

    std::vector<std::string> receiving_obstacle_position_circles_topics;
    std::vector<double> receiving_obstacle_circles_radius_;

    std::vector<std::string> receiving_obstacle_position_squares_topics;
    std::vector<std::string> receiving_obstacle_position_lines_topics;

    std::string receiving_obstacle_position_gazebo_topic;

    if (!getRequiredParameter(nh_,"/gridmap/frequency", frequency)){return errorFunc("/gridmap/frequency");};
    if (!getRequiredParameter(nh_,"/gridmap/size_x", map_size_x)){return errorFunc("/gridmap/size_x");};
    if (!getRequiredParameter(nh_,"/gridmap/size_y", map_size_y)){return errorFunc("/gridmap/size_y");};
    if (!getRequiredParameter(nh_,"/gridmap/resolution", map_resolution)){return errorFunc("/gridmap/resolution");};
    if (!getRequiredParameter(nh_,"/gridmap/map_id", map_id)){return errorFunc("/gridmap/map_id");};
    if (!getRequiredParameter(nh_,"/gridmap/map_origin_x", map_center_x)){return errorFunc("/gridmap/map_origin_x");};
    if (!getRequiredParameter(nh_,"/gridmap/map_origin_y", map_center_y)){return errorFunc("/gridmap/map_origin_y");};
    if (!getRequiredParameter(nh_,"/gridmap/publish_topic", publish_topic)){return errorFunc("/gridmap/publish_topic");};

    if (!getParameterOrDefault(nh_,"/obstacles/inflate", inflate_, false)){defaultFunc("/obstacle_create_static/square/inflate");};
    if (!getParameterOrDefault(nh_,"/obstacles/inflation_thickness", inflation_thickness_, 0.01)){defaultFunc("/obstacle_create_static/square/inflation_thickness");};
    if (!getParameterOrDefault(nh_,"/obstacles/create_static/use", static_obstacle_use, false)){defaultFunc("/obstacle/use");};

    if (!getParameterOrDefault(nh_,"/obstacles/create_static/circles/x_center", static_circles_x_, {})){defaultFunc("/obstacle/create_static/circle/x_center");};
    if (!getParameterOrDefault(nh_,"/obstacles/create_static/circles/y_center", static_circles_y_, {})){defaultFunc("/obstacle/create_static/circle/y_center");};
    if (!getParameterOrDefault(nh_,"/obstacles/create_static/circles/radius", static_circles_radius_, {})){defaultFunc("/obstacle/create_static/circle/radius");};

    if (!getParameterOrDefault(nh_,"/obstacles/create_static/squares/x_center", static_squares_x_, {})){defaultFunc("/obstacle_create_static/square/x_center");};
    if (!getParameterOrDefault(nh_,"/obstacles/create_static/squares/y_center", static_squares_y_, {})){defaultFunc("/obstacle_create_static/square/y_center");};
    if (!getParameterOrDefault(nh_,"/obstacles/create_static/squares/length", static_squares_length_, {})){defaultFunc("/obstacle_create_static/square/length");};
    if (!getParameterOrDefault(nh_,"/obstacles/create_static/squares/width", static_squares_width_, {})){defaultFunc("/obstacle_create_static/square/width");};
    if (!getParameterOrDefault(nh_,"/obstacles/create_static/squares/orientation_deg", static_squares_orientation_, {})){defaultFunc("/obstacle_create_static/square/rotation");};
    if (!getParameterOrDefault(nh_,"/obstacles/create_static/squares/line_thickness", static_squares_line_thickness_, {})){defaultFunc("/obstacle_create_static/square/line_thickness");};

    if (!getParameterOrDefault(nh_,"/obstacles/receiving_position/use", receiving_obstacle_position_use_, false)){defaultFunc("/receiving_position/use");};

    if (!getParameterOrDefault(nh_,"/obstacles/receiving_position/circles/topics", receiving_obstacle_position_circles_topics, {})){defaultFunc("/receiving_position/circles/topic");};
    if (!getParameterOrDefault(nh_,"/obstacles/receiving_position/circles/radius", receiving_obstacle_circles_radius_, {})){defaultFunc("/receiving_position/circles/radius");};

    if (!getParameterOrDefault(nh_,"/obstacles/receiving_position/rectangles/topics", receiving_obstacle_position_squares_topics, {})){defaultFunc("/receiving_position/squares/topic");};
    if (!getParameterOrDefault(nh_,"/obstacles/receiving_position/rectangles/lengths", receiving_obstacle_squares_length_, {})){defaultFunc("/receiving_position/squares/length");};
    if (!getParameterOrDefault(nh_,"/obstacles/receiving_position/rectangles/widths", receiving_obstacle_squares_width_, {})){defaultFunc("/receiving_position/squares/width");};
    if (!getParameterOrDefault(nh_,"/obstacles/receiving_position/rectangles/line_thickness", receiving_obstacle_squares_line_thickness_, {})){defaultFunc("/receiving_position/squares/line_thickness");};

    if (!getParameterOrDefault(nh_,"/obstacles/receiving_position/lines/topics", receiving_obstacle_position_lines_topics, {})){defaultFunc("/receiving_position/lines/topic");};
    if (!getParameterOrDefault(nh_,"/obstacles/receiving_position/lines/lengths", receiving_obstacle_lines_length_, {})){defaultFunc("/receiving_position/lines/length");};
    if (!getParameterOrDefault(nh_,"/obstacles/receiving_position/lines/line_thickness", receiving_obstacle_lines_line_thickness_, {})){defaultFunc("/receiving_position/lines/line_thickness");};

    if (!getParameterOrDefault(nh_,"/obstacles/receiving_position_gazebo/use", receiving_obstacle_position_gazebo_use_, false)){defaultFunc("/receiving_position_gazebo/use");};
    if (!getParameterOrDefault(nh_,"/obstacles/receiving_position_gazebo/topic", receiving_obstacle_position_gazebo_topic, std::string("temp"))){defaultFunc("/receiving_position_gazebo/topic");};
    if (!getParameterOrDefault(nh_,"/obstacles/receiving_position_gazebo/radius", receiving_obstacle_gazebo_radius_, 1.0)){defaultFunc("/receiving_position_gazebo/radius");};

    // Recording
    if (!getParameterOrDefault(nh_,"/recording/static_circle_indices_to_record", static_circle_indices_to_record_, {})){defaultFunc("/recording/static_circle_indices_to_record");};
    if (!getParameterOrDefault(nh_,"/obstacles/create_static/circles/names", static_circle_names_, {})){defaultFunc("/obstacles/create_static/circles/names");};
    if (!getParameterOrDefault(nh_,"/recording/static_square_indices_to_record", static_square_indices_to_record_, {})){defaultFunc("/recording/static_square_indices_to_record");};
    if (!getParameterOrDefault(nh_,"/obstacles/create_static/squares/names", static_square_names_, {})){defaultFunc("/obstacles/create_static/squares/names");};

    // Visualization
    if (!getParameterOrDefault(nh_,"/visualization/static_circle_indices_to_visualize", static_circle_indices_to_visualize_, {})){defaultFunc("/visualization/static_circle_indices_to_visualize");};
    if (!getParameterOrDefault(nh_,"/visualization/static_square_indices_to_visualize", static_square_indices_to_visualize_, {})){defaultFunc("/visualization/static_square_indices_to_visualize");};
    if (!getParameterOrDefault(nh_,"/visualization/marker_z", marker_z_, 1.8)){defaultFunc("/visualization/marker_z");};
    if (!getParameterOrDefault(nh_,"/visualization/marker_color", marker_color_, {0, 0, 0, 1})){defaultFunc("/visualization/marker_color");};

    gridmap_.header.stamp = nh_.now();
    gridmap_.header.frame_id = map_id;

    // All cells are initially unknown
    gridmap_.header.stamp = nh_.now();
    gridmap_.header.frame_id = map_id;

    // Gridmap_.info.map_load_time = rclcpp::Time(0);
    gridmap_.info.resolution = map_resolution;
    gridmap_.info.width = map_size_x/map_resolution;
    gridmap_.info.height = map_size_y/map_resolution;
    gridmap_.data.resize(gridmap_.info.width * gridmap_.info.height);

    gridmap_.info.origin.position.x = map_center_x - map_size_x / 2;
    gridmap_.info.origin.position.y = map_center_y - map_size_y / 2;

    // Here we create the image
    occupancy_image_ = cv::Mat(gridmap_.info.width, gridmap_.info.height, CV_8UC1, gridmap_.data.data());

    /**
     * SETUP: Static obstacles
    */
    // Check if we need to create static obstacles
    if (static_obstacle_use)
    {
        if (!(static_circles_x_.size() == static_circles_y_.size()) || !(static_circles_x_.size() == static_circles_radius_.size()))
        {
            RCLCPP_ERROR_STREAM(nh_.get_logger(), "[Occupancygrid Creator]: Parameters static circles x, y and radius dont have the same length!");
            return false;
        }

        createStaticObstacles(static_circles_x_, static_circles_y_, static_circles_radius_);

        if (!(static_squares_x_.size() == static_squares_y_.size()) || !(static_squares_x_.size() == static_squares_length_.size()) ||
            !(static_squares_x_.size() == static_squares_length_.size()) || !(static_squares_x_.size() == static_squares_width_.size()) ||
            !(static_squares_x_.size() == static_squares_orientation_.size()))
        {
            RCLCPP_ERROR_STREAM(nh_.get_logger(), "[Occupancygrid Creator]: Parameters static squares x, y, length, width and orientation dont have the same length!");
            return false;
        }

        for (long unsigned int i=0; i<static_squares_x_.size(); i++)
        {
            placeSquareInImage(gridmap_, occupancy_image_, static_squares_x_[i], static_squares_y_[i], static_squares_length_[i], static_squares_width_[i], static_squares_orientation_[i], static_squares_line_thickness_[i]);
        }
    }

    // Visualization
    std::string visualization_topic = "/grid/obs/vis";
    static_obstacles_marker_pub_.reset(new ROSMarkerPublisher(nh_, visualization_topic.c_str(), target_frame_, static_circles_x_.size()+static_squares_x_.size()));

    // Place data back in the gridmap
    gridmap_.data = std::vector<int8_t>(occupancy_image_.data, occupancy_image_.data + occupancy_image_.total());

    /**
     * SETUP: Receiving obstacles
    */
    state_msgs_stored_.resize(receiving_obstacle_position_squares_topics.size());
    state_received_.resize(receiving_obstacle_position_squares_topics.size());
    state_msgs_lines_stored_.resize(receiving_obstacle_position_lines_topics.size());
    state_received_lines_.resize(receiving_obstacle_position_lines_topics.size());

    /**
     * SETUP: ROS
    */
    pub_map_ =
        nh_.create_publisher<nav_msgs::msg::OccupancyGrid>(
            publish_topic,
            1);

    subs_vector_.resize(
        receiving_obstacle_position_squares_topics.size());

    subs_vector_lines_.resize(
        receiving_obstacle_position_lines_topics.size());

    if (receiving_obstacle_position_use_)
    {
        for (long unsigned int i = 0;
             i < receiving_obstacle_position_squares_topics.size();
             ++i)
        {
            subs_vector_[i] =
                nh_.create_subscription<
                    geometry_msgs::msg::TransformStamped>(
                    receiving_obstacle_position_squares_topics[i],
                    1,
                    [this, i](
                        geometry_msgs::msg::TransformStamped::
                            ConstSharedPtr msg)
                    {
                        callbackPositionObstacleSquares(msg, i);
                    });
        }

        for (long unsigned int i = 0;
             i < receiving_obstacle_position_lines_topics.size();
             ++i)
        {
            subs_vector_lines_[i] =
                nh_.create_subscription<
                    geometry_msgs::msg::TransformStamped>(
                    receiving_obstacle_position_lines_topics[i],
                    1,
                    [this, i](
                        geometry_msgs::msg::TransformStamped::
                            ConstSharedPtr msg)
                    {
                        callbackPositionObstacleLines(msg, i);
                    });
        }
    }

    if (receiving_obstacle_position_gazebo_use_)
    {
        sub_gazebo_ =
            nh_.create_subscription<
                gazebo_msgs::msg::ModelStates>(
                receiving_obstacle_position_gazebo_topic,
                1,
                [this](
                    gazebo_msgs::msg::ModelStates::ConstSharedPtr msg)
                {
                    callbackPositionObstacleGazebo(msg);
                });
    }

    // Create recording publishers
    obs_rec_circle_pubs_.resize(static_circles_x_.size());

    for (long unsigned int i = 0; i < static_circles_x_.size(); ++i)
    {
        if (std::find(
                static_circle_indices_to_record_.begin(),
                static_circle_indices_to_record_.end(),
                i) != static_circle_indices_to_record_.end())
        {
            obs_rec_circle_pubs_[i] =
                nh_.create_publisher<
                    geometry_msgs::msg::PoseStamped>(
                    recording_topic_base_ + static_circle_names_[i],
                    1);
        }
    }

    obs_rec_square_pubs_.resize(static_squares_x_.size());

    for (long unsigned int i = 0; i < static_squares_x_.size(); ++i)
    {
        if (std::find(
                static_square_indices_to_record_.begin(),
                static_square_indices_to_record_.end(),
                i) != static_square_indices_to_record_.end())
        {
            obs_rec_square_pubs_[i] =
                nh_.create_publisher<
                    geometry_msgs::msg::PoseStamped>(
                    recording_topic_base_ + static_square_names_[i],
                    1);
        }
    }

    timer_ =
        nh_.create_wall_timer(
            std::chrono::duration<double>(1.0 / frequency),
            std::bind(&OccupancygridCreator::createMap, this));
    return true;
}

bool OccupancygridCreator::errorFunc(const std::string name)
{
  RCLCPP_ERROR_STREAM(nh_.get_logger(), "[Occupancygrid Creator]: Parameter \""+name+"\" not defined!");
  return false;
}

void OccupancygridCreator::defaultFunc(const std::string name)
{
    RCLCPP_WARN_STREAM(
        nh_.get_logger(),
        "[Occupancygrid Creator]: Parameter \""
            << name
            << "\" not defined, using its default value.");
}

void OccupancygridCreator::createMap()
{
    if (receiving_obstacle_position_use_ && !(std::find(begin(state_received_), end(state_received_), true) == end(state_received_)))
    {
        RCLCPP_WARN_STREAM(nh_.get_logger(), "[Occupancygrid Creator]: Creating received Obstacles. ");
        // For each of the messages, place in the object
        nav_msgs::msg::OccupancyGrid new_gridmap = gridmap_;
        std::vector<double> x;
        std::vector<double> y;
        cv::Mat occupancy_image = cv::Mat(new_gridmap.info.width, new_gridmap.info.height, CV_8UC1, new_gridmap.data.data());

        for (long unsigned int i=0; i<state_msgs_stored_.size(); i++)
        {
            if (!state_received_[i]){continue;};

            double x = state_msgs_stored_[i].transform.translation.x;
            double y = state_msgs_stored_[i].transform.translation.y;

            tf2::Quaternion q_tf;
            tf2::fromMsg(state_msgs_stored_[i].transform.rotation , q_tf);
            double roll, pitch, yaw;

            tf2::Matrix3x3(q_tf).getEulerYPR(yaw, pitch, roll);

            double yaw_degree = yaw * 180 / 3.14159265358979323846;
            placeSquareInImage(new_gridmap, occupancy_image, x, y, receiving_obstacle_squares_length_[i], receiving_obstacle_squares_width_[i], yaw_degree, receiving_obstacle_squares_line_thickness_[i]);
        }

        for (long unsigned int i=0; i<state_msgs_lines_stored_.size(); i++)
        {
            if (!state_received_lines_[i]){continue;};

            double x = state_msgs_lines_stored_[i].transform.translation.x;
            double y = state_msgs_lines_stored_[i].transform.translation.y;
            RCLCPP_WARN_STREAM(nh_.get_logger(), "x: " << x << " y: " << y);

            tf2::Quaternion q_tf;
            tf2::fromMsg(state_msgs_lines_stored_[i].transform.rotation , q_tf);
            double roll, pitch, yaw;

            tf2::Matrix3x3(q_tf).getEulerYPR(yaw, pitch, roll);

            double yaw_degree = yaw * 180 / 3.14159265358979323846;
            placeLineInImage(new_gridmap, occupancy_image, x, y, receiving_obstacle_lines_length_[i], yaw_degree, receiving_obstacle_lines_line_thickness_[i]);
        }

        new_gridmap.data = std::vector<int8_t>(occupancy_image.data, occupancy_image.data + occupancy_image.total());
        pub_map_->publish(new_gridmap);

        return;
    }
    else if (receiving_obstacle_position_gazebo_use_ && state_gazebo_received_)
    {
        RCLCPP_WARN_STREAM(nh_.get_logger(), "[Occupancygrid Creator]: Creating received Gazebo Obstacles. ");

        // For each of the messages, place in the object
        nav_msgs::msg::OccupancyGrid new_gridmap = gridmap_;
        std::vector<double> x;
        std::vector<double> y;
        for (long unsigned int i=0; i<state_msgs_gazebo_stored_.name.size(); i++)
        {
            if (state_msgs_gazebo_stored_.name[i].find("cylinder") == std::string::npos){continue;};

            RCLCPP_WARN_STREAM(nh_.get_logger(), "found :" << state_msgs_gazebo_stored_.name[i]);
            double x = state_msgs_gazebo_stored_.pose[i].position.x;
            double y = state_msgs_gazebo_stored_.pose[i].position.y;
            placeObstacleInGrid(new_gridmap, x, y, receiving_obstacle_gazebo_radius_);
        }

        pub_map_->publish(new_gridmap);
        return;
    }

    pub_map_->publish(gridmap_);

    publishStaticObstacles();
    createStaticObstacleVisualizations();
    publishStaticObstacleVisualizations();
}

void OccupancygridCreator::createStaticObstacles(std::vector<double> x, std::vector<double> y, std::vector<double> radius)
{
    // First change to integer parameters of grid structure
    for (long unsigned int i=0; i<x.size(); i++)
    {
        placeObstacleInGrid(gridmap_, x[i], y[i], radius[i]);
    }
}

void OccupancygridCreator::placeObstacleInGrid(nav_msgs::msg::OccupancyGrid &gridmap, double x_cur, double y_cur, double radius_cur)
{
    double rad_steps =  std::atan2(gridmap_.info.resolution, radius_cur);

    for (double radians = 0; radians < 2*3.14159265358979323846; radians = radians + rad_steps)
    {
        double x_on_circle = std::cos(radians)*radius_cur;
        double y_on_circle = std::sin(radians)*radius_cur;

        //convert to map indexes
        int x_on_grid = (x_on_circle+x_cur-gridmap.info.origin.position.x)/gridmap.info.resolution;
        int y_on_grid = (y_on_circle+y_cur-gridmap.info.origin.position.y)/gridmap.info.resolution;

        gridmap.data[x_on_grid+y_on_grid*gridmap_.info.height] = 100;
    }
}

void OccupancygridCreator::placeSquareInImage(nav_msgs::msg::OccupancyGrid &gridmap, cv::Mat &occupancy_image, double x_cur, double y_cur, double length, double width, double orientation, double line_thickness)
{
    int x_on_grid = (x_cur-gridmap.info.origin.position.x)/gridmap.info.resolution;
    int y_on_grid = (y_cur-gridmap.info.origin.position.y)/gridmap.info.resolution;
    int length_on_grid = length/gridmap.info.resolution;
    int width_on_grid = width/gridmap.info.resolution;
    int line_thickness_on_grid = line_thickness/gridmap.info.resolution;

    int inflation_thickness_on_grid = inflation_thickness_/gridmap.info.resolution;

    // First is middle point, then length and width in meters, then orientation in degrees
    cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(x_on_grid, y_on_grid), cv::Size2f(length_on_grid,width_on_grid), orientation);
    cv::Point2f vertices[4];
    rRect.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        cv::line(occupancy_image, vertices[i], vertices[(i+1)%4], cv::Scalar(100), line_thickness_on_grid);
    }

    if(inflate_)
    {
        // First draw circles around corners
        for (int i = 0; i < 4; i++)
        {
            // Draw the corners
            cv::circle(occupancy_image, vertices[i], inflation_thickness_on_grid, cv::Scalar(100), 1);
        }

        // Draw the lines on the outside of the rectangle
        rRect = cv::RotatedRect(cv::Point2f(x_on_grid, y_on_grid), cv::Size2f(length_on_grid+2*inflation_thickness_on_grid,width_on_grid), orientation);
        rRect.points(vertices);
        cv::line(occupancy_image, vertices[0], vertices[1], cv::Scalar(100), 1);
        cv::line(occupancy_image, vertices[2], vertices[3], cv::Scalar(100), 1);

        rRect = cv::RotatedRect(cv::Point2f(x_on_grid, y_on_grid), cv::Size2f(length_on_grid,width_on_grid+2*inflation_thickness_on_grid), orientation);
        rRect.points(vertices);
        cv::line(occupancy_image, vertices[1], vertices[2], cv::Scalar(100), 1);
        cv::line(occupancy_image, vertices[3], vertices[0], cv::Scalar(100), 1);

        // Draw the lines on the inside of the rectangle if it is possible
        if (width_on_grid-2*inflation_thickness_on_grid > 0)
        {
            rRect = cv::RotatedRect(cv::Point2f(x_on_grid, y_on_grid), cv::Size2f(length_on_grid,width_on_grid-2*inflation_thickness_on_grid), orientation);
            rRect.points(vertices);
            cv::line(occupancy_image, vertices[1], vertices[2], cv::Scalar(100), 1);
            cv::line(occupancy_image, vertices[3], vertices[0], cv::Scalar(100), 1);
        }

        if (length_on_grid-2*inflation_thickness_on_grid > 0)
        {
            rRect = cv::RotatedRect(cv::Point2f(x_on_grid, y_on_grid), cv::Size2f(length_on_grid-2*inflation_thickness_on_grid,width_on_grid), orientation);
            rRect.points(vertices);
            cv::line(occupancy_image, vertices[0], vertices[1], cv::Scalar(100), 1);
            cv::line(occupancy_image, vertices[2], vertices[3], cv::Scalar(100), 1);
        }
    }
}

void OccupancygridCreator::placeInflatedSquareInImage(nav_msgs::msg::OccupancyGrid &gridmap, cv::Mat &occupancy_image, double x_cur, double y_cur, double length, double width, double orientation, double inflation_thickness)
{
    int length_on_grid = length/gridmap.info.resolution;
    int width_on_grid = width/gridmap.info.resolution;

    int inflation_thickness_on_grid = inflation_thickness/gridmap.info.resolution;

    cv::Point center(x_cur, y_cur);

    // First is middle point, then length and width in meters, then orientation in degrees
    drawRoundedRect(occupancy_image, center, width_on_grid+inflation_thickness_on_grid, length_on_grid+inflation_thickness_on_grid, inflation_thickness_on_grid, orientation, cv::Scalar(100), 1);
}

void OccupancygridCreator::drawRoundedRect(cv::Mat &image, cv::Point center, int width, int height, int cornerRadius, double angle, cv::Scalar color, int thickness)
{
    // Create a new image to draw the rounded rectangle
    cv::Mat rectImage = cv::Mat::zeros(image.size(), image.type());

    std::cerr << "width: " << width << " height: " << height << std::endl;

    // Draw the corners
    cv::circle(rectImage, cv::Point(cornerRadius, cornerRadius), cornerRadius, color, thickness);
    cv::circle(rectImage, cv::Point(width - cornerRadius, cornerRadius), cornerRadius, color, thickness);
    cv::circle(rectImage, cv::Point(cornerRadius, height - cornerRadius), cornerRadius, color, thickness);
    cv::circle(rectImage, cv::Point(width - cornerRadius, height - cornerRadius), cornerRadius, color, thickness);

    // Draw the straight lines
    cv::line(rectImage, cv::Point(center.x - width, height+cornerRadius), cv::Point(center.x + width, height+cornerRadius), color, thickness);
    cv::line(rectImage, cv::Point(center.x - width, height-cornerRadius), cv::Point(center.x + width, height-cornerRadius), color, thickness);
    cv::line(rectImage, cv::Point(0, cornerRadius), cv::Point(0, height - cornerRadius), color, thickness);
    cv::line(rectImage, cv::Point(width, cornerRadius), cv::Point(width, height - cornerRadius), color, thickness);

    // Rotate the rectangle image
    cv::Mat rotationMatrix = cv::getRotationMatrix2D(cv::Point(width / 2, height / 2), angle, 1);
    cv::warpAffine(rectImage, rectImage, rotationMatrix, rectImage.size());

    // Overlay the rotated rectangle onto the original image
    rectImage.copyTo(image, rectImage);
}

void OccupancygridCreator::placeLineInImage(nav_msgs::msg::OccupancyGrid &gridmap, cv::Mat &occupancy_image, double x_cur, double y_cur, double length, double orientation, double line_thickness)
{
    placeSquareInImage(gridmap, occupancy_image, x_cur, y_cur, 0.001, length, orientation, line_thickness);
}

void OccupancygridCreator::callbackPositionObstacleSquares(
    geometry_msgs::msg::TransformStamped::ConstSharedPtr msg,
    const long unsigned int i)
{
    state_msgs_stored_[i] = *msg;
    state_received_[i] = true;
}

void OccupancygridCreator::callbackPositionObstacleLines(
    geometry_msgs::msg::TransformStamped::ConstSharedPtr msg,
    const long unsigned int i)
{
    state_msgs_lines_stored_[i] = *msg;
    state_received_lines_[i] = true;
}

void OccupancygridCreator::callbackPositionObstacleGazebo(
    gazebo_msgs::msg::ModelStates::ConstSharedPtr msg)
{
    state_msgs_gazebo_stored_ = *msg;
    state_gazebo_received_ = true;
}

void OccupancygridCreator::publishStaticObstacles()
{
    // Create base obstacle message
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = target_frame_;
    msg.header.stamp = nh_.now();

    // Publish all static circle messages
    for (long unsigned int i=0; i<static_circles_x_.size(); i++)
    {
        if (std::find(static_circle_indices_to_record_.begin(), static_circle_indices_to_record_.end(), i) != static_circle_indices_to_record_.end())
        {
            // Fill dynamic message content
            msg.pose.position.x = static_circles_x_[i];
            msg.pose.position.y = static_circles_y_[i];
            msg.pose.position.z = 0;
            msg.pose.orientation.x = 0;
            msg.pose.orientation.y = 0;
            msg.pose.orientation.z = 0;
            msg.pose.orientation.w = 1;
            obs_rec_circle_pubs_[i]->publish(msg);
        }
    }

    // Publish all static square messages
    tf2::Quaternion q;
    for (long unsigned int i=0; i<static_squares_x_.size(); i++)
    {
        if (std::find(static_square_indices_to_record_.begin(), static_square_indices_to_record_.end(), i) != static_square_indices_to_record_.end())
        {
            // Obtain quaternion from orientation
            q.setRPY(0, 0, static_squares_orientation_[i] * M_PI / 180);

            // Fill dynamic message content
            msg.pose.position.x = static_squares_x_[i];
            msg.pose.position.y = static_squares_y_[i];
            msg.pose.position.z = 0;
            msg.pose.orientation.x = q.x();
            msg.pose.orientation.y = q.y();
            msg.pose.orientation.z = q.z();
            msg.pose.orientation.w = q.w();
            obs_rec_square_pubs_[i]->publish(msg);
        }
    }
}

void OccupancygridCreator::createStaticObstacleVisualizations()
{
    for (long unsigned int i=0; i<static_circles_x_.size(); i++)
    {
        if (std::find(static_circle_indices_to_visualize_.begin(), static_circle_indices_to_visualize_.end(), i) != static_circle_indices_to_visualize_.end())
        {
            ROSPointMarker &static_obstacle = static_obstacles_marker_pub_->getNewPointMarker("CYLINDER");
            static_obstacle.setColor(marker_color_[0], marker_color_[1], marker_color_[2], marker_color_[3]);
            static_obstacle.setScale(2*static_circles_radius_[i], 2*static_circles_radius_[i], marker_z_);
            static_obstacle.addPointMarker(Eigen::Vector3d(static_circles_x_[i], static_circles_y_[i], marker_z_/2));
        }
    }
    for (long unsigned int i=0; i<static_squares_x_.size(); i++)
    {
        if (std::find(static_square_indices_to_visualize_.begin(), static_square_indices_to_visualize_.end(), i) != static_square_indices_to_visualize_.end())
        {
            ROSPointMarker &static_obstacle = static_obstacles_marker_pub_->getNewPointMarker("CUBE");
            static_obstacle.setColor(marker_color_[0], marker_color_[1], marker_color_[2], marker_color_[3]);
            static_obstacle.setScale(static_squares_length_[i], static_squares_width_[i], marker_z_);
            static_obstacle.setOrientation(static_squares_orientation_[i] * M_PI / 180);
            static_obstacle.addPointMarker(Eigen::Vector3d(static_squares_x_[i], static_squares_y_[i], marker_z_/2));
        }
    }
}

void OccupancygridCreator::publishStaticObstacleVisualizations()
{
    static_obstacles_marker_pub_->publish();
}
