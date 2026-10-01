#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <occupancygrid_creator/occupancygrid_creator.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<rclcpp::Node>("occupancygrid_creator_node");

    OccupancygridCreator occupancygrid_creator(*node);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}