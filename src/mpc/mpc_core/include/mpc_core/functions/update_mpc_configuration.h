#ifndef MPC_CORE_UPDATE_MPC_CONFIGURATION
#define MPC_CORE_UPDATE_MPC_CONFIGURATION

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <mpc_base/configuration_mpc.h>

/**
 * Read all MPC parameters from the ROS 2 node.
 */
bool mpc_configuration_initialize(
    rclcpp::Node& nh,
    ConfigurationMPC& config);

bool errorFunc(const std::string name);

void defaultFunc(const std::string name);

void rosRecordingWarning(const std::string name);

#endif