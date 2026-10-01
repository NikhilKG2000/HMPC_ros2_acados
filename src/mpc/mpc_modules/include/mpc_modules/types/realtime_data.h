#ifndef MPC_MODULES_TYPES_REALTIME_DATA_H
#define MPC_MODULES_TYPES_REALTIME_DATA_H

#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>

#include <mpc_modules/types/tracked.h>

/**
 * @brief Stores relevant real-time data in one place
 *
 */
class RealTimeData
{

public:
    RealTimeData(){}

    virtual ~RealTimeData(){};

    void Print()
    {
        RCLCPP_WARN(
            rclcpp::get_logger("mpc_modules"),
            "========== Real Time Data =========");
        RCLCPP_INFO_STREAM(rclcpp::get_logger("mpc_modules"),"- STATIC OBSTACLES -");
        for (int k = 0; k < constraints_.cols(); k++)
        {
            for (int l = 0; l < constraints_.rows()/3; l++)
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("mpc_modules"),"Constraint " << l << " [" << constraints_(3*l, k) << ", " << constraints_(3*l+1, k) << "] b = " << constraints_(3*l+2, k));
            }
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger("mpc_modules"),"- REFERENCE INPUT TRAJECTORY -");
        for (size_t k = 0; k < ref_traj_u_.size(); k++)
        {
            std::string s_u_ref = "";
            for (size_t l = 0; l < ref_traj_u_[k].size(); l++)
            {
                s_u_ref.append(std::to_string(ref_traj_u_[k][l]));
                if (!(l == ref_traj_u_[k].size()-1))
                {
                    s_u_ref.append(", ");
                }
            }
            RCLCPP_INFO_STREAM(rclcpp::get_logger("mpc_modules"),"u_ref at stage " << k << " " << s_u_ref);
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger("mpc_modules"),"- REFERENCE STATE TRAJECTORY -");
        for (size_t k = 0; k < ref_traj_x_.size(); k++)
        {
            std::string s_x_ref = "";
            for (size_t l = 0; l < ref_traj_x_[k].size(); l++)
            {
                s_x_ref.append(std::to_string(ref_traj_x_[k][l]));
                if (!(l == ref_traj_x_[k].size()-1))
                {
                    s_x_ref.append(", ");
                }
            }
            RCLCPP_INFO_STREAM(rclcpp::get_logger("mpc_modules"),"x_ref at stage " << k << " " << s_x_ref);
        }
        RCLCPP_WARN(
            rclcpp::get_logger("mpc_modules"),
            "===================================");
    }

public:
    // Constraints
    Eigen::MatrixXd constraints_; // Eigen::VectorXd containing concatenated halfspace constraints per stage
    Eigen::VectorXi n_relevant_constraints_; // amount of relevant (non-dummy) halfspace constraints per stage
    int constraints_n_it_valid_; // validity: indicates how many iterations the communicated constraints remain valid

    // Length of (x,u) trajectory depends on horizon of lower-layer controller
    TrackedVector<std::vector<double>> ref_traj_u_;
    TrackedVector<std::vector<double>> ref_traj_x_;
    int ref_traj_n_it_valid_; // validity: indicates how many iterations the communicated reference trajectory remains valid

protected:
};

#endif
