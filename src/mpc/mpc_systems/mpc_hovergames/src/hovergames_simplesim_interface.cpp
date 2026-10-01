#include <mpc_hovergames/hovergames_simplesim_interface.h>

#include <mpc_tools/instrumentation_timer.h>
#include <mpc_tools/printing.h>

#include <chrono>

HovergamesSimpleSimInterface::HovergamesSimpleSimInterface(
    rclcpp::Node& node,
    ControllerBase* ptr_controller,
    ConfigurationMPC* ptr_config,
    SolverBase* ptr_solver,
    ModuleHandlerBase* ptr_module_handler,
    RobotRegion* ptr_robot_region,
    DataSaverJson* ptr_data_saver_json,
    std::mutex* mutex)
    : HovergamesDroneInterface(
          node,
          ptr_controller,
          ptr_config,
          ptr_solver,
          ptr_module_handler,
          ptr_robot_region,
          ptr_data_saver_json,
          mutex)
{
    MPC_WARN_ALWAYS(
        "Initializing hovergames simple sim interface");

    control_msg_.header.frame_id =
        ptr_config_->robot_base_link_;

    control_msg_.u[0] = 0.0;
    control_msg_.u[1] = 0.0;
    control_msg_.u[2] = 0.0;
    control_msg_.u[3] = 9.81;

    if (ptr_config_->layer_idx_ == 0)
    {
        command_pub_0_ =
            nh_.create_publisher<
                simple_sim::msg::DroneHovergamesControl>(
                ptr_config_->control_command_topic_,
                1);

        // In synchronized simulation the model advances only after receiving a
        // control command. Send hover commands until its first state arrives,
        // then let the normal TMPC control loop take over.
        bootstrap_timer_ = nh_.create_wall_timer(
            std::chrono::milliseconds(100),
            [this]()
            {
                if (first_state_received_)
                {
                    bootstrap_timer_->cancel();
                    return;
                }

                control_msg_.header.stamp = nh_.now();
                command_pub_0_->publish(control_msg_);
            });
    }

    MPC_WARN_ALWAYS(
        "Hovergames simple sim interface initialized");
}

void HovergamesSimpleSimInterface::OnStateCallback(
    const nav_msgs::msg::Odometry& /*msg*/)
{
    PROFILE_FUNCTION();

    if (!first_state_received_)
    {
        first_state_received_ = true;

        if (
            !first_ref_received_ &&
            ptr_config_->n_layers_ > 1 &&
            ptr_config_->layer_idx_ == 0)
        {
            start_stage_vars_ =
                ptr_solver_->last_updated_stage_vars_;

            SetReferenceTrajectoryHover();
            SetConstraintsHover();
        }

        if (ptr_config_->layer_idx_ == 0)
        {
            ptr_controller_->startTimer();
        }
    }
}

void HovergamesSimpleSimInterface::OnComputeActuation()
{
    PROFILE_FUNCTION();

    control_msg_.u[0] = u_solver_(0);
    control_msg_.u[1] = u_solver_(1);
    control_msg_.u[2] = u_solver_(2);
    control_msg_.u[3] = u_solver_(3);
}

void HovergamesSimpleSimInterface::OnActuate()
{
    PROFILE_FUNCTION();

    if (bootstrap_timer_)
    {
        bootstrap_timer_->cancel();
    }

    control_msg_.header.stamp = nh_.now();
    command_pub_0_->publish(control_msg_);
}

void HovergamesSimpleSimInterface::
OnDeployObjectiveReachedStrategy()
{
    PROFILE_FUNCTION();
}

void HovergamesSimpleSimInterface::
OnDeployEmergencyStrategy()
{
    PROFILE_FUNCTION();
}

void HovergamesSimpleSimInterface::OnActuateBrake()
{
    PROFILE_FUNCTION();
}
