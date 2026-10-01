#include <mpc_core/functions/update_mpc_configuration.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <cstdint>
#include <vector>

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
    const std::string& parameter_name)
{
    const auto& overrides =
        node.get_node_parameters_interface()
            ->get_parameter_overrides();

    return overrides.find(parameter_name) != overrides.end();
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
        value = node.declare_parameter<T>(
            name,
            default_value);
    }
    else
    {
        node.get_parameter(name, value);
    }

    return parameter_was_provided;
}

// ROS 2 represents integer parameter arrays using int64_t.
bool getParameterOrDefault(
    rclcpp::Node& node,
    const std::string& ros1_name,
    std::vector<int>& value,
    const std::vector<int>& default_value)
{
    const std::string name = ros2ParameterName(ros1_name);

    const bool parameter_was_provided =
        node.has_parameter(name) ||
        hasParameterOverride(node, name);

    const std::vector<int64_t> ros2_default_value(
        default_value.begin(),
        default_value.end());

    std::vector<int64_t> ros2_value;

    if (!node.has_parameter(name))
    {
        ros2_value =
            node.declare_parameter<std::vector<int64_t>>(
                name,
                ros2_default_value);
    }
    else
    {
        node.get_parameter(name, ros2_value);
    }

    value.assign(
        ros2_value.begin(),
        ros2_value.end());

    return parameter_was_provided;
}

std::string groundProjectionTopic(
    const std::string& topic,
    int n_layers,
    int layer_idx)
{
    if (topic.empty())
    {
        return "";
    }

    if (n_layers <= 1)
    {
        return topic + "/ground";
    }

    const std::string layer_suffix =
        "/layer_" + std::to_string(layer_idx);

    if (topic.size() >= layer_suffix.size() &&
        topic.compare(
            topic.size() - layer_suffix.size(),
            layer_suffix.size(),
            layer_suffix) == 0)
    {
        return topic.substr(
            0,
            topic.size() - layer_suffix.size()) +
            "/ground" + layer_suffix;
    }

    return topic + "/ground";
}

}  // namespace

bool mpc_configuration_initialize(
    rclcpp::Node& nh,
    ConfigurationMPC& config)
{
    RCLCPP_WARN_STREAM(rclcpp::get_logger("mpc"),"[MPC config] Initializing parameter configuration");

    /************************** PARAMS **************************/
    /* Hierarchical settings */
    if (!getParameterOrDefault(nh,"/sim", config.sim_, false)){defaultFunc("/sim");};

    /* Hierarchical settings (necessary to define here) */
    if (!getParameterOrDefault(nh,"/n_layers", config.n_layers_, 1)){defaultFunc("/n_layers");};
    if (!getParameterOrDefault(nh,"layer_idx", config.layer_idx_, 0)){defaultFunc("layer_idx");};
    if (config.n_layers_ > 1)
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("mpc"),"[MPC config] Total amount of MPC layers: " << config.n_layers_);
        RCLCPP_WARN_STREAM(rclcpp::get_logger("mpc"),"[MPC config] MPC layer: " << config.layer_idx_);
    }
    std::string layer_idx_str;
    if (config.n_layers_ > 1)
    {
        layer_idx_str = "/" + std::to_string(config.layer_idx_);
    }
    else
    {
        layer_idx_str = "";
    }

    /* Control loop and actuation */
    if (!getRequiredParameter(nh,"/params/control_loop_and_actuation/enable_output", config.enable_output_)){return errorFunc("/params/control_loop_and_actuation/enable_output");};
    if (!getRequiredParameter(nh,"/params/control_loop_and_actuation/auto_enable_plan", config.auto_enable_plan_)){return errorFunc("/params/control_loop_and_actuation/auto_enable_plan");};
    if (config.auto_enable_plan_){config.plan_ = true;} else {config.plan_ = false;};
    if (!getRequiredParameter(nh,"/params/control_loop_and_actuation/synchronized_actuation", config.synchronized_actuation_)){return errorFunc("/params/control_loop_and_actuation/synchronized_actuation");};
    if (!getParameterOrDefault(nh,"/params/control_loop_and_actuation/feedback_law_frequency", config.feedback_law_frequency_, 100.0)){defaultFunc("/params/control_loop_and_actuation/sync_mode");};
    if (!getParameterOrDefault(nh,"/params/control_loop_and_actuation/sync_mode", config.sync_mode_, true)){defaultFunc("/params/control_loop_and_actuation/sync_mode");};
    if (!getParameterOrDefault(nh,"/params/control_loop_and_actuation/sync_simple_sim", config.sync_simple_sim_, false)){defaultFunc("/params/control_loop_and_actuation/sync_simple_sim");};

    /* Solver */
    if (!getParameterOrDefault(nh,"/params/solver/use_custom_initial_guess", config.use_custom_initial_guess_, false)){defaultFunc("/params/solver/use_custom_initial_guess");};
    if (!getParameterOrDefault(nh,"/params/solver/time_shift", config.time_shift_, 1)){defaultFunc("/params/solver/time_shift");};

    /* Robot description */
    if (!getParameterOrDefault(nh,"/params/robot/n_dim", config.n_dim_, 2)){defaultFunc("/params/robot/n_dim");};
    if (!getRequiredParameter(nh,"/params/robot/length", config.robot_length_)){return errorFunc("/params/robot/length");};
    if (!getRequiredParameter(nh,"/params/robot/width", config.robot_width_)){return errorFunc("/params/robot/width");};
    if (!getRequiredParameter(nh,"/params/robot/com_to_back", config.robot_center_of_mass_to_back_)){return errorFunc("/params/robot/com_to_back");};
    if (!getRequiredParameter(nh,"/params/robot/base_link", config.robot_base_link_)){return errorFunc("/params/robot/base_link");};
    if (!getRequiredParameter(nh,"/params/robot/target_frame", config.robot_target_frame_)){return errorFunc("/params/robot/target_frame");};

    /* Modules */
    // Static polyhedron constraints
    if (!getParameterOrDefault(nh,"/params/modules/static_polyhedron_constraints/n_constraints_per_stage", config.n_constraints_per_stage_, 24)){defaultFunc("/params/modules/static_polyhedron_constraints/n_constraints_per_stage");};
    if (!getParameterOrDefault(nh,"/params/modules/static_polyhedron_constraints/occupied_threshold", config.occupied_threshold_, 70)){defaultFunc("/params/modules/static_polyhedron_constraints/occupied_threshold");};
    if (!getParameterOrDefault(nh,"/params/modules/static_polyhedron_constraints/occ_cell_selection_method", config.occ_cell_selection_method_, 1)){defaultFunc("/params/modules/static_polyhedron_constraints/occ_cell_selection_method");};
    if (!getParameterOrDefault(nh,"/params/modules/static_polyhedron_constraints/bounding_box_width", config.bounding_box_width_, 2.0)){defaultFunc("/params/modules/static_polyhedron_constraints/bounding_box_width");};
    if (!getParameterOrDefault(nh,"/params/modules/static_polyhedron_constraints/line_segment_to_stage", config.line_segment_to_stage_, -1)){defaultFunc("/params/modules/static_polyhedron_constraints/line_segment_to_stage");};
    if (!getParameterOrDefault(nh,"/params/modules/static_polyhedron_constraints/delta_segment", config.delta_segment_, 0.00001)){defaultFunc("/params/modules/static_polyhedron_constraints/delta_segment");};
    if (!getParameterOrDefault(nh,"/params/modules/static_polyhedron_constraints/safety_margin", config.safety_margin_, -1.0)){};
    if (!getParameterOrDefault(nh,"/params/modules/static_polyhedron_constraints/safety_margins", config.safety_margins_, {-1.0})){};
    if ((int)config.safety_margins_.size() != config.n_layers_)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("mpc"),"[MPC config] The provided vector of safety margins has length " << config.safety_margins_.size() << ", whereas the amount of hierarchical MPC layers equals " << config.n_layers_ << "! Exiting.");
        return false;
    }
    if (config.safety_margin_ == -1){config.safety_margin_ = config.safety_margins_[config.layer_idx_];};
    if (config.safety_margin_ == -1){defaultFunc("/params/modules/static_polyhedron_constraints/safety_margin_ or safety_margins_");}

    /* Printing */
    if (!getParameterOrDefault(nh,"/params/printing/debug_output", config.debug_output_, false)){defaultFunc("/params/printing/debug_output");};
    if (!getParameterOrDefault(nh,"/params/printing/debug_data", config.debug_data_, false)){defaultFunc("/params/printing/debug_data");};

    /* Visualization */
    // General
    if (!getParameterOrDefault(nh,"/params/visualization/general/ground_projection_marker_alpha", config.ground_projection_marker_alpha_, 0.1)){defaultFunc("/params/visualization/general/ground_projection_marker_alpha");};
    if (!getParameterOrDefault(nh,"/params/visualization/general/ground_projection_marker_z", config.ground_projection_marker_z_, 0.1)){defaultFunc("/params/visualization/general/ground_projection_marker_z");};
    // System interface
    if (!getParameterOrDefault(nh,"/params/visualization/system_interface/draw_current_position_ground_air", config.draw_current_position_ground_air_, 3)){defaultFunc("/params/visualization/system_interface/draw_current_position_ground_air");};
    if (!getParameterOrDefault(nh,"/params/visualization/system_interface/current_position_marker_color", config.current_position_marker_color_, {0.5, 0, 0.5, 1})){defaultFunc("/params/visualization/system_interface/current_position_marker_color");};
    if (!getParameterOrDefault(nh,"/params/visualization/system_interface/current_position_marker_scale", config.current_position_marker_scale_, {0.05, 0.05, 0.2})){defaultFunc("/params/visualization/system_interface/current_position_marker_scale");};
    if (!getParameterOrDefault(nh,"/params/visualization/system_interface/region_min_n_points", config.region_min_n_points_, 30)){defaultFunc("/params/visualization/system_interface/region_min_n_points");};
    if (!getParameterOrDefault(nh,"/params/visualization/system_interface/current_region_n_points", config.current_region_n_points_, 1)){defaultFunc("/params/visualization/system_interface/current_region_n_points");};
    if (config.draw_current_region_ && config.current_region_n_points_ > 1 && config.current_region_n_points_ < config.region_min_n_points_) {RCLCPP_WARN_STREAM(rclcpp::get_logger("mpc"),"[MPC config] The allowed values for current_region_n_points_ are {1,>" << config.region_min_n_points_ << "}. Current region will not be created!");};
    if (!getParameterOrDefault(nh,"/params/visualization/system_interface/predicted_regions_n_points", config.predicted_regions_n_points_, 1)){defaultFunc("/params/visualization/system_interface/predicted_regions_n_points");};
    if (config.draw_predicted_regions_ && config.predicted_regions_n_points_ > 1 && config.predicted_regions_n_points_ < config.region_min_n_points_) {RCLCPP_WARN_STREAM(rclcpp::get_logger("mpc"),"[MPC config] The allowed values for predicted_regions_n_points_ are {1,>" << config.region_min_n_points_ << "}. Current region will not be created!");};
    if (!getParameterOrDefault(nh,"/params/visualization/system_interface/draw_current_region", config.draw_current_region_, true)){defaultFunc("/params/visualization/system_interface/draw_current_region");};
    if (!getParameterOrDefault(nh,"/params/visualization/system_interface/current_region_marker_scale", config.current_region_marker_scale_, {0.05, 0.05, 0.2})){defaultFunc("/params/visualization/system_interface/current_region_marker_scale");};
    if (!getParameterOrDefault(nh,"/params/visualization/system_interface/current_region_marker_z", config.current_region_marker_z_, 0.001)){defaultFunc("/params/visualization/system_interface/current_region_marker_z");};
    if (!getParameterOrDefault(nh,"/params/visualization/system_interface/draw_predicted_positions_ground_air"+layer_idx_str, config.draw_predicted_positions_ground_air_, 3)){defaultFunc("/params/visualization/system_interface/draw_predicted_positions_ground_air"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/params/visualization/system_interface/predicted_positions_marker_color"+layer_idx_str, config.predicted_positions_marker_color_, {1, 0.6484375, 0, 0.8})){defaultFunc("/params/visualization/system_interface/predicted_positions_marker_color"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/params/visualization/system_interface/predicted_positions_marker_scale"+layer_idx_str, config.predicted_positions_marker_scale_, {0.1, 0.1, 0.16})){defaultFunc("/params/visualization/system_interface/predicted_positions_marker_scale"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/params/visualization/system_interface/draw_predicted_regions"+layer_idx_str, config.draw_predicted_regions_, false)){defaultFunc("/params/visualization/system_interface/draw_predicted_regions"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/params/visualization/system_interface/predicted_regions_marker_scale"+layer_idx_str, config.predicted_regions_marker_scale_, {0.1, 0.1, 0.16})){defaultFunc("/params/visualization/system_interface/predicted_regions_marker_scale"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/params/visualization/system_interface/predicted_regions_marker_z"+layer_idx_str, config.predicted_regions_marker_z_, 0.0005)){defaultFunc("/params/visualization/system_interface/predicted_regions_marker_z"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/params/visualization/system_interface/region_indices_to_draw", config.region_indices_to_draw_, {1})){defaultFunc("/params/visualization/system_interface/region_indices_to_draw");};
    // Modules
    if (!getParameterOrDefault(nh,"/params/visualization/modules/goal_oriented/draw_goal_position_ground_air", config.draw_goal_position_ground_air_, 3)){defaultFunc("/params/visualization/modules/goal_oriented/draw_goal_position_ground_air");};
    if (!getParameterOrDefault(nh,"/params/visualization/modules/goal_oriented/marker_color", config.goal_position_marker_color_, {0, 1, 0, 0.2})){defaultFunc("/params/visualization/modules/goal_oriented/marker_color");};
    if (!getParameterOrDefault(nh,"/params/visualization/modules/goal_oriented/marker_scale", config.goal_position_marker_scale_, {0.2, 0.2, 0.2})){defaultFunc("/params/visualization/modules/goal_oriented/marker_scale");};
    if (!getParameterOrDefault(nh,"/params/visualization/modules/reference_trajectory/draw_reference_trajectory_ground_air", config.draw_reference_trajectory_ground_air_, 3)){defaultFunc("/params/visualization/modules/reference_trajectory/draw_reference_trajectory_ground_air");};
    if (!getParameterOrDefault(nh,"/params/visualization/modules/reference_trajectory/marker_color", config.reference_trajectory_marker_color_, {1, 0, 0, 0.6})){defaultFunc("/params/visualization/modules/reference_trajectory/marker_color");};
    if (!getParameterOrDefault(nh,"/params/visualization/modules/reference_trajectory/marker_scale", config.reference_trajectory_marker_scale_, {0.12, 0.12, 0.12})){defaultFunc("/params/visualization/modules/reference_trajectory/marker_scale");};
    if (!getParameterOrDefault(nh,"/params/visualization/modules/static_polyhedron_constraints/draw_constraints"+layer_idx_str, config.draw_constraints_, true)){defaultFunc("/params/visualization/modules/static_polyhedron_constraints/draw_constraints"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/params/visualization/modules/static_polyhedron_constraints/marker_alpha_hierarchical", config.constraints_marker_alpha_hierarchical_, 0.1)){defaultFunc("/params/visualization/modules/static_polyhedron_constraints/marker_alpha_hierarchical");};
    if (!getParameterOrDefault(nh,"/params/visualization/modules/static_polyhedron_constraints/marker_scale", config.constraints_marker_scale_, {0.1, 0.1, 0.001})){defaultFunc("/params/visualization/modules/static_polyhedron_constraints/marker_scale");};
    if (!getParameterOrDefault(nh,"/params/visualization/modules/static_polyhedron_constraints/marker_z"+layer_idx_str, config.constraints_marker_z_, 0.004)){defaultFunc("/params/visualization/modules/static_polyhedron_constraints/marker_z"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/params/visualization/modules/static_polyhedron_constraints/indices_to_draw"+layer_idx_str, config.indices_to_draw_, {0, 1, 2, 3})){defaultFunc("/params/visualization/modules/static_polyhedron_constraints/indices_to_draw"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/params/visualization/modules/static_polyhedron_constraints/draw_vertices", config.draw_vertices_, true)){defaultFunc("/params/visualization/modules/static_polyhedron_constraints/draw_vertices");};

    /* Recording */
    // General
    if (!getParameterOrDefault(nh,"/params/recording/enable_ros_recordings", config.enable_ros_recordings_, false)){defaultFunc("/params/recording/enable_ros_recordings");};
    if (!getParameterOrDefault(nh,"/params/recording/enable_json_recordings", config.record_experiment_json_, false)){defaultFunc("/params/recording/enable_json_recordings");};
    if (!getParameterOrDefault(nh,"/params/recording/experiment_name_json", config.recording_name_json_, std::string("no_name"))){defaultFunc("/params/recording/experiment_name_json");};
    // System interface
    if (!getParameterOrDefault(nh,"/params/recording/system_interface/record_current_state", config.record_cur_state_, false)){defaultFunc("/params/recording/system_interface/record_current_state");};
    if (!getParameterOrDefault(nh,"/params/recording/system_interface/record_predicted_positions"+layer_idx_str, config.record_pred_traj_, false)){defaultFunc("/params/recording/system_interface/record_predicted_positions"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/params/recording/system_interface/record_slack"+layer_idx_str, config.record_slack_, false)){defaultFunc("/params/recording/system_interface/record_slack"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/params/recording/system_interface/record_time_reference"+layer_idx_str, config.record_time_reference_, false)){defaultFunc("/params/recording/system_interface/record_time_reference"+layer_idx_str);};
    // Modules
    if (!getParameterOrDefault(nh,"/params/recording/modules/goal_oriented/record_goal_position"+layer_idx_str, config.record_goal_position_, false)){defaultFunc("/params/recording/modules/goal_oriented/record_goal_position"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/params/recording/modules/reference_trajectory/record_reference_trajectory"+layer_idx_str, config.record_reference_trajectory_, false)){defaultFunc("/params/recording/modules/reference_trajectory/record_reference_trajectory"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/params/recording/modules/static_polyhedron_constraints/record_constraints"+layer_idx_str, config.record_constraints_, false)){defaultFunc("/params/recording/modules/static_polyhedron_constraints/record_constraints"+layer_idx_str);};
    // Give warning if ROS data recording is disabled, but one of the recording settings is enabled
    if (!config.enable_ros_recordings_)
    {
        if (config.record_cur_state_) rosRecordingWarning("current state");
        if (config.record_pred_traj_) rosRecordingWarning("predicted positions");
        if (config.record_slack_) rosRecordingWarning("slack");
        if (config.record_goal_position_) rosRecordingWarning("goal position");
        if (config.record_reference_trajectory_) rosRecordingWarning("reference trajectory");
        if (config.record_constraints_) rosRecordingWarning("constraints");
    }
    /************************************************************/

    /************************** TOPICS **************************/
    /* Publish */
    // System interface
    if (!getRequiredParameter(nh,"/topics/publish/system_interface/control_command", config.control_command_topic_)){return errorFunc("/topics/publish/system_interface/control_command");};
    if (!getParameterOrDefault(nh,"/topics/publish/system_interface/feedback_law", config.feedback_law_topic_, std::string("/feedback_law"))){defaultFunc("/topics/publish/system_interface/feedback_law");};
    // Modules
    if (!getParameterOrDefault(nh,"/topics/publish/modules/static_polyhedron_constraints/local_occupancy_grid", config.local_occ_grid_topic_, std::string("/mpc/local_occupancy_grid"))){defaultFunc("/topics/publish/modules/static_polyhedron_constraints/local_occupancy_grid");};

    /* Subscribe */
    // System interface
    if (!getRequiredParameter(nh,"/topics/subscribe/system_interface/state", config.state_topic_)){return errorFunc("/topics/subscribe/system_interface/state");};
    // Modules
    if (!getParameterOrDefault(nh,"/topics/subscribe/modules/goal_oriented/goal", config.goal_topic_, std::string("/goal"))){defaultFunc("/topics/subscribe/modules/goal_oriented/goal");};
    if (!getParameterOrDefault(nh,"/topics/subscribe/modules/static_polyhedron_constraints/occupancy_grid", config.occ_grid_topic_, std::string("/occupancy_grid"))){defaultFunc("/topics/subscribe/modules/static_polyhedron_constraints/occupancy_grid");};

    /* Visualization */
    // System interface
    if (!getRequiredParameter(nh,"/topics/visualization/system_interface/current_position", config.current_position_vis_topic_)){return errorFunc("/topics/visualization/system_interface/current_position");};
    config.current_position_ground_vis_topic_ = config.current_position_vis_topic_ + "/ground";
    if (!getRequiredParameter(nh,"/topics/visualization/system_interface/current_region", config.current_region_vis_topic_)){return errorFunc("/topics/visualization/system_interface/current_region");};
    if (!getRequiredParameter(nh,"/topics/visualization/system_interface/predicted_positions"+layer_idx_str, config.predicted_positions_vis_topic_)){return errorFunc("/topics/visualization/system_interface/predicted_positions"+layer_idx_str);};
    if (config.predicted_positions_vis_topic_.size() > 0)
    {
        config.predicted_positions_ground_vis_topic_ =
            groundProjectionTopic(
                config.predicted_positions_vis_topic_,
                config.n_layers_,
                config.layer_idx_);
    }
    else
    {
        config.predicted_positions_ground_vis_topic_ = "";
    }
    if (!getRequiredParameter(nh,"/topics/visualization/system_interface/predicted_regions"+layer_idx_str, config.predicted_regions_vis_topic_)){return errorFunc("/topics/visualization/system_interface/predicted_regions"+layer_idx_str);};
    // Modules
    if (!getParameterOrDefault(nh,"/topics/visualization/modules/goal_oriented/goal_position"+layer_idx_str, config.goal_position_vis_topic_, std::string("/goal_position"))){defaultFunc("/topics/visualization/modules/goal_oriented/goal_position"+layer_idx_str);};
    if (config.goal_position_vis_topic_.size() > 0)
    {
        config.goal_position_ground_vis_topic_ =
            groundProjectionTopic(
                config.goal_position_vis_topic_,
                config.n_layers_,
                config.layer_idx_);
    }
    else
    {
        config.goal_position_ground_vis_topic_ = "";
    }
    if (!getParameterOrDefault(nh,"/topics/visualization/modules/reference_trajectory/reference_trajectory"+layer_idx_str, config.reference_trajectory_vis_topic_, std::string("/reference_trajectory"))){defaultFunc("/topics/visualization/modules/reference_trajectory/reference_trajectory"+layer_idx_str);};
    if (config.reference_trajectory_vis_topic_.size() > 0)
    {
        config.reference_trajectory_ground_vis_topic_ =
            groundProjectionTopic(
                config.reference_trajectory_vis_topic_,
                config.n_layers_,
                config.layer_idx_);
    }
    else
    {
        config.reference_trajectory_ground_vis_topic_ = "";
    }
    if (!getParameterOrDefault(nh,"/topics/visualization/modules/static_polyhedron_constraints/constraints"+layer_idx_str, config.constraints_vis_topic_, std::string("/constraints"))){defaultFunc("/topics/visualization/modules/static_polyhedron_constraints/constraints"+layer_idx_str);};

    /* Recording */
    // System interface
    if (!getParameterOrDefault(nh,"/topics/recording/system_interface/current_state", config.cur_state_rec_topic_, std::string("/mpc/rec/current_state"))){defaultFunc("/topics/recording/system_interface/current_state");};
    if (!getParameterOrDefault(nh,"/topics/recording/system_interface/predicted_trajectory"+layer_idx_str, config.pred_traj_rec_topic_, std::string("/mpc/rec/predicted_trajectory"))){defaultFunc("/topics/recording/system_interface/predicted_trajectory"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/topics/recording/system_interface/slack"+layer_idx_str, config.slack_rec_topic_, std::string("/mpc/rec/slack"))){defaultFunc("/topics/recording/system_interface/slack"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/topics/recording/system_interface/time_reference"+layer_idx_str, config.time_reference_rec_topic_, std::string("/mpc/rec/time_reference"))){defaultFunc("/topics/recording/system_interface/time_reference"+layer_idx_str);};
    // Modules
    if (!getParameterOrDefault(nh,"/topics/recording/modules/goal_oriented/goal_position"+layer_idx_str, config.goal_position_rec_topic_, std::string("/mpc/rec/goal_position"))){defaultFunc("/topics/recording/modules/goal_oriented/goal_position"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/topics/recording/modules/reference_trajectory/reference_trajectory"+layer_idx_str, config.reference_trajectory_rec_topic_, std::string("/mpc/rec/reference_trajectory"))){defaultFunc("/topics/recording/modules/reference_trajectory/reference_trajectory"+layer_idx_str);};
    if (!getParameterOrDefault(nh,"/topics/recording/modules/static_polyhedron_constraints/constraints"+layer_idx_str, config.constraints_rec_topic_, std::string("/mpc/rec/reference_trajectory"))){defaultFunc("/topics/recording/modules/static_polyhedron_constraintss/constraints"+layer_idx_str);};

    /* Hierarchical communication */
    if (!getParameterOrDefault(nh,"/topics/hierarchical/start_mpc_layer", config.start_mpc_layer_topic_, std::string("/hierarchical/start_mpc_layer"))){defaultFunc("/topics/hierarchical/start_mpc_layer");};
    if (!getParameterOrDefault(nh,"/topics/hierarchical/reference", config.ref_topic_, std::string("/hierarchical/reference"))){defaultFunc("/topics/hierarchical/reference");};
    if (!getParameterOrDefault(nh,"/topics/hierarchical/pmpc_objective_reached", config.pmpc_obj_reached_topic_, std::string("/hierarchical/pmpc_objective_reached"))){defaultFunc("/topics/hierarchical/pmpc_objective_reached");};
    if (!getParameterOrDefault(nh,"/topics/hierarchical/pmpc_failure", config.pmpc_failure_topic_, std::string("/hierarchical/pmpc_failure"))){defaultFunc("/topics/hierarchical/pmpc_failure");};
    /************************************************************/

    RCLCPP_WARN_STREAM(rclcpp::get_logger("mpc"),"[MPC config] Parameter configuration initialized");

    return true;
}

bool errorFunc(const std::string name)
{
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("mpc"),"[MPC config] Parameter \""+name+"\" not defined!");
    return false;
}

void defaultFunc(const std::string name)
{
    RCLCPP_WARN_STREAM(rclcpp::get_logger("mpc"),"[MPC config] Parameter \""+name+"\" not defined, using default value (see update_mpc_configuration.cpp)!");
}

void rosRecordingWarning(const std::string name)
{
    RCLCPP_WARN_STREAM(rclcpp::get_logger("mpc"),"[MPC config] Recording \""+name+"\" is enabled, but recording ROS data is disabled, so \""+name+"\" will not be recorded!");
}
