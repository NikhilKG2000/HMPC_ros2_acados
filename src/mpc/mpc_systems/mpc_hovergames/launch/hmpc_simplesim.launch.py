"""Start the two HMPC controller layers against the simple simulator topics."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


_INTEGER_PARAMETERS = {
    "n_layers",
    "layer_idx",
    "params.solver.time_shift",
    "params.robot.n_dim",
    "params.modules.static_polyhedron_constraints.n_constraints_per_stage",
    "params.modules.static_polyhedron_constraints.occupied_threshold",
    "params.modules.static_polyhedron_constraints.occ_cell_selection_method",
    "params.modules.static_polyhedron_constraints.line_segment_to_stage",
    "params.visualization.system_interface.draw_current_position_ground_air",
    "params.visualization.system_interface.region_min_n_points",
    "params.visualization.system_interface.current_region_n_points",
    "params.visualization.system_interface.predicted_regions_n_points",
    "params.visualization.system_interface.draw_predicted_positions_ground_air.0",
    "params.visualization.system_interface.draw_predicted_positions_ground_air.1",
    "params.visualization.modules.goal_oriented.draw_goal_position_ground_air",
    "params.visualization.modules.reference_trajectory.draw_reference_trajectory_ground_air",
}

_INTEGER_ARRAY_PARAMETERS = {
    "params.visualization.system_interface.region_indices_to_draw",
    "params.visualization.modules.static_polyhedron_constraints.indices_to_draw.0",
    "params.visualization.modules.static_polyhedron_constraints.indices_to_draw.1",
}


def _normalize_value(parameter_name, value):
    """Make YAML numeric arrays valid homogeneous ROS 2 parameter arrays."""
    if isinstance(value, list):
        if value and all(
                isinstance(item, (int, float)) and not isinstance(item, bool)
                for item in value):
            if parameter_name in _INTEGER_ARRAY_PARAMETERS:
                return [int(item) for item in value]
            return [float(item) for item in value]
        return [_normalize_value(parameter_name, item) for item in value]
    if isinstance(value, int) and not isinstance(value, bool):
        if parameter_name in _INTEGER_PARAMETERS:
            return value
        return float(value)
    return value


def _flatten_parameters(mapping, prefix=""):
    """Convert the original ROS 1 parameter tree to ROS 2 dotted names."""
    parameters = {}
    for raw_key, value in mapping.items():
        key = str(raw_key).strip("/").replace("/", ".")
        full_key = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            parameters.update(_flatten_parameters(value, full_key))
        else:
            parameters[full_key] = _normalize_value(full_key, value)
    return parameters


def _load_parameters(path):
    with path.open("r", encoding="utf-8") as stream:
        contents = yaml.safe_load(stream) or {}
    return _flatten_parameters(contents)


def generate_launch_description():
    package_share = Path(get_package_share_directory("mpc_hovergames"))

    parameters = _load_parameters(package_share / "config" / "common.yaml")
    parameters.update(
        _load_parameters(package_share / "config" / "simplesim_hmpc.yaml"))
    parameters.update({"sim": True, "n_layers": 2})

    tracker_parameters = dict(parameters)
    tracker_parameters.update(
        _load_parameters(
            package_share
            / "config"
            / "dynamic_reconfigure"
            / "tmpc_reconfigure_params.yaml"
        )
    )
    tracker_parameters["layer_idx"] = 0

    planner_parameters = dict(parameters)
    planner_parameters.update(
        _load_parameters(
            package_share
            / "config"
            / "dynamic_reconfigure"
            / "pmpc_reconfigure_params.yaml"
        )
    )
    planner_parameters["layer_idx"] = 1

    return LaunchDescription([
        Node(
            package="mpc_hovergames",
            executable="mpc_control_node_2_layer",
            name="tracker_node",
            arguments=["mpc_0"],
            parameters=[tracker_parameters],
            output="screen",
        ),
        Node(
            package="mpc_hovergames",
            executable="mpc_control_node_2_layer",
            name="planner_node",
            arguments=["mpc_1"],
            parameters=[planner_parameters],
            output="screen",
        ),
    ])
