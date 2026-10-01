# HMPC ROS 2

ROS 2 implementation of the HoverGames two-layer HMPC system using acados
for the TMPC tracking solver and PMPC planning solver.

The acados solver generators use the original HMPC dynamics, objectives,
constraints, constants, and system settings. Generated solver code is created
locally before building and is not required to be committed to the repository.

## Workspace Packages

This workspace contains:

- `mpc_base`
- `mpc_core`
- `mpc_modules`
- `mpc_msgs`
- `mpc_solver`
- `mpc_tools`
- `mpc_hovergames`
- `simple_sim`
- `occupancygrid_creator`
- `DecompUtil`

## Prerequisites

Install the following software:

- Ubuntu
- ROS 2 Humble
- `colcon`
- Eigen3
- Python 3
- NumPy
- SciPy
- CasADi
- acados
- `acados_template`

ROS 2 must be sourced before building:

```bash
source /opt/ros/humble/setup.bash
```

Build acados with shared libraries, then configure its location:

```bash
export ACADOS_SOURCE_DIR=/path/to/acados
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
```

For example:

```bash
export ACADOS_SOURCE_DIR="$HOME/acados"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
```

Install the acados Python interface:

```bash
python3 -m pip install -e "$ACADOS_SOURCE_DIR/interfaces/acados_template"
```

Verify it:

```bash
python3 -c "import acados_template; print('acados_template is available')"
```

## Clone the Repository

```bash
git clone <REPOSITORY_URL>
cd hmpc_ros2
```

Source ROS 2 and configure acados:

```bash
source /opt/ros/humble/setup.bash
export ACADOS_SOURCE_DIR=/path/to/acados
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
```

## Build and Run

The generated acados C code is not stored in the repository. Generate both
solvers before running `colcon build`:

```bash
python3 src/mpc/mpc_solver/scripts/acados/generate_hovergames_tmpc.py
python3 src/mpc/mpc_solver/scripts/acados/generate_hovergames_pmpc.py
```

These commands generate the solver C code, model functions, cost functions,
constraint functions, and solver metadata for TMPC and PMPC.

`colcon build` does not generate this code automatically.

```bash
colcon build --symlink-install
source install/setup.bash
```

## Run the Simulation

Open four terminals. Run the following setup in each terminal before running
the command for that terminal:

```bash
source /opt/ros/humble/setup.bash
export ACADOS_SOURCE_DIR=/path/to/acados
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
source install/setup.bash
```

### Terminal 1: Obstacle Generator

The occupancy-grid creator loads the default HoverGames obstacle map and
publishes the grid used by the MPC obstacle constraints:

```bash
ros2 launch occupancygrid_creator occupancygrid.launch.py
```

To use another map:

```bash
ros2 launch occupancygrid_creator occupancygrid.launch.py \
	map_file:=/absolute/path/to/map.yaml
```

### Terminal 2: Simple Simulator

```bash
ros2 launch simple_sim simple_sim.launch.py
```

The default initial position is `(x, y, z) = (0.0, 0.0, 1.0)`. Custom initial
positions can be passed as launch arguments:

```bash
ros2 launch simple_sim simple_sim.launch.py \
	x_init:=0.0 y_init:=0.0 z_init:=1.0
```

### Terminal 3: HMPC Controller

```bash
ros2 launch mpc_hovergames hmpc_simplesim.launch.py
```

### Terminal 4: RViz

```bash
rviz2 -d src/mpc/mpc_systems/mpc_hovergames/rviz/hmpc_ros2_manual.rviz
```

In RViz, use the **2D Goal Pose** tool and click in the map to send a goal on
the `/goal` topic.

## Send a Goal

The controller subscribes to `/goal` using
`geometry_msgs/msg/PoseStamped`. To send a goal without RViz:

```bash
ros2 topic pub --once /goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 1.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

This sends the drone to `x = 2.0`, `y = 1.0`, and `z = 1.0` with zero yaw.

The main simulation topics are:

```text
/occupancy_grid
/drone_hovergames/state
/drone_hovergames/control
/goal
/mpc/vis/constraints/layer_0
/mpc/vis/constraints/layer_1
/mpc/vis/predicted_positions/layer_0
/mpc/vis/predicted_positions/layer_1
```

## Optional Solver Tests

```bash
ros2 run mpc_solver test_hovergames_tmpc_adapter
ros2 run mpc_solver test_hovergames_pmpc_adapter
```