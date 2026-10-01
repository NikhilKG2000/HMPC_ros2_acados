# HMPC for HoverGames on ROS 2

The HoverGames two-layer HMPC uses the original model, objectives,
constraints, and offline constants with acados-generated TMPC and PMPC
solvers. The ROS 2 workspace includes `mpc_base`, `mpc_core`, `mpc_modules`,
`mpc_msgs`, `mpc_solver`, `mpc_tools`, `mpc_hovergames`, `simple_sim`,
`occupancygrid_creator`, and `DecompUtil`. All ten packages are required
for the simulation.

## Prerequisites

Install and source ROS 2 Humble, including `ament_cmake`, `colcon`, and the
ROS message packages used by the workspace. Also install Eigen3, Python 3,
NumPy, SciPy, CasADi, and `acados_template`.

Build acados with shared libraries and set its source directory before
building this workspace:

```bash
export ACADOS_SOURCE_DIR=/path/to/acados
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
source /opt/ros/humble/setup.bash
```

The acados Python interface must be importable by the active Python
interpreter. From the acados checkout, this is commonly installed with:

```bash
python3 -m pip install -e "$ACADOS_SOURCE_DIR/interfaces/acados_template"
```

From the workspace root, after the prerequisites are available:

```bash
# Required after removing generated/build files or after a fresh checkout.
python3 src/mpc/mpc_solver/scripts/acados/generate_hovergames_tmpc.py
python3 src/mpc/mpc_solver/scripts/acados/generate_hovergames_pmpc.py
colcon build --packages-up-to mpc_hovergames --symlink-install
source install/setup.bash
ros2 run mpc_solver test_hovergames_tmpc_adapter
ros2 run mpc_solver test_hovergames_pmpc_adapter
```

The generator commands must run before `colcon build`: CMake compiles their
generated acados C code but does not generate it automatically. If the
workspace has been cleaned, remove only `build/`, `install/`, and `log/` for a
normal rebuild; removing `src/mpc/mpc_solver/acados_generated/` also requires
running both generator commands again.

Run `occupancygrid_creator/occupancygrid.launch.py`,
`simple_sim/simple_sim.launch.py`, and
`mpc_hovergames/hmpc_simplesim.launch.py` in separate sourced terminals.
The controller subscribes to `/goal` as `geometry_msgs/msg/PoseStamped`.
