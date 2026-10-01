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

## Generate the acados Solvers

The generated acados C code is not stored in the repository. Generate both
solvers before running `colcon build`:

```bash
python3 src/mpc/mpc_solver/scripts/acados/generate_hovergames_tmpc.py
python3 src/mpc/mpc_solver/scripts/acados/generate_hovergames_pmpc.py
```

These commands generate the solver C code, model functions, cost functions,
constraint functions, and solver metadata for TMPC and PMPC.

`colcon build` does not generate this code automatically.

## Build the Workspace

From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Test the Solvers

```bash
ros2 run mpc_solver test_hovergames_tmpc_adapter
ros2 run mpc_solver test_hovergames_pmpc_adapter
```

Both commands should report that the solver adapter test passed.

## Run the Simulation

Open separate terminals. In every terminal, source the required environments:

```bash
source /opt/ros/humble/setup.bash
export ACADOS_SOURCE_DIR=/path/to/acados
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
source install/setup.bash
```

Start the occupancy-grid creator:

```bash
ros2 launch occupancygrid_creator occupancygrid.launch.py
```

Start the simple simulator:

```bash
ros2 launch simple_sim simple_sim.launch.py
```

Start the HoverGames HMPC controller:

```bash
ros2 launch mpc_hovergames hmpc_simplesim.launch.py
```

The controller subscribes to `/goal` using:

```text
geometry_msgs/msg/PoseStamped
```

## Clean the Workspace

Remove normal ROS and CMake build outputs:

```bash
rm -rf build install log
```

Remove generated acados code as well:

```bash
rm -rf src/mpc/mpc_solver/acados_generated
```

After removing generated acados code, regenerate both solvers before building:

```bash
python3 src/mpc/mpc_solver/scripts/acados/generate_hovergames_tmpc.py
python3 src/mpc/mpc_solver/scripts/acados/generate_hovergames_pmpc.py
colcon build --symlink-install
source install/setup.bash
```

## Complete Fresh Setup

After cloning a clean repository, the complete order is:

```bash
cd hmpc_ros2
source /opt/ros/humble/setup.bash

export ACADOS_SOURCE_DIR=/path/to/acados
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"

python3 -m pip install -e "$ACADOS_SOURCE_DIR/interfaces/acados_template"

python3 src/mpc/mpc_solver/scripts/acados/generate_hovergames_tmpc.py
python3 src/mpc/mpc_solver/scripts/acados/generate_hovergames_pmpc.py

colcon build --symlink-install
source install/setup.bash

ros2 run mpc_solver test_hovergames_tmpc_adapter
ros2 run mpc_solver test_hovergames_pmpc_adapter
```

## Troubleshooting

### `ACADOS_SOURCE_DIR is not set`

```bash
export ACADOS_SOURCE_DIR=/path/to/acados
```

### `Could not find libacados`

```bash
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
ls "$ACADOS_SOURCE_DIR/lib"
```

The acados library directory should contain `libacados.so`, `libblasfeo.so`,
and `libhpipm.so`.

### Generated solver header not found

Run both generator commands again:

```bash
python3 src/mpc/mpc_solver/scripts/acados/generate_hovergames_tmpc.py
python3 src/mpc/mpc_solver/scripts/acados/generate_hovergames_pmpc.py
```

### `ModuleNotFoundError: acados_template`

```bash
python3 -m pip install -e "$ACADOS_SOURCE_DIR/interfaces/acados_template"
```

### ROS packages are not found

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Files Not Committed

The following files are generated or build-specific and are excluded from
Git:

```text
build/
install/
log/
src/mpc/mpc_solver/acados_generated/
```