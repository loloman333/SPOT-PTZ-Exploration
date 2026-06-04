> **Note:** This README was AI-generated and could contain inaccuracies

# ptz_exploration_sim

Simulation package for the PTZ exploration stack.

This package contains the Gazebo/ROS 2 simulation assets used to validate the active visual search pipeline without hardware. It provides:

- a configurable world launcher,
- a robot spawn launcher,
- a PTZ simulator node,
- URDF/Xacro descriptions for the base robot and sensors,
- Gazebo world files and scene variants,
- RViz and runtime configuration for simulation.

## Package contents

### Launch files

- `launch/world.launch.py`  
	Starts the Gazebo world selected by the active config file. It loads `config/config.yaml`, resolves the world name, optionally expands a `.xacro.sdf` world template, sets the Gazebo resource path, and launches `ros_gz_sim`.

- `launch/robot.launch.py`  
	Spawns the robot into the running simulation, publishes the robot description from Xacro, starts `robot_state_publisher`, creates the fixed TF links used by the sim, and sets up the Gazebo-to-ROS bridge topics. If enabled in config, it also launches RViz, the PTZ simulator node, and the synthetic 360° camera array.

### Runtime configuration

- `config/config.yaml`  
	Main simulation configuration file. It defines the default world, the initial robot pose, whether the PTZ camera and 360° cameras are simulated, RViz startup, and PTZ simulator parameters such as zoom limits, output resolution, and joint names.

- `config/sim.rviz`  
	RViz layout used during simulation development and debugging.

### Robot description

- `urdf/base.urdf.xacro`  
	Differential-drive base model with wheels, casters, odometry, joint-state publishing, and the Gazebo diff-drive plugin.

- `urdf/camera.urdf.xacro`  
	Generic fixed depth-camera macro used for the simulated 360° camera ring. It defines the optical frame and a Gazebo depth sensor.

- `urdf/ptz.urdf.xacro`  
	PTZ camera macro with pan/tilt joints, Gazebo joint position controllers, and the raw high-resolution camera sensor used by the PTZ simulator backend.

- `urdf/robot.urdf.xacro`  
	Top-level robot description that composes the base and optional sensors. The launch files toggle the PTZ camera and 360° camera ring through Xacro arguments.

### Worlds

The `worlds/` folder contains the simulation scenes used for experiments and development. The active world is selected through `config/config.yaml`.

- `depot.xacro.sdf`  
	Xacro-based world template for the depot scenario.

- `depot_objects.yaml`  
	Object placement or scene configuration used by the depot template.

- `small_house.sdf`  
	Indoor house-like scenario.

- `urban_circuit_01.sdf`  
	Urban-style navigation world.

### Python package

- `ptz_exploration_sim/ptz_simulator.py`  
	Main simulator node exposed as the `ptz_simulator` console script. It subscribes to PTZ commands, joint states, and raw camera messages; crops and resizes the high-resolution PTZ image according to the current zoom; republishes the simulated PTZ image and camera info; and publishes PTZ state plus a settled/not-settled flag.

- `ptz_exploration_sim/utils/math.py`  
	Small math helper module with matrix and quaternion conversion utilities used by the simulation code.

- `ptz_exploration_sim/utils/ign.py`  
	Reserved helper module for Gazebo/Ignition-specific utilities. Currently empty.

## How the simulation is wired

The simulation launchers rely on the shared config loader from `ptz_exploration_core`:

- `handle_config_file(...)` loads `config/config.yaml` and returns the merged ROS parameters.
- `world.launch.py` uses the selected world name to resolve the world file and optional xacro template.
- `robot.launch.py` publishes the robot description and sets up the following bridge topics:
	- `/cmd_vel`
	- `/odom`
	- `/tf`
	- `/joint_states`
	- `/clock`
	- PTZ image and camera info topics when PTZ simulation is enabled
	- depth image and point cloud topics for the 360° camera ring when enabled

The simulated PTZ backend crops the raw camera image around the center and rescales it to the configured output resolution. It also updates the camera intrinsics to match the effective zoom level, so downstream perception nodes receive a consistent image geometry.

## Running the package

Launch the Gazebo world first, then spawn the robot:

```bash
ros2 launch ptz_exploration_sim world.launch.py
ros2 launch ptz_exploration_sim robot.launch.py
```

The launch files read `config/config.yaml` by default. You can override it with `config_file:=...`.

Example:

```bash
ros2 launch ptz_exploration_sim world.launch.py config_file:=/path/to/config.yaml
ros2 launch ptz_exploration_sim robot.launch.py config_file:=/path/to/config.yaml
```

## Development notes

- The package is installed as an `ament_python` package via `setup.py`.
- The `ptz_simulator` entry point is the main node to inspect when changing PTZ behavior.
- The URDF/Xacro files are designed to be composable, so sensor combinations can be toggled without changing the core launch logic.
- Most runtime parameters are intentionally centralized in `config/config.yaml` so that new worlds or sensor settings can be tested without code changes.

If you extend the package, prefer adding new simulation behavior here and keep the exploration logic itself in `ptz_exploration_core`.
