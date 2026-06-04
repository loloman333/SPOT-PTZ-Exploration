> **Note:** This README was AI-generated and could contain inaccuracies

# ptz_exploration_core

Core package for the PTZ exploration stack. This package contains the perception, mapping/optimization, exploration and utilities that implement the active visual search pipeline.

It exposes the main runtime nodes used by simulation and real-robot runs, launch helpers and small analysis utilities.

## What this package is for

`ptz_exploration_core` implements the algorithmic components: object detection, landmark collection and factor-graph optimization, frontier-based exploration, data collection and plotting utilities used to evaluate experiments.

The package intentionally separates simulation/hardware specifics (in `ptz_exploration_sim` and `ptz_exploration_spot`) from the core algorithmic logic so nodes can run unchanged in both environments.

## Package contents

### Launch

- `launch/launch.py`  
	Single entry-point launcher that reads a config file (`config_sim.yaml` or `config_spot.yaml`), and conditionally brings up subsystem nodes: octomap, Nav2, YOLO detector, projector/optimizer (factor-graph), gazer, and other core nodes based on config flags.

### Nodes / entrypoints

#### C++ nodes

- `src/cloud_in.cpp`  
	Point cloud aggregator. Subscribes to point cloud topics from depth cameras and republishes them on a unified topic for OctoMap consumption.

- `src/projector.cpp`  
	Detection projection node. Subscribes to 2D detections and point clouds, projects detections into 3D space via OctoMap ray-casting, and publishes `DetectionMeasurementArray` messages for the optimizer.

- `src/optimizer.cpp`  
	Factor graph optimization backend using GTSAM/iSAM2. Subscribes to `DetectionMeasurementArray` messages, manages landmark data association, solves the nonlinear least-squares problem, and publishes `LandmarkArray` with optimized 3D positions and uncertainties. Also writes JSON snapshots for debugging.

- `src/gazer.cpp`  
	PTZ camera control node. Subscribes to PTZ command messages and publishes camera pan/tilt/zoom joint commands or services. Also provides visualization and settled status feedback.

#### Python nodes

- `ptz_exploration_core/detector.py`  
	YOLO-based detection node. Subscribes to camera image topics, runs the Ultralytics YOLO model, and publishes `vision_msgs/Detection2DArray` on `/detections`. Configurable model, classes, device and per-camera topics via the config file.

- `ptz_exploration_core/explorer.py`  
	Frontier-based high-level exploration node. Computes frontiers from the projected 2D occupancy map, ranks them (cells vs distance), dispatches goals to Nav2 via `nav2_simple_commander`, and orchestrates scan/confirm states (PTZ scans, body rotation, or service-based scanning).

- `ptz_exploration_core/collector.py`  
	Experiment data collector. Subscribes to `LandmarkArray` and the projected 2D map, tracks distance traveled (via TF), battery states, publishes optional ground-truth markers (simulation), and writes periodic CSV snapshots for offline analysis.

#### Analysis utilities

- `ptz_exploration_core/plotter.py`  
	Offline plotting utility that turns collector CSV output into time-series plots. Uses `config/plots.yaml` to define which metrics to plot.

- `ptz_exploration_core/plot_factor_graph.py`  
	Small utility to create 2D plots of the optimizer's JSON snapshots (poses, landmarks, edges) for debugging and presentation figures.

### Messages

Custom message types are defined in `msg/` and used across the core stack:

- `DetectionMeasurement.msg`, `DetectionMeasurementArray.msg` — internal measurement wrappers
- `Landmark.msg`, `LandmarkArray.msg` — landmark state messages published by the optimizer

### Config files

- `config/config_sim.yaml`  
	Simulation default parameters (topics, node enable flags, detector and optimizer options).

- `config/config_spot.yaml`  
	Spot/hardware-specific runtime mapping and tuned parameters.

- `config/nav2_params_sim.yaml` and `config/nav2_params_spot.yaml`  
	Nav2 tuning for simulator and Spot robot respectively.

- `config/nav2_behavior_tree.xml`  
	Behavior tree used for Nav2 during exploration runs.

- `config/plots.yaml`  
	Plot group definitions used by `plotter.py`.

### Utilities

#### Python utilities

- `ptz_exploration_core/utils/math.py`  
	Grid/world helpers, path-length computation and geometric utilities used by the explorer and collector.

- `ptz_exploration_core/utils/ros.py`  
	Common ROS helpers: `handle_config_file()` used by launch files to load merged config params, image/pointcloud helpers, and marker publishing helpers.

#### C++ utilities

- `include/utils/data_association.hpp` / `src/utils/data_association.cpp`  
	Data association logic for matching detections to landmarks using Mahalanobis distance and chi-square gating.

- `include/utils/math.hpp` / `src/utils/math.cpp`  
	Geometric and linear algebra utilities (transforms, projections, covariance handling) used by projector and optimizer.

- `include/utils/ros.hpp` / `src/utils/ros.cpp`  
	ROS message building and conversion helpers for C++ nodes.

## How the core package is wired

- The main launcher `launch/launch.py` calls `handle_config_file()` to load parameters. Depending on flags in the config it will:
	- start an OctoMap server (cloud_in + octomap_server) when `launch_octomap` is true,
	- include Nav2 bringup when `launch_nav2` is true,
	- start the YOLO `detector` node when `launch_detector` is true,
	- start `gazer`, `projector` and `optimizer` nodes for measurement projection and factor-graph optimization when requested.

- Topics and services the core nodes use (defaults from configs):
	- `/detections` — publishes `vision_msgs/Detection2DArray` from the detector
	- `/landmarks` — publishes `ptz_exploration_core/LandmarkArray` from the optimizer
	- `/projected_map` — 2D projected occupancy map used by the explorer
	- `/exploration_goal` — published PoseStamped for downstream navigation
	- `/scan_environment` (service) — trigger a PTZ or body-based scan
	- `/confirm_landmarks` (service) — verification/confirmation service for candidate landmarks

## Running the core stack

Start the full core stack (after launching the sim or hardware drivers):

```bash
ros2 launch ptz_exploration_core launch.py
```

Control which subcomponents are started by passing an alternate config file:

```bash
ros2 launch ptz_exploration_core launch.py config_file:=/path/to/config_sim.yaml
ros2 launch ptz_exploration_core launch.py config_file:=/path/to/config_spot.yaml
```

## Development notes

- Keep algorithmic changes inside this package. Simulation/hardware-specific code belongs to `ptz_exploration_sim` and `ptz_exploration_spot` respectively.
- Use `config/config_sim.yaml` as the development baseline; `config_spot.yaml` contains hardware mappings.
- The `detector` node uses Ultralytics YOLO — check `config_*` to set `yolo_model` and `yolo_device`.
- When troubleshooting data association or optimizer issues, use `plot_factor_graph.py` on optimizer JSON snapshots to visualize pose/landmark topology.
