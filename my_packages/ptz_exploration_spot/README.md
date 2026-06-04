> **Note:** This README was AI-generated and could contain inaccuracies

# ptz_exploration_spot

Spot-specific integration package for the PTZ exploration stack.

This package provides hardware drivers, sensor bridges, and configuration needed to run the active visual search pipeline on Boston Dynamics Spot robots equipped with the SpotCAM payload.

## What this package is for

`ptz_exploration_spot` concentrates on Spot-specific concerns: SD-RTC WebRTC streaming from SpotCAM, PTZ control via the Spot SDK, payload transform publishing, and hardware parameter tuning.

The core algorithmic nodes (from `ptz_exploration_core`) run unchanged; this package provides the hardware integration layer and passthrough nodes that adapt Spot's proprietary APIs to standard ROS 2 interfaces.

## Package contents

### Launch

- `launch/launch.py`  
	Main entry-point launcher that reads a config file (`config/config.yaml`), includes the standard Spot driver bringup from `our_launchers/spotty.launch.py`, and brings up the PTZ wrapper node.

### Nodes / entrypoints

- `ptz_exploration_spot/ptz_wrapper.py`  
	SpotCAM PTZ bridge. Connects to the Spot robot via the Boston Dynamics SDK, manages the PTZ hardware interface, polls position/zoom state, publishes PTZ joint state and settled flags, subscribes to ROS 2 PTZ commands, and streams video via WebRTC if enabled.

### Config files

- `config/config.yaml`  
	Spot hardware parameters: hostname, credentials, camera/stream settings, DDS configuration, PTZ tuning, and feature flags (e.g., auto-power, auto-stand, velodyne enable, SpotCAM init).

- `config/custom_dds_profile.xml`  
	DDS middleware profile for optimized communication with Spot's network stack.

- `config/spot.rviz`  
	RViz layout and display settings customized for Spot-specific topics and coordinate frames.

## How the Spot integration is wired

- `launch/launch.py` calls `handle_config_file()` to merge ROS parameters, then:
  - includes the upstream Spot driver launcher from `our_launchers` (brings up base drivers, TF, joint state, hardware interface),
  - launches the `ptz_wrapper` node configured with credentials and SpotCAM parameters.

- The `ptz_wrapper` node:
  - authenticates to the robot via the Boston Dynamics SDK,
  - discovers the SpotCAM payload and publishes its static TF,
  - creates clients for PTZ, media log (kinematics), compositor, and stream quality,
  - subscribes to `/ptz_cmd` (geometry_msgs/Point) and dispatches requests via the Spot `/spotty/set_ptz_position` service,
  - publishes `/ptz_state` (JointState) and `/ptz_settled` (Bool) status,
  - optionally streams video via WebRTC and republishes frames on `/spotty/camera/ptz/image`.

- Key topics and services:
  - `/ptz_cmd` — input Point (x=pan, y=tilt, z=zoom) from exploration stack,
  - `/ptz_state` — JointState with pan/tilt/zoom positions and velocities,
  - `/ptz_settled` — Bool flag indicating PTZ is at target,
  - `/spotty/set_ptz_position` (service) — Spot SDK-backed PTZ control,
  - `/spotty/camera/ptz/image` — WebRTC video stream (if enabled),
  - `/spotty/camera/ptz/camera_info` — camera intrinsics for the PTZ feed.

- Static transforms:
  - `spotty/body` → `spotty/spot_cam_origin` — payload mount transform from robot state,
  - `spotty/spot_cam_origin` → `spotty/ptz` — PTZ optical frame (set up by external calibration/launch files).

## Running the stack on Spot

Start the full Spot integration (ensure robot is powered and network is reachable):

```bash
ros2 launch ptz_exploration_spot launch.py
```

Pass an alternate config file if needed:

```bash
ros2 launch ptz_exploration_spot launch.py config_file:=/path/to/custom_config.yaml
```

After Spot drivers are up, launch the core exploration stack:

```bash
ros2 launch ptz_exploration_core launch.py config_file:=/path/to/config_spot.yaml
```

## Configuration notes

### Credentials and network

Edit `config/config.yaml` to set:
- `username` and `password` for Spot authentication,
- `hostname` — IP address of the Spot robot (e.g., `192.168.50.3` for ethernet, `192.168.80.3` for WiFi),
- `port` — optional non-standard Spot API port (usually 0 for default).

### SpotCAM settings

- `initialize_spot_cam` — Set to `True` if the payload module is enabled on the robot.
- `cameras_used` — List of Spot cameras to initialize (e.g., `["frontleft", "frontright", "left", "right", "back"]`).
- `publish_point_clouds` — Generate and publish pointclouds from depth data.
- `webrtc_bitrate_bps` — Video stream bitrate; reduce if local CPU struggles with decoding.

### PTZ-specific

- `ptz_name` — Name of the PTZ in Spot's camera system (typically `"mech"`).
- `stream_topic` — ROS 2 topic where WebRTC video is republished.
- `stream_publish_rate_hz` — Publishing frequency of video frames (reduce to cut CPU load).

## Dependencies

Declared in `package.xml`:
- `rclpy`, `geometry_msgs`, `sensor_msgs` (ROS 2 core),
- `spot_msgs` (Spot SDK message definitions),
- `bosdyn.client` (Boston Dynamics SDK — assumed installed in dev container).

## Development notes

- This package is intentionally thin; hardware-independent logic lives in `ptz_exploration_core`.
- The SpotCAM WebRTC stream runs in a background thread to avoid blocking the main callback loop.
- PTZ state polling uses a separate timer group to prevent interference with streaming.
- The `ptz_wrapper` uses the Spot SDK's service clients; ensure your container has `bosdyn-client` installed and credentials are valid.
- For offline development/testing, focus on `ptz_exploration_core` and `ptz_exploration_sim`; this package is only needed when deploying to hardware.

To extend: add new Spot-specific drivers or payload integrations following the same pattern as `ptz_wrapper`.
