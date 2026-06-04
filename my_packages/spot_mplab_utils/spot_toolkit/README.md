# Spot Toolkit

This package is to provide tools and utilities for working with the Spot robot. This package includes scripts, nodes, and configurations to simplify interaction with Spot in a ROS 2 environment.

## Features

- Helper function
    - Conversions between msg types
    - **Vision utilities**, that should come very handy! I suggest taking a look it may come very handy, especially the `CameraManager` class.
    - And other utilities 
- Action relay(s) for simpler control
- And more to come!

## Nodes

> [!TIP] 
> Run them all with `ros2 run spot_toolkit <node> `

### Trajectory relay

This node is subscribed to `/pose_goal` topic (with `PoseStamped` msg type) and then sends a trajectory goal to the robot. Incoming new poses cancel the previous action, so that a continous input topic feed can control the robots position. The pose can be from any frame, as the node always converts it to body.

This node is useful for object tracking an continious position-based movement control. I recommend [remapping](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#remapping) the node to the project specific topic if you are making a launchfile.

### Velocity relay

In theory `/cmd_vel` should be good to do velocity control, but ive found it stutters too much, and for some reason its inaccurate (sending it forward velocity makes it go a bit sideways(?)). So this new node is just a substitution for that. It waits for msgs on `/velocity_goal` and sends a velocity request to the robot. Any new msgs will cancel the current request and issue a new one. Until the problem with cmd_vel is not figured out, i suggest to use this node for velocity control.

### Bag recorder

A nice graphical interface with big buttons to record bags. Edit the config file for this to select what to record. The best use for this is with in pair with the Orin during recording outdoor data, as its size makes it very easy to control from the phone (using NoMachine on the orin)

### Simple spotty commander

A modified version of boston dynamics simple_spot_commander. Added a configurable service list, to have as many or as few trigger services be called from one function.

## Contributing

Contributions are welcome! Please submit a pull request with your changes.