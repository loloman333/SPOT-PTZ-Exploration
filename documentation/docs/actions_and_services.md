# Actions and Services 

One of the most important task is to properly handle **Spot**'s actions and services, as they are responsible for moving the robot around. This section focuses on the general use of these commands and quirks of some that need extra explaining.

!!! danger 
    **A faulty or misintended code can make the robot do unexpected, sometimes dangerous things that may harm itself or its surroundings. Be extra precautious when writing these commands.**

*A good overview of these interfaces are currently officially documented. Before reading further, look over the ==[Spot Driver Available Interfaces](https://github.com/bdaiinstitute/spot_ros2/wiki/Spot-Driver-Available-Interfaces)==.*

As could be read, it's a good list to familiarizes themselves with the options to command the robot, but not much else is there besides that. Down below is the extra information that should be highlighted in my opinion.

## Actions

This section is well written by the boston dynamics team, so this will be brief.

Most command can be made with ==bosdyn.client.robot_command.RobotCommandBuilder==, a class with static helper functions. Here's the [documentation](https://dev.bostondynamics.com/python/bosdyn-client/src/bosdyn/client/robot_command.html#bosdyn.client.robot_command.RobotCommandBuilder) from the sdk's wiki. Some commands are not in the Builder, so manual assembly is required. Consult the SDK in how to make these commands, either the [Python Reference Guide](https://dev.bostondynamics.com/python/readme) , the examples in the [SDK github page](https://github.com/boston-dynamics/spot-sdk/tree/master/python/examples) or the [ROS2 page](https://github.com/bdaiinstitute/spot_ros2/tree/main/spot_examples).

!!! note
    Most function have a **build_on_command** argument, that accepts an other RobotCommand. With this you can nest command into each other, so the robot can do thing all at once. (For example move forward and move its arm as well.)

## Services

Almost all services can be called trough the terminal, using the **Trigger** standard service type:

```
ros2 service call /<spot_name>/<service> std_srvs/srv/Trigger
```
The call returns **Trigger.Response**, which contains .success=True if the request was successful, False and .message="The error message" if not. 
!!! note
    *success=True does not 100% guarantee that the robot did what you asked for! Faults in the communication, the robot or the driver may give false positives. Although not that common, it's good to understand this principle.*


While offering less control than actions, they are a quick way to make the robot do things, like standing, sitting or opening its gripper.  
For example:

- **/stand** stand the robot up to a default positions, but with actions, you can specify height or to stand twisted.
- **/rollover** rolls the robot over to the right, but with actions you can specify to roll to the left.

### The Robot_command service

As it's stated in the [spot ros2 wiki](https://github.com/bdaiinstitute/spot_ros2/wiki/Spot-Driver-Available-Interfaces), the **/robot_command** service is a way to quickly send customized actions to the robot. However some important stuff was *left out.*
 
#### Time-dependent commands

Commands that don't have finalized goals (like an end point, pose etc.) need a way to tell when should they end. The most used example is **synchro_velocity_command** that after being converted form protobuf2 to a service, needs a ==duration== , to tell for how long should the robot have this velocity.  
Heres an example:

```python3
from bosdyn.client.robot_command import RobotCommandBuilder
from spot_msgs.srv import RobotCommand as RobotCommandService 
from bosdyn_msgs.conversions import convert

#Make the pb2 command
velocity_pb2_command = RobotCommandBuilder.synchro_velocity_command(v_x,v_y,w)
request = RobotCommandService.Request()
#convert it to a service request
convert(velocity_pb2_command, request.command)
#add a duration AFTER it was converted
request.duration.sec = VELOCITY_CMD_DURATION
#and then send the request..
```

The other is the drag command, that for some reason does not work the same as the carry command on a set/reset principle. This also needs a request to be sent with a duration to specify.

```
from bosdyn.api include arm_command_pb2,synchronized_command_pb2,basic_command_pb2, robot_command_pb2
from bosdyn.client.robot_command import RobotCommandBuilder
from spot_msgs.srv import RobotCommand as RobotCommandService 
from bosdyn_msgs.conversions import convert

arm_command = arm_command_pb2.ArmCommand.Request(
    arm_drag_command=basic_command_pb2.ArmDragCommand.Request())
synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
    arm_command=arm_command)
dragcommand = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)

#The process is the same as above...
```

