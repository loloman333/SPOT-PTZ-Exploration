# Move commands, trajectories

In this section we look over the ways we can move the robot.

## Velocity based

The driver's API wrapper send a duration based velocity command, so any method in the ROS2 interface follows the same suit.

### Service
You send a velocity request to the driver with a set duration, and upon completion, it sends a response.
**The method of that is shown in an example in [Actions and Services](actions_and_services.md).**

### Topic
An easier, and more a ROS2 oriented way is to publish a `Twist` msg to `<robot_name>/cmd_vel`. As `Twist` does not support duration, the driver's parameter `cmd_duration` sets how long should the command last per message sent. 

!!! note
    During testing, i've found that `cmd_duration=0.8` makes smooth movement, with okay responsiveness. Too low duration makes the robot stutter, and too high makes it unideal for real-time control, that is for example needed for SLAM. 

!!! info
    Make sure the publishing interval is not lower than the command duration! Doing so will make the robot stutter, move in non desired ways. Of course a much higher value will move a robot in a segmented way, so i recommend making the intervals `0.1-0.5` higher than the cmd_duration 

Heres an example implementation:

```python
cmd_duration = 0.8 #you either set it manually, or:
#Call a service request to /spotty/spot_ros2/getparameters, and get the duration dynamically
cmd_duration = get_params_from_srv() #example def
vel_pub = node.create_publisher(Twist,/spotty/cmd_vel)
twist = Twist()
twist.linear.x = ..
..
#Option1 - while loop
while condition: #<- add some condition (like time passed)
    vel_pub.publish(twist)
    time.sleep(cmd_duration+0.4)
vel_pub.publish(Twist()) #<- to make sure it stops after the condition passed.
#Option2 - timer
global current_twist #or make a class value
current_twist = twist
def pub_vel():
    vel_pub.publish(current_twist)

node.create_timer(cmd_duration+0.4,callback=pub_vel)
#and now the current value will make move the robot accordingly. To stop it just set the vel to 0.
global_twist.linear.x += 0.1
#...
global_twist = Twist()
```


## Coordinate based

### Understanding Coordinates
Understanding coordinate systems and frames is crucial for movement commands. All movement actions require an **origin frame** (a reference). The two root frames are *vision* and *odom*. 

Use a **TFListener** to compute transforms between frames:
```python
from bdai_ros2_wrappers.tf_listener_wrapper import TFListenerWrapper
from bdai_ros2_wrappers.utilities import namespace_with
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME

#this is in a class that receives a node! (main.node) 
robot_name = "spotty"
self.tf_listener = TFListenerWrapper(node)
self.body_frame_name = namespace_with(robot_name, BODY_FRAME_NAME)
self.vision_frame_name = namespace_with(robot_name, VISION_FRAME_NAME)

vision_to_body = tf_listener.lookup_a_tform_b(self.vision_frame_name, self.body_frame_name)
```

Use RViz's **TF** tab to visualize frames, see what you have computed is feasible in the coordinate frame, especially for camera-based movements.

**There's a guide for the robot's frames on the [official website](https://dev.bostondynamics.com/docs/concepts/geometry_and_frames.html)**

The transform output (TransformStamped) can be converted into an SE3Pose for further use. 
Following ros message type constructing, to init an SE3Pose type, each coordinate has to be set manually. I recommend making a helper function in your project to not do the conversion each time, as shown below.

```python
vision_to_body = tf_listener.lookup_a_tform_b(vision_frame_name,body_frame_name)

from bosdyn.client.math_helpers import Quat, SE2Pose, SE3Pose #and more..

def tfstamp_t_se3(tfstamp) -> SE3Pose:
    se3pose = SE3Pose (
        tfstamp.transform.translation.x
        tfstamp.transform.translation.y
        tfstamp.transform.translation.z
        Quat(
            tfstamp.transform.rotation.w
            tfstamp.transform.rotation.x
            tfstamp.transform.rotation.y
            tfstamp.transform.rotation.z
        )
    )
    return se3pose

#let's make a vector that points to (1,1,0) meters.
vision_to_body_se3 = tfstamp_t_se3(vision_to_body)
body_to_target = SE3Pose(1,1,1,Quat(1,0,0))
vision_to_target = vision_to_body_se3*body_to_target

#make it into a 2d pose vector:
vision_to_target_se2: SE2Pose = vision_to_target.get_closest_se2_transform()
```
Now we made a vector pointing (1,1)m from the body in 2D, having the vision frame as the ref frame. With this method you can issue movement commands, of course with the proper parameters. 

**Always make sure you use the current transforms! After you move the robot the vision_to_body variable will not point to the actual position of the body!** To circumvent this a good solution is to make sort of getter function, so when calling the variable it always gets actual transform.  
*An example in a class environment:*
```python
@property
def world_to_body(self) -> SE3Pose:
    world_to_body = tfstamp_t_se3( #<- converts output to SE3Pose, as told above
        self.tf_listener.lookup_a_tform_b(self.vision_frame_name,
            self.body_frame_name, timeout_sec=1))
    return world_to_body
```

### Moving the robot
Now we have a 2d pose pointing to the target, from the vision (or world) coordinate frame. The robot may move anywhere, this pose vector will stay the same.** We can now send the robot to the desired position. The robots body pose will match the target pose, so thats why it is a good idea to first build the position from the body's frame.
!!! note
    ** Technically the vector will stay the same in the world coordinate, but the world coordinate origin is deviating ever so slightly as a result of how these world coordinates are computed. Out of the two options- **vision** and **odom**(etry) - I recommend to use the vision as its more reliable IMO, contrary to that boston almost always uses odom as its world in their examples.
```python
goal = vision_to_target_se2

#This will make the robot crawl slowly.
params = RobotCommandBuilder.mobility_params(locomotion_hint=spot_command_pb2.HINT_SPEED_SELECT_CRAWL)

walk_pb2 = RobotCommandBuilder.synchro_se2_trajectory_point_command(goal_x=goal.x,
                                                    goal_y=goal.y,
                                                    goal_heading= goal.angle,
                                                    frame_name=VISION_FRAME_NAME,
                                                    params=params,
                                                    build_on_command=build_on_command)
command_goal = RobotCommand.Goal()
convert(walk_pb2, command_goal.command)
self._robot_command_client.send_goal_and_wait("Walking to target", command_goal)
#Optionally you can send it async, and could cancel or change the goal mid-way.

self._logger.info("Successfully walked to goal")
```
Aside from giving the builder the goal, parameters can be given, setting the max velocity, body height and much more. Take a look at the SDK doc and other implementations to see how it is done.