## List of common actions

I've compiled some of the things the robot can do into a list, to ease the programming. Almost all can commands can be generated through RobotCommandBuilder (from bosdyn.client.robot_command import RobotCommandBuilder), but some have to be build from the ground up.

This is just a simple list, it may be expanded or deleted, as the SDK's list every action, but some with not a lot of explaining.

|Action Name|Code|Description|
|:----------|----|-----------|
|Trajectory Command|.synchro_se2_trajectory_point_command()|Goes to a 2d point (x,y) , in reference to a world frame (vision or odom)
|Stand Command| .synchro_stand_command() | Commands robot to stand if sitting. Can specify a lot of ways to achieve that, read the arguments for more info.
Sit Command| .synchro_sit_command() | Make the robot sit. Nothing fancy.
Velocity Command|.synchro_velocity_command()| This function is a bit different, there's explanation at the end of this table.
Change Battery Pose| .battery_change_pose_command()| Rolls robot to side for battery access.
Safe Power Off| .safe_power_off_command| Makes the robot sit and power off safely. Handy for "soft" errors, if you don't want to handle EStop release.
CLAW COMMANDS|
Open| .claw_gripper_open_command() | Opens arm claw fully
Close| .claw_gripper_close_command() | Close arm fully
Open fraction| .claw_gripper_open_fraction_command() | Open arm to a float fraction. (0 is closed and 1 is open)
Open degree| .claw_gripper_open_angle_command() | Open arm using the rotation angle (0 is closed, -1.5708 is fully open).
Complex movement| .claw_gripper_command_helper| Given a set of positions (angles) and an equal set of times (seconds), create a synchro command. Useful for "animating" the gripper, e.g. imitating a waving hand.
ARM COMMANDS
Stow| .arm_stow_command() | Stows arm.
Unstow/Ready| .arm_ready_command() | Ready arm. (Sidenote: stow and unstow are simple position commands, where the position is a 'named' position)
Carry| .arm_carry_command() | Sets the arm position into the 'carry' position.
Arm pose| .arm_pose_command() | Command the arm to move to an SE3Pose. Pay extra attention to correct frame and quaternion usage.
Arm pose SE3Pose| .arm_pose_command_from_pose() | Same as above, but the input argument is just one SE3Pose type, instead of all coordinates as a separate argument.
Joint Command| .arm_joint_command | Control each joint, using two rotational arguments for each joint. I have never used it, ask Krisz, or test it out yourself.
