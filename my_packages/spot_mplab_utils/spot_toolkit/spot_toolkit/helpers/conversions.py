from typing import Type, Any, Union
from types import FrameType
from synchros2.utilities import fqn
import numpy as np
from spot_msgs.action import RobotCommand, Manipulation
from spot_msgs.srv import RobotCommand as RobotCommandSrv
from bosdyn_msgs.conversions import convert

from bosdyn.client.math_helpers import Quat, SE3Pose, Vec2, Vec3
from geometry_msgs.msg import TransformStamped, Vector3
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import rclpy
from std_msgs.msg import Header
from builtin_interfaces.msg import Time 
from rclpy.clock import Clock

def transform_to_se3pose(transform) -> SE3Pose:
    """Converts any object with .translation and .rotation to SE3Pose."""
    return SE3Pose(
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
        convert_to_target_quaternion(transform.rotation, Quat)
    )

def tfstamp_to_se3(tfstamp: TransformStamped) -> SE3Pose:
    return transform_to_se3pose(tfstamp.transform)

def tfstamp_to_msgpose(tfstamp: TransformStamped) -> PoseStamped:
    pose = se3_to_msgpose(transform_to_se3pose(tfstamp.transform))
    return PoseStamped(header=tfstamp.header, pose=pose)

def se3_to_msgpose(se3: SE3Pose) -> Pose:
    pose = Pose()
    pose.position = convert_to_target_vector(se3.position, Point)
    pose.orientation = convert_to_target_quaternion(se3.rotation, Quaternion) 
    return pose

def msgpose_to_se3(msgpose: Pose) -> SE3Pose:
    return SE3Pose(
        msgpose.position.x,
        msgpose.position.y,
        msgpose.position.z,
        convert_to_target_quaternion(msgpose.orientation, Quat)
    )


def create_header(frame_id: str, stamp : Time=None) -> Header:
    """Create a ROS Header with the given frame_id and stamp (or current time if None)."""
    header = Header()
    header.frame_id = frame_id
    header.stamp = (stamp or Clock().now()).to_msg()
    return header


def proto_to_ros_command(proto_command, ros_type : Union[RobotCommand, RobotCommandSrv, Manipulation]) -> Any:
    
    ros_obj = ros_type()
    convert(proto_command, ros_obj.command)
    return ros_obj


def convert_to_target_vector(obj: Any, target_type: Type) -> Any: 
    """ Converts any object with .x, .y, .z attributes to a target type. 
    
        target_type: either a class with an __init__(x, y, z) signature 
            or a class that allows assignment to x, y, z attributes """ 
    x, y, z = getattr(obj, "x"), getattr(obj, "y"), getattr(obj, "z") 
    try: 
        # Try to construct the target type with x, y, z in the constructor 
        return target_type(x, y, z) 
    except TypeError: 
        # Fall back to attribute assignment 
        target_obj = target_type() 
        target_obj.x = x 
        target_obj.y = y 
        target_obj.z = z 
        return target_obj
    
def convert_to_target_quaternion(obj: Any, target_type: Type) -> Any: 
    """ Converts any object with .w, .x, .y, .z attributes to a target type. 
    
        target_type: either a class with an __init__(w, x, y, z) signature 
            or a class that allows assignment to w, x, y, z attributes """ 
    w, x, y, z = getattr(obj, "w"), getattr(obj, "x"), getattr(obj, "y"), getattr(obj, "z") 
    try: 
        # Try to construct the target type with w, x, y, z in the constructor 
        return target_type(w, x, y, z) 
    except TypeError: 
        # Fall back to attribute assignment 
        target_obj = target_type() 
        target_obj.w = w 
        target_obj.x = x 
        target_obj.y = y 
        target_obj.z = z 
        return target_obj

#This is a more general version of the above two functions
def convert_by_attributes(obj: Any, target_type: Type) -> Any:
    """
    Converts any object to a target type by matching attribute names.

    - Copies all attributes that exist in both the source and the target.
    - Works with vectors (x, y, z), quaternions (w, x, y, z), or any simple data class.
    """
    # Create an instance of the target type
    try:
        # Attempt to construct with matching kwargs
        attrs = {name: getattr(obj, name) for name in dir(obj) 
                 if not name.startswith("_") and hasattr(target_type, name)}
        return target_type(**attrs)
    except TypeError:
        # Fall back to attribute assignment
        target_obj = target_type()
        for name in dir(obj):
            if not name.startswith("_") and hasattr(target_obj, name):
                setattr(target_obj, name, getattr(obj, name))
        return target_obj