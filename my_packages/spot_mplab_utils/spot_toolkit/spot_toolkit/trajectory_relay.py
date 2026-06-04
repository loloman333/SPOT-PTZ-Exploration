import math
from typing import Optional

import numpy as np
import rclpy
from bosdyn.api import geometry_pb2 as geo
from bosdyn.api import trajectory_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.action.client import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from synchros2.service import Serviced
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with
from spot_msgs.srv import RobotCommand

from spot_toolkit.helpers.conversions import (create_header, msgpose_to_se3,
                                              se3_to_msgpose, tfstamp_to_se3)
from spot_toolkit.helpers.utils import color_text

if not hasattr(np, 'float'):
    np.float = float


class TrajectoryRelay(Node):
    """
    A ROS 2 node that relays a stream of PoseStamped messages to Spot's
    robot command service. It continuously sends new trajectory goals,
    canceling the previously sent goal each time a new one arrives.
    This creates smooth, responsive motion based on the input pose stream.
    """

    def __init__(self) -> None:
        """Initializes the node, parameters, and ROS interfaces."""
        super().__init__('trajectory_relay')
        self._logger = self.get_logger()
        self._callback_group = ReentrantCallbackGroup()

        # --- State Variables ---
        self._is_streaming: bool = False
        self._last_goal_future: Optional[Future] = None

        # --- Parameters ---
        self._load_parameters()

        # --- Robot Specific Setup ---
        self._mobility_params = self._create_mobility_params()
        self._body_frame_name = namespace_with(self._robot_name, BODY_FRAME_NAME)
        self._vision_frame_name = namespace_with(self._robot_name, VISION_FRAME_NAME)

        # --- ROS 2 Interfaces ---
        self._setup_ros_interfaces()

        self._logger.info(
            color_text("Initialization complete. Waiting for goal poses on ", color="green") +
            color_text(self.resolve_topic_name('goal_pose'), color="blue") + 
            color_text(" and sending trajectories to the robot.", color="green"))

    def _load_parameters(self) -> None:
        """Declares and retrieves parameters from the parameter server."""
        self._robot_name = self.get_namespace().strip('/')

        self.declare_parameter("trajectory_duration_sec", 1.0)
        self._trajectory_duration_sec = self.get_parameter(
            'trajectory_duration_sec'
        ).get_parameter_value().double_value

    def _setup_ros_interfaces(self) -> None:
        """Initializes subscribers, service clients, and timers."""
        self._tf_listener = TFListenerWrapper(self)

        self._robot_command_client = Serviced(
            RobotCommand, 'robot_command', self, callback_group=self._callback_group
        )

        self.create_subscription(
            PoseStamped, 'goal_pose', self._pose_callback, 10, callback_group=self._callback_group
        )

        watchdog_period = self._trajectory_duration_sec * 2.0
        self._stream_watchdog = self.create_timer(
            watchdog_period, self._stream_watchdog_callback, callback_group=self._callback_group
        )

    def _pose_callback(self, msg: PoseStamped) -> None:
        """Handles incoming PoseStamped messages, creating and sending a new goal."""
        if not self._is_streaming:
            self._logger.info("Trajectory stream started.")
            self._is_streaming = True
        self._stream_watchdog.reset()

        if not self._robot_command_client.wait_for_service(timeout_sec=5.0):
            self._logger.error('Robot command service is not available. Aborting goal.')
            return

        # Cancel the previous goal if it's still active
        if self._last_goal_future:# and not self._last_goal_future.done():
            self._logger.debug("Canceling previous trajectory goal.")
            self._last_goal_future.cancel()

        # Transform the incoming pose to the vision frame
        vision_pose_stamped = self._transform_to_vision_frame(msg)
        if vision_pose_stamped is None:
            self._logger.warn("Failed to transform goal pose. Skipping this goal.")
            return

        request = self._create_robot_command_request(vision_pose_stamped)

        self._logger.debug("Sending new trajectory goal to the robot command service.")
        self._last_goal_future = self._robot_command_client.asynchronous(request)
        self._last_goal_future.add_done_callback(self._goal_done_callback)

    def _transform_to_vision_frame(self, pose_stamped: PoseStamped) -> Optional[PoseStamped]:
        """Transforms a pose from its source frame to the robot's vision frame."""
        if not pose_stamped.header.frame_id:
            self._logger.error("Incoming pose has an empty 'frame_id'. Cannot transform.")
            return None

        try:
            vision_tform_source = self._tf_listener.lookup_a_tform_b(
                self._vision_frame_name, pose_stamped.header.frame_id, timeout_sec=1.0
            )
            if not vision_tform_source:
                self._logger.error(
                    f"Failed to find transform from '{pose_stamped.header.frame_id}' to '{self._vision_frame_name}'."
                )
                return None

            source_pose_se3 = msgpose_to_se3(pose_stamped.pose)
            vision_tform_source_se3 = tfstamp_to_se3(vision_tform_source)

            # Apply the transformation: T_vision_goal = T_vision_source * T_source_goal
            vision_pose_se3 = vision_tform_source_se3 * source_pose_se3

            vision_pose_msg = PoseStamped()
            vision_pose_msg.header = create_header(self._vision_frame_name, self.get_clock().now())
            vision_pose_msg.header.frame_id = "vision" #no namespace
            vision_pose_msg.pose = se3_to_msgpose(vision_pose_se3)
            return vision_pose_msg

        except Exception as e:
            self._logger.error(f"Error during TF transformation: {e}")
            return None

    def _create_robot_command_request(self, pose_stamped: PoseStamped) -> RobotCommand.Request:
        """Builds the RobotCommand service request from a PoseStamped message."""
        se3_pose = msgpose_to_se3(pose_stamped.pose)
        se2_pose = se3_pose.get_closest_se2_transform()

        proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=se2_pose.x,
            goal_y=se2_pose.y,
            goal_heading=se2_pose.angle,
            frame_name="vision",
            params=self._mobility_params,
        )

        request = RobotCommand.Request()
        convert(proto_goal, request.command)

        sec, nanosec = divmod(self._trajectory_duration_sec, 1)
        request.duration = Duration(sec=int(sec), nanosec=int(nanosec * 1e9))

        return request

    def _goal_done_callback(self, future: Future) -> None:
        """Callback executed when the service call future is resolved."""
        if future.cancelled():
            self._logger.info("Previous trajectory goal was successfully canceled.")
            return

        try:
            response: RobotCommand.Response = future.result()
            if response is None:
                self._logger.error("Robot command service call failed with no response.")
            elif not response.success:
                self._logger.error(f"Trajectory command failed: {response.message}")
            else:
                self._logger.debug("Trajectory command succeeded.")
        except Exception as e:
            self._logger.error(f"Exception during goal callback: {e}")

    def _stream_watchdog_callback(self) -> None:
        """Resets the streaming flag if no new poses are received in time."""
        if self._is_streaming:
            self._logger.info("Trajectory stream timed out. Stopping.")
            self._is_streaming = False

    @staticmethod
    def _create_mobility_params():
        """Creates mobility parameters for the robot's trajectory command."""
        speed_limit = geo.SE2VelocityLimit(
            max_vel=geo.SE2Velocity(linear=geo.Vec2(x=1.0, y=1.0), angular=math.pi / 2.0)
        )
        body_control = TrajectoryRelay._create_default_body_control()
        return spot_command_pb2.MobilityParams(
            vel_limit=speed_limit,
            body_control=body_control,
            locomotion_hint=spot_command_pb2.HINT_TROT,
        )

    @staticmethod
    def _create_default_body_control():
        """Sets default body control to keep the robot level."""
        footprint_r_body = geo.SE3Pose(
            position=geo.Vec3(x=0.0, y=0.0, z=0.0),
            rotation=geo.Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        )
        point = trajectory_pb2.SE3TrajectoryPoint(pose=footprint_r_body)
        traj = trajectory_pb2.SE3Trajectory(points=[point])
        return spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()