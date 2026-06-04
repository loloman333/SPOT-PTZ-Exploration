import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from spot_msgs.srv import RobotCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from synchros2.service import Serviced
from bosdyn_msgs.conversions import convert
from rclpy.task import Future

import math
import numpy as np
if not hasattr(np, 'float'):
    np.float = float

from synchros2.utilities import fqn, namespace_with
from bosdyn.client.robot_command import RobotCommandBuilder


from bosdyn.client.frame_helpers import (
    BODY_FRAME_NAME, VISION_FRAME_NAME
)

from spot_toolkit.helpers.utils import color_text

class VelocityRelay(Node):
    def __init__(self):
        super().__init__('velocity_relay')

        self.declare_parameter('robot_name', "spotty")
        self._robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.declare_parameter("command_duration_sec", 1.0)
        self._command_duration_sec = self.get_parameter('command_duration_sec').get_parameter_value().double_value

        self.cb_group = ReentrantCallbackGroup()

        self.streaming = False
        self.stream_watchdog = self.create_timer(self._command_duration_sec*2, self.reset_stream_flag, callback_group=self.cb_group)
        
        self.future = Future()
        
        self.velocity_client = Serviced(RobotCommand, 'robot_command', self, callback_group=self.cb_group)
        self.subscription = self.create_subscription(
            Twist,
            'velocity_goal',
            self.target_callback,
            10,
            callback_group=self.cb_group
        )

        actual_topic_name = self.resolve_topic_name('velocity_goal')
        self.get_logger().info(
            color_text("Init done. Waiting for Twist msgs on ", color="green") +
            color_text(actual_topic_name, color="blue") + 
            color_text(" and sending them to the robot as service calls.", color="green"))

    def reset_stream_flag(self):
        if self.streaming == True:
            self.get_logger().info("Velocity stream stopped.")
        self.streaming = False

    def target_callback(self, msg: Twist):
        self._logger.debug(f"Received new goal pose")
        if not self.velocity_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"Velocity service not available. Is the robot driver running?")
            return
        
        if self.streaming == False:
            self.get_logger().info("Velocity stream started.")
        
        self.streaming = True
        self.stream_watchdog.reset()
        
        
        # Convert Twist to a body frame pos
        pb2_command = RobotCommandBuilder.synchro_velocity_command(
            v_x=msg.linear.x,
            v_y=msg.linear.y,
            v_rot=msg.angular.z,
        )
        
        reqest = RobotCommand.Request()
        convert(pb2_command, reqest.command)
        reqest.duration.sec = int(self._command_duration_sec)
        
        
        # Cancel previous future if active
        if not self.future.done():
            self.future.cancel()
        
        self.future = self.velocity_client.asynchronous(reqest)
        self.future.add_done_callback(self.goal_callback)
    
    def goal_callback(self, future: Future):
        try:
            response : RobotCommand.Response = future.result()
            if response is None:
                self.get_logger().error("Failed to send velocity command.")
                return
            if response.success:
                self.get_logger().debug("Velocity command sent successfully.")
            else:
                self.get_logger().error(f"Failed to send velocity command: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Exception while sending velocity command: {e}")
            
            
def main(args=None):
    rclpy.init(args=args)
    node = VelocityRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

