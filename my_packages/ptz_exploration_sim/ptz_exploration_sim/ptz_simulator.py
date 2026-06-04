#!/usr/bin/env python3
import math
import copy
import numpy as np
import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import Bool
from std_msgs.msg import Float64


class PtzSimulator(Node):
    SUPPORTED_IMAGE_ENCODINGS = ('rgb8', 'bgr8')

    def __init__(self):
        super().__init__('ptz_simulator')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pan_joint', 'ptz_cam_pan_joint'),
                ('tilt_joint', 'ptz_cam_tilt_joint'),
                ('ptz_cmd_topic', '/ptz_cmd'),
                ('ptz_settled_topic', '/ptz_settled'),
                ('ptz_raw_image_topic', '/ptz_cam/raw_full/image'),
                ('ptz_raw_camera_info_topic', '/ptz_cam/raw_full/camera_info'),
                ('ptz_image_topic', '/ptz_cam/raw/image'),
                ('ptz_camera_info_topic', '/ptz_cam/raw/camera_info'),
                ('ptz_output_width', 640),
                ('ptz_output_height', 360),
                ('camera_info_rate_hz', 30.0),
                ('zoom_slew_rate', 8.0),
                ('control_period_s', 0.02),
                ('min_zoom_factor', 1.0),
                ('max_zoom_factor', 30.0),
                ('settle_tolerance_rad', 0.02),
                ('settle_tolerance_zoom', 0.02),
        ])

        self.pan_joint = self.get_parameter('pan_joint').value
        self.tilt_joint = self.get_parameter('tilt_joint').value
        self.ptz_cmd_topic = self.get_parameter('ptz_cmd_topic').value
        self.ptz_settled_topic = self.get_parameter('ptz_settled_topic').value
        self.ptz_raw_image_topic = self.get_parameter('ptz_raw_image_topic').value
        self.ptz_raw_camera_info_topic = self.get_parameter('ptz_raw_camera_info_topic').value
        self.ptz_image_topic = self.get_parameter('ptz_image_topic').value
        self.ptz_camera_info_topic = self.get_parameter('ptz_camera_info_topic').value
        self.ptz_output_width = int(self.get_parameter('ptz_output_width').value)
        self.ptz_output_height = int(self.get_parameter('ptz_output_height').value)
        self.camera_info_rate_hz = float(self.get_parameter('camera_info_rate_hz').value)
        self.zoom_slew_rate = float(self.get_parameter('zoom_slew_rate').value)
        self.control_period_s = float(self.get_parameter('control_period_s').value)
        self.min_zoom_factor = self.get_parameter('min_zoom_factor').value
        self.max_zoom_factor = self.get_parameter('max_zoom_factor').value
        self.settle_tolerance_rad = float(self.get_parameter('settle_tolerance_rad').value)
        self.settle_tolerance_zoom = float(self.get_parameter('settle_tolerance_zoom').value)
        
        # Current State
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.current_zoom_factor = 1.0

        # Last published state (for measured velocity)
        self.last_state_pan = None
        self.last_state_tilt = None
        self.last_state_zoom = None
        self.last_state_time_s = None
        
        # Target State
        self.target_pan = 0.0
        self.target_tilt = 0.0
        self.target_zoom_factor = 1.0 
        self.raw_camera_info = None
        self.bridge = CvBridge()
        self.latest_camera_info_msg = None
        self.get_logger().info("PTZ resize backend: CPU")

        # Publishers
        self.pan_pub = self.create_publisher(Float64, '/ptz_cam/cmd_pan', 10)
        self.tilt_pub = self.create_publisher(Float64, '/ptz_cam/cmd_tilt', 10)
        self.state_pub = self.create_publisher(JointState, '/ptz_state', 10)
        self.settled_pub = self.create_publisher(Bool, self.ptz_settled_topic, 10)
        self.ptz_image_pub = self.create_publisher(Image, self.ptz_image_topic, 10)
        self.ptz_camera_info_pub = self.create_publisher(CameraInfo, self.ptz_camera_info_topic, 10)

        # Subscribers
        self.ptz_cmd_sub = self.create_subscription(
            Point,
            self.ptz_cmd_topic,
            self.set_ptz_callback,
            10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.raw_camera_info_sub = self.create_subscription(
            CameraInfo,
            self.ptz_raw_camera_info_topic,
            self.raw_camera_info_callback,
            10
        )
        self.raw_image_sub = self.create_subscription(
            Image,
            self.ptz_raw_image_topic,
            self.raw_image_callback,
            10
        )
        
        # Timers
        self.pt_timer = self.create_timer(self.control_period_s, self.execute_pan_tilt)
        self.z_timer = self.create_timer(self.control_period_s, self.execute_zoom)
        self.state_timer = self.create_timer(0.1, self.publish_state)
        if self.camera_info_rate_hz > 0.0:
            self.camera_info_timer = self.create_timer(1.0 / self.camera_info_rate_hz, self.publish_camera_info)
        
        self.get_logger().info("PTZ Manager Initialized.")
        
    def publish_state(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9

        pan_vel = 0.0
        tilt_vel = 0.0
        zoom_vel = 0.0

        if (
        self.last_state_time_s is not None
        and self.last_state_pan is not None
        and self.last_state_tilt is not None
        and self.last_state_zoom is not None
        ):
        dt = now_s - self.last_state_time_s
        if dt > 1e-6:
        pan_vel = self.shortest_angular_distance(self.last_state_pan, self.current_pan) / dt
        tilt_vel = (self.current_tilt - self.last_state_tilt) / dt
        zoom_vel = (self.current_zoom_factor - self.last_state_zoom) / dt

        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = ['ptz_pan', 'ptz_tilt', 'ptz_zoom']
        state_msg.position = [
        self.wrap_angle(self.current_pan),
        float(self.current_tilt),
        float(self.current_zoom_factor),
        ]
        state_msg.velocity = [
        float(pan_vel),
        float(tilt_vel),
        float(zoom_vel),
        ]
        self.state_pub.publish(state_msg)

        pan_err = abs(self.shortest_angular_distance(self.current_pan, self.target_pan))
        tilt_err = abs(self.current_tilt - self.target_tilt)
        zoom_err = abs(self.current_zoom_factor - self.target_zoom_factor)
        is_settled = (
        pan_err <= self.settle_tolerance_rad
        and tilt_err <= self.settle_tolerance_rad
        and zoom_err <= self.settle_tolerance_zoom
        )
        self.settled_pub.publish(Bool(data=is_settled))

        self.last_state_pan = self.current_pan
        self.last_state_tilt = self.current_tilt
        self.last_state_zoom = self.current_zoom_factor
        self.last_state_time_s = now_s

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def shortest_angular_distance(from_angle, to_angle):
        return PtzSimulator.wrap_angle(to_angle - from_angle)

    @staticmethod
    def nearest_equivalent_angle(target, reference):
        return reference + PtzSimulator.shortest_angular_distance(reference, target)

    def clamp_zoom(self, zoom):
        return max(self.min_zoom_factor, min(float(zoom), self.max_zoom_factor))

    @staticmethod
    def compute_center_roi(raw_w, raw_h, zoom):
        crop_w = max(1, min(raw_w, int(raw_w / zoom)))
        crop_h = max(1, min(raw_h, int(raw_h / zoom)))
        x0 = max(0, (raw_w - crop_w) // 2)
        y0 = max(0, (raw_h - crop_h) // 2)
        return x0, y0, crop_w, crop_h

    @staticmethod
    def get_joint_position(msg, joint_name):
        if joint_name not in msg.name:
            return None
        idx = msg.name.index(joint_name)
        if idx >= len(msg.position):
            return None
        return msg.position[idx]

    def execute_pan_tilt(self):
        self.pan_pub.publish(Float64(data=self.target_pan))
        self.tilt_pub.publish(Float64(data=self.target_tilt))
        
    def execute_zoom(self):
        if self.current_zoom_factor == self.target_zoom_factor:
        return

        dz = self.target_zoom_factor - self.current_zoom_factor
        max_step = self.zoom_slew_rate * self.control_period_s
        if max_step <= 0.0 or abs(dz) <= max_step:
        self.current_zoom_factor = self.target_zoom_factor
        else:
        self.current_zoom_factor += math.copysign(max_step, dz)

    def publish_camera_info(self):
        if self.latest_camera_info_msg is None:
        return
        self.ptz_camera_info_pub.publish(self.latest_camera_info_msg)

    def raw_camera_info_callback(self, msg):
        if self.raw_camera_info is not None:
        return

        self.raw_camera_info = msg
        self.get_logger().info(f"Received raw PTZ camera info from {self.ptz_raw_camera_info_topic}")

        if self.raw_camera_info_sub is not None:
        self.destroy_subscription(self.raw_camera_info_sub)
        self.raw_camera_info_sub = None

    def raw_image_callback(self, msg):
        if msg.encoding not in self.SUPPORTED_IMAGE_ENCODINGS:
        self.get_logger().info(f"Unsupported PTZ raw image encoding: {msg.encoding}")
        return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if cv_image is None:
        return

        raw_h, raw_w = cv_image.shape[:2]
        zoom = self.clamp_zoom(self.current_zoom_factor)
        x0, y0, crop_w, crop_h = self.compute_center_roi(raw_w, raw_h, zoom)

        cropped = cv_image[y0:y0 + crop_h, x0:x0 + crop_w]
        resized = cv2.resize(
        cropped,
        (self.ptz_output_width, self.ptz_output_height),
        interpolation=cv2.INTER_LINEAR,
        )

        out_img = self.bridge.cv2_to_imgmsg(resized, encoding=msg.encoding)
        out_img.header = msg.header
        self.ptz_image_pub.publish(out_img)

        if self.raw_camera_info is not None:
        out_info = self._build_zoomed_camera_info(self.raw_camera_info, msg.header, x0, y0, crop_w, crop_h)
        self.latest_camera_info_msg = out_info

    def _build_zoomed_camera_info(self, raw_info, header, crop_x, crop_y, crop_w, crop_h):
        out_info = copy.deepcopy(raw_info)
        out_info.header = header
        out_info.width = self.ptz_output_width
        out_info.height = self.ptz_output_height

        sx = float(self.ptz_output_width) / float(crop_w)
        sy = float(self.ptz_output_height) / float(crop_h)

        k = np.array(raw_info.k, dtype=np.float64).reshape(3, 3)
        fx, fy = k[0, 0], k[1, 1]
        cx, cy = k[0, 2], k[1, 2]

        fx_new = fx * sx
        fy_new = fy * sy
        cx_new = (cx - float(crop_x)) * sx
        cy_new = (cy - float(crop_y)) * sy

        k_new = k.copy()
        k_new[0, 0] = fx_new
        k_new[1, 1] = fy_new
        k_new[0, 2] = cx_new
        k_new[1, 2] = cy_new
        out_info.k = list(k_new.reshape(-1))

        p = np.array(raw_info.p, dtype=np.float64).reshape(3, 4)
        p_new = p.copy()
        p_new[0, 0] = fx_new
        p_new[1, 1] = fy_new
        p_new[0, 2] = cx_new
        p_new[1, 2] = cy_new
        p_new[0, 3] = p[0, 3] * sx
        p_new[1, 3] = p[1, 3] * sy
        out_info.p = list(p_new.reshape(-1))

        return out_info

    def joint_state_callback(self, msg):
        pan = self.get_joint_position(msg, self.pan_joint)
        if pan is not None:
        self.current_pan = pan

        tilt = self.get_joint_position(msg, self.tilt_joint)
        if tilt is not None:
        self.current_tilt = tilt

    def set_ptz_callback(self, msg):
        self.get_logger().info(f"PTZ CMD: Pan={msg.x:.2f}, Tilt={msg.y:.2f}, Zoom={msg.z:.2f}x")
        
        # Update Targets
        self.target_pan = self.nearest_equivalent_angle(float(msg.x), self.current_pan)
        self.target_tilt = float(msg.y)
        
        self.target_zoom_factor = self.clamp_zoom(msg.z)

def main(args=None):
    rclpy.init(args=args)
    node = PtzSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
