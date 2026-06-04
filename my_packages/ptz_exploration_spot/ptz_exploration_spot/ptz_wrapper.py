#!/usr/bin/env python3
import asyncio
import math
import threading
import time
import urllib3
from aiortc import RTCConfiguration

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool
import tf2_ros

import bosdyn.client
from bosdyn.client import spot_cam
from bosdyn.client.payload import PayloadClient
from bosdyn.client.spot_cam.compositor import CompositorClient
from bosdyn.client.spot_cam.media_log import MediaLogClient
from bosdyn.client.spot_cam.ptz import PtzClient
from bosdyn.client.spot_cam.streamquality import StreamQualityClient

from spot_msgs.srv import SetPtzPosition
from spot_wrapper.cam_webrtc_client import WebRTCClient
from spot_wrapper.wrapper import SpotWrapper
from cv_bridge import CvBridge

# Spot CAM uses a self-signed cert. Suppressing this warning so it doesn't 
# scream at us in the terminal every time we make an HTTP request.
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

class PTZWrapper(Node):
    def __init__(self):
        super().__init__('ptz_wrapper')

        # Allow independent timers to run concurrently.
        self.timer_callback_group = ReentrantCallbackGroup()

        # --- Parameters ---
        self._declare_and_load_parameters()

        # --- ROS 2 Publishers & Broadcasters ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster(self)
        self.cam_info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self.ptz_state_pub = self.create_publisher(JointState, self.ptz_state_topic, 10)
        self.ptz_settled_pub = self.create_publisher(Bool, self.ptz_settled_topic, 10)
        
        if self.publish_stream:
            self.image_pub = self.create_publisher(Image, self.stream_topic, 10)

        # --- ROS 2 Subscribers & Services ---
        self.ptz_cmd_sub = self.create_subscription(Point, self.ptz_cmd_topic, self.cmd_ptz_callback, 10)
        self.ptz_client = self.create_client(SetPtzPosition, self.ptz_service_name)
        self._wait_for_ptz_service()

        # --- State Variables ---
        self.current_zoom_multiplier = 1.0
        self.current_pan_raw_rad = None
        self.current_tilt_raw_rad = None
        self.current_zoom_raw = None
        self.target_pan_raw_rad = None
        self.target_tilt_raw_rad = None
        self.target_zoom_raw = None
        self.is_shutting_down = False
        self.last_state_time_s = None
        self.target_ptz_desc = None
        self.latest_video_frame = None
        self.latest_frame_lock = threading.Lock()
        self.stream_resolution_lock = threading.Lock()
        self.current_stream_width = None
        self.current_stream_height = None
        self.bridge = CvBridge()

        # --- Boot Sequence ---
        self._init_boston_dynamics_clients()
        self._publish_static_payload_transform()
        
        self._start_kinematics_timers()
        if self.publish_stream:
            self._start_webrtc_stream()

        self.get_logger().info('Spot CAM PTZWrapper locked, loaded, and spinning.')

    # ========================================================================
    # SETUP AND INITIALIZATION
    # ========================================================================

    def _declare_and_load_parameters(self):
        self.declare_parameters(
        namespace='',
        parameters=[
        ('robot_name', 'spotty'),
        ('username', 'user'),
        ('password', 'password'),
        ('hostname', '192.168.50.3'),
        ('port', 0),
        ('ptz_cmd_topic', '/ptz_cmd'),
        ('ptz_state_topic', '/ptz_state'),
        ('ptz_settled_topic', '/ptz_settled'),
        ('ptz_service_name', ''),
        ('ptz_name', 'mech'),
        ('ptz_state_poll_period_s', 0.1),
        ('settle_tolerance_rad', 0.02),
        ('settle_tolerance_zoom', 0.02),
        ('publish_stream', True),
        ('stream_topic', 'spotty/camera/ptz/image'),
        ('camera_info_topic', 'spotty/camera/ptz/camera_info'),
        ('frame_id', 'spotty/ptz'),
        ('tf_publish_rate_hz', 15.0),
        ('stream_publish_rate_hz', 30.0),
        ('webrtc_bitrate_bps', 250000),
        ('webrtc_refresh_interval', 1),
        ('webrtc_idr_interval', 15),
        ('webrtc_awb', 0),
        ]
        )

        self.robot_name = self.get_parameter('robot_name').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value
        self.hostname = self.get_parameter('hostname').value
        self.port = int(self.get_parameter('port').value)

        self.ptz_cmd_topic = self.get_parameter('ptz_cmd_topic').value
        self.ptz_state_topic = self.get_parameter('ptz_state_topic').value
        self.ptz_settled_topic = self.get_parameter('ptz_settled_topic').value
        self.ptz_service_name = self.get_parameter('ptz_service_name').value or f'/{self.robot_name}/set_ptz_position'
        self.ptz_name = self.get_parameter('ptz_name').value
        self.ptz_state_poll_period_s = float(self.get_parameter('ptz_state_poll_period_s').value)
        self.settle_tolerance_rad = float(self.get_parameter('settle_tolerance_rad').value)
        self.settle_tolerance_zoom = float(self.get_parameter('settle_tolerance_zoom').value)
        
        self.publish_stream = bool(self.get_parameter('publish_stream').value)
        self.stream_topic = self.get_parameter('stream_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.tf_publish_rate_hz = float(self.get_parameter('tf_publish_rate_hz').value)
        self.stream_publish_rate_hz = float(self.get_parameter('stream_publish_rate_hz').value)

        self.webrtc_bitrate_bps = int(self.get_parameter('webrtc_bitrate_bps').value)
        self.webrtc_refresh_interval = int(self.get_parameter('webrtc_refresh_interval').value)
        self.webrtc_idr_interval = int(self.get_parameter('webrtc_idr_interval').value)
        self.webrtc_awb = int(self.get_parameter('webrtc_awb').value)

    def _wait_for_ptz_service(self):
        while not self.ptz_client.wait_for_service(timeout_sec=10.0):
        self.get_logger().info(f"Waiting for {self.ptz_service_name} to come online...")

    def _init_boston_dynamics_clients(self):
        self.sdk = bosdyn.client.create_standard_sdk('PTZ_Wrapper_Node', cert_resource_glob=None)
        spot_cam.register_all_service_clients(self.sdk)
        
        self.robot = self.sdk.create_robot(self.hostname)
        if self.port:
        self.robot.update_secure_channel_port(self.port)
            
        SpotWrapper.authenticate(self.robot, self.username, self.password, self.get_logger())

        # Grab the specific Spot CAM clients we need
        self.payload_client = self.robot.ensure_client(PayloadClient.default_service_name)
        self.media_log_client = self.robot.ensure_client(MediaLogClient.default_service_name)
        self.spot_ptz_client = self.robot.ensure_client(PtzClient.default_service_name)
        
        if self.publish_stream:
        self.compositor_client = self.robot.ensure_client(CompositorClient.default_service_name)
        self.stream_quality_client = self.robot.ensure_client(StreamQualityClient.default_service_name)

    def _publish_static_payload_transform(self):
        payload_details = None
        for payload in self.payload_client.list_payloads():
        if payload.is_enabled and 'Spot CAM' in payload.name:
        payload_details = payload
        break

        if not payload_details:
        self.get_logger().error("Spot CAM payload not found. Is it enabled in the admin console?")
        raise SystemError('Expected an enabled Spot CAM payload.')
            
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = f'{self.robot_name}/body' 
        static_tf.child_frame_id = f'{self.robot_name}/spot_cam_origin'

        pose = payload_details.body_tform_payload
        static_tf.transform.translation.x = pose.position.x
        static_tf.transform.translation.y = pose.position.y
        static_tf.transform.translation.z = pose.position.z
        static_tf.transform.rotation.x = pose.rotation.x
        static_tf.transform.rotation.y = pose.rotation.y
        static_tf.transform.rotation.z = pose.rotation.z
        static_tf.transform.rotation.w = pose.rotation.w

        self.static_tf_broadcaster.sendTransform(static_tf)

    def _start_kinematics_timers(self):
        tf_period = 1.0 / self.tf_publish_rate_hz
        self.tf_timer = self.create_timer(
        tf_period,
        self.poll_and_publish_tf,
        callback_group=self.timer_callback_group,
        )
        
        self.state_poll_timer = self.create_timer(
        self.ptz_state_poll_period_s,
        self.poll_state,
        callback_group=self.timer_callback_group,
        )

    def _start_webrtc_stream(self):
        try:
        self.compositor_client.set_screen('mech_full')
            
        # Drop the bitrate so our local CPU doesn't choke trying to decode H.264
        self.stream_quality_client.set_stream_params(
        self.webrtc_bitrate_bps,
        self.webrtc_refresh_interval,
        self.webrtc_idr_interval,
        self.webrtc_awb,
        )
        except Exception as e:
        self.get_logger().warn(f"Spot CAM grumbled about stream params: {e}")

        # Fire up the self-healing WebRTC consumer thread
        self.webrtc_thread = threading.Thread(target=self._webrtc_thread_worker, daemon=True)
        self.webrtc_thread.start()
        
        # Publish the latest available frame on a configurable timer.
        self.stream_publish_timer = self.create_timer(
        1.0 / self.stream_publish_rate_hz,
        self.publish_latest_frame,
        callback_group=self.timer_callback_group,
        )


    # ========================================================================
    # ROS 2 CALLBACKS
    # ========================================================================

    def cmd_ptz_callback(self, msg: Point):
        request = SetPtzPosition.Request()
        request.name = self.ptz_name

        # msg.x/msg.y/msg.z are expected to be in the physical hardware frame
        target_pan = float(msg.x)
        target_tilt = float(msg.y)
        target_zoom = float(msg.z)

        # Choose nearest equivalent pan to avoid long rotations
        if self.current_pan_raw_rad is not None:
        target_pan = self._nearest_equivalent_angle(target_pan, self.current_pan_raw_rad)

        request.pan = float(math.degrees(target_pan))
        request.tilt = float(math.degrees(target_tilt))
        request.zoom = float(target_zoom)

        # Store the actual physical target we sent so settled checks match hardware.
        self.target_pan_raw_rad = target_pan
        self.target_tilt_raw_rad = target_tilt
        self.target_zoom_raw = target_zoom

        self.ptz_client.call_async(request)

    def poll_and_publish_tf(self):
        try:
        cameras = self.media_log_client.list_cameras()
        for cam in cameras:
        if cam.name == 'ptz':
            self._publish_camera_data(cam)
            break
        except Exception as e:
        # This happens frequently if the robot walks behind a wall. Just swallow it and retry.
        self.get_logger().info(f"MediaLog kinematics poll dropped a packet: {e}")

    def poll_state(self):
        try:
        ptz_desc = self._get_target_ptz_desc()
        if not ptz_desc:
        return

        ptz_pos = self.spot_ptz_client.get_ptz_position(ptz_desc)
        self._publish_ptz_state(ptz_pos)
        zoom_val = ptz_pos.zoom.value if hasattr(ptz_pos.zoom, 'value') else ptz_pos.zoom
            
        self.current_zoom_multiplier = max(1.0, float(zoom_val))
            
        except Exception as e:
        self.get_logger().info(f"Failed to poll true PTZ zoom state: {e}")

    def _publish_ptz_state(self, ptz_pos):
        try:
        pan_deg = ptz_pos.pan.value if hasattr(ptz_pos.pan, 'value') else ptz_pos.pan
        tilt_deg = ptz_pos.tilt.value if hasattr(ptz_pos.tilt, 'value') else ptz_pos.tilt
        zoom_val = ptz_pos.zoom.value if hasattr(ptz_pos.zoom, 'value') else ptz_pos.zoom

        pan_rad = float(math.radians(pan_deg))
        tilt_rad = float(math.radians(tilt_deg))
        zoom = float(zoom_val)

        prev_pan_rad = self.current_pan_raw_rad
        prev_tilt_rad = self.current_tilt_raw_rad
        prev_zoom = self.current_zoom_raw

        now_s = self.get_clock().now().nanoseconds * 1e-9
        pan_vel = 0.0
        tilt_vel = 0.0
        zoom_vel = 0.0

        if self.last_state_time_s is not None and prev_pan_rad is not None and prev_tilt_rad is not None and prev_zoom is not None:
        dt = now_s - self.last_state_time_s
        if dt > 1e-6:
            pan_vel = self._shortest_angular_distance(prev_pan_rad, pan_rad) / dt
            tilt_vel = (tilt_rad - prev_tilt_rad) / dt
            zoom_vel = (zoom - prev_zoom) / dt

        if self.target_pan_raw_rad is None:
        self.target_pan_raw_rad = pan_rad
        if self.target_tilt_raw_rad is None:
        self.target_tilt_raw_rad = tilt_rad
        if self.target_zoom_raw is None:
        self.target_zoom_raw = zoom

        pan_err = abs(self._shortest_angular_distance(pan_rad, self.target_pan_raw_rad))
        tilt_err = abs(tilt_rad - self.target_tilt_raw_rad)
        zoom_err = abs(zoom - self.target_zoom_raw)
        is_settled = (
        pan_err <= self.settle_tolerance_rad
        and tilt_err <= self.settle_tolerance_rad
        and zoom_err <= self.settle_tolerance_zoom
        )

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['ptz_pan', 'ptz_tilt', 'ptz_zoom']
        msg.position = [self._wrap_angle(pan_rad), tilt_rad, zoom]
        msg.velocity = [pan_vel, tilt_vel, zoom_vel]
        self.ptz_state_pub.publish(msg)
        self.ptz_settled_pub.publish(Bool(data=is_settled))

        self.current_pan_raw_rad = pan_rad
        self.current_tilt_raw_rad = tilt_rad
        self.current_zoom_raw = zoom
        self.last_state_time_s = now_s
        except Exception as e:
        self.get_logger().debug(f"PTZ state publish failed: {e}")

    def _get_target_ptz_desc(self):
        if self.target_ptz_desc:
        return self.target_ptz_desc

        for ptz_desc in self.spot_ptz_client.list_ptz():
        if ptz_desc.name == self.ptz_name:
        self.target_ptz_desc = ptz_desc
        break

        return self.target_ptz_desc

    def publish_latest_frame(self):
        if self.image_pub.get_subscription_count() == 0:
        return

        with self.latest_frame_lock:
        if self.latest_video_frame is None:
        return
        frame = self.latest_video_frame
        self.latest_video_frame = None

        try:
        frame = frame.to_ndarray(format='bgr24')
        except Exception as e:
        self.get_logger().warn(f'Failed converting WebRTC frame to ndarray: {e}')
        return

        published_width = int(frame.shape[1])
        published_height = int(frame.shape[0])
        with self.stream_resolution_lock:
        self.current_stream_width = published_width
        self.current_stream_height = published_height

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.image_pub.publish(msg)

    def _get_output_image_dimensions(self, native_width: int, native_height: int):
        with self.stream_resolution_lock:
        if self.current_stream_width is not None and self.current_stream_height is not None:
        return self.current_stream_width, self.current_stream_height

        # Before the first frame is published, fall back to native camera dimensions.
        return native_width, native_height
        
    # ========================================================================
    # QUATERNION MATH HELPERS
    # ========================================================================

    @staticmethod
    def _quat_mult(q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
        ]

    @staticmethod
    def _quat_inv(q):
        return [-q[0], -q[1], -q[2], q[3]]

    @staticmethod
    def _wrap_angle(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def _shortest_angular_distance(from_rad: float, to_rad: float) -> float:
        return PTZWrapper._wrap_angle(to_rad - from_rad)

    @staticmethod
    def _nearest_equivalent_angle(target_rad: float, reference_rad: float) -> float:
        return reference_rad + PTZWrapper._shortest_angular_distance(reference_rad, target_rad)


    # ========================================================================
    # DATA PROCESSING & PUBLISHING
    # ========================================================================

    def _publish_camera_data(self, cam):
        now = self.get_clock().now().to_msg()
        pose = cam.base_tform_sensor
        
        # ---------------------------------------------------------
        # THE ROTATION FIX
        # ---------------------------------------------------------
        # Standard ROS: X-forward, Y-left, Z-up
        # Optical Frame: Z-forward, X-right, Y-down
        
        # Static flip: Standard -> Optical
        q_std_to_opt = [-0.5, 0.5, -0.5, 0.5] 
        
        # Static flip: Optical -> Standard (Inverse/Conjugate of the above)
        q_opt_to_std = [0.5, -0.5, 0.5, 0.5] 

        # --- LINK 1: spot_cam_origin (Optical) -> ptz_origin (Standard) ---
        # Handles translation from the base, and applies a static rotation 
        # to force the ptz_origin into Standard ROS convention.
        t_origin = TransformStamped()
        t_origin.header.stamp = now
        t_origin.header.frame_id = f'{self.robot_name}/spot_cam_origin'
        t_origin.child_frame_id = f'{self.robot_name}/ptz_origin'       
        
        t_origin.transform.translation.x = pose.position.x
        t_origin.transform.translation.y = pose.position.y
        t_origin.transform.translation.z = pose.position.z
        
        # Static flip: Optical -> Standard
        t_origin.transform.rotation.x = q_opt_to_std[0]
        t_origin.transform.rotation.y = q_opt_to_std[1]
        t_origin.transform.rotation.z = q_opt_to_std[2]
        t_origin.transform.rotation.w = q_opt_to_std[3]

        # --- LINK 2: ptz_origin (Standard) -> ptz_lens (Optical) ---
        # Handles the dynamic PTZ hardware rotation AND applies the static optical flip back
        t_lens = TransformStamped()
        t_lens.header.stamp = now
        t_lens.header.frame_id = t_origin.child_frame_id
        t_lens.child_frame_id = self.frame_id       
        
        t_lens.transform.translation.x = 0.0
        t_lens.transform.translation.y = 0.0
        t_lens.transform.translation.z = 0.0
        
        q_raw_hardware = [pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w]
        
        # Combine the dynamic pan/tilt (Standard) with the flip back to Optical
        # Note: Depending on your _quat_mult implementation, you might need to swap the arguments
        # if the hardware rotation applies globally rather than locally.
        q_lens_final = self._quat_mult(q_std_to_opt, q_raw_hardware)
        
        t_lens.transform.rotation.x = q_lens_final[0]
        t_lens.transform.rotation.y = q_lens_final[1]
        t_lens.transform.rotation.z = q_lens_final[2]
        t_lens.transform.rotation.w = q_lens_final[3]

        # Broadcast both links simultaneously to build the chain seamlessly
        self.tf_broadcaster.sendTransform([t_origin, t_lens])

        # ========================================================================
        # CAMERA INFO PUBLISHING
        # ========================================================================
        info = CameraInfo()
        info.header.stamp = now
        info.header.frame_id = self.frame_id
        
        native_width = int(cam.resolution.x) if hasattr(cam, 'resolution') else 1920
        native_height = int(cam.resolution.y) if hasattr(cam, 'resolution') else 1080
        output_width, output_height = self._get_output_image_dimensions(native_width, native_height)
        info.width = output_width
        info.height = output_height
        info.distortion_model = 'plumb_bob'

        fx = 1542.1050771207592
        fy = 1542.6668979953038
        cx = 973.15937340522555
        cy = 558.51062252115651
        info.d = [0.202715784, 0.373427182, -1.33260691, 2.45122409, 0.0]

        try:
        if hasattr(cam, 'intrinsics') and hasattr(cam.intrinsics, 'pinhole'):
        pinhole = cam.intrinsics.pinhole
        fx = pinhole.focal_length.x
        fy = pinhole.focal_length.y
        cx = pinhole.center_point.x
        cy = pinhole.center_point.y
        info.d = [pinhole.k1, pinhole.k2, pinhole.k3, pinhole.k4, 0.0]
        except Exception:
        pass 

        scale_x = output_width / max(1.0, float(native_width))
        scale_y = output_height / max(1.0, float(native_height))

        fx = fx * getattr(self, 'current_zoom_multiplier', 1.0) * scale_x
        fy = fy * getattr(self, 'current_zoom_multiplier', 1.0) * scale_y
        cx = cx * scale_x
        cy = cy * scale_y
        
        info.k = [fx,  0.0, cx,
          0.0, fy,  cy,
          0.0, 0.0, 1.0]

        info.r = [1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0]

        info.p = [fx,  0.0, cx,  0.0,
          0.0, fy,  cy,  0.0,
          0.0, 0.0, 1.0, 0.0]

        self.cam_info_pub.publish(info)

    # ========================================================================
    # WEBRTC BACKGROUND THREAD (Self-Healing)
    # ========================================================================

    def _webrtc_thread_worker(self):
        while rclpy.ok():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
            
        try:
        loop.run_until_complete(self._run_webrtc_client())
        except Exception as e:
        self.get_logger().error(f"WebRTC stream choked on an error: {e}")
        finally:
        loop.close()
            
        if rclpy.ok():
        self.get_logger().warn("WebRTC thread died (likely a Wi-Fi blip). Attempting CPR in 3s...")
        time.sleep(3.0) # Give the robot a second to catch its breath before hammering it again

    async def _run_webrtc_client(self):
                webrtc_client = WebRTCClient(
            hostname=self.hostname,
            sdp_port=31102,
            sdp_filename='h264.sdp',
            cam_ssl_cert=False,  
            token=self.robot.user_token,  
            rtc_config=RTCConfiguration()
        )
        
        try:
            await webrtc_client.start()

            while rclpy.ok() and not self.is_shutting_down:
                try:
                    frame = await asyncio.wait_for(webrtc_client.video_frame_queue.get(), timeout=2.0)

                    while True:
                        try:
                            frame = webrtc_client.video_frame_queue.get_nowait()
                        except asyncio.QueueEmpty:
                            break

                    with self.latest_frame_lock:
                        self.latest_video_frame = frame

                except asyncio.TimeoutError:
                    # A timeout usually means the robot walked into a deadzone.
                    # Let's check if the underlying connection actually died.
                    if hasattr(webrtc_client, 'pc'):
                        state = webrtc_client.pc.connectionState
                        if state in ['failed', 'closed', 'disconnected']:
                            self.get_logger().error(f"WebRTC connection state dropped to '{state}'. Forcing reconnect.")
                            break
                    continue
                except Exception as e:
                    self.get_logger().warn(f"Exception during WebRTC frame consumption: {e}")
                    break
        finally:
            try:
                await webrtc_client.pc.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = PTZWrapper()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.is_shutting_down = True
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
