#!/usr/bin/env python3
import csv
from datetime import datetime
import math
from pathlib import Path
from typing import List, Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from ptz_exploration_core.msg import Landmark, LandmarkArray
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from spot_msgs.msg import BatteryStateArray
import yaml

class Collector(Node):
    def __init__(self):
        super().__init__('collector')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_name', 'spot'),
                ('base_frame_id', 'base_link'),
                ('reference_frame_id', 'odom'),
                ('sim_world_name', ''),
                ('distance_update_hz', 5.0),
                ('landmarks_topic', '/landmarks'),
                ('projected_map_topic', '/projected_map'),
                ('battery_states_topic', ''),
                ('ground_truth_markers_topic', '/ground_truth_markers'),
                ('exploration_done_topic', '/exploration_done'),
                ('target_objects', 5),
                ('output_dir', '~/ros2_ws/data/collector_runs'),
            ]
        )

        # Read parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.reference_frame_id = self.get_parameter('reference_frame_id').value
        self.sim_world_name = str(self.get_parameter('sim_world_name').value).strip()
        self.distance_update_hz = float(self.get_parameter('distance_update_hz').value)
        self.landmarks_topic = self.get_parameter('landmarks_topic').value
        self.projected_map_topic = self.get_parameter('projected_map_topic').value
        self.battery_states_topic = str(self.get_parameter('battery_states_topic').value).strip()
        self.ground_truth_markers_topic = self.get_parameter('ground_truth_markers_topic').value
        self.exploration_done_topic = self.get_parameter('exploration_done_topic').value
        # Number of target objects expected in the environment. If >0,
        # the collector will shut down 60s after this many landmarks are found.
        self.target_objects = int(self.get_parameter('target_objects').value)
        self.output_dir = Path(self.get_parameter('output_dir').value).expanduser()
        self.run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.is_simulation = bool(self.sim_world_name)
        if self.is_simulation:
            self.output_file_path = self.output_dir / f'run_sim_{self.sim_world_name}_{self.run_id}.csv'
        else:
            self.output_file_path = self.output_dir / f'run_spot_{self.run_id}.csv'

        self.world_objects = self.load_world_objects(self.sim_world_name) if self.is_simulation else []

        # Time and motion tracking members
        self.start_time = self.get_clock().now()
        self.distance_traveled_m = 0.0
        self.rotation_traveled_rad = 0.0
        self.last_xy = None
        self.last_yaw = None

        # TF listener (subscribes to /tf and /tf_static)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Landmark tracking members (latest state by ID)
        self.landmarks_by_id = {}

        # Projected map tracking members
        self.projected_map_known_cells = 0
        self.projected_map_known_area_m2 = 0.0

        # Geometric error tracking members
        self.geometric_error = math.nan
        self.battery_percentage = math.nan
        self.mean_covariance_trace = math.nan

        self.marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.ground_truth_marker_pub = None
        if self.is_simulation:
            self.ground_truth_marker_pub = self.create_publisher(
                MarkerArray,
                self.ground_truth_markers_topic,
                self.marker_qos,
            )

        # In-memory rows, flushed once on shutdown
        self.rows = []

        # Shutdown timer when all target objects have been found
        self.target_objects_shutdown_timer = None

        # Subscribers
        self.landmarks_sub = self.create_subscription(
            LandmarkArray,
            self.landmarks_topic,
            self.landmarks_callback,
            10
        )
        self.projected_map_sub = self.create_subscription(
            OccupancyGrid,
            self.projected_map_topic,
            self.projected_map_callback,
            10
        )

        self.battery_sub = None
        if not self.is_simulation:
            battery_topic = self.battery_states_topic or f'/{self.robot_name}/status/battery_states'
            self.battery_sub = self.create_subscription(
                BatteryStateArray,
                battery_topic,
                self.battery_callback,
                1,
            )
            self.battery_states_topic = battery_topic

        # Subscribe to exploration done signal
        # self.exploration_done_sub = self.create_subscription(
        #     Empty,
        #     self.exploration_done_topic,
        #     self.exploration_done_callback,
        #     1
        # )

        # Timer for TF integration + periodic data collection
        timer_period = 1.0 / max(self.distance_update_hz, 1e-6)
        self.collect_timer = self.create_timer(
            timer_period,
            self.collect_snapshot,
        )

        self.get_logger().info(
            f"Collector initialized with robot_name={self.robot_name}, "
            f"base_frame_id={self.base_frame_id}, "
            f"reference_frame_id={self.reference_frame_id}, "
            f"sim_world_name={self.sim_world_name}, "
            f"distance_update_hz={self.distance_update_hz}, "
            f"landmarks_topic={self.landmarks_topic}, "
            f"projected_map_topic={self.projected_map_topic}, "
            f"battery_states_topic={self.battery_states_topic}, "
            f"ground_truth_markers_topic={self.ground_truth_markers_topic}, "
            f"output_file={self.output_file_path}"
        )

        self.publish_ground_truth_markers()

    @staticmethod
    def _pose_from_object_entry(object_entry: dict) -> Optional[tuple[float, float, float]]:
        pose = object_entry.get('pose', {})
        center_offset = object_entry.get('center_offset', {})
        try:
            return (
                float(pose.get('x', 0.0)) + float(center_offset.get('x', 0.0)),
                float(pose.get('y', 0.0)) + float(center_offset.get('y', 0.0)),
                float(pose.get('z', 0.0)) + float(center_offset.get('z', 0.0)),
            )
        except (TypeError, ValueError):
            return None

    def load_world_objects(self, world_name: str) -> List[dict]:
        if not world_name:
            return []

        world_objects_path = Path(get_package_share_directory('ptz_exploration_sim')) / 'worlds' / f'{world_name}_objects.yaml'
        if not world_objects_path.exists():
            self.get_logger().warn(f'World objects file not found: {world_objects_path}')
            return []

        with world_objects_path.open('r') as yaml_file:
            loaded = yaml.safe_load(yaml_file) or {}

        objects = loaded.get('objects', [])
        if not isinstance(objects, list):
            self.get_logger().warn(f'Invalid objects format in {world_objects_path}; expected a list')
            return []

        valid_objects = []
        for object_entry in objects:
            if isinstance(object_entry, dict) and self._pose_from_object_entry(object_entry) is not None:
                # Only include objects with target=true for ground truth comparison
                if object_entry.get('target', False):
                    valid_objects.append(object_entry)
        return valid_objects

    def build_ground_truth_marker_array(self) -> MarkerArray:
        marker_array = MarkerArray()

        delete_all_marker = Marker()
        delete_all_marker.action = Marker.DELETEALL
        delete_all_marker.header.frame_id = self.sim_world_name
        delete_all_marker.header.stamp = self.get_clock().now().to_msg()
        marker_array.markers.append(delete_all_marker)

        # Only visualize target objects
        target_objects = [obj for obj in self.world_objects if obj.get('target', False)]
        for marker_id, object_entry in enumerate(target_objects):
            pose = self._pose_from_object_entry(object_entry)
            if pose is None:
                continue

            object_name = str(object_entry.get('name', f'object_{marker_id}'))

            sphere_marker = Marker()
            sphere_marker.header.frame_id = self.sim_world_name
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = 'ground_truth_objects'
            sphere_marker.id = marker_id * 2
            sphere_marker.type = Marker.CUBE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position.x = pose[0]
            sphere_marker.pose.position.y = pose[1]
            sphere_marker.pose.position.z = pose[2]
            sphere_marker.pose.orientation.w = 1.0
            sphere_marker.scale.x = 0.35
            sphere_marker.scale.y = 0.35
            sphere_marker.scale.z = 0.35
            sphere_marker.color.r = 0.1
            sphere_marker.color.g = 1.0
            sphere_marker.color.b = 0.2
            sphere_marker.color.a = 0.7
            sphere_marker.lifetime.sec = 0
            marker_array.markers.append(sphere_marker)

            text_marker = Marker()
            text_marker.header.frame_id = self.sim_world_name
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'ground_truth_objects'
            text_marker.id = marker_id * 2 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pose[0]
            text_marker.pose.position.y = pose[1]
            text_marker.pose.position.z = pose[2] + 0.6
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.25
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = object_name
            text_marker.lifetime.sec = 0
            marker_array.markers.append(text_marker)

        return marker_array

    def publish_ground_truth_markers(self):
        if not self.is_simulation or self.ground_truth_marker_pub is None:
            return

        self.ground_truth_marker_pub.publish(self.build_ground_truth_marker_array())
        self.get_logger().info(f"Published ground truth markers for {len(self.world_objects)} objects in world '{self.sim_world_name}'")

    def landmarks_callback(self, msg):
        """Track landmarks by ID, keeping only the latest state from callbacks."""
        for incoming in msg.landmarks:
            self.landmarks_by_id[int(incoming.id)] = self._copy_landmark(incoming)

        # If target_objects is configured, start a one-minute shutdown timer
        # once the required number of landmarks have been observed.
        try:
            found_count = len(self.landmarks_by_id)
        except Exception:
            found_count = 0

        if self.target_objects and found_count >= self.target_objects:
            if self.target_objects_shutdown_timer is None:
                self.get_logger().info(
                    f"Found {found_count}/{self.target_objects} target objects; scheduling shutdown in 60s"
                )
                # Schedule shutdown after 60 seconds
                self.target_objects_shutdown_timer = self.create_timer(
                    60.0,
                    self.target_objects_shutdown_callback,
                )
                
    @staticmethod
    def _copy_landmark(source_landmark: Landmark) -> Landmark:
        copied_landmark = Landmark()
        copied_landmark.id = source_landmark.id
        copied_landmark.class_name = source_landmark.class_name
        copied_landmark.confidence = source_landmark.confidence
        copied_landmark.position.x = source_landmark.position.x
        copied_landmark.position.y = source_landmark.position.y
        copied_landmark.position.z = source_landmark.position.z
        copied_landmark.covariance = list(source_landmark.covariance)
        return copied_landmark

    def projected_map_callback(self, msg: OccupancyGrid):
        """Track projected map known size from the latest occupancy grid."""
        known_cells = sum(1 for v in msg.data if v != -1)
        resolution = float(msg.info.resolution)

        self.projected_map_known_cells = known_cells
        self.projected_map_known_area_m2 = known_cells * resolution * resolution

    def battery_callback(self, msg: BatteryStateArray):
        """Track the current battery percentage from the latest battery state array."""
        if not msg.battery_states:
            self.battery_percentage = math.nan
            return

        self.battery_percentage = float(msg.battery_states[0].charge_percentage)

    def exploration_done_callback(self, msg: Empty):
        """Called when exploration is complete. Flush data and shutdown."""
        self.get_logger().info("Received exploration_done signal. Shutting down collector...")
        self.flush_to_csv()
        rclpy.shutdown()

    def target_objects_shutdown_callback(self):

        # Prevent re-entry
        if self.target_objects_shutdown_timer is not None:
            try:
                self.target_objects_shutdown_timer.cancel()
            except Exception:
                pass
            self.target_objects_shutdown_timer = None

        self.get_logger().info(
            f"Target objects threshold reached ({self.target_objects}); shutting down collector now..."
        )
        self.flush_to_csv()
        rclpy.shutdown()

    def get_elapsed_time_sec(self):
        """Return elapsed time in seconds since node startup."""
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """Convert quaternion to yaw (rotation around Z)."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_to_pi(angle_rad: float) -> float:
        """Wrap angle to [-pi, pi]."""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def update_motion_from_tf(self):
        """Integrate XY translation and yaw rotation using TF transforms."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame_id,
                self.base_frame_id,
                rclpy.time.Time(),
            )
        except TransformException as ex:
            self.get_logger().debug(
                f"TF lookup failed ({self.reference_frame_id} -> {self.base_frame_id}): {ex}"
            )
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        current_xy = (x, y)
        q = transform.transform.rotation
        current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        if self.last_xy is not None:
            dx = current_xy[0] - self.last_xy[0]
            dy = current_xy[1] - self.last_xy[1]
            self.distance_traveled_m += math.hypot(dx, dy)

        if self.last_yaw is not None:
            dyaw = self.wrap_to_pi(current_yaw - self.last_yaw)
            self.rotation_traveled_rad += abs(dyaw)

        self.last_xy = current_xy
        self.last_yaw = current_yaw

    def get_landmarks_count(self) -> int:
        return len(self.landmarks_by_id)

    def get_mean_confidence(self) -> float:
        if not self.landmarks_by_id:
            return 0.0
        return sum(float(lm.confidence) for lm in self.landmarks_by_id.values()) / len(self.landmarks_by_id)

    def get_mean_covariance_trace(self) -> float:
        if not self.landmarks_by_id:
            return math.nan

        traces = []
        for landmark in self.landmarks_by_id.values():
            covariance = getattr(landmark, 'covariance', None)
            if covariance is None or len(covariance) < 9:
                continue
            traces.append(float(covariance[0]) + float(covariance[4]) + float(covariance[8]))

        if not traces:
            return math.nan

        return sum(traces) / len(traces)

    def get_geometric_error(self) -> float:
        if not self.is_simulation or not self.world_objects or not self.landmarks_by_id:
            return math.nan

        total_error = 0.0
        valid_landmark_count = 0

        # Only compare against target objects
        target_objects = [obj for obj in self.world_objects if obj.get('target', False)]
        
        for landmark in self.landmarks_by_id.values():
            landmark_position = (
                float(landmark.position.x),
                float(landmark.position.y),
                float(landmark.position.z),
            )

            nearest_distance = math.nan
            for object_entry in target_objects:
                object_pose = self._pose_from_object_entry(object_entry)
                if object_pose is None:
                    continue

                distance = math.dist(landmark_position, object_pose)
                if math.isnan(nearest_distance) or distance < nearest_distance:
                    nearest_distance = distance

            if not math.isnan(nearest_distance):
                total_error += nearest_distance
                valid_landmark_count += 1

        if valid_landmark_count == 0:
            return math.nan

        return total_error / valid_landmark_count

    def collect_snapshot(self):
        """Update TF-derived motion and append one row with latest callback state."""
        self.update_motion_from_tf()
        self.geometric_error = self.get_geometric_error()
        self.mean_covariance_trace = self.get_mean_covariance_trace()

        self.rows.append({
            'elapsed_s': self.get_elapsed_time_sec(),
            'distance_m_cum': self.distance_traveled_m,
            'rotation_rad_cum': self.rotation_traveled_rad,
            'landmarks_count': self.get_landmarks_count(),
            'mean_confidence': self.get_mean_confidence(),
            'mean_covariance_trace': self.mean_covariance_trace,
            'geometric_error': self.geometric_error,
            'battery_percentage': self.battery_percentage,
            'projected_map_known_area_m2': self.projected_map_known_area_m2,
        })

    def flush_to_csv(self):
        """Write all collected rows to CSV once (MVP behavior)."""
        self.output_dir.mkdir(parents=True, exist_ok=True)
        fieldnames = [
            'elapsed_s',
            'distance_m_cum',
            'rotation_rad_cum',
            'landmarks_count',
            'mean_confidence',
            'mean_covariance_trace',
            'geometric_error',
            'battery_percentage',
            'projected_map_known_area_m2',
        ]

        with self.output_file_path.open('w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)

        self.get_logger().info(
            f"Saved {len(self.rows)} rows to {self.output_file_path}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Collector()
    try:
        rclpy.spin(node)
    finally:
        node.flush_to_csv()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
