#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from scipy import ndimage
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from std_srvs.srv import Trigger
from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener, TransformException
from nav2_msgs.msg import BehaviorTreeLog
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from visualization_msgs.msg import MarkerArray, Marker
from ptz_exploration_core.utils.math import (
    euclidean_distance_xy,
    grid_indices_to_world_xy,
    path_length,
    world_xy_to_point,
)
from ptz_exploration_core.utils.ros import (
    publish_pose_stamped_markers,
    publish_point_markers,
)

class Explorer(Node):

    STATE_EXPLORE_FRONTIER = 'explore_frontier'
    STATE_SCAN_ENVIRONMENT = 'scan_environment'
    STATE_CONFIRM_CANDIDATES = 'confirm_candidates'

    def __init__(self):
        super().__init__('explorer')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_name', 'spotty'),
                ('base_frame_id', 'spotty/base_link'),
                ('explore_period_s', 1.0),
                ('scan_control_period_s', 0.1),
                ('blacklist_radius_m', 1.0),
                ('min_goal_distance_from_robot_m', 0.5),
                ('map_frame_id', 'map'),
                ('projected_map_topic', '/projected_map'),
                ('frontier_map_topic', '/frontier_map'),
                ('frontier_markers_topic', '/frontier_markers'),
                ('blacklisted_markers_topic', '/blacklisted_goal_markers'),
                ('exploration_done_topic', '/exploration_done'),
                ('min_frontier_size_cells', 20),
                ('weight_cells', 1.0),
                ('weight_distance', 1.0),
                ('occupied_threshold', 50),
                ('distance_type', 'euclidean'),
                ('scan_service_name', '/scan_environment'),
                ('confirm_service_name', '/confirm_landmarks'),
                ('confirm_landmarks', True),
                ('scan_method', 'ptz_service'),
                ('scan_num_stops', 12),
                ('scan_dwell_time_s', 1.0),
                ('body_scan_cmd_vel_topic', '/cmd_vel'),
            ]
        )
        
        # Read parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.explore_period_s = self.get_parameter('explore_period_s').value
        self.scan_control_period_s = self.get_parameter('scan_control_period_s').value
        self.blacklist_radius_m = self.get_parameter('blacklist_radius_m').value
        self.min_goal_distance_from_robot_m = self.get_parameter('min_goal_distance_from_robot_m').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.projected_map_topic = self.get_parameter('projected_map_topic').value
        self.frontier_map_topic = self.get_parameter('frontier_map_topic').value
        self.frontier_markers_topic = self.get_parameter('frontier_markers_topic').value
        self.blacklisted_markers_topic = self.get_parameter('blacklisted_markers_topic').value
        self.exploration_done_topic = self.get_parameter('exploration_done_topic').value
        self.min_frontier_size_cells = int(self.get_parameter('min_frontier_size_cells').value)
        self.weight_cells = float(self.get_parameter('weight_cells').value)
        self.weight_distance = float(self.get_parameter('weight_distance').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.distance_type = self.get_parameter('distance_type').value
        self.scan_service_name = self.get_parameter('scan_service_name').value
        self.confirm_service_name = self.get_parameter('confirm_service_name').value
        self.confirm_landmarks = bool(self.get_parameter('confirm_landmarks').value)
        self.scan_method = self.get_parameter('scan_method').value
        self.scan_num_stops = int(self.get_parameter('scan_num_stops').value)
        self.scan_dwell_time_s = float(self.get_parameter('scan_dwell_time_s').value)
        self.body_scan_cmd_vel_topic = self.get_parameter('body_scan_cmd_vel_topic').value

        if self.distance_type not in ('euclidean', 'path_length'):
            self.get_logger().warn(
                f"Invalid distance_type='{self.distance_type}', falling back to 'euclidean'"
            )
            self.distance_type = 'euclidean'

        if self.scan_method not in ('ptz_service', 'body_rotation', 'none'):
            self.get_logger().warn(
                f"Invalid scan_method='{self.scan_method}', falling back to 'ptz_service'"
            )
            self.scan_method = 'ptz_service'

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.frontier_pub = self.create_publisher(
            MarkerArray,
            self.blacklisted_markers_topic,
            10
        )

        self.frontier_map_pub = self.create_publisher(
            OccupancyGrid,
            self.frontier_map_topic,
            10
        )

        self.frontier_markers_pub = self.create_publisher(
            MarkerArray,
            self.frontier_markers_topic,
            10
        )
        
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/exploration_goal',
            10
        )
        
        self.goal_map_pub = self.create_publisher(
            OccupancyGrid,
            '/goal_map',
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.body_scan_cmd_vel_topic,
            10
        )

        self.exploration_done_pub = self.create_publisher(
            Empty,
            self.exploration_done_topic,
            10
        )
                
        # Subscribers
        self.projected_map_sub = self.create_subscription(
            OccupancyGrid,
            self.projected_map_topic,
            self.projected_map_callback,
            10
        )
        
        self.bt_log_sub = self.create_subscription(
            BehaviorTreeLog, 
            '/behavior_tree_log', 
            self.bt_log_callback, 
            10
        )

        # Service clients (non-blocking)
        self.scan_client = self.create_client(Trigger, self.scan_service_name)
        self.confirm_client = self.create_client(Trigger, self.confirm_service_name)
        
        # Timers
        self.explore_timer = self.create_timer(
            self.explore_period_s,
            self.explore
        )
        self.scan_control_timer = self.create_timer(
            self.scan_control_period_s,
            self.handle_scan_state
        )

        # Internal class members (not ROS parameters)
        self.tf_lookup_timeout_s = 0.2
        self.follow_path_node_name = 'FollowPath'
        self.follow_path_running_status = 'RUNNING'
        self.occupancy_free_value = 0
        self.occupancy_unknown_value = -1
        
        # Class member variables
        self.current_map_msg = None
        self.navigator = BasicNavigator()
        self.has_started_moving = False
        self.current_goal = None
        self.goal_blacklist = []
        self.goal_in_progress = False
        self.loop_state = self.STATE_EXPLORE_FRONTIER
        self.scan_future = None
        self.confirm_future = None
        self.scan_yaw_angles = []
        self.scan_yaw_index = 0
        self.scan_yaw_start_time = None
        # Scan tuning defaults
        self.scan_angle_tolerance = 0.05
        self.scan_decel_threshold = 0.15
        self.scan_max_speed = 0.5
        self.scan_decel_speed = 0.2
        self.scan_settle_time_s = 0.5
        
        self.get_logger().info(
            f"Explorer initialized with robot_name={self.robot_name}")

    def projected_map_callback(self, msg):
        self.current_map_msg = msg
        
    def bt_log_callback(self, msg):
        for event in msg.event_log:
            if (
                event.node_name == self.follow_path_node_name
                and event.current_status == self.follow_path_running_status
            ):
                self.has_started_moving = True
                
    def explore(self):
        if self.loop_state == self.STATE_EXPLORE_FRONTIER:
            self.handle_explore_frontier_state()
            return

        if self.loop_state == self.STATE_SCAN_ENVIRONMENT:
            # Scanning is handled by the separate scan_control_timer
            return

        if self.loop_state == self.STATE_CONFIRM_CANDIDATES:
            confirm_complete = self.confirm_candidates()
            if confirm_complete:
                self.set_loop_state(self.STATE_EXPLORE_FRONTIER)
            return

        self.get_logger().warn(f"Unknown loop_state='{self.loop_state}', resetting")
        self.set_loop_state(self.STATE_EXPLORE_FRONTIER)

    def handle_scan_state(self):
        if self.loop_state != self.STATE_SCAN_ENVIRONMENT:
            return

        scan_complete = self.scan_environment()
        if scan_complete:
            if self.confirm_landmarks:
                self.set_loop_state(self.STATE_CONFIRM_CANDIDATES)
            else:
                self.set_loop_state(self.STATE_EXPLORE_FRONTIER)

    def set_loop_state(self, next_state):
        if self.loop_state != next_state:
            self.get_logger().info(f"Loop state: {self.loop_state} -> {next_state}")
        self.loop_state = next_state

    def handle_navigation_result(self):
        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
        self.get_logger().info('Arrived at frontier!')
        return True  # Scan after successful arrival

        elif result == TaskResult.FAILED:
        if not self.has_started_moving:
        self.get_logger().warn('PLANNER FAILED')
        self.goal_blacklist.append(self.current_goal)
        return False  # Skip scan, re-plan immediately
        else:
        self.get_logger().error('CONTROLLER FAILED')
        return True  # Scan after controller failure

        elif result == TaskResult.CANCELED:
        self.get_logger().info('Goal was canceled.')
        return True  # Scan after cancellation

        else:
        self.get_logger().error(f"Unknown navigation result: {result}")
        return True  # Conservative: scan on unknown result

    def handle_explore_frontier_state(self):
        if not self.goal_in_progress:
            self.explore_best_frontier()
            return

        if not self.navigator.isTaskComplete():
            return

        should_scan = self.handle_navigation_result()
        self.goal_in_progress = False
        
        if should_scan:
            self.set_loop_state(self.STATE_SCAN_ENVIRONMENT)
        else:
            # Planner failed: skip scan and immediately re-plan
            self.set_loop_state(self.STATE_EXPLORE_FRONTIER)

    def scan_environment(self):
        if self.scan_method == 'none':
            return self.scan_environment_none()

        if self.scan_method == 'body_rotation':
            return self.scan_environment_body_rotation()

        return self.scan_environment_ptz_service()

    def scan_environment_none(self):
        return True

    def scan_environment_ptz_service(self):
        if self.scan_future is None:
            if not self.scan_client.wait_for_service(timeout_sec=0.0):
                self.get_logger().warn(
                    f"Waiting for scan service: {self.scan_service_name}",
                    throttle_duration_sec=2.0,
                )
                return False

            self.get_logger().info("Calling scan_environment trigger")
            self.scan_future = self.scan_client.call_async(Trigger.Request())
            return False

        if not self.scan_future.done():
            return False

        try:
            response = self.scan_future.result()
        except Exception as ex:
            self.get_logger().error(f"scan_environment trigger failed: {ex}")
            self.scan_future = None
            return True

        self.scan_future = None
        if response is None:
            self.get_logger().warn("scan_environment returned no response")
            return True

        if response.success:
            self.get_logger().info(f"scan_environment complete: {response.message}")
        else:
            self.get_logger().warn(f"scan_environment reported failure: {response.message}")
        return True

    def scan_environment_body_rotation(self):
        frame_id = self.current_map_msg.header.frame_id if self.current_map_msg is not None else self.map_frame_id
        current_yaw = self.get_robot_yaw_in_frame(frame_id)
        if current_yaw is None:
            self.stop_body_rotation_scan()
            self.get_logger().warn("Body scan aborted: could not get robot yaw")
            return True

        # Initialize scan on first call
        if not self.scan_yaw_angles:
            num_stops = max(1, self.scan_num_stops)
            start_yaw = current_yaw
            self.scan_yaw_angles, angle_increment = self.build_scan_yaw_targets(start_yaw, num_stops)
            self.scan_yaw_index = 0
            self.scan_yaw_start_time = None
            self.get_logger().info(
                f"Starting stepwise body scan with {num_stops} stops starting_from_yaw={start_yaw:.3f}, angle_increment={angle_increment:.3f}"
            )
            return False

        # Check if we've completed all stops
        if self.scan_yaw_index >= len(self.scan_yaw_angles):
            self.stop_body_rotation_scan()
            self.get_logger().info("Body scan complete")
            return True

        target_yaw = self.scan_yaw_angles[self.scan_yaw_index]

        remaining_angle = self.get_scan_remaining_angle(current_yaw, target_yaw)

        # If within tolerance, stop and start dwell
        if remaining_angle <= self.scan_angle_tolerance:
            if self.handle_scan_dwell():
                self.scan_yaw_index += 1
                self.scan_yaw_start_time = None
                if self.scan_yaw_index < len(self.scan_yaw_angles):
                    self.get_logger().info(f"Moving to stop {self.scan_yaw_index + 1}/{len(self.scan_yaw_angles)}")
            return False

        self.publish_scan_angular_velocity(self.get_scan_command_speed(remaining_angle))
        return False

    def build_scan_yaw_targets(self, start_yaw, num_stops):
        angle_increment = 2.0 * math.pi / float(num_stops)
        targets = []

        if num_stops == 1:
            targets.append(start_yaw)
        else:
            for stop_index in range(1, num_stops):
                targets.append(start_yaw + angle_increment * stop_index)
            targets.append(start_yaw + 2.0 * math.pi)

        return targets, angle_increment

    def get_scan_remaining_angle(self, current_yaw, target_yaw):
        return (target_yaw - current_yaw) % (2.0 * math.pi)

    def get_scan_command_speed(self, remaining_angle):
        return self.scan_decel_speed if remaining_angle <= self.scan_decel_threshold else self.scan_max_speed

    def handle_scan_dwell(self):
        settle_time_s = 0.5

        if self.scan_yaw_start_time is None:
            self.scan_yaw_start_time = self.get_clock().now()
            self.publish_scan_angular_velocity(0.0)
            self.get_logger().info(
                f"Reached stop {self.scan_yaw_index + 1}/{len(self.scan_yaw_angles)}, settling..."
            )
            return False

        settle_elapsed = (self.get_clock().now() - self.scan_yaw_start_time).nanoseconds * 1e-9
        return settle_elapsed >= self.scan_settle_time_s + self.scan_dwell_time_s

    def confirm_candidates(self):
        if not self.confirm_landmarks:
            return True

        if self.confirm_future is None:
            if not self.confirm_client.wait_for_service(timeout_sec=0.0):
                self.get_logger().warn(
                    f"Waiting for confirm service: {self.confirm_service_name}",
                    throttle_duration_sec=2.0,
                )
                return False

            self.get_logger().info("Calling confirm_landmarks trigger")
            self.confirm_future = self.confirm_client.call_async(Trigger.Request())
            return False

        if not self.confirm_future.done():
            return False

        try:
            response = self.confirm_future.result()
        except Exception as ex:
            self.get_logger().error(f"confirm_landmarks trigger failed: {ex}")
            self.confirm_future = None
            return True

        self.confirm_future = None
        if response is None:
            self.get_logger().warn("confirm_landmarks returned no response")
            return True

        if response.success:
            self.get_logger().info(f"confirm_landmarks complete: {response.message}")
        else:
            self.get_logger().warn(f"confirm_landmarks reported failure: {response.message}")
        return True

    def get_robot_pose_in_frame(self, frame_id):
        # TF lookup to get robot position in requested frame
        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id,
                self.base_frame_id,
                Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_s)
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform {self.base_frame_id} -> {frame_id}: {ex}")
            return None

        robot_pose = PoseStamped()
        robot_pose.header.frame_id = frame_id
        robot_pose.header.stamp = transform.header.stamp
        robot_pose.pose.position.x = transform.transform.translation.x
        robot_pose.pose.position.y = transform.transform.translation.y
        robot_pose.pose.position.z = transform.transform.translation.z
        return robot_pose

    def get_robot_yaw_in_frame(self, frame_id):
        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id,
                self.base_frame_id,
                Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_s)
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform {self.base_frame_id} -> {frame_id}: {ex}"
            )
            return None

        q = transform.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_scan_angular_velocity(self, angular_speed_rad_s):
        cmd = Twist()
        cmd.angular.z = float(angular_speed_rad_s)
        self.cmd_vel_pub.publish(cmd)

    def stop_body_rotation_scan(self):
        self.publish_scan_angular_velocity(0.0)
        self.scan_yaw_angles = []
        self.scan_yaw_index = 0
        self.scan_yaw_start_time = None

    def dispatch_goal(self, goal_msg):
        if goal_msg is None:
            self.goal_in_progress = False
            self.set_loop_state(self.STATE_SCAN_ENVIRONMENT)
            return

        self.goal_pub.publish(goal_msg)

        self.has_started_moving = False
        self.current_goal = goal_msg
        self.goal_in_progress = True
        self.navigator.goToPose(goal_msg)

    def explore_best_frontier(self):
        if self.current_map_msg is None:
            return

        frame_id = self.current_map_msg.header.frame_id or self.map_frame_id
        robot_pose = self.get_robot_pose_in_frame(frame_id)
        if robot_pose is None:
            return

        ranked_frontiers = self.compute_ranked_frontiers(robot_pose)
        self.publish_blacklisted_goals(
            self.goal_blacklist,
            frame_id
        )
        
        # Compute the next goal based on ranked frontier points and current position
        goal_msg = self.select_goal(
            ranked_frontiers,
            robot_pose,
            frame_id
        )

        self.dispatch_goal(goal_msg)

    def compute_ranked_frontiers(self, robot_pose):
        if self.current_map_msg is None:
            return []

        occupancy_map = np.array(self.current_map_msg.data).reshape(
            (self.current_map_msg.info.height, self.current_map_msg.info.width)
        )

        # Frontier detection: free cells neighboring unknown cells (8-connectivity)
        free_mask = occupancy_map == self.occupancy_free_value
        unknown_mask = occupancy_map == self.occupancy_unknown_value
        frontier_map = ndimage.binary_dilation(unknown_mask, np.ones((3, 3))) & free_mask

        self.publish_frontier_map(frontier_map)

        robot_position_xy = (
            float(robot_pose.pose.position.x),
            float(robot_pose.pose.position.y),
        )

        # Cluster frontier cells using 8-connected components labeling
        labeled, num_features = ndimage.label(frontier_map)
        frontiers = []
        for i in range(1, num_features + 1):
            cells = np.argwhere(labeled == i)
            if len(cells) < self.min_frontier_size_cells:
                continue

            # Select representative point: cell closest to cluster centroid
            representative_indices = self.select_frontier_representative_indices(cells)
            point = self.indices_to_point(representative_indices, self.current_map_msg.info)
            
            # Compute raw features: cluster size (s_k) and distance (d_k)
            raw_cells = self.compute_cells_score(cells)

            if self.distance_type == 'path_length':
                raw_distance = self.compute_path_distance(point, robot_pose, robot_position_xy)
            else:
                raw_distance = self.compute_distance_score(point, robot_position_xy)

            frontiers.append({
                'point': point,
                'cells': raw_cells,
                'distance': raw_distance,
            })

        if not frontiers:
            self.publish_frontiers_markers([])
            self.get_logger().info('Explorer computed 0 ranked frontiers')
            return []

        # Normalize features per batch: s_tilde_k = s_k / max(s_j), d_tilde_k = 1 - d_k / max(d_j)
        normalized_cells = self.normalize_per_batch([item['cells'] for item in frontiers])
        normalized_distance = self.normalize_per_batch(
            [item['distance'] for item in frontiers],
            invert=True
        )

        # Compute weighted scores: S_k = weight_cells * s_tilde_k + weight_distance * d_tilde_k
        weighted_frontiers = []
        for idx, item in enumerate(frontiers):
            score = (
                self.weight_cells * normalized_cells[idx]
                + self.weight_distance * normalized_distance[idx]
            )
            weighted_frontiers.append((float(score), item['point']))

        # Sort by score descending
        weighted_frontiers.sort(key=lambda item: item[0], reverse=True)
        self.publish_frontiers_markers(weighted_frontiers)
        return weighted_frontiers

    def select_frontier_representative_indices(self, cells):        
        centroid = cells.mean(axis=0)
        squared_distances = np.sum((cells - centroid) ** 2, axis=1)
        representative_idx = int(np.argmin(squared_distances))
        return cells[representative_idx]

    def select_goal(self, ranked_frontiers, robot_pose=None, frame_id='map'):
        if len(ranked_frontiers) == 0:
            self.get_logger().warn("No frontiers found!")
            self.publish_exploration_done()
            return None
        
        # Filter 1: Remove frontiers within blacklist_radius_m of previously failed goals
        filtered_frontiers = []
        
        for score, frontier_pos in ranked_frontiers:
            is_blacklisted = False
            
            for blacklist_goal in self.goal_blacklist:
                blacklist_pos = blacklist_goal.pose.position
                distance = euclidean_distance_xy(
                    frontier_pos.x,
                    frontier_pos.y,
                    blacklist_pos.x,
                    blacklist_pos.y,
                )
                if distance < self.blacklist_radius_m:
                    is_blacklisted = True
                    break
            
            if not is_blacklisted:
                filtered_frontiers.append((score, frontier_pos))
        
        if len(filtered_frontiers) == 0:
            self.get_logger().warn(
                f"All frontiers are within {self.blacklist_radius_m:.2f}m of blacklisted goals!"
            )
            self.publish_exploration_done()
            return None
        
        # Filter 2: Remove frontiers closer than min_goal_distance_from_robot_m
        valid_frontiers = filtered_frontiers
        if robot_pose is not None:
            robot_pos = robot_pose.pose.position

            valid_frontiers = []
            for score, frontier_pos in filtered_frontiers:
                dist = euclidean_distance_xy(
                    robot_pos.x,
                    robot_pos.y,
                    frontier_pos.x,
                    frontier_pos.y,
                )
                if dist > self.min_goal_distance_from_robot_m:
                    valid_frontiers.append((score, frontier_pos))

            if len(valid_frontiers) == 0:
                self.get_logger().warn(
                    f"No frontier found farther than {self.min_goal_distance_from_robot_m:.2f}m from robot."
                )
                self.publish_exploration_done()
                return None

        # Greedy selection: pick the frontier with highest score S_k
        # (already sorted high->low by score from compute_ranked_frontiers)
        if len(valid_frontiers) > 0:
            return self.frontier_to_goal_pose(valid_frontiers[0][1], frame_id)
        
        return None

    def frontier_to_goal_pose(self, frontier_point, frame_id):
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = frontier_point.x
        goal.pose.position.y = frontier_point.y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        return goal

    def compute_cells_score(self, cells):
        return float(len(cells))

    def compute_distance_score(self, frontier_point, robot_position_xy):
        return euclidean_distance_xy(
            robot_position_xy[0],
            robot_position_xy[1],
            frontier_point.x,
            frontier_point.y,
        )

    def publish_frontier_map(self, frontier_map):
        if self.current_map_msg is None:
            return

        frontier_map_msg = OccupancyGrid()
        frontier_map_msg.header.stamp = self.get_clock().now().to_msg()
        frontier_map_msg.header.frame_id = self.current_map_msg.header.frame_id
        frontier_map_msg.info = self.current_map_msg.info
        frontier_map_msg.data = np.where(frontier_map, 100, 0).astype(np.int8).ravel().tolist()
        self.frontier_map_pub.publish(frontier_map_msg)

    def normalize_per_batch(self, values, invert=False):
        if not values:
            return []

        finite_values = [float(v) for v in values if math.isfinite(float(v))]
        max_value = max(finite_values) if finite_values else 0.0

        normalized = []
        for value in values:
            value_float = float(value)

            if not math.isfinite(value_float):
                # Treat non-finite values as worst possible score before optional inversion.
                normalized_value = 1.0
            elif max_value <= 0.0:
                normalized_value = 0.0
            else:
                # Max-based normalization: 0 -> 0.0, max_value -> 1.0
                normalized_value = value_float / max_value

            if invert:
                normalized_value = 1.0 - normalized_value

            normalized.append(float(max(0.0, min(1.0, normalized_value))))

        return normalized

    def publish_frontiers_markers(self, frontier_candidates):
        if self.current_map_msg is None:
            return

        marker_diameter = max(2.0 * self.current_map_msg.info.resolution, 0.10)
        points = [point for _, point in frontier_candidates]
        publish_point_markers(
            node=self,
            points=points,
            publisher=self.frontier_markers_pub,
            frame_id=self.current_map_msg.header.frame_id,
            namespace='frontiers',
            color=(1.0, 0.5, 0.0, 0.9),
            marker_type=Marker.SPHERE,
            marker_diameter=marker_diameter,
        )

    def indices_to_point(self, indices, map_info):
        world_x, world_y = grid_indices_to_world_xy(indices, map_info)
        return world_xy_to_point(world_x, world_y)

    def compute_path_distance(self, frontier_point, robot_pose, robot_position_xy):
        if robot_pose is None:
            self.get_logger().warn(
                "Could not get robot pose for path planning; falling back to Euclidean distance"
            )
            return self.compute_distance_score(frontier_point, robot_position_xy)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = robot_pose.header.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = frontier_point.x
        goal_pose.pose.position.y = frontier_point.y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        try:
            path = self.navigator.getPath(robot_pose, goal_pose, use_start=True)
            if path is None:
                self.get_logger().warn(
                    f"Path planning failed to frontier at ({frontier_point.x:.2f}, {frontier_point.y:.2f}); "
                    "returning infinity to exclude this frontier"
                )
                return float('inf')
            return self._compute_path_length(path)
        except Exception as ex:
            self.get_logger().warn(
                f"Path planning exception for frontier: {ex}; returning infinity to exclude this frontier"
            )
            return float('inf')

    def _compute_path_length(self, path):
        return path_length(path)

    def publish_goal_map(self, goal_map, map_info, frame_id):
        goal_msg = OccupancyGrid()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = frame_id
        goal_msg.info = map_info
        goal_msg.data = goal_map.flatten().tolist()
        self.goal_map_pub.publish(goal_msg)
    
    def publish_blacklisted_goals(self, blacklisted_goals, frame_id):
        publish_pose_stamped_markers(
            node=self,
            pose_stamped_list=blacklisted_goals,
            publisher=self.frontier_pub,
            frame_id=frame_id,
            namespace='blacklisted_goals',
            color=(1.0, 0.0, 0.0, 0.9),
            marker_type=Marker.CUBE,
            marker_diameter=0.1,
        )

    def publish_exploration_done(self):
        self.exploration_done_pub.publish(Empty())
        self.get_logger().info("Exploration complete - published exploration_done signal")
            
def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
