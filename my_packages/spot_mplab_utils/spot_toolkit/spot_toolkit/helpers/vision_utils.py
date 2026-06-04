import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Point
from typing import List, Optional, Tuple, Iterable


from sensor_msgs.msg import PointCloud2
import logging
from synchros2.utilities import namespace_with
import math
from typing import Optional,Tuple
import logging
import numpy as np
from sensor_msgs.msg import Image, CameraInfo

from synchros2 import subscription
from synchros2.utilities import namespace_with
from bosdyn.client.math_helpers import Quat, SE2Pose, SE3Pose, Vec2, Vec3
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

from spot_toolkit.helpers.conversions import se3_to_msgpose

logger = logging.getLogger(__name__)

class LidarProjection:
    """Handles the projection of LiDAR points into the camera frame."""

    @staticmethod
    def transform_to_rvec_tvec(transform: TransformStamped) -> Tuple[np.ndarray, np.ndarray]:
        """Converts a TransformStamped to OpenCV's rotation and translation vectors."""
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        tvec = np.array([translation.x, translation.y, translation.z], dtype=np.float64)
        rot_matrix = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix()
        rvec, _ = cv2.Rodrigues(rot_matrix)
        return rvec, tvec

    @staticmethod
    def project_points(
        lidar_points: List[List[float]],
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        transform: TransformStamped
    ) -> Optional[np.ndarray]:
        """Projects 3D LiDAR points to 2D image coordinates."""
        try:
            rvec, tvec = LidarProjection.transform_to_rvec_tvec(transform)
            points_3d_np = np.asarray(lidar_points, dtype=np.float64).reshape(-1, 1, 3)

            image_points, _ = cv2.projectPoints(points_3d_np, rvec, tvec, camera_matrix, dist_coeffs)

            if image_points is None:
                logger.warning("cv2.projectPoints returned None.")
                return None

            return image_points.reshape(-1, 2)

        except cv2.error as e:
            logger.exception(f"OpenCV error during projection: {e}")
            return None

    @staticmethod
    def filter_points_in_bbox(
        projected_points: np.ndarray,
        lidar_points: np.ndarray,
        bbox: Tuple[int, int, int, int],
        image_shape: Tuple[int, int]
    ) -> Optional[np.ndarray]:
        """Filters projected points to keep only those within the image and a bounding box."""
        height, width = image_shape
        x_min, y_min, x_max, y_max = bbox
        u, v = projected_points[:, 0], projected_points[:, 1]

        mask = (
            (u >= x_min) & (u <= x_max) &
            (v >= y_min) & (v <= y_max) &
            (u >= 0) & (u < width) &
            (v >= 0) & (v < height)
        )

        if not np.any(mask):
            logger.warning("No LiDAR points projected into the bounding box.")
            return None

        # Combine original 3D points with their 2D projections
        # Shape: (N, 5) -> [u, v, x, y, z]
        return np.hstack((projected_points[mask], lidar_points[mask]))


class SpatialAnalysis:
    """Analyzes spatial data from depth images and 3D point clouds."""

    @staticmethod
    def find_dominant_cluster_center(
        points_3d: np.ndarray,
        bin_size: float = 0.5,
        min_points_in_bin: int = 50,
        max_distance: float = 8.0
    ) -> Optional[np.ndarray]:
        """
        Finds the center of the most dominant cluster of points based on distance.
        It uses a histogram to find the distance bin with the most points.
        """
        if points_3d.shape[0] < min_points_in_bin:
            return None

        distances = np.linalg.norm(points_3d, axis=1)
        num_bins = int(np.ceil(max_distance / bin_size))
        hist, bin_edges = np.histogram(distances, bins=num_bins, range=(0, max_distance))

        # Find the bin with the most points that meets the minimum threshold
        dominant_bin_index = -1
        max_count = min_points_in_bin -1
        for i, count in enumerate(hist):
            if count > max_count:
                max_count = count
                dominant_bin_index = i

        if dominant_bin_index != -1:
            d_min, d_max = bin_edges[dominant_bin_index], bin_edges[dominant_bin_index + 1]
            mask = (distances >= d_min) & (distances < d_max)
            cluster_points = points_3d[mask]
            return cluster_points.mean(axis=0)

        logger.warning("No dominant point cluster found meeting criteria.")
        return None
    
    @staticmethod
    def estimate_dominant_depth(
        depth_img: np.ndarray,
        x_min: int,
        x_max: int,
        y_min: int,
        y_max: int,
        depth_scale: float,
        bin_size: float = 0.2,
        min_points: int = 10,
        max_distance: float = 8.0
    ) -> float:
        """
        Estimates the dominant depth value in a bounding box using histogram binning.

        Args:
            depth_img (np.ndarray): Raw depth image
            x_min, x_max, y_min, y_max (int): Bounding box coordinates
            depth_scale (float): Scale to convert depth units to meters
            bin_size (float): Bin size in meters
            min_points (int): Minimum number of points to consider a bin valid
            max_distance (float): Max depth to consider

        Returns:
            float: Mean depth of the dominant bin, or max_distance if none found
        """
        roi = depth_img[y_min:y_max, x_min:x_max].flatten()
        depths = roi / depth_scale
        depths = depths[depths > 0.0]

        if len(depths) == 0:
            return max_distance

        bins = int(np.ceil(max_distance / bin_size))
        hist, edges = np.histogram(depths, bins=bins, range=(0.0, max_distance))

        for count, (low, high) in zip(hist, zip(edges[:-1], edges[1:])):
            if count > min_points:
                valid_depths = depths[(depths > low) & (depths < high)]
                if len(valid_depths) > 0:
                    return np.mean(valid_depths)

        return max_distance
    
    @staticmethod
    def depth_to_xyz(depth, pixel_x, pixel_y, focal_length, principal_point):
        """Calculate the transform to point in image using camera intrinsics and depth"""
        x = depth * (pixel_x - principal_point.x) / focal_length.x
        y = depth * (pixel_y - principal_point.y) / focal_length.y
        z = depth
        return x, y, z

    
class PoseUtils:
    """Utility functions for generating and manipulating Pose objects."""

    @staticmethod
    def get_pose_along_ray_2d(
        target_point: Point,
        distance_margin: Optional[float] = None,
        origin_point: Optional[Point] = None
    ) -> Optional[Pose]:
        """
        Generates a 3D Pose located near a target point and oriented to face it in the 2D plane (XY).

        The orientation is calculated in 2D based on the heading from an optional origin point to the target point.
        Optionally, the pose can be placed at a fixed distance before the target point, along the ray direction.

        Args:
            target_point (Point): The target point that the pose should face.
            distance_margin (float, Optional): If provided, the pose is placed this much closer to the origin along the ray.
            origin_point (Point, Optional): Origin point from which to compute the direction. Defaults to (0, 0, 0).

        Returns:
            Pose: A 3D Pose located and oriented to face the target point in the XY plane.
        """
        
        origin = origin_point if origin_point else Point()

        norm_vec = PoseUtils._get_norm_vec(origin, target_point)
        if norm_vec is None:
            return None

        heading = PoseUtils._get_heading(norm_vec) # pass xhat as norm_vec[0] for 2D heading

        # Apply distance margin if requested
        if distance_margin is not None:
            pos = np.array([
                target_point.x - norm_vec[0] * distance_margin,
                target_point.y - norm_vec[1] * distance_margin,
                target_point.z - norm_vec[2] * distance_margin
            ])
        else:
            pos = np.array([target_point.x, target_point.y, target_point.z])

        # Build rotation around Z (yaw only)
        q = R.from_euler("z", heading).as_quat()  # [x, y, z, w]

        # Use helpers
        mat = PoseUtils._make_pose(pos, R.from_quat(q))

        return PoseUtils._matrix_to_pose(mat)
    
    @staticmethod
    def _get_heading(xhat) -> float:
        """Computes the heading angle in radians based on the xhat vector.
        Args:
            xhat (np.ndarray): A 3D vector representing the direction in the XY plane """
        zhat = [0.0, 0.0, 1.0]
        yhat = np.cross(zhat, xhat)
        mat = np.array([xhat, yhat, zhat]).transpose()
        return Quat.from_matrix(mat).to_yaw()
 
    def get_pose_along_ray_3d(
        target_point: Point,
        distance_margin: Optional[float] = None,
        origin_point: Optional[Point] = None,
    ) -> Optional[Pose]:
        """
        Generates a 3D Pose located near a target point and oriented to face it in 3D space.

        The orientation is calculated in 3D based on the direction from an optional origin point to the target point.
        Optionally, the pose can be placed at a fixed distance before the target point, along the ray direction.

        Args:
            target_point (Point): The target point that the pose should face.
            distance_margin (float, Optional): If provided, the pose is placed this much closer to the origin along the ray.
            origin_point (Point, Optional): Origin point from which to compute the direction. Defaults to (0, 0, 0)."""

        origin = origin_point if origin_point else Point()
        norm_vec = PoseUtils._get_norm_vec(origin, target_point)
        if norm_vec is None:
            return None

        # Apply distance margin if requested
        if distance_margin is not None:
            pos = np.array([
                target_point.x - norm_vec[0] * distance_margin,
                target_point.y - norm_vec[1] * distance_margin,
                target_point.z - norm_vec[2] * distance_margin
            ])
        else:
            pos = np.array([target_point.x, target_point.y, target_point.z])

        u = Vec3(x=1, y=0, z=0)
        v = Vec3(norm_vec[0], norm_vec[1], norm_vec[2])

        #Returns a quaternion representing the rotation from u to v.
        quat = Quat.from_two_vectors(u, v)
        final = SE3Pose(pos[0],pos[1],pos[2],quat)

        
        return se3_to_msgpose(final)


    def _get_norm_vec(from_pt: Point, to_pt: Point) -> Optional[np.ndarray]:
        """Helper to compute normalized vector from one point to another."""
        dx = to_pt.x - from_pt.x
        dy = to_pt.y - from_pt.y
        dz = to_pt.z - from_pt.z
        vec = np.array([dx, dy, dz])
        norm = np.linalg.norm(vec)
        if norm == 0:
            return None
        return vec / norm
            
    @staticmethod
    def smooth_and_limit_stream(
        pose_stream: Iterable[Pose],
        alpha: float = 0.2,
        alpha_yaw: float = 0.2,
        max_translation_jump: float = 0.5,
        max_rotation_jump_deg: float = 45.0,
        max_translation_step: float = 0.05,
        max_rotation_step_deg: float = 10.0,
    ) -> List[Pose]:
        """
        Smooths and rate-limits a stream of Pose messages using EMA on XY and Yaw.

        This function applies three steps sequentially for each new pose:
        1.  **Outlier Rejection**: Discards new poses that are too far from the
            previous one in terms of XY distance and yaw angle.
        2.  **EMA Smoothing**: Applies an Exponential Moving Average to translation
            (X, Y, Z) and yaw for smoother transitions.
        3.  **Rate Limiting**: Clamps the final step size in translation (XY plane)
            and rotation (yaw) to ensure the output is kinematically feasible.

        Args:
            pose_stream: An iterable of geometry_msgs/Pose messages.
            alpha: The smoothing factor for EMA (lower is smoother).
            max_translation_jump: Max distance to be considered a valid pose (for outlier rejection).
            max_rotation_jump_deg: Max yaw change to be considered a valid pose (for outlier rejection).
            max_translation_step: The maximum allowed translational distance (XY) in one step.
            max_rotation_step_deg: The maximum allowed yaw rotation in one step.

        Returns:
            A list of the smoothed and rate-limited Pose messages.
        """
        smoothed_poses: List[Pose] = []
        last_output_matrix: np.ndarray = None

        # Convert step limits to radians for calculations
        max_rotation_jump_rad = np.deg2rad(max_rotation_jump_deg)
        max_rotation_step_rad = np.deg2rad(max_rotation_step_deg)

        for current_pose_msg in pose_stream:
            current_input_matrix = PoseUtils._pose_to_matrix(current_pose_msg)

            if last_output_matrix is None:
                smoothed_poses.append(current_pose_msg)
                last_output_matrix = current_input_matrix
                continue

            # --- Extract current and previous states (translation and yaw) ---
            t_new_in = current_input_matrix[:3, 3]
            t_last_out = last_output_matrix[:3, 3]
            yaw_new_in = R.from_matrix(current_input_matrix[:3, :3]).as_euler('xyz')[2]
            yaw_last_out = R.from_matrix(last_output_matrix[:3, :3]).as_euler('xyz')[2]

            # --- 1. Outlier Rejection ---
            translation_jump = np.linalg.norm(t_new_in[:2] - t_last_out[:2]) # Check in 2D
            
            # Calculate shortest angle difference for yaw jump
            yaw_jump = yaw_new_in - yaw_last_out
            yaw_jump = (yaw_jump + np.pi) % (2 * np.pi) - np.pi

            if translation_jump > max_translation_jump or abs(yaw_jump) > max_rotation_jump_rad:
                # If outlier, hold the last pose and skip processing
                smoothed_poses.append(PoseUtils._matrix_to_pose(last_output_matrix))
                continue

            # --- 2. EMA Smoothing ---
            # Smooth translation components directly
            t_ema = alpha * t_new_in + (1 - alpha) * t_last_out
            
            # Smooth yaw by interpolating along the shortest path
            yaw_ema = yaw_last_out + alpha_yaw * yaw_jump
            yaw_ema = (yaw_ema + np.pi) % (2 * np.pi) - np.pi # Re-normalize

            # --- 3. Rate Limiting (Clamping) ---
            # Limit the rate of change from the last output to the new smoothed target
            px, py, _ = t_last_out
            sx_ema, sy_ema, sz_ema = t_ema

            # Calculate change in the 2D plane (XY)
            dx, dy = sx_ema - px, sy_ema - py
            translation_step = math.hypot(dx, dy)

            # Calculate yaw change, handling angle wrapping
            dyaw = yaw_ema - yaw_last_out
            dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi

            # Clamp translation step
            if translation_step > max_translation_step:
                scale = max_translation_step / translation_step
                sx_clamped = px + dx * scale
                sy_clamped = py + dy * scale
            else:
                sx_clamped, sy_clamped = sx_ema, sy_ema

            # Clamp rotation step
            if abs(dyaw) > max_rotation_step_rad:
                final_yaw = yaw_last_out + math.copysign(max_rotation_step_rad, dyaw)
            else:
                final_yaw = yaw_ema

            # --- 4. Reconstruct Final Pose Matrix ---
            final_pose_matrix = np.eye(4)
            # Use clamped XY, smoothed Z, and clamped yaw for planar motion
            final_pose_matrix[:3, 3] = [sx_clamped, sy_clamped, sz_ema]
            final_pose_matrix[:3, :3] = R.from_euler('z', final_yaw).as_matrix()

            # Append the result and update the state for the next iteration
            smoothed_poses.append(PoseUtils._matrix_to_pose(final_pose_matrix))
            last_output_matrix = final_pose_matrix

        return smoothed_poses
    
    @staticmethod
    def _pose_to_matrix(msg: Pose):
        """Convert ROS2 Pose msg to 4x4 SE3 matrix"""
        t = np.array([msg.position.x, msg.position.y, msg.position.z])
        q = [msg.orientation.x, msg.orientation.y,
             msg.orientation.z, msg.orientation.w]
        Rm = R.from_quat(q).as_matrix()
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = t
        return T

    @staticmethod
    def _matrix_to_pose(mat):
        """Convert 4x4 SE3 matrix to ROS2 Pose msg"""
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = mat[:3, 3]
        q = R.from_matrix(mat[:3, :3]).as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        return pose

    @staticmethod
    def _make_pose(t, q: R):
        """Build SE3 4x4 matrix from translation and Rotation object"""
        T = np.eye(4)
        T[:3, :3] = q.as_matrix()
        T[:3, 3] = t
        return T
class Camera:
    def __init__(self, name: str, robot_name: str, rot: Optional[int] = None,
                 frame_name: Optional[str] = None, depth_frame: Optional[str] = None,
                 intrinsics: Optional[CameraInfo] = None) -> None:
        self.name = name
        self.rgb = namespace_with(robot_name, "camera", name, "image")
        self.rgbinfo = namespace_with(robot_name, "camera", name, "camera_info")
        self.depth = namespace_with(robot_name, "depth_registered", name, "image")
        self.depthinfo = namespace_with(robot_name, "depth_registered", name, "camera_info")

        self.frame = namespace_with(robot_name, frame_name) if frame_name else \
                     namespace_with(robot_name, name + "_fisheye")

        self.depth_frame = namespace_with(robot_name, depth_frame) if depth_frame else self.frame
        self.rot = rot
        self.instrinsics = intrinsics


class CameraManager:
    """Manages a selected set of Camera objects for a robot."""
    
    _all_camera_configs = {
        "back":       {"frame_name": "back"},
        "right":      {"rot": 1},
        "left":       {"rot": 0},
        "frontleft":  {"rot": 0},
        "frontright": {"rot": 0},
        "hand":       {"frame_name": "hand_color_image_sensor", "depth_frame": "hand_depth_sensor"}
    }

    def __init__(self, robot_name: str, enabled_cameras: Optional[List[str]] = None):
        self.robot_name = robot_name
        enabled = enabled_cameras or list(self._all_camera_configs.keys())

        self.cameras = []
        for cam_name in enabled:
            if cam_name not in self._all_camera_configs:
                raise ValueError(f"Unknown camera name: {cam_name}")
            config = self._all_camera_configs[cam_name]
            self.cameras.append(Camera(cam_name, robot_name, **config))

    def get_all_intrinsics(self):
        subs_camera_topics = [cam.rgbinfo for cam in self.cameras]
        subs_camera_msgtypes = [CameraInfo] * len(self.cameras)
        caminfos = subscription.wait_for_messages(subs_camera_topics, subs_camera_msgtypes)
        for cam, info in zip(self.cameras, caminfos):
            cam.instrinsics = info
        return self.cameras

    def find_camera_index(self, name: str) -> int:
        for i, cam in enumerate(self.cameras):
            if cam.name == name:
                return i
        return -1

    def get_camera(self, name: str) -> Optional[Camera]:
        idx = self.find_camera_index(name)
        return self.cameras[idx] if idx >= 0 else None