import os
import yaml
import numpy as np
from rclpy.clock import Clock
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo
from launch_ros.actions import SetRemap
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray


def evaluate_pattern(pattern: str, **kwargs) -> str:
    if not pattern:
        return pattern
    return pattern.format(**{k: v for k, v in kwargs.items() if v is not None})

def handle_config_file(context, pkg_path):
    
    # Get the config file path
    config_path = LaunchConfiguration('config_file').perform(context)
    
    # Load YAML config
    if not os.path.exists(config_path):
        
        LogInfo(msg=f"Config file not found at specified path. Checking default config location.").execute(context)
        config_path = os.path.join(pkg_path, 'config', os.path.basename(config_path))
        
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")
    
    LogInfo(msg=f"Using config file: {config_path}").execute(context)
    
    config = None
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
        config = {**config.get('launch', {}).get('ros__parameters', {}), **config.get('/**', {}).get('ros__parameters', {})}
    
    remaps = list(config['remappings'].items()) if 'remappings' in config and config['remappings'] else None
    if remaps is None:
        return config, config_path, []
        
    remap_actions = []
    for from_topic, to_topic in remaps:
        LogInfo(msg=f"Adding remapping: {from_topic} -> {to_topic}").execute(context)
        remap_actions.append(
            SetRemap(from_topic, to_topic)
        )
    
    return config, config_path, remap_actions

def imgmsg_to_bgr(bridge, msg):
    return bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def depth_imgmsg_to_float32(bridge, msg):
    if msg.encoding == '16UC1':
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        return depth_image.astype(np.float32) / 1000.0
    elif msg.encoding == '32FC1':
        return bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
    else:
        return None

def point_cloud(points: np.ndarray, parent_frame: str, stamp=None) -> PointCloud2:
        ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [
        PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate("xyzrgba")
    ]

    header = Header(frame_id=parent_frame, stamp=stamp or Clock().now().to_msg())

    return PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 7),
        row_step=(itemsize * 7 * points.shape[0]),
        data=data,
    )

def pointcloud2_to_array(msg: PointCloud2) -> np.ndarray:
        try:
        points_arr = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if points_arr.size == 0:
            return np.array([]).reshape(0, 3)
        if getattr(points_arr.dtype, "names", None):
            xyz = np.vstack([
                points_arr["x"].astype(np.float32),
                points_arr["y"].astype(np.float32),
                points_arr["z"].astype(np.float32),
            ]).T
            return xyz

        points_arr = np.asarray(points_arr, dtype=np.float32)
        if points_arr.ndim == 1:
            points_arr = points_arr.reshape(-1, 3)
        return points_arr[:, :3]
    except Exception:
        points_iter = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(points_iter))
        if points.size == 0:
            return np.array([]).reshape(0, 3)
        if getattr(points.dtype, "names", None):
            return np.vstack([
                points["x"].astype(np.float32),
                points["y"].astype(np.float32),
                points["z"].astype(np.float32),
            ]).T
        points = points.astype(np.float32)
        if points.ndim == 1:
            points = points.reshape(-1, 3)
        return points[:, :3]


def transform_points(points: np.ndarray, transform) -> np.ndarray:
        if points.size == 0:
        return points

    finite_mask = np.isfinite(points).all(axis=1)
    if not np.any(finite_mask):
        return np.array([]).reshape(0, 3)

    points = points[finite_mask]

    t = transform.transform.translation
    r = transform.transform.rotation
    rotation_matrix = R.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
    translation = np.array([t.x, t.y, t.z], dtype=np.float32)

    return (rotation_matrix @ points.T).T + translation


def array_to_pointcloud2(points: np.ndarray, frame_id: str, timestamp) -> PointCloud2:
        msg = PointCloud2()
    msg.header = Header(frame_id=frame_id, stamp=timestamp)

    msg.height = 1
    msg.width = len(points)

    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)

    point_data = points.tobytes()
    msg.data = point_data

    return msg


def _publish_marker_poses(
    node,
    poses,
    publisher,
    frame_id,
    namespace='markers',
    color=(1.0, 1.0, 1.0, 0.9),
    marker_type=Marker.SPHERE,
    marker_diameter=0.1,
):
    marker_array = MarkerArray()

    clear_marker = Marker()
    clear_marker.header.stamp = node.get_clock().now().to_msg()
    clear_marker.header.frame_id = frame_id
    clear_marker.ns = namespace
    clear_marker.action = Marker.DELETEALL
    marker_array.markers.append(clear_marker)

    for marker_id, pose in enumerate(poses, start=1):
        marker = Marker()
        marker.header.stamp = node.get_clock().now().to_msg()
        marker.header.frame_id = frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = marker_diameter
        marker.scale.y = marker_diameter
        marker.scale.z = marker_diameter
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker_array.markers.append(marker)

    publisher.publish(marker_array)


def publish_pose_stamped_markers(
    node,
    pose_stamped_list,
    publisher,
    frame_id,
    namespace='markers',
    color=(1.0, 1.0, 1.0, 0.9),
    marker_type=Marker.SPHERE,
    marker_diameter=0.1,
):
    poses = [item.pose for item in pose_stamped_list]
    _publish_marker_poses(
        node=node,
        poses=poses,
        publisher=publisher,
        frame_id=frame_id,
        namespace=namespace,
        color=color,
        marker_type=marker_type,
        marker_diameter=marker_diameter,
    )


def publish_point_markers(
    node,
    points,
    publisher,
    frame_id,
    namespace='markers',
    color=(1.0, 1.0, 1.0, 0.9),
    marker_type=Marker.SPHERE,
    marker_diameter=0.1,
):
    poses = []
    for point in points:
        pose = Pose()
        pose.position = point
        pose.orientation.w = 1.0
        poses.append(pose)

    _publish_marker_poses(
        node=node,
        poses=poses,
        publisher=publisher,
        frame_id=frame_id,
        namespace=namespace,
        color=color,
        marker_type=marker_type,
        marker_diameter=marker_diameter,
    )

