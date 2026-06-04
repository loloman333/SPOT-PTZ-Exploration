import numpy as np
from geometry_msgs.msg import Point


def quaternion_to_yaw(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def grid_indices_to_world_xy(indices, map_info):
    indices_array = np.asarray(indices)
    if indices_array.shape[-1] != 2:
        raise ValueError(f"Expected grid indices with shape (..., 2), got {indices_array.shape}")

    row = indices_array[..., 0]
    col = indices_array[..., 1]
    origin = map_info.origin

    map_x = (col.astype(float) + 0.5) * map_info.resolution
    map_y = (row.astype(float) + 0.5) * map_info.resolution

    yaw = quaternion_to_yaw(origin.orientation)
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)

    world_x = cos_yaw * map_x - sin_yaw * map_y + origin.position.x
    world_y = sin_yaw * map_x + cos_yaw * map_y + origin.position.y

    if indices_array.ndim == 1:
        return float(world_x), float(world_y)

    return world_x, world_y


def world_xy_to_point(world_x, world_y):
    point_msg = Point()
    point_msg.x = float(world_x)
    point_msg.y = float(world_y)
    point_msg.z = 0.0
    return point_msg


def normalize_minmax(values, invert=False):
    values = [float(v) for v in values]
    if not values:
        return []

    v_min = min(values)
    v_max = max(values)

    if np.isclose(v_max, v_min):
        return [1.0 for _ in values]

    if invert:
        return [(v_max - v) / (v_max - v_min) for v in values]

    return [(v - v_min) / (v_max - v_min) for v in values]


def euclidean_distance_xy(x0, y0, x1, y1):
    return float(np.hypot(float(x1) - float(x0), float(y1) - float(y0)))


def path_length(path):
    total = 0.0
    poses = path.poses
    for i in range(1, len(poses)):
        p0 = poses[i - 1].pose.position
        p1 = poses[i].pose.position
        dx = p1.x - p0.x
        dy = p1.y - p0.y
        total += np.sqrt(dx * dx + dy * dy)
    return float(total)
