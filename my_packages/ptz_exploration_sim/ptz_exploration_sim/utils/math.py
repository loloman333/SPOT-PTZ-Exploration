import numpy as np

def quaternion_to_matrix(qx, qy, qz, qw, tx, ty, tz):
        # Normalize quaternion
    norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    
    # Convert to rotation matrix
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
    ])
    
    M = np.eye(4)
    M[:3, :3] = R
    M[0, 3] = tx
    M[1, 3] = ty
    M[2, 3] = tz
    return M

def euler_to_matrix(r, p, y, tx, ty, tz):
    cx, sx = np.cos(r), np.sin(r)
    cy, sy = np.cos(p), np.sin(p)
    cz, sz = np.cos(y), np.sin(y)
    
    # Combined Rotation Matrix Rz * Ry * Rx
    R = np.array([
    [cy*cz, cz*sx*sy - cx*sz, cx*cz*sy + sx*sz],
    [cy*sz, cx*cz + sx*sy*sz, -cz*sx + cx*sy*sz],
    [-sy, cy*sx, cx*cy]
    ])
    
    M = np.eye(4)
    M[:3, :3] = R
    M[0, 3] = tx
    M[1, 3] = ty
    M[2, 3] = tz
    return M

def matrix_to_transform(M):
    x, y, z = M[0][3], M[1][3], M[2][3]
    
    # Extract Pitch
    # M[2][0] is -sin(pitch)
    sy = np.sqrt(M[0][0]*M[0][0] + M[1][0]*M[1][0])
    singular = sy < 1e-6

    if not singular:
    r = np.arctan2(M[2][1], M[2][2])
    p = np.arctan2(-M[2][0], sy)
    yaw = np.arctan2(M[1][0], M[0][0])
    else:
    # Gimbal Lock case (Pitch +/- 90)
    r = np.arctan2(-M[1][2], M[1][1])
    p = np.arctan2(-M[2][0], sy)
    yaw = 0.0

    return x, y, z, r, p, yaw

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z
