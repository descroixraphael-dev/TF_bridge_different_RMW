"""
Homogeneous-transform helpers shared by both relay processes.

All transforms in this module are 4x4 numpy arrays:

    T = [[ R  | t ]
         [ 0 0 0 1]]

representing "transform from child frame coordinates into parent frame
coordinates" -- i.e. T_parent_child, matching the usual ROS/tf2 convention
(parent_T_child @ point_in_child == point_in_parent).
"""

import numpy as np


# ---------------------------------------------------------------------------
# STATIC OFFSETS -- measure these once for your physical setup and edit here.
# Both default to identity, which is only correct if the marker is mounted
# exactly at ASTRO's base_link origin/orientation, and the Tello's camera is
# exactly at its base_link origin/orientation. Replace with real translation
# (meters) and rotation (as a 3x3 matrix, or build with quaternion_to_matrix)
# once you've measured the actual mounts.
# ---------------------------------------------------------------------------

def _identity_transform():
    return np.eye(4)

def quaternion_to_matrix(x, y, z, w):
    """[qx, qy, qz, qw] -> 3x3 rotation matrix."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    X, Y, Z = x * s, y * s, z * s
    wX, wY, wZ = w * X, w * Y, w * Z
    xX, xY, xZ = x * X, x * Y, x * Z
    yY, yZ = y * Y, y * Z
    zZ = z * Z
    return np.array([
        [1.0 - (yY + zZ), xY - wZ, xZ + wY],
        [xY + wZ, 1.0 - (xX + zZ), yZ - wX],
        [xZ - wY, yZ + wX, 1.0 - (xX + yY)],
    ])


def matrix_to_quaternion(R):
    """
    3x3 rotation matrix -> [qx, qy, qz, qw].

    Shepperd's method (branches on the largest diagonal term), same
    approach used in aruco_detector.py's rotation_matrix_to_quaternion --
    numerically stable for the full rotation range, unlike a small-angle
    Euler approximation.
    """
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return [qx, qy, qz, qw]


# ---------------------------------------------------------------------------
# Homogeneous transform helpers
# ---------------------------------------------------------------------------

def translation_quaternion_to_matrix(t, q):
    """t=[x,y,z], q=[qx,qy,qz,qw] -> 4x4 homogeneous transform."""
    T = np.eye(4)
    T[:3, :3] = quaternion_to_matrix(*q)
    T[:3, 3] = t
    return T


def matrix_to_translation_quaternion(T):
    """4x4 homogeneous transform -> (t=[x,y,z], q=[qx,qy,qz,qw])."""
    t = T[:3, 3].tolist()
    q = matrix_to_quaternion(T[:3, :3])
    return t, q


def invert_transform(T):
    """Inverse of a 4x4 homogeneous transform (uses R^T, not a full inv())."""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv





# T_astro_base_link_marker: marker pose expressed in ASTRO's base_link frame.
ASTRO_BASE_LINK_TO_MARKER =  translation_quaternion_to_matrix(
    t=[0.0, 0.0, 0.0],
    q=[0.0, 0.0, -0.7071068, 0.7071068],  # -90 deg about z
)

# T_tello_base_link_camera: Tello camera pose (ROS/body-axis convention,
# NOT optical convention) expressed in the Tello's own base_link frame.
TELLO_BASE_LINK_TO_CAMERA =_identity_transform()

# Fixed rotation from OpenCV's optical-frame axis convention
# (x-right, y-down, z-forward) to the ROS body-axis convention
# (x-forward, y-left, z-up), for a camera frame at the same physical
# location/orientation. cv2.solvePnP's rvec/tvec (as used in
# aruco_detector.py) are expressed in the optical convention, so any pose
# coming out of /tello/marker_pose needs this applied before it can be
# composed with frames that follow REP-103 (i.e. everything else in tf2).
_OPTICAL_TO_ROS_R = np.array([
    [0.0,  0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])
OPTICAL_TO_ROS = np.eye(4)
OPTICAL_TO_ROS[:3, :3] = _OPTICAL_TO_ROS_R

def ros_transform_to_matrix(transform):
    """geometry_msgs/Transform -> 4x4 homogeneous transform."""
    t = [transform.translation.x, transform.translation.y, transform.translation.z]
    q = [transform.rotation.x, transform.rotation.y,
         transform.rotation.z, transform.rotation.w]
    return translation_quaternion_to_matrix(t, q)


def fill_ros_transform_from_matrix(transform, T):
    """Write a 4x4 homogeneous transform into a geometry_msgs/Transform in place."""
    t, q = matrix_to_translation_quaternion(T)
    transform.translation.x, transform.translation.y, transform.translation.z = t
    transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w = q


def compose_drone_pose_in_map(t_map_astro_base_link, marker_pos, marker_quat):
    """
    Full composition chain: given ASTRO's current map->base_link transform
    (as a 4x4 matrix) and the raw marker detection (position + quaternion,
    still in OpenCV optical-frame convention, exactly as published on
    /tello/marker_pose), return T_map_tello_base_link as a 4x4 matrix.
    """
    T_camera_optical_marker = translation_quaternion_to_matrix(marker_pos, marker_quat)
    T_camera_ros_marker = OPTICAL_TO_ROS @ T_camera_optical_marker
    T_marker_camera_ros = invert_transform(T_camera_ros_marker)

    T_map_tello_base_link = (
        t_map_astro_base_link
        @ ASTRO_BASE_LINK_TO_MARKER
        @ T_marker_camera_ros
        @ invert_transform(TELLO_BASE_LINK_TO_CAMERA)
    )
    return T_map_tello_base_link

