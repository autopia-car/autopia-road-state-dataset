"""Visualization helpers for the ARSD dataset.

Includes LiDAR-to-camera projection using the extrinsic calibration
published in the ``static_transform.launch.py`` file.
"""

import numpy as np
from scipy.spatial.transform import Rotation

# ── Extrinsic calibration (from static_transform.launch.py) ──────────
# static_transform_publisher args: x y z yaw pitch roll parent child
# Convention: intrinsic ZYX Euler angles, transform from parent to child.

_CALIB_LIDAR_TO_CAM = {
    "front": {
        "t": np.array([0.161445, 0.0174674, -0.257808]),
        "rpy_ypr": np.array([0.024, 0.15, 0.01]),
    },
    "left": {
        "t": np.array([0.171841, 0.142724, -0.26274]),
        "rpy_ypr": np.array([0.693539, 0.120547, 0.0217674]),
    },
    "right": {
        "t": np.array([0.142016, -0.131321, -0.241215]),
        "rpy_ypr": np.array([-0.551337, 0.169142, -0.0122094]),
    },
}

# Rotation from ROS camera-link frame to optical frame (z-forward, x-right, y-down)
_T_OPTICAL = np.eye(4)
_T_OPTICAL[:3, :3] = Rotation.from_euler("ZYX", [-np.pi / 2, 0, -np.pi / 2]).as_matrix()


def _build_tf(t, ypr):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("ZYX", ypr).as_matrix()
    T[:3, 3] = t
    return T


def get_lidar_to_camera_tf(side="front"):
    """Return the 4x4 transform from the Rubyplus LiDAR frame to the
    camera optical frame for *side* (front / left / right).

    The launch file defines camera-link pose in the LiDAR frame
    (child-to-parent), so we compose with the optical rotation and
    invert to obtain lidar-to-camera-optical.
    """
    c = _CALIB_LIDAR_TO_CAM[side]
    T_link = _build_tf(c["t"], c["rpy_ypr"])  # camera-link → lidar
    T_cam_to_lidar = T_link @ _T_OPTICAL       # optical → camera-link → lidar
    return np.linalg.inv(T_cam_to_lidar)        # lidar → optical


def project_lidar_to_image(points, K, D, T_lidar_to_cam, width, height):
    """Project LiDAR points onto a camera image.

    Parameters
    ----------
    points : (N, 3+) array – at least [x, y, z] in the LiDAR frame.
    K : (3, 3) camera intrinsic matrix.
    D : distortion coefficients.
    T_lidar_to_cam : (4, 4) transform from LiDAR to camera optical frame.
    width, height : image dimensions in pixels.

    Returns
    -------
    uv : (M, 2) pixel coordinates of visible points.
    depths : (M,) depth in the camera frame.
    indices : (M,) indices into the original *points* array.
    """
    import cv2

    xyz = np.asarray(points[:, :3], dtype=np.float64)
    ones = np.ones((xyz.shape[0], 1), dtype=np.float64)
    pts_cam = (T_lidar_to_cam @ np.hstack([xyz, ones]).T).T[:, :3]

    in_front = pts_cam[:, 2] > 0.5
    pts_cam = pts_cam[in_front]
    idx = np.where(in_front)[0]

    if len(pts_cam) == 0:
        return np.empty((0, 2)), np.empty(0), np.empty(0, dtype=int)

    uv, _ = cv2.projectPoints(
        pts_cam, np.zeros(3), np.zeros(3),
        K.astype(np.float64), D.astype(np.float64),
    )
    uv = uv.reshape(-1, 2)
    depths = pts_cam[:, 2]

    visible = (uv[:, 0] >= 0) & (uv[:, 0] < width) & (uv[:, 1] >= 0) & (uv[:, 1] < height)
    return uv[visible], depths[visible], idx[visible]


def overlay_lidar_on_image(image, points, K, D, T_lidar_to_cam, radius=2,
                           depth_range=(1.0, 40.0)):
    """Return a copy of *image* with LiDAR points drawn as depth-coloured dots."""
    import cv2

    h, w = image.shape[:2]
    uv, depths, _ = project_lidar_to_image(points, K, D, T_lidar_to_cam, w, h)
    if len(uv) == 0:
        return image.copy()

    overlay = image.copy()
    d_norm = np.clip((depths - depth_range[0]) / (depth_range[1] - depth_range[0]), 0, 1)
    colors = cv2.applyColorMap(
        (d_norm * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET
    ).reshape(-1, 3)

    for i in range(len(uv)):
        cv2.circle(overlay, (int(uv[i, 0]), int(uv[i, 1])), radius,
                   tuple(int(v) for v in colors[i]), -1)
    return overlay
