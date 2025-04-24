import numpy as np
import open3d as o3d

def rotate_pointcloud(points, angle_deg, axis="z"):
    """
    Rotates the point cloud by a specified angle around a given axis.
    - points: Nx3 NumPy array of 3D points.
    - angle_deg: Angle in degrees.
    - axis: Axis of rotation ('x', 'y', 'z').
    Returns: Rotated point cloud as an Nx3 NumPy array.
    """
    angle_rad = np.radians(angle_deg)
    if axis == "x":
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(angle_rad), -np.sin(angle_rad)],
            [0, np.sin(angle_rad), np.cos(angle_rad)]
        ])
    elif axis == "y":
        rotation_matrix = np.array([
            [np.cos(angle_rad), 0, np.sin(angle_rad)],
            [0, 1, 0],
            [-np.sin(angle_rad), 0, np.cos(angle_rad)]
        ])
    elif axis == "z":
        rotation_matrix = np.array([
            [np.cos(angle_rad), -np.sin(angle_rad), 0],
            [np.sin(angle_rad), np.cos(angle_rad), 0],
            [0, 0, 1]
        ])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'.")

    return np.dot(points, rotation_matrix.T)