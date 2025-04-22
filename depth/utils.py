from collections.abc import Iterable
import functools
import itertools
import cv2
from numba import jit, prange
import numpy as np


@jit(nopython=True, parallel=True)
def reprojection(depth_image, source_intrinsics, extrinsics, target_intrinsics):
    """
    Transform a depth map from a source camera coordinate frame to a target camera coordinate frame
    by projecting the depth to 3D space and projecting back to target camera.
    """
    height, width = depth_image.shape
    image = np.zeros((height, width), np.uint16)
    if (extrinsics[0][3] > 0):
        sign = 1
    else:
        sign = -1
    for i in prange(0, height):
        for j in prange(0, width):
            if sign == 1:
                # Reverse the order of the pixels
                j = width - j - 1
            d = depth_image[i][j]
            if (d == 0):
                continue
            # Convert pixel to 3d point
            x = (j - source_intrinsics[0][2]) * d / source_intrinsics[0][0]
            y = (i - source_intrinsics[1][2]) * d / source_intrinsics[1][1]
            z = d

            # Move the point to the target camera frame
            x1 = extrinsics[0][0] * x + extrinsics[0][1] * y + extrinsics[0][2] * z + extrinsics[0][3]
            y1 = extrinsics[1][0] * x + extrinsics[1][1] * y + extrinsics[1][2] * z + extrinsics[1][3]
            z1 = extrinsics[2][0] * x + extrinsics[2][1] * y + extrinsics[2][2] * z + extrinsics[2][3]

            u = target_intrinsics[0][0] * (x1 / z1) + target_intrinsics[0][2]
            v = target_intrinsics[1][1] * (y1 / z1) + target_intrinsics[1][2]
            int_u = round(u)
            int_v = round(v)

            if int_u >= 0 and int_u < (len(image[0]) - 1) and int_v >= 0 and int_v < len(image):
                image[int_v][int_u] = z1
                image[int_v][int_u + sign] = z1
    return image


def align_depth(depth, M1, d1, M2, d2, T, R, dst_size):
    """Align depth from source to target camera.

    Args:
     - depth: depth map
     - M1, d1: source camera intrinsics and distortion coefficients
     - M2, d2: target camera ...
        - d1 and/or d2 can be None if the source or target are undistorted
     - T, R: extrinsics calibration from source to target camera
     - dst_size: size (w, h) of the target camera image
    """
    R1, R2, *_ = cv2.stereoRectify(M1, d1, M2, d2, depth.shape[::-1], R, T)
    map_x, map_y = cv2.initUndistortRectifyMap(M1, None, R1, M1, depth.shape[::-1], cv2.CV_32FC1)
    rectified_depth = cv2.remap(depth, map_x, map_y, cv2.INTER_NEAREST)
    combinedExtrinsics = np.eye(4)
    combinedExtrinsics[0:3, 0:3] = R2 @ R @ R1.T
    combinedExtrinsics[0:3, 3] = R2 @ T
    aligned_depth = reprojection(rectified_depth, M1, combinedExtrinsics, M1)
    map_x, map_y = cv2.initUndistortRectifyMap(M1, None, R2.T, M2, dst_size, cv2.CV_32FC1)
    return cv2.remap(aligned_depth, map_x, map_y, cv2.INTER_NEAREST)


def colorize_depth(depth, *, min_depth=500, max_depth=15000, colormap=cv2.COLORMAP_JET):
    """
    Convert depth image into an RGB image with the depth visualized with different colors.
    Args:
        depth     - depth frame
        min_depth - [mm]
        max_depth - [mm]
        colormap  - opencv colormap to use
    """
    # Log the depth, min_depth and max_depth
    log_depth = np.log(depth, where=depth != 0)
    log_min_depth = np.log(min_depth)
    log_max_depth = np.log(max_depth)
    depth_color = np.interp(log_depth, (log_min_depth, log_max_depth), (0, 255)).astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_color, colormap)
    # Set invalid depth pixels to black
    depth_color[depth == 0] = 0
    return depth_color


def rgetattr(obj, attr, *args):
    """Recursive getattr"""

    def _getattr(obj, attr):
        return getattr(obj, attr, *args)

    return functools.reduce(_getattr, [obj] + attr.split('.'))


def rsetattr(obj, attr, val):
    """Recursive setattr"""
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)


def str2value(v):
    """Convert string to actual value. Simple version of builtin eval."""
    if v == "True":
        v = True
    elif v == "False":
        v = False
    else:
        try:
            v = int(v)
        except (ValueError, TypeError):
            pass
    return v


# Added to itertools in python 3.12
def batched(iterable, n):
    "Batch data into lists of length n. The last batch may be shorter."
    # batched('ABCDEFG', 3) --> ABC DEF G
    it = iter(iterable)
    while True:
        batch = tuple(itertools.islice(it, n))
        if not batch:
            return
        yield batch


def cv2_resize(src, dsize, *, interpolation=cv2.INTER_LINEAR):
    """
    Resize src of NxHxW shape to Nx(shape) using opencv2 resize, which is limited to 512 channels.
    Args:
        - src: HxWxC image
        - dsize: width x height
    """
    N = src.shape[-1]
    if N < 512:
        dst = cv2.resize(src, dsize=dsize, interpolation=interpolation)
        dst = dst.reshape(*dst.shape[:2], N)
        return dst

    dst = np.empty((dsize[1], dsize[0], src.shape[-1]), dtype=src.dtype)
    for f, *_, l in batched(range(N), 512):
        dst[..., f:l + 1] = cv2.resize(src[..., f:l + 1], dsize=dsize, interpolation=interpolation)
    return dst