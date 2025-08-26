import cv2
import numpy as np
from numba import jit, prange
import open3d as o3d

@jit(nopython=True, parallel=True)
def reprojection(depth_image, depth_camera_intrinsics, camera_extrinsics, color_camera_intrinsics, depth_image_show=None):
    height = len(depth_image)
    width = len(depth_image[0])
    if depth_image_show is not None:
        image = np.zeros((height, width), np.uint8)
    else:
        image = np.zeros((height, width), np.uint16)
    if(camera_extrinsics[0][3] > 0):
        sign = 1
    else:
        sign = -1
    for i in prange(0, height):
        for j in prange(0, width):
            if sign == 1:
                # Reverse the order of the pixels
                j = width - j - 1
            d = depth_image[i][j]
            if(d == 0):
                continue
            # Convert pixel to 3d point
            x = (j - depth_camera_intrinsics[0][2]) * d / depth_camera_intrinsics[0][0]
            y = (i - depth_camera_intrinsics[1][2]) * d / depth_camera_intrinsics[1][1]
            z = d

            # Move the point to the camera frame
            x1 = camera_extrinsics[0][0] * x + camera_extrinsics[0][1] * y + camera_extrinsics[0][2] * z + camera_extrinsics[0][3]
            y1 = camera_extrinsics[1][0] * x + camera_extrinsics[1][1] * y + camera_extrinsics[1][2] * z + camera_extrinsics[1][3]
            z1 = camera_extrinsics[2][0] * x + camera_extrinsics[2][1] * y + camera_extrinsics[2][2] * z + camera_extrinsics[2][3]

            u = color_camera_intrinsics[0][0] * (x1  / z1) + color_camera_intrinsics[0][2]
            v = color_camera_intrinsics[1][1] * (y1  / z1) + color_camera_intrinsics[1][2]
            int_u = round(u)
            int_v = round(v)
            if(int_v != i):
                print(f'v -> {v} and i -> {i}') # This should never be printed
            if int_u >= 0 and int_u < (len(image[0]) - 1) and int_v >= 0 and int_v < len(image):
                if depth_image_show is not None:
                    image[int_v][int_u] = depth_image_show[i][j][0]
                    image[int_v][int_u + sign] = depth_image_show[i][j][0]
                else:
                    image[int_v][int_u] = z1
                    image[int_v][int_u + sign] = z1
    return image

def getAlignedDepth(frameDepth, datas, depthSize, size):
    M1, D1, M2, D2, T, R, TARGET_MATRIX, _ = datas
    R1, R2, _, _, _, _, _ = cv2.stereoRectify(M1, D1, M2, D2, (100, 100), R, T)  # The (100,100) doesn't matter as it is not used for calculating the rotation matrices
    leftMapX, leftMapY = cv2.initUndistortRectifyMap(M1, None, R1, TARGET_MATRIX, depthSize, cv2.CV_32FC1)
    depthRect = cv2.remap(frameDepth, leftMapX, leftMapY, cv2.INTER_NEAREST)
    newR = np.dot(R2, np.dot(R, R1.T))  # Should be very close to identity
    newT = np.dot(R2, T)
    combinedExtrinsics = np.eye(4)
    combinedExtrinsics[0:3, 0:3] = newR
    combinedExtrinsics[0:3, 3] = newT
    depthAligned = reprojection(depthRect, TARGET_MATRIX, combinedExtrinsics, TARGET_MATRIX)
    # Rotate the depth to the RGB frame
    R_back = R2.T
    mapX, mapY = cv2.initUndistortRectifyMap(TARGET_MATRIX, None, R_back, M2, size, cv2.CV_32FC1)
    outputAligned = cv2.remap(depthAligned, mapX, mapY, cv2.INTER_NEAREST)
    return outputAligned


def rgbd_to_projection(aligned_depth_map, rgb, intrinsic_matrix,
                       width, height, downsample = False, remove_noise = False):
    rgb_o3d = o3d.geometry.Image(rgb)
    depth_o3d = o3d.geometry.Image(aligned_depth_map)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgb_o3d, depth_o3d, convert_rgb_to_intensity=(len(rgb.shape) != 3), depth_trunc=20000, depth_scale=1000.0
    )

    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(width,
                                                                      height,
                                                                      intrinsic_matrix[0][0],
                                                                      intrinsic_matrix[1][1],
                                                                      intrinsic_matrix[0][2],
                                                                      intrinsic_matrix[1][2])

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)

    if downsample: pcd = pcd.voxel_down_sample(voxel_size=0.01)
    if remove_noise: pcd = pcd.remove_statistical_outlier(30, 0.1)[0]

    return pcd.points, pcd.colors
