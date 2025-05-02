import numpy as np
import depthai as dai
import cv2
import matplotlib.pyplot as plt
import open3d as o3d
from utils.pointcloud import rotate_pointcloud
from tkinter import *
import tkinter as tk

def extract_calibration_values_old(json_data, width=None, height=None):
    depthSize = (1280, 800)
    rgbSize = (1920, 1080)
    sockets = [0, 1, 2]  # a list of possible sockets for aok d
    left_socket = json_data['stereoRectificationData']['leftCameraSocket']
    right_socket = json_data['stereoRectificationData']['rightCameraSocket']
    sockets.remove(left_socket)
    sockets.remove(right_socket)
    rgb_socket = sockets[0]  # removed the known ones and the last remaining is color socket

    print(rgb_socket)

    ALIGN_SOCKET = right_socket
    RGB_SOCKET = rgb_socket

    # Extract the camera data for the ALIGN_SOCKET
    align_camera_data = next((item[1] for item in json_data['cameraData'] if item[0] == ALIGN_SOCKET), None)
    print(align_camera_data)
    # Extract the camera data for the RGB_SOCKET
    rgb_camera_data = next((item[1] for item in json_data['cameraData'] if item[0] == RGB_SOCKET), None)

    if not align_camera_data or not rgb_camera_data:
        raise ValueError("Camera data for required sockets not found.")

    # M1 and D1
    M1 = np.array(align_camera_data['intrinsicMatrix'])
    D1 = np.array(align_camera_data['distortionCoeff'])

    # M2 and D2
    M2 = np.array(rgb_camera_data['intrinsicMatrix'])/2  # so it matches the different resolution -migh need fixing in the future
    M2[-1][-1] = 1
    D2 = np.array(rgb_camera_data['distortionCoeff'])

    # Translation and Rotation from ALIGN_SOCKET to RGB_SOCKET
    align_extrinsics = align_camera_data['extrinsics']
    rgb_extrinsics = rgb_camera_data['extrinsics']

    T_align_to_rgb = np.array([align_extrinsics['translation']['x'],
                               align_extrinsics['translation']['y'],
                               align_extrinsics['translation']['z']])

    T = T_align_to_rgb * 10  # to mm for matching the depth

    R = np.array(align_extrinsics['rotationMatrix'])[0:3, 0:3]

    # TARGET_MATRIX
    TARGET_MATRIX = M1

    # lensPosition
    lensPosition = rgb_camera_data['lensPosition']

    return M1, D1, M2, D2, T, R, TARGET_MATRIX, lensPosition

def extract_calibration_values(calib, width, height, rgb_width, rgb_height, align="right"):
    RGB_SOCKET = dai.CameraBoardSocket.CAM_A
    LEFT_SOCKET = dai.CameraBoardSocket.CAM_B
    RIGHT_SOCKET = dai.CameraBoardSocket.CAM_C
    color_intrinsics = np.array(calib.getCameraIntrinsics(RGB_SOCKET, rgb_width, rgb_height))
    color_distortion = np.array(calib.getDistortionCoefficients(RGB_SOCKET))
    lensPosition = np.array(calib.getLensPosition(RGB_SOCKET))

    if align == "left": ALIGN_SOCKET = LEFT_SOCKET
    elif align == "right": ALIGN_SOCKET = RIGHT_SOCKET
    else: raise ValueError(f"Alignment to {align} not supported, choose left or right.")

    mono_intrinsics = np.array(calib.getCameraIntrinsics(ALIGN_SOCKET, width, height))
    mono_distortion = np.array(calib.getDistortionCoefficients(ALIGN_SOCKET))
    mono2rgb_translation = np.array(calib.getCameraTranslationVector(ALIGN_SOCKET, RGB_SOCKET, False))
    mono2rgb_translation *= 10  # convert to mm
    mono_rotation = np.array(calib.getCameraExtrinsics(ALIGN_SOCKET, RGB_SOCKET, False))[0:3, 0:3]
    TARGET_MATRIX = mono_intrinsics

    # rewriting in the naming convention used throughout the scripts
    M1, D1, M2, D2, T, R, TARGET_MATRIX, lensPosition = (
        mono_intrinsics, mono_distortion, color_intrinsics, color_distortion,
        mono2rgb_translation, mono_rotation, TARGET_MATRIX, lensPosition)

    return M1, D1, M2, D2, T, R, TARGET_MATRIX, lensPosition

def get_calibration_between_sockets(calib, SOCKET1, SOCKET2, size1, size2):
    """
    SOCKET1 is usually LEFT_SOCKET or RIGHT_SOCKET
    SOCKET2 is usually RGB_SOCKET
    """
    M1 = np.array(calib.getCameraIntrinsics(SOCKET1, *size1))
    M2 = np.array(calib.getCameraIntrinsics(SOCKET2, *size2))
    D1 = np.array(calib.getDistortionCoefficients(SOCKET1))
    D2 = np.array(calib.getDistortionCoefficients(SOCKET2))

    try:
        T = np.array(calib.getCameraTranslationVector(SOCKET1, SOCKET2, False))*10 # convert to mm
    except RuntimeError:
        T = np.array([0.0, 0.0, 0.001])
    try:
        R = np.array(calib.getCameraExtrinsics(SOCKET1, SOCKET2, False))[0:3, 0:3]
    except RuntimeError:
        R = np.eye(3)
    lensPosition = np.array(calib.getLensPosition(SOCKET2))

    TARGET_MATRIX = M1

    return M1, D1, M2, D2, T, R, TARGET_MATRIX, lensPosition

def generate_color_scale_with_annotations_v2(range_min, range_max, image_height):
    """Generate a color scale with annotated values and convert to OpenCV image."""
    # Create a gradient for the scale
    print(range_min, range_max)
    gradient = np.linspace(range_min, range_max, image_height)
    gradient = gradient[:, np.newaxis]  # Make it vertical

    # Set up the Matplotlib figure
    fig, ax = plt.subplots(figsize=(1.5, image_height / 100))  # Adjust figure size for the scale
    fig.subplots_adjust(left=0.2, right=0.4, top=0.95, bottom=0.05)  # Adjust to fit the annotations

    # Plot the gradient with annotations
    ax.imshow(gradient, aspect="auto", cmap="jet", extent=(0, 1, range_min, range_max))
    ax.yaxis.tick_right()
    ax.set_xticks([])
    # ax.set_ylabel("Values")
    ax.tick_params(axis="y", labelsize=20)  # Adjust tick font size

    # Remove unnecessary borders
    ax.spines["top"].set_visible(False)
    ax.spines["left"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["bottom"].set_visible(False)

    # Convert the plot to an OpenCV-compatible image
    fig.canvas.draw()
    scale_image = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    scale_image = scale_image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    # scale_image = cv2.cvtColor(scale_image, cv2.COLOR_RGB2BGR)  # Convert RGB to BGR for OpenCV
    plt.close(fig)  # Close the figure to free memory
    return scale_image

def colorize_depth(image, type, label=True, min_val=None, max_val=None, colormap=cv2.COLORMAP_JET, color_noise_percent_removal=1):
    valid_pixels = image[image > 0]
    if valid_pixels.size == 0:
        return np.zeros_like(image, dtype=np.uint8), None, None

    if colormap is None:
        range_min, range_max = np.percentile(image[image > 0], [0, 100 - color_noise_percent_removal])
        normalized_image = np.clip(image, range_min, range_max)
        normalized_image = ((normalized_image - range_min) / (range_max - range_min) * 255).astype(np.uint8)
        return normalized_image, range_min, range_max
    if type == "depth" or type == "tof_depth":
        if (np.any(image > 0)):
            range_min, range_max = np.percentile(image[image > 0], [0, 100-color_noise_percent_removal])
        else:
            range_min, range_max = 0, 0
        if min_val is not None and max_val is not None:
            range_min, range_max = min_val, max_val
        # print("depth min max:", range_min, range_max)
    elif type == "disparity" or type == "neural_disparity" or type == "disparity_rescaled":
        min_disparity, max_disparity = np.percentile(image[image > 0], [0, 100-color_noise_percent_removal])
        if min_val is not None and max_val is not None:
            min_disparity, max_disparity = min_val, max_val

        # Normalize disparity to a fixed range without dynamic adjustments
        normalized = np.clip((image - min_disparity) / (max_disparity - min_disparity), 0, 1)
        scaled_disparity = (normalized * 255).astype(np.uint8)
        colored_image = cv2.applyColorMap(scaled_disparity, colormap)
        range_min, range_max = min_disparity, max_disparity
    elif type == 'difference':
        if np.all(image == 0):
            return image, 0, 0
        else:
            range_min, range_max = np.percentile(image[image > 0], [0, 100 - color_noise_percent_removal])
            if min_val is not None and max_val is not None:
                range_min, range_max = min_val, max_val
    else:
        range_min, range_max = np.percentile(image[image > 0], [0, 100])

    if "disparity" not in type:  # temporary fix
        # Normalize the image to the range [0, 255]
        normalized_image = np.clip(image, range_min, range_max)
        normalized_image = ((normalized_image - range_min) / (range_max - range_min) * 255).astype(np.uint8)

        # Apply the colormap
        colored_image = cv2.applyColorMap(normalized_image, colormap)

        # Create masks for the specific values
        mask_zero = (image == 0)  # Mask for zero values
        mask_below = (image < range_min)  # Mask for values below the lower bound
        mask_above = (image > range_max)  # Mask for values above the upper bound

        colored_image[mask_below] = 255 # white
        colored_image[mask_above] = 255 # white
        colored_image[mask_zero] = 0  # black

        if type == "difference":
            # Generate a color scale
            scale_colored = generate_color_scale_with_annotations_v2(range_min, range_max, colored_image.shape[0])
            # Resize the scale to match the image height
            scale_colored = cv2.resize(scale_colored, (80, colored_image.shape[0]), interpolation=cv2.INTER_AREA)

            colored_image = cv2.hconcat([colored_image, scale_colored])

    # Add text to the image
    if label:
        cv2.putText(colored_image, type, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)

    return colored_image, range_min, range_max

def calculate_scaled_dimensions(original_dimensions, max_width, max_height):
    original_width, original_height = original_dimensions
    scale_factor = min(max_width / original_width, max_height / original_height)
    return int(original_width * scale_factor), int(original_height * scale_factor)

def format_json_for_replay(config_json):
    if type(config_json) == dict:
        return config_json
    config_json = config_json.replace("False", "false")
    config_json = config_json.replace("True", "true")
    config_json = config_json.replace("'", '"')
    return config_json

def create_placeholder_frame(size, text):
    placeholder_image = np.ones((size[1], size[0], 3), dtype=np.uint8) * 255
    font = cv2.FONT_HERSHEY_SIMPLEX
    max_width, max_height = size[0] * 0.9, size[1] * 0.9

    font_scale = 1.0
    thickness = 2
    while True:
        text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
        if text_size[0] <= max_width and text_size[1] <= max_height:
            font_scale += 0.1
        else:
            font_scale -= 0.1
            break

    text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
    text_x = (size[0] - text_size[0]) // 2
    text_y = (size[1] + text_size[1]) // 2

    cv2.putText(placeholder_image, text, (text_x, text_y), font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)
    return placeholder_image

def device_connected():
    print("connecting to device...")
    try:
        print("Not trying to connect!!!!")
        # device = dai.Device()
        # print(f"Device connected: {device.getDeviceName()}")
        return True
    except RuntimeError:
        print("No device connected. Connect a camera to use REPLAY features.")
        return False


def get_min_max_depths(depth1, depth2, color_noise_percent_removal=1):
    if depth1 is None and depth2 is None:
        return None, None

    if depth1 is not None and depth2 is not None:
        if depth1[depth1 > 0].size == 0 and depth2[depth2 > 0].size == 0:
            return 0, 0
        if depth1[depth1 > 0].size == 0 and depth2[depth2 > 0].size != 0:
            range_min2, range_max2 = np.percentile(depth2[depth2 > 0], [0, 100 - color_noise_percent_removal])
            return range_min2, range_max2
        if depth1[depth1 > 0].size != 0 and depth2[depth2 > 0].size == 0:
            range_min1, range_max1 = np.percentile(depth1[depth1 > 0], [0, 100 - color_noise_percent_removal])
            return range_min1, range_max1

        range_min1, range_max1 = np.percentile(depth1[depth1 > 0], [0, 100 - color_noise_percent_removal])
        range_min2, range_max2 = np.percentile(depth2[depth2 > 0], [0, 100 - color_noise_percent_removal])
        depth_range_max = max(range_max1, range_max2)
        depth_range_min = min(range_min1, range_min2)
        return depth_range_min, depth_range_max
    elif depth1 is None and depth2 is not None:
        range_min2, range_max2 = np.percentile(depth2[depth2 > 0], [0, 100 - color_noise_percent_removal])
        return range_min2, range_max2
    elif depth1 is not None and depth2 is None:
        range_min1, range_max1 = np.percentile(depth1[depth1 > 0], [0, 100 - color_noise_percent_removal])
        return range_min1, range_max1
    else:
        raise ValueError("depth1 and depth2")


def process_pointcloud(pcl, depth, color=None, aligned_to_rgb=False):
    pcl[:, 0] = -pcl[:, 0]  # switch because it goes wrong from the camera
    pcl = rotate_pointcloud(pcl, 180, axis="y")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcl)

    if color is not None and aligned_to_rgb:
        # Resize `isp_frame` to match the number of points
        isp_height, isp_width, _ = color.shape
        # Calculate the target dimensions
        target_height = depth.shape[0]
        target_width = depth.shape[1]

        # Resize and reshape the ISP frame
        aligned_isp_frame = cv2.resize(color, (target_width, target_height))
        aligned_isp_frame = cv2.cvtColor(aligned_isp_frame, cv2.COLOR_BGR2RGB)
        aligned_colors = aligned_isp_frame.reshape(-1, 3)

        pcd.colors = o3d.utility.Vector3dVector(aligned_colors / 255.0)  # Normalize to [0, 1]
    return pcd

def create_depth_range_frame(root, label_text, update_function):
    default_min, default_max = 0, 15000

    frame = Frame(root)

    Label(frame, text=label_text + " range").pack()

    min_var = StringVar(value=str(default_min))
    max_var = StringVar(value=str(default_max))

    Entry(frame, textvariable=min_var, width=6).pack(side=LEFT)
    Entry(frame, textvariable=max_var, width=6).pack(side=LEFT)

    Button(
        frame, text=f"Colorize\n{label_text}",
        bg="#FFFF00", activebackground="#FFFF90",
        command=lambda: update_function(int(min_var.get()), int(max_var.get())
        )
    ).pack(side=RIGHT, padx=10)

    return frame

from screeninfo import get_monitors

def get_current_monitor_size(root):
    root.update_idletasks()
    win_x = root.winfo_rootx()
    win_y = root.winfo_rooty()

    for monitor in get_monitors():
        if (monitor.x <= win_x < monitor.x + monitor.width and
            monitor.y <= win_y < monitor.y + monitor.height):
            return monitor.width, monitor.height

    # fallback to primary if not matched
    primary = get_monitors()[0]
    return primary.width, primary.height

