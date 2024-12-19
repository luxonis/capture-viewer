import open3d as o3d
import numpy as np
import os
from tkinter import *
import depthai as dai
from PIL import Image, ImageTk
import cv2
import time
import threading
import subprocess
import json


from capture_viewer_tools.alignment import getAlignedDepth, rgbd_to_projection
from capture_viewer_tools.stereo import stereo_rectify, undistort
from capture_viewer_tools.capture_tools import get_calibration_between_sockets, colorize_depth, device_connected
from capture_viewer_tools.ReplayVisualizer import ReplayVisualizer

def resize_image(image, square_size):
    if image.dtype != np.uint8:
        image = (image / np.max(image) * 255).astype(np.uint8)
    pil_image = Image.fromarray(image)
    if image.shape[0] > image.shape[1]:
        target_size = [square_size[1], int(square_size[0] * image.shape[0] / image.shape[1])]
        if target_size[1] > square_size[0]:
            target_size = [int(square_size[0] * image.shape[1] / image.shape[0]), square_size[0]]
    else:
        target_size = [int(square_size[0] * image.shape[1] / image.shape[0]), square_size[0]]
        if target_size[0] > square_size[1]:
            target_size = [square_size[1], int(square_size[0] * image.shape[0] / image.shape[1])]
    resized_image = pil_image.resize(target_size, Image.Resampling.LANCZOS)
    resized_array = np.array(resized_image)
    return resized_array

def show_blended(rgb_image, depth_aligned2rgb):
    # cv2.imshow("BGR", frameBGR)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # Scaling to fit my small screen
    screen_width = 1920  # Example width of your screen resolution
    screen_height = 1080  # Example height of your screen resolution
    max_width = screen_width - 100  # Margin to make sure it fits
    max_height = screen_height - 100  # Margin to make sure it fits
    scale_factor = min(max_width / rgb_image.shape[1], max_height / rgb_image.shape[0])
    new_width = int(rgb_image.shape[1] * scale_factor)
    new_height = int(rgb_image.shape[0] * scale_factor)

    # colorise depth
    alignedDepth_colored, range_min, range_max = colorize_depth(depth_aligned2rgb, "depth", label=False, color_noise_percent_removal=0.5)

    # Resize images
    frameBGR_resized = cv2.resize(rgb_image, (new_width, new_height))
    alignedDepth_colored_resized = cv2.resize(alignedDepth_colored, (new_width, new_height))

    def update_transparency(x):
        alpha = x / 100  # Slider value from 0 to 100 -> alpha range 0 to 1
        blended = cv2.addWeighted(alignedDepth_colored_resized, alpha, frameBGR_resized, 1 - alpha, 0)
        cv2.imshow('Blended Depth and RGB', blended)

    cv2.namedWindow('Blended Depth and RGB', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Blended Depth and RGB', new_width, new_height)

    # cv2.createTrackbar('Transparency', 'Blended Depth and RGB', 0, 100, update_transparency)
    # cv2.setTrackbarPos('Transparency', 'Blended Depth and RGB', 50)
    # update_transparency(0)

    cv2.createTrackbar('Transparency', 'Blended Depth and RGB', 50, 100, update_transparency)
    update_transparency(50)

    while True:
        key = cv2.waitKey(1) & 0xFF # Check for key press (small delay of 1ms)
        if key == ord('q'): break

        if key == ord('r'):
            return True

        # Break the loop if the window is closed (via 'X' button)
        if cv2.getWindowProperty('Blended Depth and RGB', cv2.WND_PROP_VISIBLE) < 1:
            break

    cv2.destroyAllWindows()


def on_pointcloud_rgb(root, view_info, current_view, downsample=False):
    # Get aligned depth and RGB frame
    frameBGR, alignedDepth = on_alignment(root, view_info, current_view, pointcloud=True)

    intrinsic_matrix = view_info['calibration_parameters'][2]
    depthSize = view_info['depth_size']

    # Convert RGB to the correct format for Open3D
    RGB = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2RGB)

    # Generate the 3D points and their corresponding colors
    points, colors = rgbd_to_projection(alignedDepth, RGB, intrinsic_matrix, *depthSize, downsample=downsample)

    # Create a PointCloud object
    pointcloud = o3d.geometry.PointCloud()
    pointcloud.points = o3d.utility.Vector3dVector(points)
    pointcloud.colors = o3d.utility.Vector3dVector(colors)

    # Save the point cloud to a temporary location
    temporary_pointcloud_path = os.path.join(os.getcwd(), "temp_pointcloud.ply")
    o3d.io.write_point_cloud(temporary_pointcloud_path, pointcloud)

    # Get the directory of the script
    script_directory = os.path.dirname(os.path.abspath(__file__))
    visualize_pointcloud_path = os.path.join(script_directory, 'visualize_pointcloud.py')

    # Run the visualization script with the point cloud and custom JSON info
    subprocess.Popen(['python', visualize_pointcloud_path, temporary_pointcloud_path,
                      json.dumps(view_info["metadata"], indent=4)])

    # Optional sleep to ensure the subprocess starts properly
    time.sleep(1)

def on_alignment(root, view_info, current_view, pointcloud=False):
    """
    camera returns distorted (unrectified) rgb and depth
    the depth can be aligned somewhere - default it's aligned to rectified RIGHT
    it can also be aligned to unrectified RIGHT/LEFT or rectified LEFT
    """
    depthSize = view_info['depth_size']
    stereoAlign_used_in_capture = view_info['stereoAlign_used_in_capture']
    capture_folder = view_info["capture_folder"]

    RIGHT_SOCKET = view_info["RIGHT_SOCKET"]
    if current_view["alignSocket"] == "COLOR":
        SOCKET = view_info["RGB_SOCKET"]
        frame = current_view['isp']
        size = view_info['rgb_size']
    elif current_view["alignSocket"] == "RIGHT":
        SOCKET = view_info["RIGHT_SOCKET"]
        frame = current_view['right']
        frame = np.stack((frame,) * 3, axis=-1)
        size = view_info['mono_size']
    elif current_view["alignSocket"] == "LEFT":
        SOCKET = view_info["LEFT_SOCKET"]
        frame = current_view['left']
        frame = np.stack((frame,) * 3, axis=-1)
        size = view_info['mono_size']
    else:
        print(f'ERROR - ALIGNMENT SOCKET {current_view["alignSocket"]} NOT FOUND')
        return False

    frameDepth = current_view['depth']

    if frameDepth is None or frame is None:
        print("Depth or RGB images not available")
    else:
        calib = dai.CalibrationHandler(f'{capture_folder}/calib.json')
        datas = get_calibration_between_sockets(calib, RIGHT_SOCKET, SOCKET, depthSize, size)
        if RIGHT_SOCKET != SOCKET: alignedDepth = getAlignedDepth(frameDepth, datas, depthSize, size) # alignedDepth is in the shape of rgbSize
        else: alignedDepth = frameDepth

        print("UNDISTORTION")
        frame = undistort(calib, frame, RIGHT_SOCKET, SOCKET, depthSize, size)

        if not pointcloud: show_blended(frame, alignedDepth)
        else: return frame, alignedDepth  # for usage of aligned depth in pointcloud visualiazation

def on_alignment_stereo(root, view_info, current_view, R=False):
    depthSize = view_info['depth_size']
    capture_folder = view_info["capture_folder"]

    RIGHT_SOCKET = view_info["RIGHT_SOCKET"]
    if current_view["alignSocket"] == "COLOR":
        SOCKET = view_info["RGB_SOCKET"]
        frame = current_view['isp']
        size = view_info['rgb_size']
    elif current_view["alignSocket"] == "RIGHT":
        SOCKET = view_info["RIGHT_SOCKET"]
        frame = current_view['right']
        frame = np.stack((frame,) * 3, axis=-1)
        size = view_info['mono_size']
    elif current_view["alignSocket"] == "LEFT":
        SOCKET = view_info["LEFT_SOCKET"]
        frame = current_view['left']
        frame = np.stack((frame,) * 3, axis=-1)
        size = view_info['mono_size']
    else:
        print(f'ERROR - ALIGNMENT SOCKET {current_view["alignSocket"]} NOT FOUND')
        return False

    frameDepth = current_view['depth']

    if frameDepth is None or frame is None:
        print("Depth or RGB images not available")
    else:
        calib = dai.CalibrationHandler(f'{capture_folder}/calib.json')
        datas = get_calibration_between_sockets(calib, RIGHT_SOCKET, SOCKET, depthSize, size)
        if RIGHT_SOCKET != SOCKET: alignedDepth = getAlignedDepth(frameDepth, datas, depthSize, size)
        else: alignedDepth = frameDepth

        print("UNDISTORT + stereo align")
        calib = dai.CalibrationHandler(f'{capture_folder}/calib.json')
        if RIGHT_SOCKET != SOCKET: frame = stereo_rectify(calib, frame, RIGHT_SOCKET, SOCKET, depthSize, size, switch_R=R)
        else: frame = frame

        if show_blended(frame, alignedDepth):  # if you press R when viewing stereo alignment it uses the other R
            R = not R
            print("showing R switch:", R)
            on_alignment_stereo(root, view_info, current_view, R=R)

def on_alignment_ref(root, view_info, current_view):
    depthSize = view_info['depth_size']
    capture_folder = view_info["capture_folder"]
    frameDepth = current_view['depth']

    RIGHT_SOCKET = view_info["RIGHT_SOCKET"]  # todo - depth can be aligned to many places

    if current_view["alignSocket"] == "COLOR":
        SOCKET = view_info["RGB_SOCKET"]
        frame = current_view['isp']
        size = view_info['rgb_size']
    elif current_view["alignSocket"] == "RIGHT":
        SOCKET = view_info["RIGHT_SOCKET"]
        frame = current_view['right']
        frame = np.stack((frame,) * 3, axis=-1)
        size = view_info['mono_size']
    elif current_view["alignSocket"] == "LEFT":
        SOCKET = view_info["LEFT_SOCKET"]
        frame = current_view['left']
        frame = np.stack((frame,) * 3, axis=-1)
        size = view_info['mono_size']
    else:
        print(f'ERROR - ALIGNMENT SOCKET {current_view["alignSocket"]} NOT FOUND')
        return False

    if frameDepth is None or frame is None:
        print("Depth or RGB images not available")
    else:
        calib = dai.CalibrationHandler(f'{capture_folder}/calib.json')
        datas = get_calibration_between_sockets(calib, RIGHT_SOCKET, SOCKET, depthSize, size)
        if RIGHT_SOCKET != SOCKET: alignedDepth = getAlignedDepth(frameDepth, datas, depthSize, size)
        else: alignedDepth = frameDepth

        print("UNDISTORTION")
        frame = undistort(calib, frame, RIGHT_SOCKET, SOCKET, depthSize, size)

        show_blended(frame, alignedDepth)



def display_images(root, canvas, view_info, current_view, min_val=None, max_val=None):
    data = view_info['data']
    types = view_info['types']
    timestamps = view_info['timestamps']
    canvas_height = view_info['canvas_height']
    canvas_width = view_info['canvas_width']
    current_index = view_info["current_index"]
    view_info["max_slider"] = max_val
    view_info["min_slider"] = min_val

    canvas.delete("all")
    timestamp = timestamps[current_index]
    images = data[timestamp]

    if len(types) == 2:
        rows, columns = 1, 2
    elif len(types) == 1:
        rows, columns = 1, 1
    elif len(types) == 4 or len(types) == 3:
        rows, columns = 2, 2
    else:
        rows, columns = 2, int(len(types) / 2) + 1

    image_size = (int(canvas_height / rows), int(canvas_width / columns))

    positions = [(i // columns * image_size[0], (i % columns) * image_size[1]) for i in range(len(types))]

    images_ref = []  # List to store image references
    canvas.image_coords = []  # Store image coordinates and dimensions for hover detection

    for i, key in enumerate(types):
        if key in images:
            image_path = images[key]
            image = np.load(image_path)
            current_view[key] = image.copy()
            if key == 'isp':
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            original_image = image.copy()  # Keep original for depth value extraction
            if len(image.shape) == 2:  # if grayscale, convert to BGR
                image = np.stack((image,)*3, axis=-1)
            if key == 'depth' or key == 'disparity' or key == 'tof' or key == 'neural_disparity' or key == 'disparity_rescaled':
                image, range_min, range_max = colorize_depth(image, key, False, min_val, max_val)
                # image, type, label = True, min_val = None, max_val = None, color_noise_percent_removal = 1

            resized_image = resize_image(image, image_size)
            im = ImageTk.PhotoImage(image=Image.fromarray(resized_image))
            canvas.create_image(positions[i][1], positions[i][0], anchor=NW, image=im)
            images_ref.append(im)  # Append to the list
            canvas.image_coords.append((positions[i][1], positions[i][0], resized_image.shape[1], resized_image.shape[0], original_image, key))  # Append position and dimensions

    canvas.images_ref = images_ref  # Store the list in canvas to keep references
    canvas.create_text(10, 10, anchor=NW, text=f"Timestamp: {timestamp.split('.npy')[0]}", fill="white", font=("Helvetica", 16))

def on_next(root, canvas, view_info, current_view):
    timestamps = view_info["timestamps"]
    view_info["current_index"] += 1
    if view_info["current_index"] >= len(timestamps):
        view_info["current_index"] = len(timestamps) - 1
    display_images(root, canvas, view_info, current_view)

def on_prev(root, canvas, view_info, current_view):
    view_info["current_index"] -= 1
    if view_info["current_index"] < 0:
        view_info["current_index"] = 0
    display_images(root, canvas, view_info, current_view)

def update_markers(canvas, x, y, x_other=None, y_other=None):
    canvas.delete("marker")
    canvas.create_oval(x - 5,
                       y - 5,
                       x + 5,
                       y + 5,
                       outline='black', fill='yellow', tags="marker")
    if x_other is not None and y_other is not None:
        canvas.create_oval(x_other - 5,
                           y_other - 5,
                           x_other + 5,
                           y_other + 5,
                           outline='white', fill='blue', tags="marker")

def on_mouse_move(event, canvas):
    depth_value, disparity_value = None, None
    depth_coords, disparity_coords = None, None
    x_other, y_other = None, None
    key1, key2 = None, None
    for (x, y, w, h, original_image, key) in canvas.image_coords:
        if x <= event.x < x + w and y <= event.y < y + h:
            relative_x = int((event.x - x) * original_image.shape[1] / w) # relative in the image without scaling
            relative_y = int((event.y - y) * original_image.shape[0] / h)
            image_location = x, y

            if key == 'depth' or key == 'tof' or key=='neural_disparity':
                depth_value = original_image[relative_y, relative_x]
                depth_coords = (relative_x, relative_y, x, y, w, h)
                key1 = key
            elif key == 'disparity' or key == 'disparity_rescaled':
                disparity_value = original_image[relative_y, relative_x]
                disparity_coords = (relative_x, relative_y, x, y, w, h)
                key2 = key
    # go through the frames again and calculate the location for the other dot
    if depth_value is not None or disparity_value is not None:
        for (x, y, w, h, original_image, key) in canvas.image_coords:
            # print((x, y, w, h, "original_image", key))  # i want x and y
            if depth_coords is None:
                if key == 'depth' or key == 'tof' or key=='neural_disparity':
                    depth_value = original_image[relative_y, relative_x]
                    depth_coords = (relative_x, relative_y, x, y, w, h)
                    x_other, y_other = event.x - image_location[0] + x, event.y - image_location[1] + y
                    key1 = key
            elif disparity_coords is None:
                if key == 'disparity' or key == 'disparity_rescaled':
                    disparity_value = original_image[relative_y, relative_x]
                    disparity_coords = (relative_x, relative_y, x, y, w, h)
                    x_other, y_other = event.x - image_location[0] + x, event.y - image_location[1] + y
                    key2 = key
    if depth_coords and disparity_coords:
        canvas.tooltip.config(text=f"{key1}: {depth_value}, {key2}: {disparity_value}")
        canvas.tooltip.place(x=event.x + 15, y=event.y + 15)
        # print(event.x, event.y, x_other, y_other)
        update_markers(canvas, event.x, event.y, x_other, y_other)
    elif depth_coords:
        canvas.tooltip.config(text=f"{key1}: {depth_value}")
        canvas.tooltip.place(x=event.x + 15, y=event.y + 15)
        update_markers(canvas, event.x, event.y)
    elif disparity_coords:
        canvas.tooltip.config(text=f"{key2}: {disparity_value}")
        canvas.tooltip.place(x=event.x + 15, y=event.y + 15)
        update_markers(canvas, event.x, event.y)
    else:
        canvas.tooltip.place_forget()
        canvas.delete("marker")


def on_replay(root, view_info, current_view):
    # check if device connected
    if not view_info["device_connected"]:
        if not device_connected(): return False
        else: view_info["device_connected"] = True

    # initialize the app
    repVis = ReplayVisualizer(root, view_info, current_view)
    repVis.create_layout()
    repVis.refresh_display(label="Generate Depth")


class LoadingHandler:
    def __init__(self, root, label):
        self.root = root
        self.loading_window = None
        self.label = label

    def loading_animation(self, loading_label):
        chars = ["|", "/", "-", "\\"]
        idx = 0
        while True:
            if self.loading_window is None or not self.loading_window.winfo_exists():
                break
            loading_label.config(text=f"{self.label} {chars[idx % len(chars)]}")
            idx += 1
            time.sleep(0.1)

    def show_loading(self):
        self.loading_window = Toplevel(self.root)
        self.loading_window.title(self.label)
        self.loading_window.geometry("200x100")
        loading_label = Label(self.loading_window, text=self.label, font=("Arial", 12))
        loading_label.pack(expand=True)

        # Start animation in a separate thread
        threading.Thread(target=self.loading_animation, args=(loading_label,), daemon=True).start()

    def execute_with_loading(self, func, *args):
        self.show_loading()
        # Start a thread to run the long-running function
        threading.Thread(target=self.run_function, args=(func, *args), daemon=True).start()

    def run_function(self, func, *args):
        func(*args)  # Call the actual function
        self.loading_window.destroy()


# Tkinter application setup
if __name__ == "__main__":
    root = Tk()
    root.geometry("400x200")

    loading_handler = LoadingHandler(root, label="Sleeping")

    # Call the loading handler with the long-running task and its parameters
    loading_handler.execute_with_loading(time.sleep, 5)

    root.mainloop()
