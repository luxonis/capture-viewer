import numpy as np
import cv2
import open3d as o3d
import tkinter as tk
from tkinter import ttk
import os
import json
import glob
from datetime import datetime
from PIL import Image, ImageTk
import subprocess
import time

from capture_viewer_tools.capture_tools import (colorize_depth, calculate_scaled_dimensions,
                                                format_json_for_replay, create_placeholder_frame)
from capture_viewer_tools.convertor_capture2replay_json import config2settings
from capture_viewer_tools.ReplaySettings import open_replay_settings_screen
from capture_viewer_tools.pointcloud import rotate_pointcloud

from depth.replay_depth import replay

class ReplayVisualizer:
    def __init__(self, root, view_info, current_view):
        self.window = tk.Toplevel(root)
        self.window.title("Depth Visualization")
        self.window.geometry("2200x1200")

        max_image_width = 640
        max_image_height = 400
        self.scaled_original_size = calculate_scaled_dimensions(view_info['depth_size'], max_image_width, max_image_height)
        # self.scaled_original_size = view_info['depth_size']

        self.view_info = view_info
        self.current_view = current_view

        self.depth = current_view['depth']
        self.generated_depth = None
        self.pcl_path = None
        self.config_json = None

        self.settings_received = False

        self.output_dir = view_info["capture_folder"] + "/replay_outputs"
        if not os.path.exists(self.output_dir): os.makedirs(self.output_dir)

        self.output_folder = None
        self.already_generated = False

        self.image_labels = {
            'generated_img_label': tk.Label(),
            'original_img_label': tk.Label(),
            'difference_img_label' : tk.Label(),
            'generated_json_label' : tk.Label(),
            'original_json_label' : tk.Label(),
        }

    def create_layout(self):
        def show_pointcloud():
            if self.pcl_path is None: return False
            script_directory = os.path.dirname(os.path.abspath(__file__))
            visualize_pointcloud_path = os.path.join(script_directory, 'visualize_pointcloud.py')
            subprocess.Popen(['python', visualize_pointcloud_path, self.pcl_path, str(self.config_json)])  # o3d.visualization.draw_geometries([pointcloud])
            time.sleep(1)
        original_img_label = tk.Label(self.window)
        original_img_label.grid(row=1, column=0, padx=10, pady=10)
        generated_img_label = tk.Label(self.window)
        generated_img_label.grid(row=1, column=1, padx=10, pady=10)
        difference_img_label = tk.Label(self.window)
        difference_img_label.grid(row=1, column=3, padx=10, pady=10)

        # JSON data for each depth image
        original_settings = self.view_info["metadata"]["settings"]
        generated_settings = self.view_info["metadata"].get("config2settings", {})
        original_json_str = json.dumps(original_settings, indent=4)
        generated_json_str = json.dumps(generated_settings, indent=4)

        # Display Original Settings
        original_json_label = tk.Label(
            self.window,
            text=original_json_str,
            bg="white",
            fg="black",
            font=("Courier", 8),
            justify="left",
            anchor="nw"
        )
        original_json_label.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")

        # Generated Settings Label (dynamically updated)
        generated_json_label = tk.Label(
            self.window,
            text=generated_json_str,
            bg="white",
            fg="black",
            font=("Courier", 8),
            justify="left",
            anchor="nw"
        )
        generated_json_label.grid(row=2, column=1, padx=10, pady=10, sticky="nsew")

        # Assign the wrapper function to the button command
        settings_button = tk.Button(self.window, text="Settings", command=self.open_settings_get_config)
        settings_button.grid(row=1, column=2)
        # Assign the wrapper function to the button command
        pointcloud_button = tk.Button(self.window, text="Pointcloud", command=show_pointcloud)
        pointcloud_button.grid(row=1, column=2, pady=(50, 0))

        self.image_labels['generated_img_label'] = generated_img_label
        self.image_labels['original_img_label'] = original_img_label
        self.image_labels['difference_img_label'] = difference_img_label
        self.image_labels['original_json_label'] = original_json_label
        self.image_labels['generated_json_label'] = generated_json_label
    def refresh_display(self, label=None):
        print("Refresh display:", label)
        if self.generated_depth is None:
            colorized_generated_depth = create_placeholder_frame(self.scaled_original_size, label)
            colorized_difference = None
        else:
            colorized_generated_depth = colorize_depth(self.generated_depth, type="depth", label=0)
            if self.generated_depth.shape != self.depth.shape:  # decimation filter downsamples, this upsamples
                if self.depth.shape != self.generated_depth.shape:
                    height, width = self.depth.shape
                    self.generated_depth = cv2.resize(self.generated_depth, (width, height),
                                                      interpolation=cv2.INTER_NEAREST)
            depth_difference = np.abs(self.depth - self.generated_depth)
            colorized_difference = colorize_depth(depth_difference, type="difference", label=0)

        # Update images
        # update original depth
        colorized_depth = colorize_depth(self.depth, type="depth", label=0)
        resized_depth = cv2.resize(colorized_depth, self.scaled_original_size,
                                             interpolation=cv2.INTER_AREA)
        im_original = ImageTk.PhotoImage(image=Image.fromarray(resized_depth))
        self.image_labels['original_img_label'].configure(image=im_original)
        self.image_labels['original_img_label'].image = im_original

        # update generated depth
        resized_generated_depth = cv2.resize(colorized_generated_depth, self.scaled_original_size,
                                             interpolation=cv2.INTER_AREA)
        im_generated = ImageTk.PhotoImage(image=Image.fromarray(resized_generated_depth))
        self.image_labels['generated_img_label'].configure(image=im_generated)
        self.image_labels['generated_img_label'].image = im_generated

        if self.generated_depth is not None:
            resized_difference_depth = cv2.resize(colorized_difference, self.scaled_original_size, interpolation=cv2.INTER_AREA)
            im_difference = ImageTk.PhotoImage(image=Image.fromarray(resized_difference_depth))
            self.image_labels['difference_img_label'].configure(image=im_difference)
            self.image_labels['difference_img_label'].image = im_difference

        # Update settings labels
        generated_settings = self.view_info["metadata"].get("config2settings", {})
        generated_json_str = json.dumps(generated_settings, indent=4)
        self.image_labels['generated_json_label'].configure(text=generated_json_str)
    def get_output_folder(self):
        def compare_json(file_path, json_data):
            """
            Compare the content of a JSON file with a given JSON data object.
            This checks both keys and values for equality, element by element.
            """

            if isinstance(json_data, str):
                try:
                    json_data = json.loads(json_data)  # Parse if it's a string
                except json.JSONDecodeError as e:
                    print(f"Error parsing input JSON data: {e}")
                    return False

            with open(file_path, "r") as f: # Open and parse the file as a JSON object
                try:
                    file_json = json.load(f)
                except json.JSONDecodeError as e:
                    print(f"Error parsing file JSON: {e}")
                    return False

            if set(file_json.keys()) != set(json_data.keys()): return False
            for key in file_json:
                if file_json[key] != json_data[key]: return False

            return True
        def check_folder_for_timestamp(dir_path, current_timestamp):
            """
            Check if files with the current timestamp are present in the folder.
            """
            depth_pattern = os.path.join(dir_path, f"depth_{current_timestamp}")
            depth_files = glob.glob(depth_pattern)
            return bool(depth_files)

        self.already_generated = False
        self.output_folder = None
        currect_timestamp = self.view_info['timestamps'][self.view_info['current_index']]
        for dir_path in os.listdir(self.output_dir):
            dir_path = os.path.join(self.output_dir, dir_path)
            if not os.path.isdir(dir_path):
                continue
            if os.path.exists(os.path.join(dir_path, "config.json")):
                if compare_json(os.path.join(dir_path, "config.json"), format_json_for_replay(str(self.config_json))):
                    self.output_folder = dir_path
                    # Check if files with the current timestamp exist in the folder
                    if check_folder_for_timestamp(dir_path, currect_timestamp):
                        self.already_generated = True
                        print(f"PREVIOUSLY GENERATED: {dir_path}")
                        break
                    else:
                        print("Folder exists but timestamp not generated")
                        break
        return self.already_generated, self.output_folder

    def generate_save_depth_replay(self, output_folder=None):
        print("Generating NEW outputs...")
        if output_folder is not None:
            output_folder = output_folder
        else:
            date_time = datetime.now().strftime("%Y%m%d%H%M%S")
            output_folder = os.path.join(self.output_dir, date_time)
            os.makedirs(output_folder, exist_ok=False)

        left = self.current_view["left"]
        right = self.current_view["right"]
        color = self.current_view["isp"]
        config = self.config_json
        calib = self.view_info["calib"]

        # old version which doesnt work for decimation filter
        # replayed = next(
        #     replay(((left, right),), outputs={'depth', 'pcl'}, calib=calib, stereo_config=json.dumps(config)))
        # replayed = next(
        #     replay(((left, right),), outputs={'depth', 'pcl'}, calib=calib, stereo_config=json.dumps(config)))
        # depth = replayed['depth']
        # pcl = replayed['pcl']

        # FIXED - sends two frames and takes the second, works with decimation filter
        replayed = tuple(replay(
            (
                (left, right, color),
                (left, right, color)),
            outputs={'depth', 'pcl'},
            calib=calib,
            stereo_config=json.dumps(config)
        ))
        depth = replayed[1]['depth']
        pcl = replayed[1]['pcl']

        pcl[:, 0] = -pcl[:, 0]  # switch because it goes wrong from the camera
        pcl = rotate_pointcloud(pcl, 180, axis="y")

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcl)

        print(config)
        if color is not None and config['stereo.setDepthAlign'] == "dai.CameraBoardSocket.CAM_A":
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


        self.generated_depth = depth

        # Save depth data
        timestep = self.view_info['timestamps'][self.view_info['current_index']].split('.npy')[0]
        np.save(os.path.join(output_folder, f"depth_{timestep}.npy"), self.generated_depth)
        cv2.imwrite(os.path.join(output_folder, f"depth_{timestep}.png"), self.generated_depth)
        o3d.io.write_point_cloud(os.path.join(output_folder, f"pcl_{timestep}.ply"), pcd)

        with open(os.path.join(output_folder, f'config.json'), 'w') as f:
            json.dump(config, f, indent=4)

        print("GENERATED")
        return output_folder


    def load_or_generate(self):
        self.refresh_display(label="Loading...")
        self.window.update_idletasks()
        current_timestamp = self.view_info['timestamps'][self.view_info['current_index']]
        already_generated, self.output_folder = self.get_output_folder()
        if not already_generated:
            self.output_folder = self.generate_save_depth_replay(output_folder=self.output_folder)
        print(f"LOADING TIMESTEP: {current_timestamp}")
        self.generated_depth = np.load(os.path.join(self.output_folder, f"depth_{current_timestamp}"))
        self.pcl_path = os.path.join(self.output_folder, f"pcl_{current_timestamp.split('.npy')[0]}.ply")
        self.refresh_display(label="Updated")
        self.settings_received = False  # Reset after processing
        self.window.update_idletasks()

    def open_settings_get_config(self):
        def config_add_depthai(config_json):
            """ adding dai. at the beginning of strings so they can be validated as depthai objects in replay"""
            new_config = {}
            for key in config_json.keys():
                config_value = config_json[key]
                try: config_value = int(config_value)
                except Exception: pass
                if type(config_value) == str and config_value[0] != '[': new_config[key] = "dai." + config_value
                else: new_config[key] = config_value
            return new_config

        config_json = {}
        sucess = open_replay_settings_screen(config_json, original_config=self.view_info['metadata']['settings'])  # Call open_popup and capture the returned JSON
        print("\nSelected config_json:", config_json)
        # config_json = str(config_json)
        if not sucess:
            self.settings_received = False
            return False
        self.view_info["metadata"]["config2settings"] = config2settings(format_json_for_replay(config_json),
                                                                   self.view_info["metadata"]["settings"])
        config_json = config_add_depthai(config_json)
        self.config_json = config_json
        self.generated_depth = None
        self.load_or_generate()

if __name__ == '__main__':
    root = tk.Tk()
    root.title("Main Application")
    root.geometry("300x200")

    # Example data for the depth visualization
    metadata_settings = {
        "config2settings": {"example_key": "config_value"},  # Add more example config values as needed
        "settings": {"parameter1": "value1", "parameter2": "value2"}  # Replace with actual settings values
    }

    view_info = {
        'depth': np.random.rand(400, 200),  # Original depth array
        'depth_size': (400, 200),  # Depth image size
        'metadata': metadata_settings,  # Metadata settings included
        'capture_folder': '/mnt/nas/calibration/datasets/20240905_office4/20240905_scene_lady/OAK-D-PRO_20240905143124'
    }
    # generated_depth = np.random.rand(400, 200)  # Generated depth array
    generated_depth = None

    # Button to open the depth visualization pop-up window
    def on_open_popup():
        repVis = ReplayVisualizer(root, view_info, view_info)
        repVis.create_layout()
        repVis.refresh_display(label="Generate Depth")

    open_popup_button = ttk.Button(root, text="Open Depth Visualization", command=on_open_popup)
    open_popup_button.pack(pady=50)

    root.mainloop()