import numpy as np
import cv2
import open3d as o3d
import tkinter as tk
from tkinter import ttk, Tk, filedialog
import threading
import os
import tempfile
import glob
from datetime import datetime
from PIL import Image, ImageTk
import subprocess
import time
import platform

from capture_viewer_tools.capture_tools import (colorize_depth, calculate_scaled_dimensions, get_min_max_depths,
                                                format_json_for_replay, create_placeholder_frame, process_pointcloud)

from capture_viewer_tools.convertor_capture2replay_json import config2settings
from capture_viewer_tools.ReplaySettings import *

from capture_viewer_tools.convertor_capture2replay_json import settings2config, handle_dict, decimation_set_dict, CT_kernel_dict
from capture_viewer_tools.popup_info import show_popup

from capture_viewer_tools.capture_tools import create_depth_range_frame, get_current_monitor_size

from depth.replay_depth import Replay
from depth.stereo_config import StereoConfig

class ReplayVisualizer:
    def __init__(self, root, view_info, current_view):
        self.toplLevel = tk.Toplevel(root)
        self.toplLevel.title("REPLAY")
        self.toplLevel.protocol("WM_DELETE_WINDOW", self.close)

        self.screen_width, self.screen_height = get_current_monitor_size(self.toplLevel)
        print(self.screen_width, self.screen_height)
        self.toplLevel.geometry(f"{self.screen_width}x{self.screen_height}")
        self.toplLevel.resizable(True, True)

        max_image_width = int(self.screen_width / 3) - 100
        max_image_height = int(self.screen_height / 3) - 100
        self.scaled_original_size = calculate_scaled_dimensions(view_info['depth_size'], max_image_width, max_image_height)

        self.view_info = view_info
        self.current_view = current_view

        self.device_info = view_info['device_info']

        print(self.device_info)

        self.last_generated_depth = None
        self.generated_depth1 = None
        self.generated_depth2 = None

        self.last_generated_pcl_path = None
        self.pcl_path1 = None
        self.pcl_path2 = None

        self.generated_depth_image1 = None
        self.generated_depth_image2 = None

        self.fixed_depth_range = False
        self.fixed_depth_range_min = 0
        self.fixed_depth_range_max = 15000

        self.selected_colormap = tk.StringVar(value="JET")

        self.config_json = None
        self.config_json1 = None
        self.config_json2 = None

        self.loaded_config_name1 = tk.StringVar(value="No config loaded")
        self.loaded_config_name2 = tk.StringVar(value="No config loaded")

        self.depth_resolution1 = tk.StringVar(value="")
        self.depth_resolution2 = tk.StringVar(value="")

        self.last_config = None

        self.output_dir = os.path.join(view_info["capture_folder"], "replay_outputs")
        if not os.path.exists(self.output_dir): os.makedirs(self.output_dir)

        self.output_folder = None
        self.depth_in_replay_outputs = False

        self.button_values1 = {"settings_section_number" : 1}  # button_key: value
        self.button_values2 = {"settings_section_number" : 2}  # button_key: value

        self.depth_generate_thread1 = threading.Thread(target=lambda: None)
        self.depth_generate_thread2 = threading.Thread(target=lambda: None)

        self.replayer = Replay(device_info=self.device_info, outputs={'depth', 'pcl'}, stereo_config=StereoConfig({'stereo.setDepthAlign': 'dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT'}))

    def close(self):
        del self.replayer
        self.toplLevel.destroy()

    def get_initial_config(self, original_config):
        if self.last_config is None and original_config is not None:
            current_config = settings2config(original_config)
        elif self.last_config is not None:
            current_config = self.last_config
        else:
            current_config = default_config
        return current_config

    def convert_current_button_values_to_config(self, button_values, frame):
        config = {}
        config['stereo.setDepthAlign'] = button_values['depth_align'].get()
        if button_values['profile_preset'].get() != 'None':
            config['stereo.setDefaultProfilePreset'] = button_values['profile_preset'].get()

        if button_values['custom_settings_val'].get():
            if button_values['filtering_order_enable'].get():  # needs to be the first item for the config to not be initialised in case of wrong filtering order
                if not check_valid_filtering_order(
                        [button_values['decimation_order'].get(), button_values['median_order'].get(), button_values['speckle_order'].get(), button_values['spatial_order'].get(),
                         button_values['temporal_order'].get()]):
                    show_popup("Warning", "FILTERING ORDER IS NOT VALID", frame)
                    return config
                else:
                    config['cfg.postProcessing.filteringOrder'] = get_filter_order(button_values['decimation_order'].get(),
                                                                                   button_values['median_order'].get(),
                                                                                   button_values['speckle_order'].get(),
                                                                                   button_values['spatial_order'].get(),
                                                                                   button_values['temporal_order'].get())

            config['stereo.setRectification'] = button_values['rectificationBox_val'].get()
            config['stereo.setLeftRightCheck'] = button_values['LRBox_val'].get()

            config['stereo.setExtendedDisparity'] = button_values['extendedBox_val'].get()
            config['stereo.setSubpixel'] = button_values['subpixelBox_val'].get()

            if button_values['subpixelBox_val'].get():
                config['stereo.setSubpixelFractionalBits'] = button_values['fractional_bits_val'].get()

            # FILTERS

            if button_values['median_filter_enable'].get():
                config['stereo.initialConfig.setMedianFilter'] = button_values['median_val'].get()
            else:
                config['stereo.initialConfig.setMedianFilter'] = "dai.MedianFilter.MEDIAN_OFF"

            # Bilateral filter
            # todo - [1944301021AA992E00] [1.2.2] [9.439] [StereoDepth(2)] [warning] Bilateral filter is deprecated!
            # config['cfg.postProcessing.bilateralSigmaValue'] = bilateral_sigma_val.get()
            # if bilateral_filter_enable.get():
            #     # is this all settings?
            #     pass

            # Brightness filter
            if button_values['brightness_filter_enable'].get():  # todo can we disable brightness filter?
                config['cfg.postProcessing.brightnessFilter.maxBrightness'] = button_values['max_brightness_slider'].get()
                config['cfg.postProcessing.brightnessFilter.minBrightness'] = button_values['min_brightness_slider'].get()

            # Speckle filter
            config['cfg.postProcessing.speckleFilter.enable'] = button_values['speckle_filter_enable'].get()
            if button_values['speckle_filter_enable'].get():
                config['cfg.postProcessing.speckleFilter.speckleRange'] = button_values['speckle_range_slider'].get()
                config[
                    'cfg.postProcessing.speckleFilter.differenceThreshold'] = button_values['speckle_difference_threshold'].get()

            # Spacial filter
            config['cfg.postProcessing.spatialFilter.enable'] = button_values['spatial_filter_enable'].get()
            if button_values['spatial_filter_enable'].get():
                config['cfg.postProcessing.spatialFilter.holeFillingRadius'] = button_values['hole_filling_radius_slider'].get()
                config['cfg.postProcessing.spatialFilter.numIterations'] = button_values['num_iterations_slider'].get()
                config['cfg.postProcessing.spatialFilter.alpha'] = button_values['alpha_slider'].get()
                config['cfg.postProcessing.spatialFilter.delta'] = button_values['delta_slider'].get()

            # Temporal filter
            config['cfg.postProcessing.temporalFilter.enable'] = button_values['temporal_filter_enable'].get()
            if button_values['temporal_filter_enable'].get():
                config['cfg.postProcessing.temporalFilter.alpha'] = button_values['temporal_alpha_slider'].get()
                config['cfg.postProcessing.temporalFilter.delta'] = button_values['temporal_delta_slider'].get()

            # Threshold filter
            if button_values['threshold_filter_enable'].get():  # todo same as brightness filter
                config['cfg.postProcessing.thresholdFilter.minRange'] = button_values['min_range_val'].get()
                config['cfg.postProcessing.thresholdFilter.maxRange'] = button_values['max_range_val'].get()

            # Decimation filter
            config['cfg.postProcessing.decimationFilter.decimationFactor'] = 1  # by default do not decimate
            if button_values['decimation_filter_enable'].get():
                config['cfg.postProcessing.decimationFilter.decimationFactor'] = button_values['decimation_factor_val'].get()
                config['cfg.postProcessing.decimationFilter.decimationMode'] = handle_dict(
                    button_values['decimation_mode_val'].get(), decimation_set_dict)  # leave this is correct

        if button_values['advanced_settings_enable'].get():
            config["cfg.censusTransform.kernelSize"] = handle_dict(button_values['CT_kernel_val'].get(), CT_kernel_dict)
            config["cfg.censusTransform.enableMeanMode"] = button_values['mean_mode_enable'].get()
            config["cfg.censusTransform.threshold"] = button_values['CT_threshold_val'].get()

            config["cfg.costAggregation.divisionFactor"] = button_values['division_factor_val'].get()
            config["cfg.costAggregation.horizontalPenaltyCostP1"] = button_values['horizontal_penalty_p1_val'].get()
            config["cfg.costAggregation.horizontalPenaltyCostP2"] = button_values['horizontal_penalty_p2_val'].get()
            config["cfg.costAggregation.verticalPenaltyCostP1"] = button_values['vertical_penalty_p1_val'].get()
            config["cfg.costAggregation.verticalPenaltyCostP2"] = button_values['vertical_penalty_p2_val'].get()

            config["cfg.costMatching.confidenceThreshold"] = button_values['confidence_threshold_val'].get()
            config["cfg.costMatching.linearEquationParameters.alpha"] = button_values['CM_alpha_val'].get()
            config["cfg.costMatching.linearEquationParameters.beta"] = button_values['CM_beta_val'].get()
            config["cfg.costMatching.linearEquationParameters.threshold"] = button_values['matching_threshold_val'].get()
            config["cfg.costMatching.enableCompanding"] = button_values['enableCompanding_val'].get()
            config["cfg.algorithmControl.leftRightCheckThreshold"] = button_values['leftRightCheckThreshold_val'].get()

        return config

    def start_checker_thread(self, frame):
        def checker():
            for i in range(5):
                alive1 = self.depth_generate_thread1.is_alive()
                alive2 = self.depth_generate_thread2.is_alive()
                if not alive1 and not alive2:
                    return
                time.sleep(0.5)
            print("Threads still running after 5 checks.")
            show_popup("Warning", "Depth processing thread is already running, please wait for replay on camera to finish.", frame)

        threading.Thread(target=checker, daemon=True).start()

    def generate_replay_depth_threads(self, button_values, frame=None):
        self.start_checker_thread(frame)

        if button_values["settings_section_number"] == 1:
            self.depth_generate_thread1 = threading.Thread(target=self._on_generate, args=(button_values, frame,))
            self.depth_generate_thread1.start()
        elif button_values["settings_section_number"] == 2:
            self.depth_generate_thread2 = threading.Thread(target=self._on_generate, args=(button_values, frame,))
            self.depth_generate_thread2.start()
    def _on_generate(self, button_values, frame=None):
        settings_section_number = button_values['settings_section_number']
        print(f"GENERATE THREAD: {settings_section_number}")

        self.last_config = self.config_json
        self.config_json = self.convert_current_button_values_to_config(button_values, frame)

        print(self.config_json)

        # self.config_json = add_depthai_to_config(self.config_json)

        self.last_generated_depth = None

        if settings_section_number == 1:
            self.config_json1 = self.config_json
            self.generated_depth1 = None
            self.pcl_path1 = None
        elif settings_section_number == 2:
            self.config_json2 = self.config_json
            self.generated_depth2 = None
            self.pcl_path2 = None


        self.refresh_display(label="Loading...")
        self.main_frame.update_idletasks()

        self.replay_generate_one_frame()

        if settings_section_number == 1:
            self.generated_depth1 = self.last_generated_depth
            self.pcl_path1 = self.last_generated_pcl_path
        elif settings_section_number == 2:
            self.generated_depth2 = self.last_generated_depth
            self.pcl_path2 = self.last_generated_pcl_path
        
        self.depth_range_min, self.depth_range_max = get_min_max_depths(self.generated_depth1, self.generated_depth2)

        self.refresh_display(label="Updated")
        self.main_frame.update_idletasks()
        print(f"Thread {settings_section_number} finished")

    def on_mouse_wheel(self, event):
        # print(event.widget)
        if event.delta > 0:
            self.on_mouse_wheel_up(event)
        elif event.delta < 0:
            self.on_mouse_wheel_down(event)
    def on_mouse_wheel_up(self, event):
        if "settings_canvas1" in str(event.widget):
            self.settings_canvas1.yview_scroll(-1, "units")
        elif "settings_canvas2" in str(event.widget):
            self.settings_canvas2.yview_scroll(-1, "units")
        elif "config_frame1" in str(event.widget):
            self.config_canvas1.yview_scroll(-1, "units")
        elif "config_frame2" in str(event.widget):
            self.config_canvas2.yview_scroll(-1, "units")
    def on_mouse_wheel_down(self, event):
        if "settings_canvas1" in str(event.widget):
            self.settings_canvas1.yview_scroll(1, "units")
        elif "settings_canvas2" in str(event.widget):
            self.settings_canvas2.yview_scroll(1, "units")
        elif "config_frame1" in str(event.widget):
            self.config_canvas1.yview_scroll(1, "units")
        elif "config_frame2" in str(event.widget):
            self.config_canvas2.yview_scroll(1, "units")

    def bind_scrolling(self):
        # --- BINDING FOR SCROLLING
        if platform.system() == "Linux":
            self.settings_frame_custom1.bind("<Button-4>", self.on_mouse_wheel_up)
            self.settings_frame_custom1.bind("<Button-5>", self.on_mouse_wheel_down)

            self.settings_frame_custom2.bind("<Button-4>", self.on_mouse_wheel_up)
            self.settings_frame_custom2.bind("<Button-5>", self.on_mouse_wheel_down)

            self.config_frame1.bind("<Button-4>", self.on_mouse_wheel_up)
            self.config_frame1.bind("<Button-5>", self.on_mouse_wheel_down)

            self.config_frame2.bind("<Button-4>", self.on_mouse_wheel_up)
            self.config_frame2.bind("<Button-5>", self.on_mouse_wheel_down)

            def bind_recursively(frame):
                for widget in frame.winfo_children():
                    if isinstance(widget, ttk.Spinbox) or isinstance(widget, ttk.Combobox):
                        widget.bind("<Button-4>", 'break')
                        widget.bind("<Button-5>", 'break')
                        continue
                    widget.bind("<Button-4>", self.on_mouse_wheel_up)
                    widget.bind("<Button-5>", self.on_mouse_wheel_down)
                    bind_recursively(widget)

            bind_recursively(self.settings_frame_custom1)
            bind_recursively(self.settings_frame_custom2)
            bind_recursively(self.config_frame1)
            bind_recursively(self.config_frame2)

        else:
            self.settings_canvas1.bind_all("<MouseWheel>", self.on_mouse_wheel)
            self.settings_canvas2.bind_all("<MouseWheel>", self.on_mouse_wheel)

        self.settings_canvas1.focus_set()
        self.settings_canvas2.focus_set()

    def on_pointcloud_button(self, pcl_path, config_json):
        if pcl_path is None: return False
        script_directory = os.path.dirname(os.path.abspath(__file__))
        visualize_pointcloud_path = os.path.join(script_directory, 'visualize_pointcloud.py')
        subprocess.Popen(['python', visualize_pointcloud_path, pcl_path, str(config_json)])  # o3d.visualization.draw_geometries([pointcloud])
        time.sleep(1)

    def create_depth_section(self, column_in_main_frame):
        generated_depth_frame = tk.Frame(self.main_frame)
        generated_depth_frame.grid(row=1, column=column_in_main_frame, padx=5, pady=5, sticky='nsew')

        generated_depth_text_label1 = tk.Label(generated_depth_frame, text="Generated Depth", font=("Arial", 12, "bold"))
        generated_depth_text_label1.grid(row=0, column=0, padx=10, pady=(10, 0), sticky="nsew")

        generated_depth_image = tk.Label(generated_depth_frame, image=None)
        generated_depth_image.image = None
        generated_depth_image.grid(row=1, column=0, padx=10, pady=10)

        button_frame = tk.Frame(generated_depth_frame)
        button_frame.grid(row=1, column=1, padx=5, pady=5, sticky='nsew')

        if column_in_main_frame == 0:
            pointcloud_button = tk.Button(button_frame, text="Point Cloud", command=lambda: self.on_pointcloud_button(self.pcl_path1, self.config_json1))
            depth_resolution = tk.Label(button_frame, textvariable=self.depth_resolution1, font=("Helvetica", 12, "bold"))
        else:
            pointcloud_button = tk.Button(button_frame, text="Point Cloud", command=lambda: self.on_pointcloud_button(self.pcl_path2, self.config_json2))
            depth_resolution = tk.Label(button_frame, textvariable=self.depth_resolution2, font=("Helvetica", 12, "bold"))

        depth_text = tk.Label(button_frame, text="Resolution:", font=("Helvetica", 12, "bold"))
        pointcloud_button.grid(row=2, column=0, sticky='ew')
        depth_text.grid(row=0, column=0, sticky='w')
        depth_resolution.grid(row=0, column=1, sticky='w')

        button_frame.rowconfigure((0, 3), weight=1)
        button_frame.columnconfigure(2, weight=1)


        return generated_depth_frame, generated_depth_image
    def create_settings_section(self, collumn_in_main_frame, frame_name, button_values):
        settings_frame_custom = tk.Frame(self.main_frame, name=frame_name)
        settings_frame_custom.grid(row=2, column=collumn_in_main_frame, sticky="nsew")

        settings_frame_custom.grid_rowconfigure(0, weight=1, minsize=self.screen_height/2)
        settings_frame_custom.grid_columnconfigure(0, weight=1, minsize=self.screen_width/3)

        settings_canvas = tk.Canvas(settings_frame_custom)
        settings_canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar = tk.Scrollbar(settings_frame_custom, orient="vertical", command=settings_canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")

        scrollbar_h = tk.Scrollbar(settings_frame_custom, orient="horizontal", command=settings_canvas.xview)
        scrollbar_h.grid(row=1, column=0, sticky="ew")

        settings_canvas.configure(yscrollcommand=scrollbar.set, xscrollcommand=scrollbar_h.set)

        content_frame = tk.Frame(settings_canvas)
        initial_config = self.get_initial_config(original_config=None)
        inicialize_button_values(initial_config, button_values)
        def fallback_generate_function(*args):
            self.generate_replay_depth_threads(button_values, frame=settings_frame_custom)

        add_trace_to_button_values(button_values, fallback_generate_function)
        create_settings_layout(content_frame, button_values)

        settings_canvas.create_window((0, 0), window=content_frame, anchor="nw")
        content_frame.update_idletasks()

        settings_canvas.config(scrollregion=settings_canvas.bbox("all"))

        content_frame2 = tk.Frame(settings_frame_custom)
        content_frame2.grid(row=2, column=0, sticky="n")

        return settings_frame_custom, settings_canvas
    def create_config_section(self, collumn_in_main_frame, config_frame_name):
        config_frame = tk.Frame(self.main_frame, name=config_frame_name)
        config_frame.grid(row=3, column=collumn_in_main_frame, sticky='nsew')

        config_frame.grid_rowconfigure(0, weight=1)
        config_frame.grid_columnconfigure(0, weight=1, minsize=self.screen_width/3)

        height = int(self.screen_height/7)
        width = int(self.screen_width/3)

        config_canvas = tk.Canvas(config_frame, height=height, width=width)
        config_canvas.grid(row=0, column=0, sticky="nsew")

        v_scrollbar = tk.Scrollbar(config_frame, orient="vertical", command=config_canvas.yview)
        v_scrollbar.grid(row=0, column=1, sticky="ns")

        config_canvas.configure(yscrollcommand=v_scrollbar.set)

        # Create a content frame inside the canvas
        config_json_frame = tk.Frame(config_canvas)
        config_window = config_canvas.create_window((0, 0), window=config_json_frame, anchor="nw")

        # JSON data for each depth image (example)
        generated_settings = self.view_info["metadata"].get("config2settings", {})
        generated_json_str = json.dumps(generated_settings, indent=4)

        # Create a Text widget to display the JSON data
        generated_json_text = tk.Text(config_json_frame,
                                      bg="white",
                                      fg="black",
                                      font=("Courier", 12),
                                      wrap="word") # fix width so it matches
        generated_json_text.insert(tk.END, generated_json_str)
        generated_json_text.config(state=tk.DISABLED)  # Disable editing
        generated_json_text.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Ensure wrapping applies when resizing
        config_json_frame.grid_columnconfigure(0, weight=1)
        config_json_frame.grid_rowconfigure(0, weight=1)

        # Update scroll region dynamically
        def update_scroll_region(event):
            config_canvas.config(scrollregion=config_canvas.bbox("all"))

        config_json_frame.bind("<Configure>", update_scroll_region)

        return generated_json_text, config_frame, config_canvas
    def create_difference_section(self, collumn_in_main_frame):
        difference_frame = tk.Frame(self.main_frame)
        difference_frame.grid(row=1, column=collumn_in_main_frame, padx=5, pady=5, sticky='nsew')

        difference_depth_text_label = tk.Label(difference_frame, text="Absolute Difference", font=("Arial", 12, "bold"))
        difference_depth_text_label.grid(row=0, column=0, padx=10, pady=(10, 0), sticky="nsew")

        difference_image = tk.Label(difference_frame, image=None)
        difference_image.image = None
        difference_image.grid(row=1, column=0, padx=10, pady=10)

        return difference_frame, difference_image

    def update_depth_range(self, min, max):
        self.fixed_depth_range_max = max
        self.fixed_depth_range_min = min

        self.fixed_depth_range = True

        self.refresh_generated_depth_or_placeholder(self.generated_depth1, self.generated_depth_image1, "Recolor")
        self.refresh_generated_depth_or_placeholder(self.generated_depth2, self.generated_depth_image2, "Recolor")
    def update_resolution(self):
        if self.generated_depth1 is not None:
            self.depth_resolution1.set(f"{self.generated_depth1.shape}")
        if self.generated_depth2 is not None:
            self.depth_resolution2.set(f"{self.generated_depth2.shape}")

    def create_edits_section(self, collumn_in_main_frame):
        edits_frame = tk.Frame(self.main_frame)
        edits_frame.grid(row=2, column=collumn_in_main_frame, padx=5, pady=5, sticky='nsew')
        depth_frame = create_depth_range_frame(edits_frame, 'Depth', self.update_depth_range)
        depth_frame.grid(row=0, column=0, sticky='ew', padx=10)

        def on_colormap_change(event):
            self.refresh_generated_depth_or_placeholder(self.generated_depth1, self.generated_depth_image1, "Colormap Change")
            self.refresh_generated_depth_or_placeholder(self.generated_depth2, self.generated_depth_image2, "Colormap Change")
            self.refresh_difference_or_placeholder(self.generated_depth1, self.generated_depth2, self.difference_image)

        options = ["JET", "DEEPGREEN", "GREYSCALE"]
        colormap_selection_dropdown = ttk.Combobox(edits_frame, textvariable=self.selected_colormap, values=options, state="readonly")
        colormap_selection_dropdown.grid(row=1, column=0, sticky="nsew")

        colormap_selection_dropdown.bind("<<ComboboxSelected>>", on_colormap_change)

        return edits_frame

    def create_layout(self):
        self.main_frame = tk.Frame(self.toplLevel)
        self.main_frame.grid(row=0, column=0, sticky="nsew")

        # depth 1 and depth 2
        self.generated_depth_frame1, self.generated_depth_image1 = self.create_depth_section(0)
        self.generated_depth_frame2, self.generated_depth_image2 = self.create_depth_section(1)

        self.difference_frame, self.difference_image = self.create_difference_section(2)

        # SETTINGS
        self.settings_frame_custom1, self.settings_canvas1 = self.create_settings_section(0, "settings_canvas1", self.button_values1)
        self.settings_frame_custom2, self.settings_canvas2 = self.create_settings_section(1, "settings_canvas2", self.button_values2)

        # CONFIG
        self.generated_json_text1, self.config_frame1, self.config_canvas1 = self.create_config_section(0, "config_frame1")
        self.generated_json_text2, self.config_frame2, self.config_canvas2 = self.create_config_section(1, "config_frame2")

        self.edits_frame = self.create_edits_section(2)

        self.bind_scrolling()

    def initialize_depth(self):
        self.toplLevel.after(100, lambda: self.generate_replay_depth_threads(self.button_values1))
        self.toplLevel.after(200, lambda: self.generate_replay_depth_threads(self.button_values2))

    def get_colormap(self):
        if self.selected_colormap.get() == "GREYSCALE":
            return None
        colormap_name = f"COLORMAP_{self.selected_colormap.get()}"
        colormap = getattr(cv2, colormap_name)
        return colormap

    def refresh_generated_depth_or_placeholder(self, generated_depth, generated_depth_image, label):
        if generated_depth is None:
            resized_generated_depth = create_placeholder_frame(self.scaled_original_size, label)
        else:
            if self.fixed_depth_range:
                colorized_generated_depth, _, _ = colorize_depth(generated_depth, min_val=self.fixed_depth_range_min, max_val=self.fixed_depth_range_max, colormap=self.get_colormap(), type="depth", label=0)
            else:
                colorized_generated_depth, _, _ = colorize_depth(generated_depth, min_val=self.depth_range_min, max_val=self.depth_range_max, colormap=self.get_colormap(), type="depth", label=0)
            # update generated depth
            resized_generated_depth = cv2.resize(colorized_generated_depth, self.scaled_original_size, interpolation=cv2.INTER_AREA)

        tk_image = ImageTk.PhotoImage(image=Image.fromarray(resized_generated_depth))
        generated_depth_image.configure(image=tk_image)
        generated_depth_image.image = tk_image
    def refresh_difference_or_placeholder(self, depth1, depth2, difference_image):
        label = "Absolute Difference"
        if depth1 is None or depth2 is None: difference = None
        elif depth1.shape != depth2.shape:
            label = "Depth maps have incompatible shapes"
            difference = None
        else: difference = np.abs(depth1 - depth2)

        if difference is None:
            colorized_difference = create_placeholder_frame(self.scaled_original_size, label)
        else:
            colorized_difference, _, _ = colorize_depth(difference, min_val=self.depth_range_min, max_val=self.depth_range_max, colormap=self.get_colormap(), type="depth", label=0)

        resized_difference = cv2.resize(colorized_difference, self.scaled_original_size, interpolation=cv2.INTER_AREA)

        tk_image = ImageTk.PhotoImage(image=Image.fromarray(resized_difference))

        difference_image.configure(image=tk_image)
        difference_image.image = tk_image
    def refresh_json_text(self, generated_json_text, config_json):
        updated_json_str = json.dumps(config_json, indent=4)
        generated_json_text.config(state=tk.NORMAL)  # Enable editing temporarily
        generated_json_text.delete('1.0', tk.END)  # Clear the existing text
        generated_json_text.insert(tk.END, updated_json_str)  # Insert updated JSON
        generated_json_text.config(state=tk.DISABLED)  # Disable editing again

    def refresh_display(self, label=None):
        print("Refresh display:", label)

        self.refresh_generated_depth_or_placeholder(self.generated_depth1, self.generated_depth_image1, label)
        self.refresh_generated_depth_or_placeholder(self.generated_depth2, self.generated_depth_image2, label)

        self.refresh_difference_or_placeholder(self.generated_depth1, self.generated_depth2, self.difference_image)

        self.refresh_json_text(self.generated_json_text1, self.config_json1)
        self.refresh_json_text(self.generated_json_text2, self.config_json2)

        self.update_resolution()


    def get_or_create_output_folder(self):
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

        self.depth_in_replay_outputs = False
        output_folder = None
        current_timestamp = self.view_info['timestamps'][self.view_info['current_index']]
        for dir_path in os.listdir(self.output_dir):
            dir_path = os.path.join(self.output_dir, dir_path)
            if not os.path.isdir(dir_path):
                continue
            if os.path.exists(os.path.join(dir_path, "config.json")):
                if compare_json(os.path.join(dir_path, "config.json"), format_json_for_replay(str(self.config_json))):
                    output_folder = dir_path
                    # Check if files with the current timestamp exist in the folder
                    if check_folder_for_timestamp(dir_path, current_timestamp):
                        self.depth_in_replay_outputs = True
                        print(f"PREVIOUSLY GENERATED: {dir_path}")
                        break
                    else:
                        print("Folder exists but timestamp not generated")
                        break
        if not self.depth_in_replay_outputs:
            date_time = datetime.now().strftime("%Y%m%d%H%M%S")
            output_folder = os.path.join(self.output_dir, date_time)
            os.makedirs(output_folder, exist_ok=False)
        return output_folder

    def save_depth_pcl(self, depth, pcl, config):
        np.save(os.path.join(self.output_folder, f"depth_{timestamp}.npy"), depth)
        colorized_depth, _, _ = colorize_depth(depth, "depth", label=False, color_noise_percent_removal=0)
        cv2.imwrite(os.path.join(self.output_folder, f"depth_{timestamp}.png"), colorized_depth)
        o3d.io.write_point_cloud(os.path.join(self.output_folder, f"pcl_{timestamp}.ply"), pcl)

        with open(os.path.join(self.output_folder, f'config.json'), 'w') as f:
            json.dump(config, f, indent=4)

    def replay_generate_one_frame(self, output_folder=None):
        print("Generating depth for ONE timestamp")
        left = self.current_view["left"]
        right = self.current_view["right"]

        # if len(left.shape) == 3 and len(right.shape) == 3:
        #     left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        #     right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

        color = self.current_view["rgb"]
        config = self.config_json
        calib = self.view_info["calib"]

        if left is None or right is None:
            print("No left or right frames provided, closing replay")
            return

        if calib is None:
            print("No calibration provided, closing replay")
            return

        if color is None:
            print("No color specified")
            color = left

        # FIXED - sends two frames and takes the second, works with decimation filter
        replayed = tuple(self.replayer.replay(
            (
                (left, right, color),
                (left, right, color)),
            calib=calib,
            stereo_config=StereoConfig(config)
        ))
        depth = replayed[1]['depth']
        pcl = replayed[1]['pcl']

        aligned_to_rgb = self.config_json['stereo.setDepthAlign'] == "dai.CameraBoardSocket.CAM_A"
        pcl = process_pointcloud(pcl, depth, color, aligned_to_rgb)

        with tempfile.NamedTemporaryFile(delete=False, suffix='.ply') as tmp_file:
            o3d.io.write_point_cloud(tmp_file.name, pcl)

        self.last_generated_depth = depth
        self.last_generated_pcl_path = tmp_file.name

        print("GENERATED")
        return output_folder

if __name__ == '__main__':
    import depthai as dai
    from oak_capture_show import load_data, extract_timestamps

    root = tk.Tk()
    root.title("Main Application")
    root.geometry("300x200")

    ip = None

    parent_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    capture_folder = os.path.join(parent_path, "example_data/OAK-D-S2_194430105176F91200_20250423162047")

    selected_types = ['left', 'right', 'depth', 'rgb']

    data, height, width, rgb_width, rgb_height = load_data(capture_folder, selected_types)
    timestamps = extract_timestamps(data)

    metadata_file = os.path.join(capture_folder, 'metadata.json')
    if os.path.exists(metadata_file):
        with open(metadata_file, 'r') as f:
            metadata = json.load(f)

    calib = ''
    if os.path.exists(f'{capture_folder}/calib.json'):
        calib = dai.CalibrationHandler(f'{capture_folder}/calib.json')

    view_info = {
        "capture_folder": capture_folder,
        "types": selected_types,
        "timestamps": timestamps,
        "current_index": 0,
        "data": data,
        "metadata": metadata,
        "calib": calib,
        "canvas_width": 0,
        "canvas_height": 0,
        "depth_size": (width, height),
        "rgb_size": (rgb_width, rgb_height),
        "mono_size": (width, height),
        # "stereoAlign_used_in_capture": check_settings(capture_folder),
        # "calibration_parameters": datas,
        "RGB_SOCKET": dai.CameraBoardSocket.CAM_A,
        "LEFT_SOCKET": dai.CameraBoardSocket.CAM_B,
        "RIGHT_SOCKET": dai.CameraBoardSocket.CAM_C,
        "device_connected": False,
        "device_info": dai.DeviceInfo(ip) if ip else None,
    }

    current_view = {
        "rgb": None,
        "depth": None,
        "disparity": None,
        "left": None,
        "right": None,
        "alignSocket": "COLOR"
    }

    timestamp = '1441065'
    for key in selected_types:
        image_path = os.path.join(capture_folder, f"{key}_{timestamp}.npy")
        image = np.load(image_path)
        current_view[key] = image.copy()

    print("initialization...")
    repVis = ReplayVisualizer(root, view_info, current_view)
    repVis.create_layout()
    repVis.refresh_display(label="Generate Depth")
    repVis.initialize_depth()

    root.mainloop()
