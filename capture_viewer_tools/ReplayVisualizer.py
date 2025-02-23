import numpy as np
import cv2
import open3d as o3d
import tkinter as tk
from tkinter import ttk
import threading
import os
import json
import glob
from datetime import datetime
from PIL import Image, ImageTk
import subprocess
import time
import platform

from capture_viewer_tools.capture_tools import (colorize_depth, calculate_scaled_dimensions,
                                                format_json_for_replay, create_placeholder_frame)
from capture_viewer_tools.convertor_capture2replay_json import config2settings
from capture_viewer_tools.ReplaySettings import *
from capture_viewer_tools.pointcloud import rotate_pointcloud

from depth.replay_depth import replay

class ReplayVisualizer:
    def __init__(self, root, view_info, current_view):
        self.toplLevel = tk.Toplevel(root)
        self.toplLevel.title("REPLAY")
        self.toplLevel.geometry("2450x1200")

        max_image_width = 840
        max_image_height = 400
        self.scaled_original_size = calculate_scaled_dimensions(view_info['depth_size'], max_image_width, max_image_height)

        self.view_info = view_info
        self.current_view = current_view

        self.last_generated_depth = None
        self.generated_depth1 = None
        self.generated_depth2 = None

        self.last_generated_pcl_path = None
        self.pcl_path1 = None
        self.pcl_path2 = None

        self.generated_depth_image1 = None
        self.generated_depth_image2 = None

        self.config_json = None
        self.config_json1 = None
        self.config_json2 = None

        self.last_config = None

        self.output_dir = os.path.join(view_info["capture_folder"], "replay_outputs")
        if not os.path.exists(self.output_dir): os.makedirs(self.output_dir)

        self.output_folder = None
        self.depth_in_replay_outputs = False

        self.button_values1 = {"settings_section_number" : 1}  # button_key: value
        self.button_values2 = {"settings_section_number" : 2}  # button_key: value

        self.depth_generate_thread1 = threading.Thread(target=lambda: None)
        self.depth_generate_thread2 = threading.Thread(target=lambda: None)

    def add_depthai_to_config(self, config_json):
        """ adding dai. at the beginning of strings so they can be validated as depthai objects in replay"""
        new_config = {}
        for key in config_json.keys():
            config_value = config_json[key]
            print(key, config_value, type(config_value))
            try: config_value = int(config_value)
            except Exception: pass
            if type(config_value) == str and config_value[0] != '[': new_config[key] = "dai." + config_value
            else: new_config[key] = config_value
        return new_config

    def get_initial_config(self, original_config):
        if self.last_config is None and original_config is not None:
            current_config = settings2config(original_config)
        elif self.last_config is not None:
            current_config = self.last_config
        else:
            current_config = default_config
        return current_config

    def inicialize_button_values(self, current_config, button_values):
        # ----------------------------------- Initialize the UI elements with default values from current_config -----------------------------------
        # Initialize tkinter variables, falling back to default_config if a key is missing in current_config
        button_values['depth_align'] = tk.StringVar(value=current_config.get('stereo.setDepthAlign', default_config['stereo.setDepthAlign']))
        button_values['profile_preset'] = tk.StringVar(value=current_config.get('stereo.setDefaultProfilePreset', default_config['stereo.setDefaultProfilePreset']))

        button_values['custom_settings_val'] = tk.BooleanVar(value=False)

        button_values['rectificationBox_val'] = tk.BooleanVar(value=current_config.get('stereo.setRectification', True))  # Default to True if not found
        button_values['LRBox_val'] = tk.BooleanVar(value=current_config.get('stereo.setLeftRightCheck', default_config['stereo.setLeftRightCheck']))
        button_values['extendedBox_val'] = tk.BooleanVar(value=current_config.get('stereo.setExtendedDisparity', default_config['stereo.setExtendedDisparity']))
        button_values['subpixelBox_val'] = tk.BooleanVar(value=current_config.get('stereo.setSubpixel', default_config['stereo.setSubpixel']))
        button_values['fractional_bits_val'] = tk.IntVar(value=current_config.get('stereo.setSubpixelFractionalBits', default_config['stereo.setSubpixelFractionalBits']))

        # FILTERS -----------------------------------------------------------------------------------
        button_values['filtering_order_enable'] = tk.BooleanVar(value=(True if 'cfg.postProcessing.filteringOrder' in current_config else False))
        if 'cfg.postProcessing.filteringOrder' in current_config:
            button_values['initial_filter_order'] = get_filter_order_back(current_config['cfg.postProcessing.filteringOrder'])
        else:
            button_values['initial_filter_order'] = get_filter_order_back(default_config['cfg.postProcessing.filteringOrder'])

        button_values['decimation_order'] = tk.IntVar(value=button_values['initial_filter_order'][0])
        button_values['median_order'] = tk.IntVar(value=button_values['initial_filter_order'][1])
        button_values['speckle_order'] = tk.IntVar(value=button_values['initial_filter_order'][2])
        button_values['spatial_order'] = tk.IntVar(value=button_values['initial_filter_order'][3])
        button_values['temporal_order'] = tk.IntVar(value=button_values['initial_filter_order'][4])

        button_values['median_filter_enable'] = tk.BooleanVar(value=(current_config.get('stereo.initialConfig.setMedianFilter', "MedianFilter.MEDIAN_OFF") != "MedianFilter.MEDIAN_OFF"))
        button_values['median_val'] = tk.StringVar(value=current_config.get('stereo.initialConfig.setMedianFilter', default_config['stereo.initialConfig.setMedianFilter']))

        # bilateral_filter_enable = tk.BooleanVar(value=current_config.get('cfg.postProcessing.bilateralFilter.enable', False))
        # bilateral_sigma_val = tk.IntVar(value=current_config.get('cfg.postProcessing.bilateralSigmaValue', default_config['cfg.postProcessing.bilateralSigmaValue']))

        button_values['brightness_filter_enable'] = tk.BooleanVar(value=current_config.get('cfg.postProcessing.brightnessFilter.enable', False))
        button_values['min_brightness_slider'] = tk.IntVar(value=current_config.get('cfg.postProcessing.brightnessFilter.minBrightness', default_config['cfg.postProcessing.brightnessFilter.minBrightness']))
        button_values['max_brightness_slider'] = tk.IntVar(value=current_config.get('cfg.postProcessing.brightnessFilter.maxBrightness', default_config['cfg.postProcessing.brightnessFilter.maxBrightness']))

        button_values['speckle_filter_enable'] = tk.BooleanVar(value=current_config.get('cfg.postProcessing.speckleFilter.enable', False))
        button_values['speckle_range_slider'] = tk.IntVar(value=current_config.get('cfg.postProcessing.speckleFilter.speckleRange', default_config['cfg.postProcessing.speckleFilter.speckleRange']))
        button_values['speckle_difference_threshold'] = tk.IntVar(value=current_config.get('cfg.postProcessing.speckleFilter.differenceThreshold', default_config['cfg.postProcessing.speckleFilter.differenceThreshold']))

        button_values['spatial_filter_enable'] = tk.BooleanVar(value=current_config.get('cfg.postProcessing.spatialFilter.enable', False))
        button_values['hole_filling_radius_slider'] = tk.IntVar(value=current_config.get('cfg.postProcessing.spatialFilter.holeFillingRadius', default_config['cfg.postProcessing.spatialFilter.holeFillingRadius']))
        button_values['num_iterations_slider'] = tk.IntVar(value=current_config.get('cfg.postProcessing.spatialFilter.numIterations', default_config['cfg.postProcessing.spatialFilter.numIterations']))
        button_values['alpha_slider'] = tk.DoubleVar(value=current_config.get('cfg.postProcessing.spatialFilter.alpha', default_config['cfg.postProcessing.spatialFilter.alpha']))
        button_values['delta_slider'] = tk.IntVar(value=current_config.get('cfg.postProcessing.spatialFilter.delta', default_config['cfg.postProcessing.spatialFilter.delta']))

        button_values['temporal_filter_enable'] = tk.BooleanVar(value=current_config.get('cfg.postProcessing.temporalFilter.enable', False))
        button_values['temporal_alpha_slider'] = tk.DoubleVar(value=current_config.get('cfg.postProcessing.temporalFilter.alpha', default_config['cfg.postProcessing.temporalFilter.alpha']))
        button_values['temporal_delta_slider'] = tk.IntVar(value=current_config.get('cfg.postProcessing.temporalFilter.delta', default_config['cfg.postProcessing.temporalFilter.delta']))

        button_values['threshold_filter_enable'] = tk.BooleanVar(value=(True if 'cfg.postProcessing.thresholdFilter.minRange' in current_config else False))
        button_values['min_range_val'] = tk.IntVar(value=current_config.get('cfg.postProcessing.thresholdFilter.minRange', default_config['cfg.postProcessing.thresholdFilter.minRange']))
        button_values['max_range_val'] = tk.IntVar(value=current_config.get('cfg.postProcessing.thresholdFilter.maxRange', default_config['cfg.postProcessing.thresholdFilter.maxRange']))

        button_values['decimation_filter_enable'] = tk.BooleanVar(value=(True if 'cfg.postProcessing.decimationFilter.decimationFactor' in current_config else False))
        button_values['decimation_factor_val'] = tk.IntVar(value=current_config.get('cfg.postProcessing.decimationFilter.decimationFactor', default_config['cfg.postProcessing.decimationFilter.decimationFactor']))
        button_values['decimation_mode_val'] = tk.StringVar(
            value=handle_dict(current_config.get('cfg.postProcessing.decimationFilter.decimationMode', default_config['cfg.postProcessing.decimationFilter.decimationMode']), decimation_set_dict, reverse=True))

        # ------------------------------------------------------ ADVANCED STEREO SETTINGS -------------------------------------------------------
        button_values['advanced_settings_enable'] = tk.BooleanVar(value=False)
        button_values['mean_mode_enable'] = tk.BooleanVar(value=True)
        button_values['CT_kernel_val'] = tk.StringVar(value='KERNEL_AUTO')
        button_values['CT_threshold_val'] = tk.IntVar(value=0)
        button_values['division_factor_val'] = tk.IntVar(value=1)
        button_values['horizontal_penalty_p1_val'] = tk.IntVar(value=250)
        button_values['horizontal_penalty_p2_val'] = tk.IntVar(value=500)
        button_values['vertical_penalty_p1_val'] = tk.IntVar(value=250)
        button_values['vertical_penalty_p2_val'] = tk.IntVar(value=500)
        button_values['confidence_threshold_val'] = tk.IntVar(value=245)
        button_values['CM_alpha_val'] = tk.IntVar(value=0)
        button_values['CM_beta_val'] = tk.IntVar(value=2)
        button_values['matching_threshold_val'] = tk.IntVar(value=127)

    def convert_current_button_values_to_config(self, button_values):
        config = {}
        config['stereo.setDepthAlign'] = button_values['depth_align'].get()
        if button_values['profile_preset'].get() != 'None':
            config['stereo.setDefaultProfilePreset'] = button_values['profile_preset'].get()

        if button_values['custom_settings_val'].get():
            if button_values['filtering_order_enable'].get():  # needs to be the first item for the config to not be initialised in case of wrong filtering order
                if not check_valid_filtering_order(
                        [button_values['decimation_order'].get(), button_values['median_order'].get(), button_values['speckle_order'].get(), button_values['spatial_order'].get(),
                         button_values['temporal_order'].get()]):
                    warning()
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
                config['stereo.initialConfig.setMedianFilter'] = "MedianFilter.MEDIAN_OFF"

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

        return config

    def on_generate_button_keydown(self, button_values, frame=None):
        if self.depth_generate_thread1.is_alive() or self.depth_generate_thread2.is_alive():
            print("Depth Thread is already running")
            show_popup("Warning", "Depth processing thread is already running, please wait for replay on camera to finish.", frame)
            return
        if button_values["settings_section_number"] == 1:
            self.depth_generate_thread1 = threading.Thread(target=self.on_generate, args=(button_values,))
            self.depth_generate_thread1.start()
        elif button_values["settings_section_number"] == 2:
            self.depth_generate_thread2 = threading.Thread(target=self.on_generate, args=(button_values,))
            self.depth_generate_thread2.start()
    def on_generate(self, button_values):
        settings_section_number = button_values['settings_section_number']
        print(f"GENERATE THREAD: {settings_section_number}")

        self.last_config = self.config_json
        self.config_json = self.convert_current_button_values_to_config(button_values)

        print(self.config_json)

        self.config_json = self.add_depthai_to_config(self.config_json)

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

        self.load_or_generate()

        if settings_section_number == 1:
            self.generated_depth1 = self.last_generated_depth
            self.pcl_path1 = self.last_generated_pcl_path
        elif settings_section_number == 2:
            self.generated_depth2 = self.last_generated_depth
            self.pcl_path2 = self.last_generated_pcl_path
        
        self.depth_range_min, self.depth_range_max = self.get_min_max_depths(self.generated_depth1, self.generated_depth2)

        self.refresh_display(label="Updated")
        self.main_frame.update_idletasks()
        print(f"Thread {settings_section_number} finished")

    def create_settings_layout(self, frame, button_values):
        def update_label(var, label, form="int"):
            if label is None: return
            if form == "int":
                label.config(text=str(int(var.get())))
            elif form == "float":
                label.config(text=str(round(float(var.get()), 1)))

        def toggle_custom_frame_settings():
            toggle_frame_settings(button_values['custom_settings_val'], custom_settings_frame)
            if button_values['custom_settings_val'].get():
                toggle_frame_settings(button_values['decimation_filter_enable'], decimation_frame)
                toggle_frame_settings(button_values['median_filter_enable'], median_frame)
                toggle_frame_settings(button_values['speckle_filter_enable'], speckle_frame)
                toggle_frame_settings(button_values['spatial_filter_enable'], spatial_frame)
                toggle_frame_settings(button_values['temporal_filter_enable'], temporal_frame)
                toggle_frame_settings(button_values['threshold_filter_enable'], threshold_frame)
                # toggle_frame_settings(bilateral_filter_enable, bilateral_frame)
                toggle_frame_settings(button_values['brightness_filter_enable'], brightness_frame)
                toggle_frame_settings(button_values['filtering_order_enable'], order_frame)

        def toggle_frame_settings(frame_on, frame):
            if frame_on.get(): enable_frame_widgets_recursively(frame, True)
            else: enable_frame_widgets_recursively(frame, False)

        def toggle_advanced_settings():
            if button_values['advanced_settings_enable'].get():
                enable_frame_widgets_recursively(advanced_stereo_setting_frame, True)
            else:
                enable_frame_widgets_recursively(advanced_stereo_setting_frame, False)

        def enable_frame_widgets_recursively(frame, state):
            for widget in frame.winfo_children():
                if isinstance(widget, ttk.LabelFrame):
                    enable_frame_widgets_recursively(widget, state)
                    widget.state(['!disabled'] if state else ['disabled'])
                else:
                    widget.state(['!disabled'] if state else ['disabled'])
            frame.state(['!disabled'] if state else ['disabled'])

        popup_window = frame

        # ----------------------------------------------------------------- BUTTONS -------------------------------------------------------------

        current_row = 0

        # Add radiobuttons for left/right choice with a label
        ttk.Label(popup_window, text="setDepthAlign").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
        left_radiobutton = ttk.Radiobutton(popup_window, text="Left", variable=button_values['depth_align'], value="CameraBoardSocket.LEFT")
        right_radiobutton = ttk.Radiobutton(popup_window, text="Right", variable=button_values['depth_align'], value="CameraBoardSocket.RIGHT")
        rec_left_radiobutton = ttk.Radiobutton(popup_window, text="Rec Left", variable=button_values['depth_align'], value='StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT')
        rec_right_radiobutton = ttk.Radiobutton(popup_window, text="Rec Right", variable=button_values['depth_align'], value='StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT')
        rgb_radiobutton = ttk.Radiobutton(popup_window, text="RGB", variable=button_values['depth_align'], value="CameraBoardSocket.CAM_A")
        left_radiobutton.grid(row=current_row, column=1, padx=10, pady=5, sticky="w")
        right_radiobutton.grid(row=current_row, column=2, padx=10, pady=5, sticky="w")
        rec_left_radiobutton.grid(row=current_row, column=3, padx=10, pady=5, sticky="w")
        rec_right_radiobutton.grid(row=current_row, column=4, padx=10, pady=5, sticky="w")
        rgb_radiobutton.grid(row=current_row, column=5, padx=10, pady=5, sticky="w")
        current_row += 1

        #
        ttk.Label(popup_window, text="setDefaultProfilePreset").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
        Hdef_radiobutton = ttk.Radiobutton(popup_window, text="DEFAULT", variable=button_values['profile_preset'], value="node.StereoDepth.PresetMode.DEFAULT")
        HA_radiobutton = ttk.Radiobutton(popup_window, text="HIGH_ACCURACY", variable=button_values['profile_preset'], value="node.StereoDepth.PresetMode.HIGH_ACCURACY")
        HD_radiobutton = ttk.Radiobutton(popup_window, text="HIGH_DENSITY", variable=button_values['profile_preset'], value="node.StereoDepth.PresetMode.HIGH_DENSITY")
        HR_radiobutton = ttk.Radiobutton(popup_window, text="ROBOTICS", variable=button_values['profile_preset'], value="node.StereoDepth.PresetMode.ROBOTICS")
        HDE_radiobutton = ttk.Radiobutton(popup_window, text="HIGH_DETAIL", variable=button_values['profile_preset'], value="node.StereoDepth.PresetMode.HIGH_DETAIL")
        HF_radiobutton = ttk.Radiobutton(popup_window, text="FACE", variable=button_values['profile_preset'], value="node.StereoDepth.PresetMode.FACE")
        HN_radiobutton = ttk.Radiobutton(popup_window, text="None", variable=button_values['profile_preset'], value="None")
        Hdef_radiobutton.grid(row=current_row, column=1, padx=10, pady=5, sticky="w")
        HR_radiobutton.grid(row=current_row, column=2, padx=10, pady=5, sticky="w")
        HDE_radiobutton.grid(row=current_row, column=3, padx=10, pady=5, sticky="w")
        HF_radiobutton.grid(row=current_row, column=4, padx=10, pady=5, sticky="w")
        current_row += 1
        HA_radiobutton.grid(row=current_row, column=1, padx=10, pady=5, sticky="w")
        HD_radiobutton.grid(row=current_row, column=2, padx=10, pady=5, sticky="w")
        HN_radiobutton.grid(row=current_row, column=3, padx=10, pady=5, sticky="w")
        current_row += 1

        #
        ttk.Label(popup_window, text="Use Custom Settings").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
        cust_box = ttk.Checkbutton(popup_window, variable=button_values['custom_settings_val'], command=toggle_custom_frame_settings)
        cust_box.grid(row=current_row, column=1, padx=10, pady=10, sticky="w")
        current_row += 1

        custom_settings_frame = ttk.LabelFrame(popup_window, text="Custom Settings", padding=(10, 10))
        custom_settings_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="w")

        # # Add checkbuttons for rectification
        ttk.Label(custom_settings_frame, text="setRectification").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
        recbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['rectificationBox_val'])
        recbox.grid(row=current_row, column=1, padx=10, pady=10,  sticky="w")
        current_row += 1

        # LR CHECK
        ttk.Label(custom_settings_frame, text="setLRcheck").grid(row=current_row, column=0, padx=10, pady=10,sticky="w")
        LRbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['LRBox_val'])
        LRbox.grid(row=current_row, column=1, padx=10, pady=10,  sticky="w")
        current_row += 1

        # EXTENDED DISPARITY
        ttk.Label(custom_settings_frame, text="setExtendedDisparity").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
        extbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['extendedBox_val'])
        extbox.grid(row=current_row, column=1, padx=10, pady=10,  sticky="w")
        current_row += 1

        # SetSubpixel Label and Checkbox
        ttk.Label(custom_settings_frame, text="setSubpixel").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
        subbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['subpixelBox_val'])
        subbox.grid(row=current_row, column=1, padx=10, pady=10,  sticky="w")


        def dropdown(frame, row, col, val, options):
            combo = ttk.Combobox(frame, textvariable=val, values=options, state="readonly")
            combo.grid(row=row, column=col, padx=10, pady=10,  sticky="w")
            combo.bind("<MouseWheel>", lambda event: "break")
            combo.bind("<Button-4>", lambda event: "break")
            combo.bind("<Button-5>", lambda event: "break")

        def spinbox(frame, row, col, slider, label, range):
            spinbox = ttk.Spinbox(frame, from_=range[0], to=range[1], textvariable=slider, command=lambda: update_label(slider, label))
            spinbox.grid(row=row, column=col, padx=10, pady=10,  sticky="w")
            spinbox.bind("<MouseWheel>", lambda event: "break")
            spinbox.bind("<Button-4>", lambda event: "break")
            spinbox.bind("<Button-5>", lambda event: "break")

        # Add subpixelFractionalBits combobox
        ttk.Label(custom_settings_frame, text="subpixelFractionalBits").grid(row=current_row, column=2, padx=10, pady=10, sticky="w")
        dropdown(custom_settings_frame, current_row, 3, button_values['fractional_bits_val'], [3, 4, 5])

        current_row += 1

        # Decimation Filter Enable
        ttk.Label(custom_settings_frame, text="Decimation Filter Enable").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
        decimation_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['decimation_filter_enable'], command=lambda: toggle_frame_settings(button_values['decimation_filter_enable'], decimation_frame))
        decimation_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10,  sticky="w")

        current_row += 1

        # Decimation Filter Group (in a LabelFrame with a black border)
        decimation_frame = ttk.LabelFrame(custom_settings_frame, text="Decimation Filter", padding=(10, 10))
        decimation_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

        # Decimation Filter (Factor and Mode on one row)
        ttk.Label(decimation_frame, text="Factor").grid(row=1, column=0, padx=10, pady=10, sticky="w")
        decimation_factor_label = ttk.Label(decimation_frame)
        decimation_factor_label.grid(row=1, column=2, padx=10, pady=10, sticky="w")
        dropdown(decimation_frame, 1, 1, button_values['decimation_factor_val'], [1, 2, 3, 4])
        ttk.Label(decimation_frame, text="Mode").grid(row=1, column=3, padx=10, pady=10, sticky="w")
        dropdown(decimation_frame, 1, 4, button_values['decimation_mode_val'], ['NON_ZERO_MEAN','NON_ZERO_MEDIAN','PIXEL_SKIPPING'])

        current_row += 1

        # Median Filter
        ttk.Label(custom_settings_frame, text="Median Enable").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
        median_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['median_filter_enable'], command=lambda: toggle_frame_settings(button_values['median_filter_enable'], median_frame))
        median_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10,  sticky="w")
        current_row += 1

        # Median Filter Frame
        median_frame = ttk.LabelFrame(custom_settings_frame, text="Median Filter", padding=(10, 10))
        median_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

        ttk.Label(median_frame, text="Median").grid(row=0, column=0, padx=10, pady=10, sticky="w")
        median_3_button = ttk.Radiobutton(median_frame, text="MEDIAN_3x3", variable=button_values['median_val'], value="MedianFilter.KERNEL_3x3")
        median_5_button = ttk.Radiobutton(median_frame, text="MEDIAN_5x5", variable=button_values['median_val'], value="MedianFilter.KERNEL_5x5")
        median_7_button = ttk.Radiobutton(median_frame, text="MEDIAN_7x7", variable=button_values['median_val'], value="MedianFilter.KERNEL_7x7")
        median_3_button.grid(row=0, column=1, padx=10, pady=5, sticky="w")
        median_5_button.grid(row=0, column=2, padx=10, pady=5, sticky="w")
        median_7_button.grid(row=0, column=3, padx=10, pady=5, sticky="w")
        current_row += 1

        # Speckle Filter Enable
        ttk.Label(custom_settings_frame, text="Speckle Filter Enable").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
        speckle_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['speckle_filter_enable'], command=lambda: toggle_frame_settings(button_values['speckle_filter_enable'], speckle_frame))
        speckle_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10,  sticky="w")
        current_row += 1

        # Speckle Filter Frame
        speckle_frame = ttk.LabelFrame(custom_settings_frame, text="Speckle Filter", padding=(10, 10))
        speckle_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

        # Speckle Range
        ttk.Label(speckle_frame, text="Speckle Range").grid(row=1, column=0, padx=10, pady=10, sticky="w")
        speckle_range_label = ttk.Label(speckle_frame, text=str(button_values['speckle_range_slider'].get()))
        spinbox(speckle_frame, 1, 1, button_values['speckle_range_slider'], speckle_range_label, [0, 255])
        ttk.Label(speckle_frame, text="(0, 256)").grid(row=1, column=2, padx=10, pady=10, sticky="w")
        current_row += 1

        # Speckle Difference Threshold
        ttk.Label(speckle_frame, text="Speckle Difference Threshold").grid(row=2, column=0, padx=10, pady=10, sticky="w")
        speckle_difference_label = ttk.Label(speckle_frame, text=str(button_values['speckle_difference_threshold'].get()))
        spinbox(speckle_frame, 2, 1, button_values['speckle_difference_threshold'], speckle_difference_label, [0, 255])
        ttk.Label(speckle_frame, text="(0, 256)").grid(row=2, column=2, padx=10, pady=10, sticky="w")
        current_row += 1

        # Spatial Filter Enable
        ttk.Label(custom_settings_frame, text="Spatial Filter Enable").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
        spatial_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['spatial_filter_enable'], command=lambda: toggle_frame_settings(button_values['spatial_filter_enable'], spatial_frame))
        spatial_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10,  sticky="w")
        current_row += 1

        # Spatial Filter Frame
        spatial_frame = ttk.LabelFrame(custom_settings_frame, text="Spatial Filter", padding=(10, 10))
        spatial_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

        # Spatial Filter Settings
        ttk.Label(spatial_frame, text="Hole Filling Radius").grid(row=1, column=0, padx=10, pady=10, sticky="w")
        hole_filling_radius_label = ttk.Label(spatial_frame, text=str(button_values['hole_filling_radius_slider'].get()))
        spinbox(spatial_frame, 1, 1, button_values['hole_filling_radius_slider'], hole_filling_radius_label, [0, 255])
        ttk.Label(spatial_frame, text="(0, 255)").grid(row=1, column=2, padx=10, pady=10, sticky="w")

        ttk.Label(spatial_frame, text="Num Iterations").grid(row=1, column=3, padx=10, pady=10, sticky="w")
        num_iterations_label = ttk.Label(spatial_frame, text=str(button_values['num_iterations_slider'].get()))
        num_iterations_label.grid(row=1, column=5, padx=10, pady=10, sticky="w")
        ttk.Scale(spatial_frame, from_=1, to=10, variable=button_values['num_iterations_slider'], orient="horizontal", command=lambda x: update_label(button_values['num_iterations_slider'], num_iterations_label)).grid(row=1, column=4, padx=10, pady=10,  sticky="w")
        current_row += 1

        ttk.Label(spatial_frame, text="Alpha").grid(row=2, column=0, padx=10, pady=10, sticky="w")
        alpha_label = ttk.Label(spatial_frame, text=str(button_values['alpha_slider'].get()))
        alpha_label.grid(row=2, column=2, padx=10, pady=10, sticky="w")
        ttk.Scale(spatial_frame, from_=0, to=1, variable=button_values['alpha_slider'], orient="horizontal", command=lambda x: update_label(button_values['alpha_slider'], alpha_label, form="float")).grid(row=2, column=1, padx=10, pady=10,  sticky="w")

        ttk.Label(spatial_frame, text="Delta").grid(row=2, column=3, padx=10, pady=10, sticky="w")
        delta_label = ttk.Label(spatial_frame, text=str(button_values['delta_slider'].get()))
        spinbox(spatial_frame, 2, 4, button_values['delta_slider'], delta_label, [0, 255])
        ttk.Label(spatial_frame, text="(0, 255)").grid(row=2, column=5, padx=10, pady=10, sticky="w")

        current_row += 1

        # Temporal Filter Frame
        ttk.Label(custom_settings_frame, text="Temporal Filter Enable").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
        temporal_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['temporal_filter_enable'], command=lambda: toggle_frame_settings(button_values['temporal_filter_enable'], temporal_frame))
        temporal_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10,  sticky="w")
        current_row += 1

        # Spatial Filter Frame
        temporal_frame = ttk.LabelFrame(custom_settings_frame, text="Temporal Filter", padding=(10, 10))
        temporal_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

        # Temporal Filter Enable
        ttk.Label(temporal_frame, text="Alpha").grid(row=2, column=0, padx=10, pady=10, sticky="w")
        temporal_alpha_label = ttk.Label(temporal_frame, text=str(button_values['temporal_alpha_slider'].get()))
        temporal_alpha_label.grid(row=2, column=2, padx=10, pady=10, sticky="w")
        ttk.Scale(temporal_frame, from_=0, to=1, variable=button_values['temporal_alpha_slider'], orient="horizontal",
                  command=lambda x: update_label(button_values['temporal_alpha_slider'], temporal_alpha_label, form="float")).grid(row=2, column=1, padx=10, pady=10,  sticky="w")

        ttk.Label(temporal_frame, text="Delta").grid(row=2, column=3, padx=10, pady=10, sticky="w")
        temporal_delta_label = ttk.Label(temporal_frame, text=str(button_values['temporal_delta_slider'].get()))
        spinbox(temporal_frame, 2, 4, button_values['temporal_delta_slider'], temporal_delta_label, [0, 255])
        ttk.Label(temporal_frame, text="(0, 255)").grid(row=2, column=5, padx=10, pady=10, sticky="w")

        current_row += 1

        # Threshold Filter Enable
        ttk.Label(custom_settings_frame, text="Threshold Filter Enable").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
        threshold_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['threshold_filter_enable'], command=lambda: toggle_frame_settings(button_values['threshold_filter_enable'], threshold_frame))
        threshold_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10,  sticky="w")
        current_row += 1

        # Threshold Filter Frame
        threshold_frame = ttk.LabelFrame(custom_settings_frame, text="Threshold Filter", padding=(10, 10))
        threshold_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

        # Threshold Filter Min and Max with Spinboxes
        ttk.Label(threshold_frame, text="Min").grid(row=1, column=0, padx=10, pady=10, sticky="w")
        spinbox(threshold_frame, 1, 1, button_values['min_range_val'], None, [0, button_values['max_range_val'].get()])
        ttk.Label(threshold_frame, text="(0, " + str(button_values['max_range_val'].get()) + ")").grid(row=1, column=2, padx=10, pady=10, sticky="w")

        ttk.Label(threshold_frame, text="Max").grid(row=1, column=3, padx=10, pady=10, sticky="w")
        spinbox(threshold_frame, 1, 4, button_values['max_range_val'], None, [0, button_values['max_range_val'].get()])
        ttk.Label(threshold_frame, text="(" + str(button_values['min_range_val'].get()) + ", 65535)").grid(row=1, column=5, padx=10, pady=10, sticky="w")

        current_row += 1

        # # Bilateral Filter Enable
        # ttk.Label(custom_settings_frame, text="Bilateral Filter Enable (NOT TESTED IN GUI)").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
        # bilateral_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=bilateral_filter_enable, command=lambda: toggle_frame_settings(bilateral_filter_enable, bilateral_frame))
        # bilateral_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10,  sticky="w")
        # current_row += 1
        #
        # # Bilateral Filter Frame
        # bilateral_frame = ttk.LabelFrame(custom_settings_frame, text="Bilateral Filter", padding=(10, 10))
        # bilateral_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")
        #
        # # Bilateral Sigma Value
        # ttk.Label(bilateral_frame, text="Bilateral Sigma Value").grid(row=1, column=0, padx=10, pady=10, sticky="w")
        # bilateral_sigma_label = ttk.Label(bilateral_frame, text=str(bilateral_sigma_val.get()))
        # bilateral_sigma_label.grid(row=1, column=2, padx=10, pady=10, sticky="w")
        # bilateral_sigma_slider = ttk.Scale(bilateral_frame, from_=0, to=20, variable=bilateral_sigma_val,
        #                                    orient="horizontal",
        #                                    command=lambda x: update_label(bilateral_sigma_val, bilateral_sigma_label))
        # bilateral_sigma_slider.grid(row=1, column=1, padx=10, pady=10,  sticky="w")
        # current_row += 1

        # Brightness Filter Enable
        ttk.Label(custom_settings_frame, text="Brightness Filter Enable (NOT TESTED IN GUI)").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
        brightness_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['brightness_filter_enable'], command=lambda: toggle_frame_settings(button_values['brightness_filter_enable'], brightness_frame))
        brightness_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10,  sticky="w")
        current_row += 1

        # Brightness Filter Frame
        brightness_frame = ttk.LabelFrame(custom_settings_frame, text="Brightness Filter", padding=(10, 10))
        brightness_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

        # Brightness Filter Sliders
        ttk.Label(brightness_frame, text="Min Brightness").grid(row=1, column=0, padx=10, pady=10, sticky="w")
        min_brightness_label = ttk.Label(brightness_frame, text=str(button_values['min_brightness_slider'].get()))
        min_brightness_label.grid(row=1, column=2, padx=10, pady=10, sticky="w")
        ttk.Scale(brightness_frame, from_=0, to=int(button_values['max_brightness_slider'].get()), variable=button_values['min_brightness_slider'], orient="horizontal",
                  command=lambda x: update_label(button_values['min_brightness_slider'], min_brightness_label)).grid(row=1, column=1,padx=10, pady=10,  sticky="w")

        ttk.Label(brightness_frame, text="Max Brightness").grid(row=1, column=3, padx=10, pady=10, sticky="w")
        max_brightness_label = ttk.Label(brightness_frame, text=str(button_values['max_brightness_slider'].get()))
        max_brightness_label.grid(row=1, column=5, padx=10, pady=10, sticky="w")
        ttk.Scale(brightness_frame, from_=int(button_values['min_brightness_slider'].get()), to=255, variable=button_values['max_brightness_slider'], orient="horizontal",
                  command=lambda x: update_label(button_values['max_brightness_slider'], max_brightness_label)).grid(row=1, column=4, padx=10, pady=10,  sticky="w")
        current_row += 1

        # Filter Order Enable
        ttk.Label(custom_settings_frame, text="Filter Order Selection Enable (NOT TESTED IN GUI)").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
        send_filter_order_checkbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['filtering_order_enable'], command=lambda: toggle_frame_settings(button_values['filtering_order_enable'], order_frame))
        send_filter_order_checkbox.grid(row=current_row, column=1, padx=10, pady=10,  sticky="w")
        current_row += 1

        # Filter order
        order_frame = ttk.LabelFrame(custom_settings_frame, text="Filtering Order", padding=(10, 10))
        order_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

        row = 1
        ttk.Label(order_frame, text="Decimation").grid(row=row, column=0, padx=10, pady=10, sticky="w")
        order_label = ttk.Label(order_frame)
        order_label.grid(row=row, column=2, padx=10, pady=10, sticky="w")
        dropdown(order_frame, row, 1, button_values['decimation_order'], [1, 2, 3, 4, 5])

        row = 2
        ttk.Label(order_frame, text="Median").grid(row=row, column=0, padx=10, pady=10, sticky="w")
        order_label = ttk.Label(order_frame)
        order_label.grid(row=row, column=2, padx=10, pady=10, sticky="w")
        dropdown(order_frame, row, 1, button_values['median_order'], [1, 2, 3, 4, 5])

        row = 3
        ttk.Label(order_frame, text="Speckle").grid(row=row, column=0, padx=10, pady=10, sticky="w")
        order_label = ttk.Label(order_frame)
        order_label.grid(row=row, column=2, padx=10, pady=10, sticky="w")
        dropdown(order_frame, row, 1, button_values['speckle_order'], [1, 2, 3, 4, 5])

        row = 4
        ttk.Label(order_frame, text="Spatial").grid(row=row, column=0, padx=10, pady=10, sticky="w")
        order_label = ttk.Label(order_frame)
        order_label.grid(row=row, column=2, padx=10, pady=10, sticky="w")
        dropdown(order_frame, row, 1, button_values['spatial_order'], [1, 2, 3, 4, 5])

        row = 5
        ttk.Label(order_frame, text="Temporal").grid(row=row, column=0, padx=10, pady=10, sticky="w")
        order_label = ttk.Label(order_frame)
        order_label.grid(row=row, column=2, padx=10, pady=10, sticky="w")
        dropdown(order_frame, row, 1, button_values['temporal_order'], [1, 2, 3, 4, 5])

        current_row += 1

        # STEREO ALGORITHM ADVANCED SETTINGS -------------------------------------------------------------------------------
        ttk.Label(popup_window, text="Enable Advanced Settings (I know what I'm doing)").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
        advanced_settings_checkbox = ttk.Checkbutton(popup_window, variable=button_values['advanced_settings_enable'], command=toggle_advanced_settings)
        advanced_settings_checkbox.grid(row=current_row, column=1, padx=10, pady=10,  sticky="w")

        current_row += 1

        inner_row = 1

        advanced_stereo_setting_frame = ttk.LabelFrame(popup_window, text="Advanced Settings", padding=(10, 10))
        advanced_stereo_setting_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

        ttk.Label(advanced_stereo_setting_frame, text="censusTransform.kernelSize").grid(row=inner_row, column=0, padx=10, pady=10, sticky="w")
        dropdown(advanced_stereo_setting_frame, inner_row, 1, button_values['CT_kernel_val'], ['KERNEL_AUTO', 'KERNEL_5x5', 'KERNEL_7x7', 'KERNEL_7x9'])

        inner_row += 1

        ttk.Label(advanced_stereo_setting_frame, text="censusTransform.enableMeanMode").grid(row=inner_row, column=0, padx=10, pady=10, sticky="w")
        mean_mode_checkbox = ttk.Checkbutton(advanced_stereo_setting_frame, variable=button_values['mean_mode_enable'])
        mean_mode_checkbox.grid(row=inner_row, column=1, padx=10, pady=10,  sticky="w")

        inner_row += 1

        ttk.Label(advanced_stereo_setting_frame, text="censusTransform.threshold").grid(row=inner_row, column=0, padx=10, pady=10, sticky="w")
        spinbox(advanced_stereo_setting_frame, inner_row, 1, button_values['CT_threshold_val'], None, [0, 255])
        ttk.Label(advanced_stereo_setting_frame, text="(0, 255)").grid(row=3, column=2, padx=10, pady=10, sticky="w")

        inner_row += 1

        ttk.Label(advanced_stereo_setting_frame, text="costAggregation.divisionFactor").grid(row=inner_row, column=0, padx=10, pady=10, sticky="w")
        spinbox(advanced_stereo_setting_frame, inner_row, 1, button_values['division_factor_val'], None, [0, 100])
        ttk.Label(advanced_stereo_setting_frame, text="(1, 100)").grid(row=inner_row, column=2, padx=10, pady=10, sticky="w")

        inner_row += 1

        ttk.Label(advanced_stereo_setting_frame, text="costAggregation.horizontalPenaltyCostP1").grid(row=inner_row, column=0, padx=10, pady=10, sticky="w")
        spinbox(advanced_stereo_setting_frame, inner_row, 1, button_values['horizontal_penalty_p1_val'], None, [0, 500])
        ttk.Label(advanced_stereo_setting_frame, text="(0, 500)").grid(row=inner_row, column=2, padx=10, pady=10, sticky="w")

        inner_row += 1

        ttk.Label(advanced_stereo_setting_frame, text="costAggregation.horizontalPenaltyCostP2").grid(row=inner_row, column=0, padx=10, pady=10, sticky="w")
        spinbox(advanced_stereo_setting_frame, inner_row, 1, button_values['horizontal_penalty_p2_val'], None, [0, 1000])
        ttk.Label(advanced_stereo_setting_frame, text="(0, 1000)").grid(row=inner_row, column=2, padx=10, pady=10, sticky="w")

        inner_row += 1

        ttk.Label(advanced_stereo_setting_frame, text="costAggregation.verticalPenaltyCostP1").grid(row=inner_row, column=0, padx=10, pady=10, sticky="w")
        spinbox(advanced_stereo_setting_frame, inner_row, 1, button_values['vertical_penalty_p1_val'], None, [0, 500])
        ttk.Label(advanced_stereo_setting_frame, text="(0, 500)").grid(row=inner_row, column=2, padx=10, pady=10, sticky="w")

        inner_row += 1

        ttk.Label(advanced_stereo_setting_frame, text="costAggregation.verticalPenaltyCostP2").grid(row=inner_row, column=0,padx=10, pady=10, sticky="w")
        spinbox(advanced_stereo_setting_frame, inner_row, 1, button_values['vertical_penalty_p2_val'], None, [0, 1000])
        ttk.Label(advanced_stereo_setting_frame, text="(0, 1000)").grid(row=inner_row, column=2, padx=10, pady=10, sticky="w")

        inner_row += 1

        ttk.Label(advanced_stereo_setting_frame, text="costMatching.confidenceThreshold").grid(row=inner_row, column=0, padx=10, pady=10, sticky="w")
        spinbox(advanced_stereo_setting_frame, inner_row, 1, button_values['confidence_threshold_val'], None, [0, 255])
        ttk.Label(advanced_stereo_setting_frame, text="(0, 255)").grid(row=inner_row, column=2, padx=10, pady=10, sticky="w")

        inner_row += 1

        # Alpha Slider
        ttk.Label(advanced_stereo_setting_frame, text="costMatching.linearEquationParameters.alpha").grid(row=inner_row, column=0, padx=10, pady=10, sticky="w")
        CM_alpha_slider = ttk.Scale(advanced_stereo_setting_frame, from_=0, to=10, variable=button_values['CM_alpha_val'], orient="horizontal", command=lambda x: update_label(CM_alpha_slider, CM_alpha_label))
        CM_alpha_slider.grid(row=inner_row, column=1, padx=10, pady=10, sticky="ew")
        CM_alpha_label = ttk.Label(advanced_stereo_setting_frame, text=str(int(button_values['CM_alpha_val'].get())))
        CM_alpha_label.grid(row=inner_row, column=2, padx=10, pady=10, sticky="w")

        inner_row += 1

        # Beta Slider
        ttk.Label(advanced_stereo_setting_frame, text="costMatching.linearEquationParameters.beta").grid(row=inner_row, column=0, padx=10, pady=10, sticky="w")
        CM_beta_slider = ttk.Scale(advanced_stereo_setting_frame, from_=0, to=10, variable=button_values['CM_beta_val'], orient="horizontal", command=lambda x: update_label(CM_beta_slider, CM_beta_label))
        CM_beta_slider.grid(row=inner_row, column=1, padx=10, pady=10, sticky="ew")
        CM_beta_label = ttk.Label(advanced_stereo_setting_frame, text=str(int(button_values['CM_beta_val'].get())))
        CM_beta_label.grid(row=inner_row, column=2, padx=10, pady=10, sticky="w")

        inner_row += 1

        ttk.Label(advanced_stereo_setting_frame, text="costMatching.linearEquationParameters.threshold").grid(row=inner_row, column=0, padx=10, pady=10, sticky="w")
        spinbox(advanced_stereo_setting_frame, inner_row, 1, button_values['matching_threshold_val'], None, [0, 255])
        ttk.Label(advanced_stereo_setting_frame, text="(0, 255)").grid(row=inner_row, column=2, padx=10, pady=10, sticky="w")

        inner_row += 1

        current_row += 1

        # ------------------------------------------------------------------------------------------------------------------------------

        toggle_custom_frame_settings()
        toggle_advanced_settings()  # turn off by default

        # popup_window.grab_set()  # Make the window modal (disable interaction with the main window)
        # popup_window.wait_window()  # Wait for the popup window to be destroyed


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

    def show_pointcloud(self, pcl_path, config_json):
        if pcl_path is None: return False
        script_directory = os.path.dirname(os.path.abspath(__file__))
        visualize_pointcloud_path = os.path.join(script_directory, 'visualize_pointcloud.py')
        subprocess.Popen(['python', visualize_pointcloud_path, pcl_path, str(config_json)])  # o3d.visualization.draw_geometries([pointcloud])
        time.sleep(1)

    def create_depth_section(self, collumn_in_main_frame):
        generated_depth_frame = tk.Frame(self.main_frame)
        generated_depth_frame.grid(row=1, column=collumn_in_main_frame, padx=5, pady=5, sticky='nsew')

        generated_depth_text_label1 = tk.Label(generated_depth_frame, text="Generated Depth", font=("Arial", 12, "bold"))
        generated_depth_text_label1.grid(row=0, column=0, padx=10, pady=(10, 0), sticky="nsew")

        generated_depth_image = tk.Label(generated_depth_frame, image=None)
        generated_depth_image.image = None
        generated_depth_image.grid(row=1, column=0, padx=10, pady=10)

        if collumn_in_main_frame == 0:
            pointcloud_button = tk.Button(generated_depth_frame, text="Pointcloud", command=lambda: self.show_pointcloud(self.pcl_path1, self.config_json1))
        else:
            pointcloud_button = tk.Button(generated_depth_frame, text="Pointcloud", command=lambda: self.show_pointcloud(self.pcl_path2, self.config_json2))
        pointcloud_button.grid(row=1, column=2, pady=(50, 0))

        return generated_depth_frame, generated_depth_image
    def create_settings_section(self, collumn_in_main_frame, frame_name, button_values):
        settings_frame_custom = tk.Frame(self.main_frame, name=frame_name)
        settings_frame_custom.grid(row=2, column=collumn_in_main_frame, sticky="nsew")

        settings_frame_custom.grid_rowconfigure(0, weight=1)
        settings_frame_custom.grid_columnconfigure(0, weight=1, minsize=850)

        # ---- CANVAS FOR SETTINGS
        settings_canvas = tk.Canvas(settings_frame_custom)
        settings_canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar = tk.Scrollbar(settings_frame_custom, orient="vertical", command=settings_canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")

        settings_canvas.configure(yscrollcommand=scrollbar.set)

        content_frame = tk.Frame(settings_canvas)
        initial_config = self.get_initial_config(original_config=None)
        self.inicialize_button_values(initial_config, button_values)
        self.create_settings_layout(content_frame, button_values)
        settings_canvas.create_window((0, 0), window=content_frame, anchor="nw")
        content_frame.update_idletasks()

        settings_canvas.config(scrollregion=settings_canvas.bbox("all"))

        # Add a "GENERATE" button at the bottom
        generate_button = tk.Button(settings_frame_custom, text="GENERATE", bg="green2", activebackground="green4", command=lambda: self.on_generate_button_keydown(button_values, frame=settings_frame_custom))
        generate_button.grid(row=1, column=0, columnspan=2, pady=20)

        # # Add a "GENERATE" button at the bottom
        # generate_button = tk.Button(popup_window, text="GENERATE", bg="blue2", activebackground="blue4", command=lambda: self.on_generate_button_keydown(button_values))  # todo
        # generate_button.grid(row=current_row, column=1, columnspan=2, pady=20)

        return settings_frame_custom, settings_canvas
    def create_config_section(self, collumn_in_main_frame, config_frame_name):
        config_frame = tk.Frame(self.main_frame, name=config_frame_name)
        config_frame.grid(row=3, column=collumn_in_main_frame, padx=5, pady=5, sticky='nsew')

        config_frame.grid_rowconfigure(0, weight=1)
        config_frame.grid_columnconfigure(0, weight=1, minsize=850)

        config_canvas = tk.Canvas(config_frame, height=150) # set max height
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
                                      font=("Courier", 8),
                                      wrap="word",  # Enables text wrapping
                                      height=20,  # Adjust based on needs
                                      width=64)  # Adjust width as necessary
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

        self.bind_scrolling()

    def refresh_generated_depth_or_placeholder(self, generated_depth, generated_depth_image, label):
        if generated_depth is None:
            colorized_generated_depth = create_placeholder_frame(self.scaled_original_size, label)
        else:
            colorized_generated_depth, _, _ = colorize_depth(generated_depth, min_val=self.depth_range_min, max_val=self.depth_range_max, type="depth", label=0)
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
            colorized_difference, _, _ = colorize_depth(difference, min_val=self.depth_range_min, max_val=self.depth_range_max, type="depth", label=0)

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

    def get_min_max_depths(self, depth1, depth2, color_noise_percent_removal=1):
        if depth1 is None and depth2 is None:
            print("Both NONE")
            return None, None
        elif depth1 is not None and depth2 is not None:
            print("Both not NONE")
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

    def refresh_display(self, label=None):
        print("Refresh display:", label)

        self.refresh_generated_depth_or_placeholder(self.generated_depth1, self.generated_depth_image1, label)
        self.refresh_generated_depth_or_placeholder(self.generated_depth2, self.generated_depth_image2, label)

        self.refresh_difference_or_placeholder(self.generated_depth1, self.generated_depth2, self.difference_image)

        self.refresh_json_text(self.generated_json_text1, self.config_json1)
        self.refresh_json_text(self.generated_json_text2, self.config_json2)


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


    def generate_save_depth_replay_one_frame(self, output_folder=None):
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

        if left is None or right is None:
            print("No left or right frames provided, closing replay")
            return

        if calib is None:
            print("No calibration provided, closing replay")
            return

        if color is None:
            print("No color specified")
            color = left

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


        self.last_generated_depth = depth

        # Save depth data
        timestep = self.view_info['timestamps'][self.view_info['current_index']].split('.npy')[0]
        np.save(os.path.join(output_folder, f"depth_{timestep}.npy"), self.last_generated_depth)
        cv2.imwrite(os.path.join(output_folder, f"depth_{timestep}.png"), self.last_generated_depth)
        o3d.io.write_point_cloud(os.path.join(output_folder, f"pcl_{timestep}.ply"), pcd)

        with open(os.path.join(output_folder, f'config.json'), 'w') as f:
            json.dump(config, f, indent=4)

        print("GENERATED")
        return output_folder

    def generate_and_save_depth_replay(self):
        def load_all_frames(data, timestamps):
            frames = {}
            for timestamp in timestamps:
                frames[timestamp] = {}
                images = data[timestamp]
                image_path = images['left']
                frames[timestamp]['left'] = np.load(image_path)
                image_path = images['right']
                frames[timestamp]['right'] = np.load(image_path)
                if 'isp' not in images:
                    continue
                image_path = images['isp']
                frames[timestamp]['isp'] = np.load(image_path)
            return frames

        def process_pointcloud(pcl, depth, color=None):
            pcl[:, 0] = -pcl[:, 0]  # switch because it goes wrong from the camera
            pcl = rotate_pointcloud(pcl, 180, axis="y")

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pcl)

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
            return pcd

        print("Generating NEW outputs for the whole folder")
        print("Lading data for generation")
        timestamps = self.view_info['timestamps']
        data = self.view_info['data']
        config = self.config_json
        calib = self.view_info["calib"]
        if calib is None: raise ValueError("No calibration provided")

        batch_size = 100

        for batch in range(0, len(timestamps), batch_size):
            frames = load_all_frames(data, timestamps[batch:batch + batch_size])
            print("Processing by batches:", len(frames))

            # sending first frame twice because it used to cause some error when using decimation filter
            batch_frames = ([(frames[timestamps[batch]]["left"], frames[timestamps[batch]]["right"], frames[timestamps[batch]].get("isp", frames[timestamps[batch]]["left"]))] +
                            [(frames[t]["left"], frames[t]["right"], frames[t].get("isp", frames[t]["left"]))
                            for t in timestamps[batch:batch + batch_size]])
            replayed = tuple(replay(
                tuple(batch_frames),
                outputs={'depth', 'pcl'},
                calib=calib,
                stereo_config=json.dumps(config)
            ))

            for i in range(1, len(batch_frames)): # dropping first frame as incorrectly processed
                depth = replayed[i]['depth']
                pcl = replayed[i]['pcl']
                timestamp = timestamps[batch + i - 1] # i starts at one for batch frames to skip double but there is no double for frames
                pcl = process_pointcloud(pcl, depth, frames[timestamp].get("isp", None))

                timestamp = timestamp.split(".npy")[0]
                np.save(os.path.join(self.output_folder, f"depth_{timestamp}.npy"), depth)
                colorized_depth, _, _ = colorize_depth(depth, "depth", label=False, min_val=0, max_val=7000)
                cv2.imwrite(os.path.join(self.output_folder, f"depth_{timestamp}.png"), colorized_depth)
                o3d.io.write_point_cloud(os.path.join(self.output_folder, f"pcl_{timestamp}.ply"), pcl)

        with open(os.path.join(self.output_folder, f'config.json'), 'w') as f:
            json.dump(config, f, indent=4)

        print("GENERATED")


    def load_or_generate(self):
        self.output_folder = self.get_or_create_output_folder()

        if not self.depth_in_replay_outputs:
            self.generate_and_save_depth_replay()

        current_timestamp = self.view_info['timestamps'][self.view_info['current_index']]
        print(f"LOADING TIMESTEP: {current_timestamp}")
        self.last_generated_depth = np.load(os.path.join(self.output_folder, f"depth_{current_timestamp}"))
        self.last_generated_pcl_path = os.path.join(self.output_folder, f"pcl_{current_timestamp.split('.npy')[0]}.ply")

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