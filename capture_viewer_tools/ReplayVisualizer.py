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
        self.toplLevel.geometry("2200x1200")

        self.main_frame = tk.Frame(self.toplLevel)
        self.main_frame.grid(row=0, column=0, sticky="nsew")

        max_image_width = 840
        max_image_height = 400
        self.scaled_original_size = calculate_scaled_dimensions(view_info['depth_size'], max_image_width, max_image_height)
        # self.scaled_original_size = view_info['depth_size']

        self.view_info = view_info
        self.current_view = current_view

        self.depth = current_view['depth']
        self.generated_depth = None
        self.pcl_path = None
        self.config_json = None
        self.last_config = None

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

    def process_config(self):
        def config_add_depthai(config_json):
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

        config_json = config_add_depthai(self.config_json)
        self.config_json = config_json
        self.generated_depth = None

    def layout_settings(self, frame, original_config=None):
        if self.last_config is None and original_config is not None:
            current_config = settings2config(original_config)
        elif self.last_config is not None:
            current_config = self.last_config
        else:
            current_config = default_config

        def create_settings_layout(frame):
            def update_label(var, label, form="int"):
                """Update the label when slider is moved."""
                if form == "int":
                    label.config(text=str(int(var.get())))
                elif form == "float":
                    label.config(text=str(round(float(var.get()), 1)))

            def on_generate(config):
                print("GENERATE")
                self.config_json = tkinter_settings_to_config({})
                self.last_config = config

                print(self.config_json)

                self.process_config()
                self.load_or_generate()

            def tkinter_settings_to_config(config):
                if not custom_settings_val.get():
                    config['stereo.setDepthAlign'] = depth_align.get()
                    if profile_preset.get() != 'None':
                        config['stereo.setDefaultProfilePreset'] = profile_preset.get()
                    return config

                if filtering_order_enable.get():  # needs to be the first item for the config to not be initialised in case of wrong filtering order
                    if not check_valid_filtering_order(
                            [decimation_order.get(), median_order.get(), speckle_order.get(), spatial_order.get(),
                             temporal_order.get()]):
                        warning()
                        return config
                    else:
                        config['cfg.postProcessing.filteringOrder'] = get_filter_order(decimation_order.get(),
                                                                                       median_order.get(),
                                                                                       speckle_order.get(),
                                                                                       spatial_order.get(),
                                                                                       temporal_order.get())

                config['stereo.setDepthAlign'] = depth_align.get()
                if profile_preset.get() != 'None':
                    config['stereo.setDefaultProfilePreset'] = profile_preset.get()
                config['stereo.setRectification'] = rectificationBox_val.get()
                config['stereo.setLeftRightCheck'] = LRBox_val.get()

                config['stereo.setExtendedDisparity'] = extendedBox_val.get()
                config['stereo.setSubpixel'] = subpixelBox_val.get()

                if subpixelBox_val.get():
                    config['stereo.setSubpixelFractionalBits'] = fractional_bits_combo.get()

                # FILTERS

                if median_filter_enable.get():
                    config['stereo.initialConfig.setMedianFilter'] = median_val.get()
                else:
                    config['stereo.initialConfig.setMedianFilter'] = "MedianFilter.MEDIAN_OFF"

                # Bilateral filter
                # todo - [1944301021AA992E00] [1.2.2] [9.439] [StereoDepth(2)] [warning] Bilateral filter is deprecated!
                # config['cfg.postProcessing.bilateralSigmaValue'] = bilateral_sigma_val.get()
                # if bilateral_filter_enable.get():
                #     # is this all settings?
                #     pass

                # Brightness filter
                if brightness_filter_enable.get():  # todo can we disable brightness filter?
                    config['cfg.postProcessing.brightnessFilter.maxBrightness'] = max_brightness_slider.get()
                    config['cfg.postProcessing.brightnessFilter.minBrightness'] = min_brightness_slider.get()

                # Speckle filter
                config['cfg.postProcessing.speckleFilter.enable'] = speckle_filter_enable.get()
                if speckle_filter_enable.get():
                    config['cfg.postProcessing.speckleFilter.speckleRange'] = speckle_range_slider.get()
                    config[
                        'cfg.postProcessing.speckleFilter.differenceThreshold'] = speckle_difference_threshold.get()

                # Spacial filter
                config['cfg.postProcessing.spatialFilter.enable'] = spatial_filter_enable.get()
                if spatial_filter_enable.get():
                    config['cfg.postProcessing.spatialFilter.holeFillingRadius'] = hole_filling_radius_slider.get()
                    config['cfg.postProcessing.spatialFilter.numIterations'] = num_iterations_slider.get()
                    config['cfg.postProcessing.spatialFilter.alpha'] = alpha_slider.get()
                    config['cfg.postProcessing.spatialFilter.delta'] = delta_slider.get()

                # Temporal filter
                config['cfg.postProcessing.temporalFilter.enable'] = temporal_filter_enable.get()
                if temporal_filter_enable.get():
                    config['cfg.postProcessing.temporalFilter.alpha'] = temporal_alpha_slider.get()
                    config['cfg.postProcessing.temporalFilter.delta'] = temporal_delta_slider.get()

                # Threshold filter
                if threshold_filter_enable.get():  # todo same as brightness filter
                    config['cfg.postProcessing.thresholdFilter.minRange'] = min_range_val.get()
                    config['cfg.postProcessing.thresholdFilter.maxRange'] = max_range_val.get()

                # Decimation filter
                config['cfg.postProcessing.decimationFilter.decimationFactor'] = decimation_factor_val.get()
                if decimation_filter_enable.get():  # todo decimation filter has no attribute enable
                    config['cfg.postProcessing.decimationFilter.decimationFactor'] = decimation_factor_val.get()
                    config['cfg.postProcessing.decimationFilter.decimationMode'] = handle_dict(
                        decimation_mode_val.get(), decimation_set_dict)  # leave this is correct

                # ADVANCED SETTINGS
                if advanced_settings_enable.get():
                    config["cfg.censusTransform.kernelSize"] = handle_dict(CT_kernel_val.get(), CT_kernel_dict)
                    config["cfg.censusTransform.enableMeanMode"] = mean_mode_enable.get()
                    config["cfg.censusTransform.threshold"] = CT_threshold_val.get()

                    config["cfg.costAggregation.divisionFactor"] = division_factor_val.get()
                    config["cfg.costAggregation.horizontalPenaltyCostP1"] = horizontal_penalty_p1_val.get()
                    config["cfg.costAggregation.horizontalPenaltyCostP2"] = horizontal_penalty_p2_val.get()
                    config["cfg.costAggregation.verticalPenaltyCostP1"] = vertical_penalty_p1_val.get()
                    config["cfg.costAggregation.verticalPenaltyCostP2"] = vertical_penalty_p2_val.get()

                    config["cfg.costMatching.confidenceThreshold"] = confidence_threshold_val.get()
                    config["cfg.costMatching.linearEquationParameters.alpha"] = CM_alpha_val.get()
                    config["cfg.costMatching.linearEquationParameters.beta"] = CM_beta_val.get()
                    config["cfg.costMatching.linearEquationParameters.threshold"] = matching_threshold_val.get()
                return config

            popup_window = frame

            # ----------------------------------- Initialize the UI elements with default values from current_config -----------------------------------
            # Initialize tkinter variables, falling back to default_config if a key is missing in current_config
            depth_align = tk.StringVar(
                value=current_config.get('stereo.setDepthAlign', default_config['stereo.setDepthAlign']))
            profile_preset = tk.StringVar(value=current_config.get('stereo.setDefaultProfilePreset',
                                                                   default_config['stereo.setDefaultProfilePreset']))

            custom_settings_val = tk.BooleanVar(value=False)

            rectificationBox_val = tk.BooleanVar(
                value=current_config.get('stereo.setRectification', True))  # Default to True if not found
            LRBox_val = tk.BooleanVar(
                value=current_config.get('stereo.setLeftRightCheck', default_config['stereo.setLeftRightCheck']))
            extendedBox_val = tk.BooleanVar(
                value=current_config.get('stereo.setExtendedDisparity', default_config['stereo.setExtendedDisparity']))
            subpixelBox_val = tk.BooleanVar(
                value=current_config.get('stereo.setSubpixel', default_config['stereo.setSubpixel']))
            print(current_config)
            fractional_bits = tk.StringVar(value=current_config.get('stereo.setSubpixelFractionalBits',
                                                                 default_config['stereo.setSubpixelFractionalBits']))

            # FILTERS -----------------------------------------------------------------------------------
            filtering_order_enable = tk.BooleanVar(
                value=(True if 'cfg.postProcessing.filteringOrder' in current_config else False))
            if 'cfg.postProcessing.filteringOrder' in current_config:
                initial_filter_order = get_filter_order_back(current_config['cfg.postProcessing.filteringOrder'])
            else:
                initial_filter_order = get_filter_order_back(default_config['cfg.postProcessing.filteringOrder'])

            decimation_order = tk.IntVar(value=initial_filter_order[0])
            median_order = tk.IntVar(value=initial_filter_order[1])
            speckle_order = tk.IntVar(value=initial_filter_order[2])
            spatial_order = tk.IntVar(value=initial_filter_order[3])
            temporal_order = tk.IntVar(value=initial_filter_order[4])

            median_filter_enable = tk.BooleanVar(value=(current_config.get('stereo.initialConfig.setMedianFilter',
                                                                           "MedianFilter.MEDIAN_OFF") != "MedianFilter.MEDIAN_OFF"))
            median_val = tk.StringVar(value=current_config.get('stereo.initialConfig.setMedianFilter',
                                                               default_config['stereo.initialConfig.setMedianFilter']))

            # bilateral_filter_enable = tk.BooleanVar(value=current_config.get('cfg.postProcessing.bilateralFilter.enable', False))
            # bilateral_sigma_val = tk.IntVar(value=current_config.get('cfg.postProcessing.bilateralSigmaValue',
            #                                                          default_config['cfg.postProcessing.bilateralSigmaValue']))

            brightness_filter_enable = tk.BooleanVar(
                value=current_config.get('cfg.postProcessing.brightnessFilter.enable', False))
            min_brightness_slider = tk.IntVar(value=current_config.get('cfg.postProcessing.brightnessFilter.minBrightness',
                                                                       default_config[
                                                                           'cfg.postProcessing.brightnessFilter.minBrightness']))
            max_brightness_slider = tk.IntVar(value=current_config.get('cfg.postProcessing.brightnessFilter.maxBrightness',
                                                                       default_config[
                                                                           'cfg.postProcessing.brightnessFilter.maxBrightness']))

            speckle_filter_enable = tk.BooleanVar(
                value=current_config.get('cfg.postProcessing.speckleFilter.enable', False))
            speckle_range_slider = tk.IntVar(value=current_config.get('cfg.postProcessing.speckleFilter.speckleRange',
                                                                      default_config[
                                                                          'cfg.postProcessing.speckleFilter.speckleRange']))
            speckle_difference_threshold = tk.IntVar(
                value=current_config.get('cfg.postProcessing.speckleFilter.differenceThreshold',
                                         default_config['cfg.postProcessing.speckleFilter.differenceThreshold']))

            spatial_filter_enable = tk.BooleanVar(
                value=current_config.get('cfg.postProcessing.spatialFilter.enable', False))
            hole_filling_radius_slider = tk.IntVar(
                value=current_config.get('cfg.postProcessing.spatialFilter.holeFillingRadius',
                                         default_config['cfg.postProcessing.spatialFilter.holeFillingRadius']))
            num_iterations_slider = tk.IntVar(value=current_config.get('cfg.postProcessing.spatialFilter.numIterations',
                                                                       default_config[
                                                                           'cfg.postProcessing.spatialFilter.numIterations']))
            alpha_slider = tk.DoubleVar(value=current_config.get('cfg.postProcessing.spatialFilter.alpha',
                                                                 default_config['cfg.postProcessing.spatialFilter.alpha']))
            delta_slider = tk.IntVar(value=current_config.get('cfg.postProcessing.spatialFilter.delta',
                                                              default_config['cfg.postProcessing.spatialFilter.delta']))

            temporal_filter_enable = tk.BooleanVar(
                value=current_config.get('cfg.postProcessing.temporalFilter.enable', False))
            temporal_alpha_slider = tk.DoubleVar(value=current_config.get('cfg.postProcessing.temporalFilter.alpha',
                                                                          default_config[
                                                                              'cfg.postProcessing.temporalFilter.alpha']))
            temporal_delta_slider = tk.IntVar(value=current_config.get('cfg.postProcessing.temporalFilter.delta',
                                                                       default_config[
                                                                           'cfg.postProcessing.temporalFilter.delta']))

            threshold_filter_enable = tk.BooleanVar(
                value=(True if 'cfg.postProcessing.thresholdFilter.minRange' in current_config else False))
            min_range_val = tk.IntVar(value=current_config.get('cfg.postProcessing.thresholdFilter.minRange',
                                                               default_config[
                                                                   'cfg.postProcessing.thresholdFilter.minRange']))
            max_range_val = tk.IntVar(value=current_config.get('cfg.postProcessing.thresholdFilter.maxRange',
                                                               default_config[
                                                                   'cfg.postProcessing.thresholdFilter.maxRange']))

            decimation_filter_enable = tk.BooleanVar(
                value=(True if 'cfg.postProcessing.decimationFilter.decimationFactor' in current_config else False))
            decimation_factor_val = tk.IntVar(
                value=current_config.get('cfg.postProcessing.decimationFilter.decimationFactor',
                                         default_config['cfg.postProcessing.decimationFilter.decimationFactor']))
            decimation_mode_val = tk.StringVar(value=handle_dict(
                current_config.get('cfg.postProcessing.decimationFilter.decimationMode',
                                   default_config['cfg.postProcessing.decimationFilter.decimationMode']),
                decimation_set_dict, reverse=True))

            # ----------------------------------------------------------------- BUTTONS -------------------------------------------------------------

            current_row = 0

            # Add radiobuttons for left/right choice with a label
            ttk.Label(popup_window, text="setDepthAlign").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
            left_radiobutton = ttk.Radiobutton(popup_window, text="Left", variable=depth_align,
                                               value="CameraBoardSocket.LEFT")
            right_radiobutton = ttk.Radiobutton(popup_window, text="Right", variable=depth_align,
                                                value="CameraBoardSocket.RIGHT")
            rec_left_radiobutton = ttk.Radiobutton(popup_window, text="Rec Left", variable=depth_align,
                                                   value='StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT')
            rec_right_radiobutton = ttk.Radiobutton(popup_window, text="Rec Right", variable=depth_align,
                                                    value='StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT')
            rgb_radiobutton = ttk.Radiobutton(popup_window, text="RGB", variable=depth_align,
                                              value="CameraBoardSocket.CAM_A")
            left_radiobutton.grid(row=current_row, column=1, padx=10, pady=5, sticky="w")
            right_radiobutton.grid(row=current_row, column=2, padx=10, pady=5, sticky="w")
            rec_left_radiobutton.grid(row=current_row, column=3, padx=10, pady=5, sticky="w")
            rec_right_radiobutton.grid(row=current_row, column=4, padx=10, pady=5, sticky="w")
            rgb_radiobutton.grid(row=current_row, column=5, padx=10, pady=5, sticky="w")
            current_row += 1

            #
            ttk.Label(popup_window, text="setDefaultProfilePreset").grid(row=current_row, column=0, padx=10, pady=10,
                                                                         sticky="w")
            Hdef_radiobutton = ttk.Radiobutton(popup_window, text="DEFAULT", variable=profile_preset,
                                               value="node.StereoDepth.PresetMode.DEFAULT")
            HA_radiobutton = ttk.Radiobutton(popup_window, text="HIGH_ACCURACY", variable=profile_preset,
                                             value="node.StereoDepth.PresetMode.HIGH_ACCURACY")
            HD_radiobutton = ttk.Radiobutton(popup_window, text="HIGH_DENSITY", variable=profile_preset,
                                             value="node.StereoDepth.PresetMode.HIGH_DENSITY")
            HR_radiobutton = ttk.Radiobutton(popup_window, text="ROBOTICS", variable=profile_preset,
                                             value="node.StereoDepth.PresetMode.ROBOTICS")
            HDE_radiobutton = ttk.Radiobutton(popup_window, text="HIGH_DETAIL", variable=profile_preset,
                                              value="node.StereoDepth.PresetMode.HIGH_DETAIL")
            HF_radiobutton = ttk.Radiobutton(popup_window, text="FACE", variable=profile_preset,
                                             value="node.StereoDepth.PresetMode.FACE")
            HN_radiobutton = ttk.Radiobutton(popup_window, text="None", variable=profile_preset, value="None")
            Hdef_radiobutton.grid(row=current_row, column=1, padx=10, pady=5, sticky="w")
            HA_radiobutton.grid(row=current_row, column=2, padx=10, pady=5, sticky="w")
            HD_radiobutton.grid(row=current_row, column=3, padx=10, pady=5, sticky="w")
            HR_radiobutton.grid(row=current_row, column=4, padx=10, pady=5, sticky="w")
            HDE_radiobutton.grid(row=current_row, column=5, padx=10, pady=5, sticky="w")
            HF_radiobutton.grid(row=current_row, column=6, padx=10, pady=5, sticky="w")
            HN_radiobutton.grid(row=current_row, column=7, padx=10, pady=5, sticky="w")
            current_row += 1

            def toggle_custom_frame_settings():
                toggle_frame_settings(custom_settings_val, custom_settings_frame)
                if custom_settings_val.get():
                    toggle_frame_settings(decimation_filter_enable, decimation_frame)
                    toggle_frame_settings(median_filter_enable, median_frame)
                    toggle_frame_settings(speckle_filter_enable, speckle_frame)
                    toggle_frame_settings(spatial_filter_enable, spatial_frame)
                    toggle_frame_settings(temporal_filter_enable, temporal_frame)
                    toggle_frame_settings(threshold_filter_enable, threshold_frame)
                    # toggle_frame_settings(bilateral_filter_enable, bilateral_frame)
                    toggle_frame_settings(brightness_filter_enable, brightness_frame)
                    toggle_frame_settings(filtering_order_enable, order_frame)

            def toggle_frame_settings(frame_on, frame):
                # Enable or disable all frames and their widgets based on the checkbox state
                if frame_on.get():
                    # Enable custom settings and all nested frames/widgets
                    enable_frame_widgets(frame, True)
                else:
                    # Disable custom settings and all nested frames/widgets
                    enable_frame_widgets(frame, False)

            def enable_frame_widgets(frame, state):
                """Recursively enable/disable all widgets in the frame, including nested frames."""
                for widget in frame.winfo_children():
                    if isinstance(widget, ttk.LabelFrame):  # If widget is a frame, call the function recursively
                        enable_frame_widgets(widget, state)
                        widget.state(['!disabled'] if state else ['disabled'])
                    else:
                        widget.state(['!disabled'] if state else ['disabled'])
                frame.state(['!disabled'] if state else ['disabled'])

            #
            ttk.Label(popup_window, text="Use Custom Settings").grid(row=current_row, column=0, padx=10, pady=10,
                                                                     sticky="w")
            cust_box = ttk.Checkbutton(popup_window, variable=custom_settings_val, command=toggle_custom_frame_settings)
            cust_box.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
            current_row += 1

            custom_settings_frame = ttk.LabelFrame(popup_window, text="Custom Settings", padding=(10, 10))
            custom_settings_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

            # # Add checkbuttons for rectification
            ttk.Label(custom_settings_frame, text="setRectification").grid(row=current_row, column=0, padx=10, pady=10,
                                                                           sticky="w")
            recbox = ttk.Checkbutton(custom_settings_frame, variable=rectificationBox_val)
            recbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
            current_row += 1

            # LR CHECK
            ttk.Label(custom_settings_frame, text="setLRcheck").grid(row=current_row, column=0, padx=10, pady=10,
                                                                     sticky="w")
            LRbox = ttk.Checkbutton(custom_settings_frame, variable=LRBox_val)
            LRbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
            current_row += 1

            # EXTENDED DISPARITY
            ttk.Label(custom_settings_frame, text="setExtendedDisparity").grid(row=current_row, column=0, padx=10, pady=10,
                                                                               sticky="w")
            extbox = ttk.Checkbutton(custom_settings_frame, variable=extendedBox_val)
            extbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
            current_row += 1

            # SetSubpixel Label and Checkbox
            ttk.Label(custom_settings_frame, text="setSubpixel").grid(row=current_row, column=0, padx=10, pady=10,
                                                                      sticky="w")
            subbox = ttk.Checkbutton(custom_settings_frame, variable=subpixelBox_val)
            subbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")

            # Add subpixelFractionalBits combobox
            ttk.Label(custom_settings_frame, text="subpixelFractionalBits").grid(row=current_row, column=2, padx=10,
                                                                                 pady=10, sticky="w")
            fractional_bits_combo = ttk.Combobox(custom_settings_frame, textvariable=fractional_bits, values=["3", "4", "5"],
                                                 state="readonly")
            fractional_bits_combo.grid(row=current_row, column=3, padx=10, pady=10, sticky="e")

            current_row += 1  # Increment the row

            # Decimation Filter Enable
            ttk.Label(custom_settings_frame, text="Decimation Filter Enable").grid(row=current_row, column=0, padx=10,
                                                                                   pady=10, sticky="w")
            decimation_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=decimation_filter_enable,
                                                         command=lambda: toggle_frame_settings(decimation_filter_enable,
                                                                                               decimation_frame))
            decimation_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")

            current_row += 1  # Increment the row

            # Decimation Filter Group (in a LabelFrame with a black border)
            decimation_frame = ttk.LabelFrame(custom_settings_frame, text="Decimation Filter", padding=(10, 10))
            decimation_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

            # Decimation Filter (Factor and Mode on one row)
            ttk.Label(decimation_frame, text="Factor").grid(row=1, column=0, padx=10, pady=10, sticky="w")
            decimation_factor_label = ttk.Label(decimation_frame)
            decimation_factor_label.grid(row=1, column=2, padx=10, pady=10, sticky="w")
            decimation_factor_dropdown = ttk.Combobox(decimation_frame, values=[1, 2, 3, 4],
                                                      textvariable=decimation_factor_val,
                                                      state="readonly")
            decimation_factor_dropdown.grid(row=1, column=1, padx=10, pady=10, sticky="e")
            ttk.Label(decimation_frame, text="Mode").grid(row=1, column=3, padx=10, pady=10, sticky="w")
            decimation_mode_dropdown = ttk.Combobox(decimation_frame, textvariable=decimation_mode_val, values=[
                'NON_ZERO_MEAN',
                'NON_ZERO_MEDIAN',
                'PIXEL_SKIPPING'
            ], state="readonly")
            decimation_mode_dropdown.grid(row=1, column=4, padx=10, pady=10, sticky="e")

            current_row += 1

            # Median Filter
            ttk.Label(custom_settings_frame, text="Median Enable").grid(row=current_row, column=0, padx=10, pady=10,
                                                                        sticky="w")
            median_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=median_filter_enable,
                                                     command=lambda: toggle_frame_settings(median_filter_enable,
                                                                                           median_frame))
            median_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
            current_row += 1

            # Median Filter Frame
            median_frame = ttk.LabelFrame(custom_settings_frame, text="Median Filter", padding=(10, 10))
            median_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

            ttk.Label(median_frame, text="Median").grid(row=0, column=0, padx=10, pady=10, sticky="w")
            # median_off_button = ttk.Radiobutton(median_frame, text="MEDIAN_OFF", variable=median_val, value="MEDIAN_OFF")
            median_3_button = ttk.Radiobutton(median_frame, text="MEDIAN_3x3", variable=median_val,
                                              value="MedianFilter.KERNEL_3x3")
            median_5_button = ttk.Radiobutton(median_frame, text="MEDIAN_5x5", variable=median_val,
                                              value="MedianFilter.KERNEL_5x5")
            median_7_button = ttk.Radiobutton(median_frame, text="MEDIAN_7x7", variable=median_val,
                                              value="MedianFilter.KERNEL_7x7")
            # median_off_button.grid(row=current_row, column=1, padx=10, pady=5, sticky="w")
            median_3_button.grid(row=0, column=1, padx=10, pady=5, sticky="w")
            median_5_button.grid(row=0, column=2, padx=10, pady=5, sticky="w")
            median_7_button.grid(row=0, column=3, padx=10, pady=5, sticky="w")
            current_row += 1

            # Speckle Filter Enable
            ttk.Label(custom_settings_frame, text="Speckle Filter Enable").grid(row=current_row, column=0, padx=10, pady=10,
                                                                                sticky="w")
            speckle_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=speckle_filter_enable,
                                                      command=lambda: toggle_frame_settings(speckle_filter_enable,
                                                                                            speckle_frame))
            speckle_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
            current_row += 1

            # Speckle Filter Frame
            speckle_frame = ttk.LabelFrame(custom_settings_frame, text="Speckle Filter", padding=(10, 10))
            speckle_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

            # Speckle Range
            ttk.Label(speckle_frame, text="Speckle Range").grid(row=1, column=0, padx=10, pady=10, sticky="w")
            speckle_range_label = ttk.Label(speckle_frame, text=str(speckle_range_slider.get()))
            speckle_range_spinbox = ttk.Spinbox(speckle_frame, from_=0, to=256, textvariable=speckle_range_slider,
                                                command=lambda: update_label(speckle_range_slider, speckle_range_label))
            speckle_range_spinbox.grid(row=1, column=1, padx=10, pady=10, sticky="e")
            ttk.Label(speckle_frame, text="(0, 256)").grid(row=1, column=2, padx=10, pady=10, sticky="w")
            current_row += 1

            # Speckle Difference Threshold
            ttk.Label(speckle_frame, text="Speckle Difference Threshold").grid(row=2, column=0, padx=10, pady=10,
                                                                               sticky="w")
            speckle_difference_label = ttk.Label(speckle_frame, text=str(speckle_difference_threshold.get()))
            speckle_difference_spinbox = ttk.Spinbox(speckle_frame, from_=0, to=256,
                                                     textvariable=speckle_difference_threshold,
                                                     command=lambda: update_label(speckle_difference_threshold,
                                                                                  speckle_difference_label))
            speckle_difference_spinbox.grid(row=2, column=1, padx=10, pady=10, sticky="e")
            ttk.Label(speckle_frame, text="(0, 256)").grid(row=2, column=2, padx=10, pady=10, sticky="w")
            current_row += 1

            # Spatial Filter Enable
            ttk.Label(custom_settings_frame, text="Spatial Filter Enable").grid(row=current_row, column=0, padx=10, pady=10,
                                                                                sticky="w")
            spatial_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=spatial_filter_enable,
                                                      command=lambda: toggle_frame_settings(spatial_filter_enable,
                                                                                            spatial_frame))
            spatial_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
            current_row += 1

            # Spatial Filter Frame
            spatial_frame = ttk.LabelFrame(custom_settings_frame, text="Spatial Filter", padding=(10, 10))
            spatial_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

            # Spatial Filter Settings
            ttk.Label(spatial_frame, text="Hole Filling Radius").grid(row=1, column=0, padx=10, pady=10, sticky="w")
            hole_filling_radius_label = ttk.Label(spatial_frame, text=str(hole_filling_radius_slider.get()))
            hole_filling_radius_spinbox = ttk.Spinbox(spatial_frame, from_=0, to=255,
                                                      textvariable=hole_filling_radius_slider,
                                                      command=lambda: update_label(hole_filling_radius_slider,
                                                                                   hole_filling_radius_label))
            hole_filling_radius_spinbox.grid(row=1, column=1, padx=10, pady=10, sticky="e")
            ttk.Label(spatial_frame, text="(0, 255)").grid(row=1, column=2, padx=10, pady=10, sticky="w")

            ttk.Label(spatial_frame, text="Num Iterations").grid(row=1, column=3, padx=10, pady=10, sticky="w")
            num_iterations_label = ttk.Label(spatial_frame, text=str(num_iterations_slider.get()))
            num_iterations_label.grid(row=1, column=5, padx=10, pady=10, sticky="w")
            ttk.Scale(spatial_frame, from_=1, to=10, variable=num_iterations_slider, orient="horizontal",
                      command=lambda x: update_label(num_iterations_slider, num_iterations_label)).grid(row=1, column=4,
                                                                                                        padx=10, pady=10,
                                                                                                        sticky="e")
            current_row += 1

            ttk.Label(spatial_frame, text="Alpha").grid(row=2, column=0, padx=10, pady=10, sticky="w")
            alpha_label = ttk.Label(spatial_frame, text=str(alpha_slider.get()))
            alpha_label.grid(row=2, column=2, padx=10, pady=10, sticky="w")
            ttk.Scale(spatial_frame, from_=0, to=1, variable=alpha_slider, orient="horizontal",
                      command=lambda x: update_label(alpha_slider, alpha_label, form="float")).grid(row=2, column=1,
                                                                                                    padx=10, pady=10,
                                                                                                    sticky="e")

            ttk.Label(spatial_frame, text="Delta").grid(row=2, column=3, padx=10, pady=10, sticky="w")
            delta_label = ttk.Label(spatial_frame, text=str(delta_slider.get()))
            delta_spinbox = ttk.Spinbox(spatial_frame, from_=0, to=255, textvariable=delta_slider,
                                        command=lambda: update_label(delta_slider, delta_label))
            delta_spinbox.grid(row=2, column=4, padx=10, pady=10, sticky="e")
            ttk.Label(spatial_frame, text="(0, 255)").grid(row=2, column=5, padx=10, pady=10, sticky="w")

            current_row += 1

            # Temporal Filter Frame

            # # Temporal filter
            # config['cfg.postProcessing.temporalFilter.enable'] = temporal_filter_enable.get()
            # if spatial_filter_enable.get():
            #     config['cfg.postProcessing.temporalFilter.alpha'] = temporal_alpha_slider.get()
            #     config['cfg.postProcessing.temporalFilter.delta'] = temporal_delta_slider.get()

            ttk.Label(custom_settings_frame, text="Temporal Filter Enable").grid(row=current_row, column=0, padx=10,
                                                                                 pady=10, sticky="w")
            temporal_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=temporal_filter_enable,
                                                       command=lambda: toggle_frame_settings(temporal_filter_enable,
                                                                                             temporal_frame))
            temporal_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
            current_row += 1

            # Spatial Filter Frame
            temporal_frame = ttk.LabelFrame(custom_settings_frame, text="Temporal Filter", padding=(10, 10))
            temporal_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

            # Temporal Filter Enable
            ttk.Label(temporal_frame, text="Alpha").grid(row=2, column=0, padx=10, pady=10, sticky="w")
            temporal_alpha_label = ttk.Label(temporal_frame, text=str(temporal_alpha_slider.get()))
            temporal_alpha_label.grid(row=2, column=2, padx=10, pady=10, sticky="w")
            ttk.Scale(temporal_frame, from_=0, to=1, variable=temporal_alpha_slider, orient="horizontal",
                      command=lambda x: update_label(temporal_alpha_slider, temporal_alpha_label, form="float")).grid(row=2,
                                                                                                                      column=1,
                                                                                                                      padx=10,
                                                                                                                      pady=10,
                                                                                                                      sticky="e")

            ttk.Label(temporal_frame, text="Delta").grid(row=2, column=3, padx=10, pady=10, sticky="w")
            temporal_delta_label = ttk.Label(temporal_frame, text=str(temporal_delta_slider.get()))
            delta_spinbox = ttk.Spinbox(temporal_frame, from_=0, to=255, textvariable=temporal_delta_slider,
                                        command=lambda: update_label(temporal_delta_slider, temporal_delta_label))
            delta_spinbox.grid(row=2, column=4, padx=10, pady=10, sticky="e")
            ttk.Label(temporal_frame, text="(0, 255)").grid(row=2, column=5, padx=10, pady=10, sticky="w")

            current_row += 1

            # Threshold Filter Enable
            ttk.Label(custom_settings_frame, text="Threshold Filter Enable").grid(row=current_row, column=0, padx=10,
                                                                                  pady=10, sticky="w")
            threshold_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=threshold_filter_enable,
                                                        command=lambda: toggle_frame_settings(threshold_filter_enable,
                                                                                              threshold_frame))
            threshold_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
            current_row += 1

            # Threshold Filter Frame
            threshold_frame = ttk.LabelFrame(custom_settings_frame, text="Threshold Filter", padding=(10, 10))
            threshold_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

            # Threshold Filter Min and Max with Spinboxes
            ttk.Label(threshold_frame, text="Min").grid(row=1, column=0, padx=10, pady=10, sticky="w")
            min_range_spinbox = ttk.Spinbox(threshold_frame, from_=0, to=int(max_range_val.get()),
                                            textvariable=min_range_val,
                                            width=10)
            min_range_spinbox.grid(row=1, column=1, padx=10, pady=10, sticky="e")
            ttk.Label(threshold_frame, text="(0, " + str(max_range_val.get()) + ")").grid(row=1, column=2, padx=10, pady=10,
                                                                                          sticky="w")

            ttk.Label(threshold_frame, text="Max").grid(row=1, column=3, padx=10, pady=10, sticky="w")
            max_range_spinbox = ttk.Spinbox(threshold_frame, from_=int(min_range_val.get()), to=65535,
                                            textvariable=max_range_val, width=10)
            max_range_spinbox.grid(row=1, column=4, padx=10, pady=10, sticky="e")
            ttk.Label(threshold_frame, text="(" + str(min_range_val.get()) + ", 65535)").grid(row=1, column=5, padx=10,
                                                                                              pady=10,
                                                                                              sticky="w")

            current_row += 1

            # # Bilateral Filter Enable
            # ttk.Label(custom_settings_frame, text="Bilateral Filter Enable (NOT TESTED IN GUI)").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
            # bilateral_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=bilateral_filter_enable, command=lambda: toggle_frame_settings(bilateral_filter_enable, bilateral_frame))
            # bilateral_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
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
            # bilateral_sigma_slider.grid(row=1, column=1, padx=10, pady=10, sticky="e")
            # current_row += 1

            # Brightness Filter Enable
            ttk.Label(custom_settings_frame, text="Brightness Filter Enable (NOT TESTED IN GUI)").grid(row=current_row,
                                                                                                       column=0, padx=10,
                                                                                                       pady=10, sticky="w")
            brightness_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=brightness_filter_enable,
                                                         command=lambda: toggle_frame_settings(brightness_filter_enable,
                                                                                               brightness_frame))
            brightness_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
            current_row += 1

            # Brightness Filter Frame
            brightness_frame = ttk.LabelFrame(custom_settings_frame, text="Brightness Filter", padding=(10, 10))
            brightness_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

            # Brightness Filter Sliders
            ttk.Label(brightness_frame, text="Min Brightness").grid(row=1, column=0, padx=10, pady=10, sticky="w")
            min_brightness_label = ttk.Label(brightness_frame, text=str(min_brightness_slider.get()))
            min_brightness_label.grid(row=1, column=2, padx=10, pady=10, sticky="w")
            ttk.Scale(brightness_frame, from_=0, to=int(max_brightness_slider.get()), variable=min_brightness_slider,
                      orient="horizontal",
                      command=lambda x: update_label(min_brightness_slider, min_brightness_label)).grid(row=1, column=1,
                                                                                                        padx=10, pady=10,
                                                                                                        sticky="e")

            ttk.Label(brightness_frame, text="Max Brightness").grid(row=1, column=3, padx=10, pady=10, sticky="w")
            max_brightness_label = ttk.Label(brightness_frame, text=str(max_brightness_slider.get()))
            max_brightness_label.grid(row=1, column=5, padx=10, pady=10, sticky="w")
            ttk.Scale(brightness_frame, from_=int(min_brightness_slider.get()), to=255, variable=max_brightness_slider,
                      orient="horizontal",
                      command=lambda x: update_label(max_brightness_slider, max_brightness_label)).grid(row=1, column=4,
                                                                                                        padx=10, pady=10,
                                                                                                        sticky="e")
            current_row += 1

            # Filter Order Enable
            ttk.Label(custom_settings_frame, text="Filter Order Selection Enable (NOT TESTED IN GUI)").grid(row=current_row,
                                                                                                            column=0,
                                                                                                            padx=10,
                                                                                                            pady=10,
                                                                                                            sticky="w")
            send_filter_order_checkbox = ttk.Checkbutton(custom_settings_frame, variable=filtering_order_enable,
                                                         command=lambda: toggle_frame_settings(filtering_order_enable,
                                                                                               order_frame))
            send_filter_order_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
            current_row += 1

            # Filter order
            order_frame = ttk.LabelFrame(custom_settings_frame, text="Filtering Order", padding=(10, 10))
            order_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

            row = 1
            ttk.Label(order_frame, text="Decimation").grid(row=row, column=0, padx=10, pady=10, sticky="w")
            median_order_label = ttk.Label(order_frame)
            median_order_label.grid(row=row, column=2, padx=10, pady=10, sticky="w")
            median_order_dropdown = ttk.Combobox(order_frame, values=[1, 2, 3, 4, 5], textvariable=decimation_order,
                                                 state="readonly")
            median_order_dropdown.grid(row=row, column=1, padx=10, pady=10, sticky="e")

            row = 2
            ttk.Label(order_frame, text="Median").grid(row=row, column=0, padx=10, pady=10, sticky="w")
            median_order_label = ttk.Label(order_frame)
            median_order_label.grid(row=row, column=2, padx=10, pady=10, sticky="w")
            median_order_dropdown = ttk.Combobox(order_frame, values=[1, 2, 3, 4, 5], textvariable=median_order,
                                                 state="readonly")
            median_order_dropdown.grid(row=row, column=1, padx=10, pady=10, sticky="e")

            row = 3
            ttk.Label(order_frame, text="Speckle").grid(row=row, column=0, padx=10, pady=10, sticky="w")
            median_order_label = ttk.Label(order_frame)
            median_order_label.grid(row=row, column=2, padx=10, pady=10, sticky="w")
            median_order_dropdown = ttk.Combobox(order_frame, values=[1, 2, 3, 4, 5], textvariable=speckle_order,
                                                 state="readonly")
            median_order_dropdown.grid(row=row, column=1, padx=10, pady=10, sticky="e")

            row = 4
            ttk.Label(order_frame, text="Spatial").grid(row=row, column=0, padx=10, pady=10, sticky="w")
            median_order_label = ttk.Label(order_frame)
            median_order_label.grid(row=row, column=2, padx=10, pady=10, sticky="w")
            median_order_dropdown = ttk.Combobox(order_frame, values=[1, 2, 3, 4, 5], textvariable=spatial_order,
                                                 state="readonly")
            median_order_dropdown.grid(row=row, column=1, padx=10, pady=10, sticky="e")

            row = 5
            ttk.Label(order_frame, text="Temporal").grid(row=row, column=0, padx=10, pady=10, sticky="w")
            median_order_label = ttk.Label(order_frame)
            median_order_label.grid(row=row, column=2, padx=10, pady=10, sticky="w")
            median_order_dropdown = ttk.Combobox(order_frame, values=[1, 2, 3, 4, 5], textvariable=temporal_order,
                                                 state="readonly")
            median_order_dropdown.grid(row=row, column=1, padx=10, pady=10, sticky="e")

            current_row += 1

            # STEREO ALGORITHM ADVANCED SETTINGS -------------------------------------------------------------------------------
            advanced_settings_enable = tk.BooleanVar(value=False)
            mean_mode_enable = tk.BooleanVar(value=True)
            CT_kernel_val = tk.StringVar(value='KERNEL_AUTO')
            CT_threshold_val = tk.IntVar(value=0)
            division_factor_val = tk.IntVar(value=1)
            horizontal_penalty_p1_val = tk.IntVar(value=250)
            horizontal_penalty_p2_val = tk.IntVar(value=500)
            vertical_penalty_p1_val = tk.IntVar(value=250)
            vertical_penalty_p2_val = tk.IntVar(value=500)
            confidence_threshold_val = tk.IntVar(value=245)
            CM_alpha_val = tk.IntVar(value=0)
            CM_beta_val = tk.IntVar(value=2)
            matching_threshold_val = tk.IntVar(value=127)

            def toggle_advanced_settings():
                if advanced_settings_enable.get():
                    enable_frame_widgets(advanced_stereo_setting_frame, True)
                else:
                    enable_frame_widgets(advanced_stereo_setting_frame, False)

            ttk.Label(popup_window, text="Enable Advanced Settings (I know what I'm doing)").grid(row=current_row, column=0,
                                                                                                  padx=10, pady=10,
                                                                                                  sticky="w")
            advanced_settings_checkbox = ttk.Checkbutton(popup_window, variable=advanced_settings_enable,
                                                         command=toggle_advanced_settings)
            advanced_settings_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")

            current_row += 1

            advanced_stereo_setting_frame = ttk.LabelFrame(popup_window, text="Advanced Settings", padding=(10, 10))
            advanced_stereo_setting_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

            ttk.Label(advanced_stereo_setting_frame, text="censusTransform.kernelSize").grid(row=1, column=0, padx=10,
                                                                                             pady=10, sticky="w")
            CT_kernel_dropdown = ttk.Combobox(advanced_stereo_setting_frame, textvariable=CT_kernel_val, values=[
                'KERNEL_AUTO',
                'KERNEL_5x5',
                'KERNEL_7x7',
                'KERNEL_7x9'
            ], state="readonly")
            CT_kernel_dropdown.grid(row=1, column=1, padx=10, pady=10, sticky="e")

            ttk.Label(advanced_stereo_setting_frame, text="censusTransform.enableMeanMode").grid(row=2, column=0, padx=10,
                                                                                                 pady=10,
                                                                                                 sticky="w")
            mean_mode_checkbox = ttk.Checkbutton(advanced_stereo_setting_frame, variable=mean_mode_enable)
            mean_mode_checkbox.grid(row=2, column=1, padx=10, pady=10, sticky="e")

            ttk.Label(advanced_stereo_setting_frame, text="censusTransform.threshold").grid(row=3, column=0, padx=10,
                                                                                            pady=10,
                                                                                            sticky="w")
            threshold_spinbox = ttk.Spinbox(advanced_stereo_setting_frame, from_=0, to=255,
                                            textvariable=CT_threshold_val, width=10  # You can adjust the width as needed
                                            )
            threshold_spinbox.grid(row=3, column=1, padx=10, pady=10, sticky="e")
            ttk.Label(advanced_stereo_setting_frame, text="(0, 255)").grid(row=3, column=2, padx=10, pady=10, sticky="w")

            ttk.Label(advanced_stereo_setting_frame, text="costAggregation.divisionFactor").grid(row=4, column=0, padx=10,
                                                                                                 pady=10, sticky="w")
            division_factor_spinbox = ttk.Spinbox(advanced_stereo_setting_frame, from_=1, to=100,
                                                  textvariable=division_factor_val, width=10)
            division_factor_spinbox.grid(row=4, column=1, padx=10, pady=10, sticky="e")
            ttk.Label(advanced_stereo_setting_frame, text="(1, 100)").grid(row=4, column=2, padx=10, pady=10, sticky="w")

            ttk.Label(advanced_stereo_setting_frame, text="costAggregation.horizontalPenaltyCostP1").grid(row=5, column=0,
                                                                                                          padx=10, pady=10,
                                                                                                          sticky="w")
            horizontal_penalty_p1_spinbox = ttk.Spinbox(advanced_stereo_setting_frame, from_=0, to=500,
                                                        textvariable=horizontal_penalty_p1_val, increment=10, width=10)
            horizontal_penalty_p1_spinbox.grid(row=5, column=1, padx=10, pady=10, sticky="e")
            ttk.Label(advanced_stereo_setting_frame, text="(0, 500)").grid(row=5, column=2, padx=10, pady=10, sticky="w")

            ttk.Label(advanced_stereo_setting_frame, text="costAggregation.horizontalPenaltyCostP2").grid(row=6, column=0,
                                                                                                          padx=10, pady=10,
                                                                                                          sticky="w")
            horizontal_penalty_p2_spinbox = ttk.Spinbox(advanced_stereo_setting_frame, from_=0, to=1000,
                                                        textvariable=horizontal_penalty_p2_val, increment=10, width=10)
            horizontal_penalty_p2_spinbox.grid(row=6, column=1, padx=10, pady=10, sticky="e")
            ttk.Label(advanced_stereo_setting_frame, text="(0, 1000)").grid(row=6, column=2, padx=10, pady=10, sticky="w")

            ttk.Label(advanced_stereo_setting_frame, text="costAggregation.verticalPenaltyCostP1").grid(row=7, column=0,
                                                                                                        padx=10, pady=10,
                                                                                                        sticky="w")
            vertical_penalty_p1_spinbox = ttk.Spinbox(advanced_stereo_setting_frame, from_=0, to=500,
                                                      textvariable=vertical_penalty_p1_val, increment=10, width=10)
            vertical_penalty_p1_spinbox.grid(row=7, column=1, padx=10, pady=10, sticky="e")
            ttk.Label(advanced_stereo_setting_frame, text="(0, 500)").grid(row=7, column=2, padx=10, pady=10, sticky="w")

            ttk.Label(advanced_stereo_setting_frame, text="costAggregation.verticalPenaltyCostP2").grid(row=8, column=0,
                                                                                                        padx=10, pady=10,
                                                                                                        sticky="w")
            vertical_penalty_p2_spinbox = ttk.Spinbox(advanced_stereo_setting_frame, from_=0, to=1000,
                                                      textvariable=vertical_penalty_p2_val, increment=10, width=10)
            vertical_penalty_p2_spinbox.grid(row=8, column=1, padx=10, pady=10, sticky="e")
            ttk.Label(advanced_stereo_setting_frame, text="(0, 1000)").grid(row=8, column=2, padx=10, pady=10, sticky="w")

            ttk.Label(advanced_stereo_setting_frame, text="costMatching.confidenceThreshold").grid(row=9, column=0, padx=10,
                                                                                                   pady=10, sticky="w")
            confidence_threshold_spinbox = ttk.Spinbox(advanced_stereo_setting_frame, from_=0, to=255,
                                                       textvariable=confidence_threshold_val, width=10)
            confidence_threshold_spinbox.grid(row=9, column=1, padx=10, pady=10, sticky="e")
            ttk.Label(advanced_stereo_setting_frame, text="(0, 255)").grid(row=9, column=2, padx=10, pady=10, sticky="w")

            # Alpha Slider
            ttk.Label(advanced_stereo_setting_frame, text="costMatching.linearEquationParameters.alpha").grid(row=10,
                                                                                                              column=0,
                                                                                                              padx=10,
                                                                                                              pady=10,
                                                                                                              sticky="w")
            CM_alpha_slider = ttk.Scale(advanced_stereo_setting_frame, from_=0, to=10, variable=CM_alpha_val,
                                        orient="horizontal",
                                        command=lambda x: update_label(CM_alpha_slider, CM_alpha_label))
            CM_alpha_slider.grid(row=10, column=1, padx=10, pady=10, sticky="ew")
            CM_alpha_label = ttk.Label(advanced_stereo_setting_frame, text=str(int(CM_alpha_val.get())))
            CM_alpha_label.grid(row=10, column=2, padx=10, pady=10, sticky="w")

            # Beta Slider
            ttk.Label(advanced_stereo_setting_frame, text="costMatching.linearEquationParameters.beta").grid(row=11,
                                                                                                             column=0,
                                                                                                             padx=10,
                                                                                                             pady=10,
                                                                                                             sticky="w")
            CM_beta_slider = ttk.Scale(advanced_stereo_setting_frame, from_=0, to=10, variable=CM_beta_val,
                                       orient="horizontal",
                                       command=lambda x: update_label(CM_beta_slider, CM_beta_label))
            CM_beta_slider.grid(row=11, column=1, padx=10, pady=10, sticky="ew")
            CM_beta_label = ttk.Label(advanced_stereo_setting_frame, text=str(int(CM_beta_val.get())))
            CM_beta_label.grid(row=11, column=2, padx=10, pady=10, sticky="w")

            ttk.Label(advanced_stereo_setting_frame, text="costMatching.linearEquationParameters.threshold").grid(row=12,
                                                                                                                  column=0,
                                                                                                                  padx=10,
                                                                                                                  pady=10,
                                                                                                                  sticky="w")
            matching_threshold_spinbox = ttk.Spinbox(advanced_stereo_setting_frame, from_=0, to=255,
                                                     textvariable=matching_threshold_val, width=10)
            matching_threshold_spinbox.grid(row=12, column=1, padx=10, pady=10, sticky="e")
            ttk.Label(advanced_stereo_setting_frame, text="(0, 255)").grid(row=12, column=2, padx=10, pady=10, sticky="w")

            current_row += 1

            # ------------------------------------------------------------------------------------------------------------------------------

            # Add a "GENERATE" button at the bottom
            generate_button = tk.Button(popup_window, text="GENERATE", bg="green2", activebackground="green4",
                                        command=lambda: on_generate(self.config_json))
            generate_button.grid(row=current_row, column=0, columnspan=2, pady=20)

            # Add a "GENERATE" button at the bottom
            generate_button = tk.Button(popup_window, text="GENERATE", bg="blue2", activebackground="blue4",
                                        command=lambda: on_generate(self.config_json))  # todo
            generate_button.grid(row=current_row, column=1, columnspan=2, pady=20)
            current_row += 1
            # ------------------------------------------------------------------------------------------------------------------------------

            toggle_custom_frame_settings()
            toggle_advanced_settings()  # turn off by default

            # popup_window.grab_set()  # Make the window modal (disable interaction with the main window)
            # popup_window.wait_window()  # Wait for the popup window to be destroyed

        create_settings_layout(frame)

    # Scroll handler function
    def on_mouse_wheel(self, event):
        if event.delta > 0:
            self.canvas.yview_scroll(-1, "units")  # Scroll up
        elif event.delta < 0:
            self.canvas.yview_scroll(1, "units")  # Scroll down

    def on_mouse_wheel_up(self, event):
        self.canvas.yview_scroll(-1, "units")

    def on_mouse_wheel_down(self, event):
        self.canvas.yview_scroll(1, "units")

    def create_layout(self):
        def show_pointcloud():
            if self.pcl_path is None: return False
            script_directory = os.path.dirname(os.path.abspath(__file__))
            visualize_pointcloud_path = os.path.join(script_directory, 'visualize_pointcloud.py')
            subprocess.Popen(['python', visualize_pointcloud_path, self.pcl_path, str(self.config_json)])  # o3d.visualization.draw_geometries([pointcloud])
            time.sleep(1)

        generated_depth_text_label = tk.Label(self.main_frame, text="Generated Depth", font=("Arial", 12, "bold"))
        generated_depth_text_label.grid(row=0, column=0, padx=10, pady=(10, 0), sticky="nsew")

        self.generated_depth_frame = tk.Frame(self.main_frame)
        self.generated_depth_frame.grid(row=1, column=0, padx=5, pady=5, sticky='nsew')

        generated_img_label = tk.Label(self.generated_depth_frame, text="Generated Depth HERE", font=("Arial", 12, "bold"))
        generated_img_label.grid(row=1, column=1, padx=10, pady=10)

        # Configure grid weights to allow for proper expansion
        self.main_frame.grid_rowconfigure(0, weight=0)  # Row for the label (no need to expand)
        self.main_frame.grid_rowconfigure(1, weight=1, uniform="equal")  # Row for generated_depth_frame
        self.main_frame.grid_rowconfigure(2, weight=1, uniform="equal")  # Row for settings_frame_custom
        self.main_frame.grid_columnconfigure(0, weight=1,  uniform="equal")  # Column for the left side (containing labels & settings_frame_custom)
        self.main_frame.grid_columnconfigure(1, weight=1, uniform="equal")  # Column for the right side (containing settings_frame)

        self.settings_frame_custom = tk.Frame(self.main_frame)
        self.settings_frame_custom.grid(row=2, column=0, sticky="nsew")

        self.settings_frame_custom.grid_rowconfigure(0, weight=1)
        self.settings_frame_custom.grid_columnconfigure(0, weight=1, minsize=1000)

        # ----

        self.canvas = tk.Canvas(self.settings_frame_custom)
        self.scrollbar = tk.Scrollbar(self.settings_frame_custom, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        # Create a frame to hold the content inside the canvas
        self.content_frame = tk.Frame(self.canvas)

        # Add widgets or content layout to content_frame
        self.layout_settings(frame=self.content_frame, original_config=self.view_info['metadata']['settings'])

        # Pack canvas and scrollbar inside settings_frame_custom
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.scrollbar.grid(row=0, column=1, sticky="ns")

        # Place the content_frame inside the canvas
        self.canvas.create_window((0, 0), window=self.content_frame, anchor="nw")

        # Update the canvas scroll region when the content size changes
        self.content_frame.update_idletasks()
        self.canvas.config(scrollregion=self.canvas.bbox("all"))

        # Bind mouse scrolling to scroll the canvas based on the platform
        if platform.system() == "Linux":
            # On Linux, use Button-4 (up) and Button-5 (down)
            self.canvas.bind_all("<Button-4>", self.on_mouse_wheel_up)  # Scroll up
            self.canvas.bind_all("<Button-5>", self.on_mouse_wheel_down)  # Scroll down  #
        else:
            # On other systems, use MouseWheel (Windows/macOS)
            self.canvas.bind("<MouseWheel>", self.on_mouse_wheel)

        # Make sure the canvas can accept focus and capture events
        self.canvas.focus_set()

        # ----

        # self.create_settings_layout(frame=self.settings_frame_custom, original_config=self.view_info['metadata']['settings'])

        self.settings_frame = tk.Frame(self.main_frame)
        self.settings_frame.grid(row=3, column=0, padx=5, pady=5, sticky='nsew')

        # JSON data for each depth image
        original_settings = self.view_info["metadata"]["settings"]
        generated_settings = self.view_info["metadata"].get("config2settings", {})
        original_json_str = json.dumps(original_settings, indent=4)
        generated_json_str = json.dumps(generated_settings, indent=4)

        # Generated Settings Label (dynamically updated)
        generated_json_label = tk.Label(
            self.settings_frame,
            text=generated_json_str,
            bg="white",
            fg="black",
            font=("Courier", 8),
            justify="left",
            anchor="nw"
        )
        generated_json_label.grid(row=2, column=1, padx=10, pady=10, sticky="nsew")

        # Assign the wrapper function to the button command
        # settings_button = tk.Button(self.generated_depth_frame, text="Settings", command=self.open_settings_get_config)
        # settings_button.grid(row=1, column=2)

        # Assign the wrapper function to the button command
        pointcloud_button = tk.Button(self.generated_depth_frame, text="Pointcloud", command=show_pointcloud)
        pointcloud_button.grid(row=1, column=2, pady=(50, 0))

        self.image_labels['generated_img_label'] = generated_img_label
        self.image_labels['generated_json_label'] = generated_json_label

    def refresh_display_old(self, label=None):
        print("Refresh display:", label)
        if self.generated_depth is None:
            colorized_generated_depth = create_placeholder_frame(self.scaled_original_size, label)
            colorized_difference = None
            range_min, range_max = 0, 0
        else:
            _, range_min, range_max = colorize_depth(self.generated_depth, type="depth", label=0)
            try:
                if self.generated_depth.shape != self.depth.shape:  # decimation filter downsamples, this upsamples
                    if self.depth.shape != self.generated_depth.shape:
                        height, width = self.depth.shape
                        self.generated_depth = cv2.resize(self.generated_depth, (width, height),
                                                          interpolation=cv2.INTER_NEAREST)
                depth_difference = np.abs(self.depth - self.generated_depth)
                colorized_difference, _, _ = colorize_depth(depth_difference, type="difference", label=0)
            except ValueError:
                colorized_difference = np.zeros_like(self.generated_depth)
                print("Depth shape problems, not calculating difference")

        # Update images
        # update original depth
        if self.generated_depth is not None:
            _, range_min_original, range_max_original = colorize_depth(self.depth, type="depth", label=0)
            range_min = min(range_min_original, range_min)
            range_max = max(range_max_original, range_max)

            colorized_generated_depth, _, _ = colorize_depth(self.generated_depth, type="depth", label=0, min_val=range_min, max_val=range_max)
            colorized_depth, _, _ = colorize_depth(self.depth, type="depth", label=0, min_val=range_min, max_val=range_max)
        else:
            colorized_depth, range_min, range_max = colorize_depth(self.depth, type="depth", label=0)

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

    def refresh_display(self, label=None):
        print("Refresh display:", label)
        if self.generated_depth is None:
            colorized_generated_depth = create_placeholder_frame(self.scaled_original_size, label)
        else:
            colorized_generated_depth, range_min, range_max = colorize_depth(self.generated_depth, type="depth", label=0)
        # update generated depth
        resized_generated_depth = cv2.resize(colorized_generated_depth, self.scaled_original_size,
                                             interpolation=cv2.INTER_AREA)
        im_generated = ImageTk.PhotoImage(image=Image.fromarray(resized_generated_depth))
        self.image_labels['generated_img_label'].configure(image=im_generated)
        self.image_labels['generated_img_label'].image = im_generated

        # Update settings labels
        self.image_labels['generated_json_label'].configure(text=json.dumps(self.config_json, indent=4))

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

    def generate_save_depth_replay(self, output_folder=None):
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

        if output_folder is not None:
            output_folder = output_folder
        else:
            date_time = datetime.now().strftime("%Y%m%d%H%M%S")
            output_folder = os.path.join(self.output_dir, date_time)
            os.makedirs(output_folder, exist_ok=False)

        print("Generating NEW outputs for the whole folder")
        print("Lading data for generation")
        timestamps = self.view_info['timestamps']
        data = self.view_info['data']
        config = self.config_json
        calib = self.view_info["calib"]
        if calib is None: raise ValueError("No calibration provided")

        batch_size = 40

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
                np.save(os.path.join(output_folder, f"depth_{timestamp}.npy"), depth)
                colorized_depth, _, _ = colorize_depth(depth, "depth", label=False, min_val=0, max_val=7000)
                cv2.imwrite(os.path.join(output_folder, f"depth_{timestamp}.png"), colorized_depth)
                o3d.io.write_point_cloud(os.path.join(output_folder, f"pcl_{timestamp}.ply"), pcl)

        with open(os.path.join(output_folder, f'config.json'), 'w') as f:
            json.dump(config, f, indent=4)

        print("GENERATED")
        return output_folder


    def load_or_generate(self):
        self.refresh_display(label="Loading...")
        self.main_frame.update_idletasks()
        current_timestamp = self.view_info['timestamps'][self.view_info['current_index']]
        already_generated, self.output_folder = self.get_output_folder()
        if not already_generated:
            self.output_folder = self.generate_save_depth_replay(output_folder=self.output_folder)
        print(f"LOADING TIMESTEP: {current_timestamp}")
        self.generated_depth = np.load(os.path.join(self.output_folder, f"depth_{current_timestamp}"))
        self.pcl_path = os.path.join(self.output_folder, f"pcl_{current_timestamp.split('.npy')[0]}.ply")
        self.refresh_display(label="Updated")
        self.settings_received = False  # Reset after processing
        self.main_frame.update_idletasks()

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