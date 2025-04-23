import tkinter as tk
from tkinter import ttk, Tk, filedialog

import json
import os

from capture_viewer_tools.convertor_capture2replay_json import handle_dict, decimation_set_dict, CT_kernel_dict

# Define a dictionary of default settings
default_config = {
    'stereo.setDepthAlign': 'dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT',
    'stereo.setDefaultProfilePreset': "dai.node.StereoDepth.PresetMode.HIGH_DENSITY",
    'stereo.setRectification': True,
    'stereo.setLeftRightCheck': True,
    'stereo.setExtendedDisparity': False,
    'stereo.setSubpixel': False,
    'stereo.setSubpixelFractionalBits': 3,
    'cfg.postProcessing.filteringOrder': "[dai.StereoDepthConfig.PostProcessing.Filter.DECIMATION, dai.StereoDepthConfig.PostProcessing.Filter.MEDIAN, dai.StereoDepthConfig.PostProcessing.Filter.TEMPORAL, dai.StereoDepthConfig.PostProcessing.Filter.SPECKLE, dai.StereoDepthConfig.PostProcessing.Filter.SPATIAL]",
    'cfg.postProcessing.medianFilter.enable': False,
    'stereo.initialConfig.setMedianFilter': "dai.MedianFilter.KERNEL_3x3",
    'cfg.postProcessing.bilateralFilter.enable': False,
    'cfg.postProcessing.bilateralSigmaValue': 10,
    'cfg.postProcessing.brightnessFilter.enable': False,
    'cfg.postProcessing.brightnessFilter.maxBrightness': 255,
    'cfg.postProcessing.brightnessFilter.minBrightness': 0,
    'cfg.postProcessing.speckleFilter.enable': False,
    'cfg.postProcessing.speckleFilter.speckleRange': 200,
    'cfg.postProcessing.speckleFilter.differenceThreshold': 2,
    'cfg.postProcessing.spatialFilter.enable': False,
    'cfg.postProcessing.spatialFilter.holeFillingRadius': 1,
    'cfg.postProcessing.spatialFilter.numIterations': 1,
    'cfg.postProcessing.spatialFilter.alpha': 0.5,
    'cfg.postProcessing.spatialFilter.delta': 3,
    'cfg.postProcessing.temporalFilter.enable': False,
    'cfg.postProcessing.temporalFilter.alpha': 0.5,
    'cfg.postProcessing.temporalFilter.delta': 3,
    'cfg.postProcessing.thresholdFilter.enable': False,
    'cfg.postProcessing.thresholdFilter.minRange': 0,
    'cfg.postProcessing.thresholdFilter.maxRange': 65535,
    'cfg.postProcessing.decimationFilter.enable': False,
    'cfg.postProcessing.decimationFilter.decimationFactor': 1,
    'cfg.postProcessing.decimationFilter.decimationMode': 'dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.PIXEL_SKIPPING',
    'cfg.censusTransform.enableMeanMode': True,
    'cfg.censusTransform.kernelSize': "dai.StereoDepthConfig.CensusTransform.KernelSize.AUTO",
    'cfg.censusTransform.threshold': 0,
    'cfg.costAggregation.divisionFactor': 1,
    'cfg.costAggregation.horizontalPenaltyCostP1': 250,
    'cfg.costAggregation.horizontalPenaltyCostP2': 500,
    'cfg.costAggregation.verticalPenaltyCostP1': 250,
    'cfg.costAggregation.verticalPenaltyCostP2': 500,
    # 'cfg.costMatching.confidenceThreshold': 245,
    'stereo.setConfidenceThreshold': 245,
    'cfg.costMatching.linearEquationParameters.alpha': 0,
    'cfg.costMatching.linearEquationParameters.beta': 2,
    'cfg.costMatching.linearEquationParameters.threshold': 127,
    'cfg.costMatching.enableCompanding': False,
    'cfg.algorithmControl.leftRightCheckThreshold': 5
}


def inicialize_button_values(current_config, button_values):
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
    button_values['filtering_order_enable'] = tk.BooleanVar(value=False)
    if 'cfg.postProcessing.filteringOrder' in current_config:
        button_values['initial_filter_order'] = get_filter_order_back(current_config['cfg.postProcessing.filteringOrder'])
    else:
        button_values['initial_filter_order'] = get_filter_order_back(default_config['cfg.postProcessing.filteringOrder'])

    button_values['decimation_order'] = tk.IntVar(value=button_values['initial_filter_order'][0])
    button_values['median_order'] = tk.IntVar(value=button_values['initial_filter_order'][1])
    button_values['speckle_order'] = tk.IntVar(value=button_values['initial_filter_order'][2])
    button_values['spatial_order'] = tk.IntVar(value=button_values['initial_filter_order'][3])
    button_values['temporal_order'] = tk.IntVar(value=button_values['initial_filter_order'][4])

    button_values['median_filter_enable'] = tk.BooleanVar(value=(current_config.get('stereo.initialConfig.setMedianFilter', "dai.MedianFilter.MEDIAN_OFF") != "dai.MedianFilter.MEDIAN_OFF"))
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

    button_values['threshold_filter_enable'] = tk.BooleanVar(value=False)
    button_values['min_range_val'] = tk.IntVar(value=current_config.get('cfg.postProcessing.thresholdFilter.minRange', default_config['cfg.postProcessing.thresholdFilter.minRange']))
    button_values['max_range_val'] = tk.IntVar(value=current_config.get('cfg.postProcessing.thresholdFilter.maxRange', default_config['cfg.postProcessing.thresholdFilter.maxRange']))

    button_values['decimation_filter_enable'] = tk.BooleanVar(value=(True if (current_config.get('cfg.postProcessing.decimationFilter.decimationFactor', 1) > 1) else False))
    button_values['decimation_factor_val'] = tk.IntVar(value=current_config.get('cfg.postProcessing.decimationFilter.decimationFactor', default_config['cfg.postProcessing.decimationFilter.decimationFactor']))
    button_values['decimation_mode_val'] = tk.StringVar(
        value=handle_dict(current_config.get('cfg.postProcessing.decimationFilter.decimationMode', default_config['cfg.postProcessing.decimationFilter.decimationMode']), decimation_set_dict, reverse=True))

    # ------------------------------------------------------ ADVANCED STEREO SETTINGS -------------------------------------------------------
    button_values['advanced_settings_enable'] = tk.BooleanVar(value=False)

    button_values['mean_mode_enable'] = tk.BooleanVar(value=default_config['cfg.censusTransform.enableMeanMode'])
    button_values['CT_kernel_val'] = tk.StringVar(value=handle_dict(default_config['cfg.censusTransform.kernelSize'], CT_kernel_dict, reverse=True))
    button_values['CT_threshold_val'] = tk.IntVar(value=default_config['cfg.censusTransform.threshold'])

    button_values['division_factor_val'] = tk.IntVar(value=default_config['cfg.costAggregation.divisionFactor'])
    button_values['horizontal_penalty_p1_val'] = tk.IntVar(value=default_config['cfg.costAggregation.horizontalPenaltyCostP1'])
    button_values['horizontal_penalty_p2_val'] = tk.IntVar(value=default_config['cfg.costAggregation.horizontalPenaltyCostP2'])
    button_values['vertical_penalty_p1_val'] = tk.IntVar(value=default_config['cfg.costAggregation.verticalPenaltyCostP1'])
    button_values['vertical_penalty_p2_val'] = tk.IntVar(value=default_config['cfg.costAggregation.verticalPenaltyCostP2'])

    button_values['confidence_threshold_val'] = tk.IntVar(value=default_config['stereo.setConfidenceThreshold'])
    button_values['CM_alpha_val'] = tk.IntVar(value=default_config['cfg.costMatching.linearEquationParameters.alpha'])
    button_values['CM_beta_val'] = tk.IntVar(value=default_config['cfg.costMatching.linearEquationParameters.beta'])
    button_values['matching_threshold_val'] = tk.IntVar(value=default_config['cfg.costMatching.linearEquationParameters.threshold'])
    button_values['enableCompanding_val'] = tk.BooleanVar(value=default_config['cfg.costMatching.enableCompanding'])
    button_values['leftRightCheckThreshold_val'] = tk.IntVar(value=default_config['cfg.algorithmControl.leftRightCheckThreshold'])

    button_values['loaded_config'] = tk.StringVar(value='')

def update_button_values(new_config, button_values):
    button_values['depth_align'].set(new_config.get('stereo.setDepthAlign', default_config['stereo.setDepthAlign']))
    button_values['profile_preset'].set(new_config.get('stereo.setDefaultProfilePreset', default_config['stereo.setDefaultProfilePreset']))

    button_values['rectificationBox_val'].set(new_config.get('stereo.setRectification', True))
    button_values['LRBox_val'].set(new_config.get('stereo.setLeftRightCheck', default_config['stereo.setLeftRightCheck']))
    button_values['extendedBox_val'].set(new_config.get('stereo.setExtendedDisparity', default_config['stereo.setExtendedDisparity']))
    button_values['subpixelBox_val'].set(new_config.get('stereo.setSubpixel', default_config['stereo.setSubpixel']))
    button_values['fractional_bits_val'].set(new_config.get('stereo.setSubpixelFractionalBits', default_config['stereo.setSubpixelFractionalBits']))

    filter_order_enabled = 'cfg.postProcessing.filteringOrder' in new_config
    button_values['filtering_order_enable'].set(filter_order_enabled)

    if filter_order_enabled:
        initial_filter_order = get_filter_order_back(new_config['cfg.postProcessing.filteringOrder'])
    else:
        initial_filter_order = get_filter_order_back(default_config['cfg.postProcessing.filteringOrder'])

    button_values['decimation_order'].set(initial_filter_order[0])
    button_values['median_order'].set(initial_filter_order[1])
    button_values['speckle_order'].set(initial_filter_order[2])
    button_values['spatial_order'].set(initial_filter_order[3])
    button_values['temporal_order'].set(initial_filter_order[4])

    button_values['median_filter_enable'].set(new_config.get('stereo.initialConfig.setMedianFilter', "MedianFilter.MEDIAN_OFF") != "MedianFilter.MEDIAN_OFF")
    button_values['median_val'].set(new_config.get('stereo.initialConfig.setMedianFilter', default_config['stereo.initialConfig.setMedianFilter']))

    button_values['brightness_filter_enable'].set(new_config.get('cfg.postProcessing.brightnessFilter.enable', False))
    button_values['min_brightness_slider'].set(new_config.get('cfg.postProcessing.brightnessFilter.minBrightness', default_config['cfg.postProcessing.brightnessFilter.minBrightness']))
    button_values['max_brightness_slider'].set(new_config.get('cfg.postProcessing.brightnessFilter.maxBrightness', default_config['cfg.postProcessing.brightnessFilter.maxBrightness']))

    button_values['speckle_filter_enable'].set(new_config.get('cfg.postProcessing.speckleFilter.enable', False))
    button_values['speckle_range_slider'].set(new_config.get('cfg.postProcessing.speckleFilter.speckleRange', default_config['cfg.postProcessing.speckleFilter.speckleRange']))
    button_values['speckle_difference_threshold'].set(new_config.get('cfg.postProcessing.speckleFilter.differenceThreshold', default_config['cfg.postProcessing.speckleFilter.differenceThreshold']))

    button_values['spatial_filter_enable'].set(new_config.get('cfg.postProcessing.spatialFilter.enable', False))
    button_values['hole_filling_radius_slider'].set(new_config.get('cfg.postProcessing.spatialFilter.holeFillingRadius', default_config['cfg.postProcessing.spatialFilter.holeFillingRadius']))
    button_values['num_iterations_slider'].set(new_config.get('cfg.postProcessing.spatialFilter.numIterations', default_config['cfg.postProcessing.spatialFilter.numIterations']))
    button_values['alpha_slider'].set(new_config.get('cfg.postProcessing.spatialFilter.alpha', default_config['cfg.postProcessing.spatialFilter.alpha']))
    button_values['delta_slider'].set(new_config.get('cfg.postProcessing.spatialFilter.delta', default_config['cfg.postProcessing.spatialFilter.delta']))

    button_values['temporal_filter_enable'].set(new_config.get('cfg.postProcessing.temporalFilter.enable', False))
    button_values['temporal_alpha_slider'].set(new_config.get('cfg.postProcessing.temporalFilter.alpha', default_config['cfg.postProcessing.temporalFilter.alpha']))
    button_values['temporal_delta_slider'].set(new_config.get('cfg.postProcessing.temporalFilter.delta', default_config['cfg.postProcessing.temporalFilter.delta']))

    button_values['threshold_filter_enable'].set('cfg.postProcessing.thresholdFilter.minRange' in new_config)
    button_values['min_range_val'].set(new_config.get('cfg.postProcessing.thresholdFilter.minRange', default_config['cfg.postProcessing.thresholdFilter.minRange']))
    button_values['max_range_val'].set(new_config.get('cfg.postProcessing.thresholdFilter.maxRange', default_config['cfg.postProcessing.thresholdFilter.maxRange']))

    button_values['decimation_filter_enable'].set('cfg.postProcessing.decimationFilter.decimationFactor' in new_config)
    button_values['decimation_factor_val'].set(new_config.get('cfg.postProcessing.decimationFilter.decimationFactor', default_config['cfg.postProcessing.decimationFilter.decimationFactor']))
    button_values['decimation_mode_val'].set(
        handle_dict(new_config.get('cfg.postProcessing.decimationFilter.decimationMode', default_config['cfg.postProcessing.decimationFilter.decimationMode']), decimation_set_dict, reverse=True)
    )

    button_values['mean_mode_enable'].set(new_config.get('cfg.censusTransform.enableMeanMode', default_config['cfg.censusTransform.enableMeanMode']))
    button_values['CT_kernel_val'].set(handle_dict(new_config.get('cfg.censusTransform.kernelSize', default_config['cfg.censusTransform.kernelSize']), CT_kernel_dict, reverse=True))
    button_values['CT_threshold_val'].set(new_config.get('cfg.censusTransform.threshold', default_config['cfg.censusTransform.threshold']))

    button_values['division_factor_val'].set(new_config.get('cfg.costAggregation.divisionFactor', default_config['cfg.costAggregation.divisionFactor']))
    button_values['horizontal_penalty_p1_val'].set(new_config.get('cfg.costAggregation.horizontalPenaltyCostP1', default_config['cfg.costAggregation.horizontalPenaltyCostP1']))
    button_values['horizontal_penalty_p2_val'].set(new_config.get('cfg.costAggregation.horizontalPenaltyCostP2', default_config['cfg.costAggregation.horizontalPenaltyCostP2']))
    button_values['vertical_penalty_p1_val'].set(new_config.get('cfg.costAggregation.verticalPenaltyCostP1', default_config['cfg.costAggregation.verticalPenaltyCostP1']))
    button_values['vertical_penalty_p2_val'].set(new_config.get('cfg.costAggregation.verticalPenaltyCostP2', default_config['cfg.costAggregation.verticalPenaltyCostP2']))

    # button_values['confidence_threshold_val'].set(new_config.get('cfg.costMatching.confidenceThreshold', default_config['cfg.costMatching.confidenceThreshold']))
    button_values['confidence_threshold_val'].set(new_config.get('stereo.setConfidenceThreshold', default_config['stereo.setConfidenceThreshold']))

    button_values['CM_alpha_val'].set(new_config.get('cfg.costMatching.linearEquationParameters.alpha', default_config['cfg.costMatching.linearEquationParameters.alpha']))
    button_values['CM_beta_val'].set(new_config.get('cfg.costMatching.linearEquationParameters.beta', default_config['cfg.costMatching.linearEquationParameters.beta']))
    button_values['matching_threshold_val'].set(new_config.get('cfg.costMatching.linearEquationParameters.threshold', default_config['cfg.costMatching.linearEquationParameters.threshold']))
    button_values['enableCompanding_val'].set(new_config.get('cfg.costMatching.enableCompanding', default_config['cfg.costMatching.enableCompanding']))
    button_values['leftRightCheckThreshold_val'].set(new_config.get('cfg.algorithmControl.leftRightCheckThreshold', default_config['cfg.algorithmControl.leftRightCheckThreshold']))

def add_trace_to_button_values(button_values, fallback_function):
    for button, value in button_values.items():
        try:
            button_values[button].trace_add("write", fallback_function)
        except:
            print("Failed to add trace to button {}".format(button))

def create_settings_layout(frame, button_values):
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
        if frame_on.get():
            enable_frame_widgets_recursively(frame, True)
        else:
            enable_frame_widgets_recursively(frame, False)

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

    def dropdown(frame, row, col, val, options):
        combo = ttk.Combobox(frame, textvariable=val, values=options, state="readonly")
        combo.grid(row=row, column=col, padx=10, pady=10, sticky="w")
        combo.bind("<MouseWheel>", lambda event: "break")
        combo.bind("<Button-4>", lambda event: "break")
        combo.bind("<Button-5>", lambda event: "break")

    def spinbox(frame, row, col, slider, label, range_of_spinbox, is_float=False):
        if is_float:
            values = [f"{i / 10:.1f}" for i in range(int(range_of_spinbox[0] * 10), int(range_of_spinbox[1] * 10) + 1)]
            spinbox = ttk.Spinbox(frame, values=values, textvariable=slider, command=lambda: update_label(slider, label, form="float"))
        else:
            spinbox = ttk.Spinbox(frame, from_=range_of_spinbox[0], to=range_of_spinbox[1], textvariable=slider, command=lambda: update_label(slider, label))
        spinbox.grid(row=row, column=col, padx=10, pady=10, sticky="w")
        spinbox.bind("<MouseWheel>", lambda event: "break")
        spinbox.bind("<Button-4>", lambda event: "break")
        spinbox.bind("<Button-5>", lambda event: "break")

    def on_load_custom_config(parent, button_values):
        dialog_window = tk.Toplevel(parent)
        dialog_window.withdraw()

        file_path = filedialog.askopenfilename(parent=dialog_window, filetypes=[("JSON files", "*.json")])
        dialog_window.destroy()

        if file_path:
            with open(file_path, 'r') as file:
                loaded_config = json.load(file)
                update_button_values(loaded_config, button_values)

            filename = os.path.basename(file_path)
            button_values['loaded_config'].set(filename)

            if not button_values['custom_settings_val'].get():
                cust_box.invoke()
            if not button_values['advanced_settings_enable'].get():
                advanced_settings_checkbox.invoke()

    popup_window = frame

    # ----------------------------------------------------------------- BUTTONS -------------------------------------------------------------

    current_row = 0

    #
    ttk.Label(popup_window, text="setDepthAlign").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    align_options = {
        "Left": "dai.CameraBoardSocket.LEFT",
        "Right": "dai.CameraBoardSocket.RIGHT",
        "Rec Left": "dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT",
        "Rec Right": "dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT",
        "RGB": "dai.CameraBoardSocket.CAM_A"
    }
    display_val_align = tk.StringVar(value={v: k for k, v in align_options.items()}.get(button_values['depth_align'].get(), "Left"))
    def on_select_align(*args):
        button_values['depth_align'].set(align_options[display_val_align.get()])
    dropdown(popup_window, current_row, 1, display_val_align, list(align_options.keys()))
    display_val_align.trace_add("write", on_select_align)

    current_row += 1

    #
    ttk.Label(popup_window, text="setDefaultProfilePreset").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    preset_options = {
        "DEFAULT": "dai.node.StereoDepth.PresetMode.DEFAULT",
        "HIGH_ACCURACY": "dai.node.StereoDepth.PresetMode.HIGH_ACCURACY",
        "HIGH_DENSITY": "dai.node.StereoDepth.PresetMode.HIGH_DENSITY",
        "ROBOTICS": "dai.node.StereoDepth.PresetMode.ROBOTICS",
        "HIGH_DETAIL": "dai.node.StereoDepth.PresetMode.HIGH_DETAIL",
        "FACE": "dai.node.StereoDepth.PresetMode.FACE",
        "None": "None"
    }
    display_val_preset = tk.StringVar(value={v: k for k, v in preset_options.items()}.get(button_values['profile_preset'].get(), "DEFAULT"))
    def on_select_preset(*args):
        button_values['profile_preset'].set(preset_options[display_val_preset.get()])
    dropdown(popup_window, current_row, 1, display_val_preset, list(preset_options.keys()))
    display_val_preset.trace_add("write", on_select_preset)

    current_row += 1

    load_config_button = tk.Button(popup_window, text="Load Config", command=lambda: on_load_custom_config(popup_window, button_values))
    load_config_button.grid(row=current_row, column=1, sticky='ew', padx=10)
    config_label = tk.Label(popup_window, textvariable=button_values['loaded_config'])
    config_label.grid(row=current_row, column=2, sticky='ew', padx=10)

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
    recbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="w")
    current_row += 1

    # LR CHECK
    ttk.Label(custom_settings_frame, text="setLRcheck").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    LRbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['LRBox_val'])
    LRbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="w")
    current_row += 1

    # EXTENDED DISPARITY
    ttk.Label(custom_settings_frame, text="setExtendedDisparity").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    extbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['extendedBox_val'])
    extbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="w")
    current_row += 1

    # SetSubpixel Label and Checkbox
    ttk.Label(custom_settings_frame, text="setSubpixel").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    subbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['subpixelBox_val'])
    subbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="w")

    # Add subpixelFractionalBits combobox
    ttk.Label(custom_settings_frame, text="subpixelFractionalBits").grid(row=current_row, column=2, padx=10, pady=10, sticky="w")
    dropdown(custom_settings_frame, current_row, 3, button_values['fractional_bits_val'], [3, 4, 5])

    current_row += 1

    # Decimation Filter Enable
    ttk.Label(custom_settings_frame, text="Decimation Filter Enable").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    decimation_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['decimation_filter_enable'], command=lambda: toggle_frame_settings(button_values['decimation_filter_enable'], decimation_frame))
    decimation_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="w")

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
    dropdown(decimation_frame, 1, 4, button_values['decimation_mode_val'], ['NON_ZERO_MEAN', 'NON_ZERO_MEDIAN', 'PIXEL_SKIPPING'])

    current_row += 1

    # Median Filter
    ttk.Label(custom_settings_frame, text="Median Enable").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    median_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['median_filter_enable'], command=lambda: toggle_frame_settings(button_values['median_filter_enable'], median_frame))
    median_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="w")
    current_row += 1

    # Median Filter Frame
    median_frame = ttk.LabelFrame(custom_settings_frame, text="Median Filter", padding=(10, 10))
    median_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

    ttk.Label(median_frame, text="Median").grid(row=0, column=0, padx=10, pady=10, sticky="w")
    median_3_button = ttk.Radiobutton(median_frame, text="MEDIAN_3x3", variable=button_values['median_val'], value="dai.MedianFilter.KERNEL_3x3")
    median_5_button = ttk.Radiobutton(median_frame, text="MEDIAN_5x5", variable=button_values['median_val'], value="dai.MedianFilter.KERNEL_5x5")
    median_7_button = ttk.Radiobutton(median_frame, text="MEDIAN_7x7", variable=button_values['median_val'], value="dai.MedianFilter.KERNEL_7x7")
    median_3_button.grid(row=0, column=1, padx=10, pady=5, sticky="w")
    median_5_button.grid(row=0, column=2, padx=10, pady=5, sticky="w")
    median_7_button.grid(row=0, column=3, padx=10, pady=5, sticky="w")
    current_row += 1

    # Speckle Filter Enable
    ttk.Label(custom_settings_frame, text="Speckle Filter Enable").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    speckle_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['speckle_filter_enable'], command=lambda: toggle_frame_settings(button_values['speckle_filter_enable'], speckle_frame))
    speckle_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="w")
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
    spatial_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="w")
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
    ttk.Scale(spatial_frame, from_=1, to=10, variable=button_values['num_iterations_slider'], orient="horizontal", command=lambda x: update_label(button_values['num_iterations_slider'], num_iterations_label)).grid(row=1, column=4, padx=10,
                                                                                                                                                                                                                      pady=10, sticky="w")
    current_row += 1

    ttk.Label(spatial_frame, text="Alpha").grid(row=2, column=0, padx=10, pady=10, sticky="w")
    alpha_label = ttk.Label(spatial_frame, text=str(button_values['alpha_slider'].get()))
    spinbox(spatial_frame, 2, 1, button_values['alpha_slider'], alpha_label, [0.0, 1.0], is_float=True)
    ttk.Label(spatial_frame, text="(0.0, 1.0)").grid(row=2, column=2, padx=10, pady=10, sticky="w")

    ttk.Label(spatial_frame, text="Delta").grid(row=2, column=3, padx=10, pady=10, sticky="w")
    delta_label = ttk.Label(spatial_frame, text=str(button_values['delta_slider'].get()))
    spinbox(spatial_frame, 2, 4, button_values['delta_slider'], delta_label, [0, 255])
    ttk.Label(spatial_frame, text="(0, 255)").grid(row=2, column=5, padx=10, pady=10, sticky="w")

    current_row += 1

    # Temporal Filter Frame
    ttk.Label(custom_settings_frame, text="Temporal Filter Enable").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    temporal_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['temporal_filter_enable'], command=lambda: toggle_frame_settings(button_values['temporal_filter_enable'], temporal_frame))
    temporal_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="w")
    current_row += 1

    temporal_frame = ttk.LabelFrame(custom_settings_frame, text="Temporal Filter", padding=(10, 10))
    temporal_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

    ttk.Label(temporal_frame, text="Alpha").grid(row=2, column=0, padx=10, pady=10, sticky="w")
    temporal_alpha_label = ttk.Label(temporal_frame, text=str(button_values['temporal_alpha_slider'].get()))
    spinbox(temporal_frame, 2, 1, button_values['temporal_alpha_slider'], temporal_alpha_label, [0.0, 1.0], is_float=True)
    ttk.Label(temporal_frame, text="(0.0, 1.0)").grid(row=2, column=2, padx=10, pady=10, sticky="w")

    ttk.Label(temporal_frame, text="Delta").grid(row=2, column=3, padx=10, pady=10, sticky="w")
    temporal_delta_label = ttk.Label(temporal_frame, text=str(button_values['temporal_delta_slider'].get()))
    spinbox(temporal_frame, 2, 4, button_values['temporal_delta_slider'], temporal_delta_label, [0, 255])
    ttk.Label(temporal_frame, text="(0, 255)").grid(row=2, column=5, padx=10, pady=10, sticky="w")

    current_row += 1

    # Threshold Filter Enable
    ttk.Label(custom_settings_frame, text="Threshold Filter Enable").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    threshold_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['threshold_filter_enable'], command=lambda: toggle_frame_settings(button_values['threshold_filter_enable'], threshold_frame))
    threshold_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="w")
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
    brightness_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="w")
    current_row += 1

    # Brightness Filter Frame
    brightness_frame = ttk.LabelFrame(custom_settings_frame, text="Brightness Filter", padding=(10, 10))
    brightness_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

    # Brightness Filter Sliders
    ttk.Label(brightness_frame, text="Min Brightness").grid(row=1, column=0, padx=10, pady=10, sticky="w")
    min_brightness_label = ttk.Label(brightness_frame, text=str(button_values['min_brightness_slider'].get()))
    min_brightness_label.grid(row=1, column=2, padx=10, pady=10, sticky="w")
    ttk.Scale(brightness_frame, from_=0, to=int(button_values['max_brightness_slider'].get()), variable=button_values['min_brightness_slider'], orient="horizontal",
              command=lambda x: update_label(button_values['min_brightness_slider'], min_brightness_label)).grid(row=1, column=1, padx=10, pady=10, sticky="w")

    ttk.Label(brightness_frame, text="Max Brightness").grid(row=1, column=3, padx=10, pady=10, sticky="w")
    max_brightness_label = ttk.Label(brightness_frame, text=str(button_values['max_brightness_slider'].get()))
    max_brightness_label.grid(row=1, column=5, padx=10, pady=10, sticky="w")
    ttk.Scale(brightness_frame, from_=int(button_values['min_brightness_slider'].get()), to=255, variable=button_values['max_brightness_slider'], orient="horizontal",
              command=lambda x: update_label(button_values['max_brightness_slider'], max_brightness_label)).grid(row=1, column=4, padx=10, pady=10, sticky="w")
    current_row += 1

    # Filter Order Enable
    ttk.Label(custom_settings_frame, text="Filter Order Selection Enable (NOT TESTED IN GUI)").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    send_filter_order_checkbox = ttk.Checkbutton(custom_settings_frame, variable=button_values['filtering_order_enable'], command=lambda: toggle_frame_settings(button_values['filtering_order_enable'], order_frame))
    send_filter_order_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="w")
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
    advanced_settings_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="w")

    current_row += 1

    inner_row = 1

    advanced_stereo_setting_frame = ttk.LabelFrame(popup_window, text="Advanced Settings", padding=(10, 10))
    advanced_stereo_setting_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

    ttk.Label(advanced_stereo_setting_frame, text="censusTransform.kernelSize").grid(row=inner_row, column=0, padx=10, pady=10, sticky="w")
    dropdown(advanced_stereo_setting_frame, inner_row, 1, button_values['CT_kernel_val'], ['KERNEL_AUTO', 'KERNEL_5x5', 'KERNEL_7x7', 'KERNEL_7x9'])

    inner_row += 1

    ttk.Label(advanced_stereo_setting_frame, text="censusTransform.enableMeanMode").grid(row=inner_row, column=0, padx=10, pady=10, sticky="w")
    mean_mode_checkbox = ttk.Checkbutton(advanced_stereo_setting_frame, variable=button_values['mean_mode_enable'])
    mean_mode_checkbox.grid(row=inner_row, column=1, padx=10, pady=10, sticky="w")

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

    ttk.Label(advanced_stereo_setting_frame, text="costAggregation.verticalPenaltyCostP2").grid(row=inner_row, column=0, padx=10, pady=10, sticky="w")
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

    ttk.Label(advanced_stereo_setting_frame, text="costMatching.enableCompanding").grid(row=inner_row, column=0, padx=10, pady=10, sticky="w")
    companding_checkbox = ttk.Checkbutton(advanced_stereo_setting_frame, variable=button_values['enableCompanding_val'])
    companding_checkbox.grid(row=inner_row, column=1, padx=10, pady=10, sticky="w")

    inner_row += 1

    ttk.Label(advanced_stereo_setting_frame, text="algorithmControl.leftRightCheckThreshold").grid(row=inner_row, column=0, padx=10, pady=10, sticky="w")
    spinbox(advanced_stereo_setting_frame, inner_row, 1, button_values['leftRightCheckThreshold_val'], None, [0, 255])
    ttk.Label(advanced_stereo_setting_frame, text="(0, 255)").grid(row=inner_row, column=2, padx=10, pady=10, sticky="w")

    current_row += 1

    # ------------------------------------------------------------------------------------------------------------------------------

    toggle_custom_frame_settings()
    toggle_advanced_settings()  # turn off by default

    # popup_window.grab_set()  # Make the window modal (disable interaction with the main window)
    # popup_window.wait_window()  # Wait for the popup window to be destroyed


def check_valid_filtering_order(order):
    new_order = order.copy()
    new_order.sort()
    return new_order == [1, 2, 3, 4, 5]

def get_filter_order(dec_ord, med_ord, speckle_ord, spatial_ord, temporal_ord):
    order = [0, 0, 0, 0, 0]
    print(dec_ord, med_ord, speckle_ord, spatial_ord, temporal_ord)
    order[dec_ord - 1] = 'dai.StereoDepthConfig.PostProcessing.Filter.DECIMATION'
    order[med_ord - 1] = 'dai.StereoDepthConfig.PostProcessing.Filter.MEDIAN'
    order[speckle_ord - 1] = 'dai.StereoDepthConfig.PostProcessing.Filter.SPECKLE'
    order[spatial_ord - 1] = 'dai.StereoDepthConfig.PostProcessing.Filter.SPATIAL'
    order[temporal_ord - 1] = 'dai.StereoDepthConfig.PostProcessing.Filter.TEMPORAL'
    final_str = '['
    print(order)
    for item in order:
        assert type(item) == str
        final_str += item
        final_str += ','
    final_str += ']'
    print(final_str)
    return final_str

def get_filter_order_back(order_string):
    result = [0, 0, 0, 0, 0]
    names = ["DECIMATION", "MEDIAN", "SPECKLE", "TEMPORAL", "SPATIAL"]
    for i, item in enumerate(order_string.split(',')):
        for j in range(len(names)):
            if names[j] in item:
                result[j] = i+1
    return result

if __name__ == '__main__':
    # Example usage
    root = tk.Tk()
    root.title("Main Application")
    root.geometry("300x200")


    # Button to open the settings pop-up window
    def on_open_popup():
        settings = open_replay_settings_screen({})
        print("Settings returned to main:", settings)


    open_popup_button = ttk.Button(root, text="Open Settings", command=on_open_popup)
    open_popup_button.pack(pady=50)

    root.mainloop()

metadata_settings = {
    "config2settings": {"example_key": "config_value"},  # Add more example config values as needed
    "settings": {"parameter1": "value1", "parameter2": "value2"}  # Replace with actual settings values
}
