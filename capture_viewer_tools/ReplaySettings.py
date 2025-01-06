import tkinter as tk
from tkinter import ttk

from capture_viewer_tools.convertor_capture2replay_json import settings2config, handle_dict, decimation_set_dict, CT_kernel_dict
from capture_viewer_tools.popup_info import show_popup

# Define a dictionary of default settings
default_config = {
    'stereo.setDepthAlign': 'StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT',
    'stereo.setDefaultProfilePreset': "node.StereoDepth.PresetMode.HIGH_DENSITY",
    'stereo.setRectification': True,
    'stereo.setLeftRightCheck': True,
    'stereo.setExtendedDisparity': False,
    'stereo.setSubpixel': False,
    'stereo.setSubpixelFractionalBits': 3,
    'cfg.postProcessing.filteringOrder': "[dai.StereoDepthConfig.PostProcessing.Filter.DECIMATION, dai.StereoDepthConfig.PostProcessing.Filter.MEDIAN, dai.StereoDepthConfig.PostProcessing.Filter.TEMPORAL, dai.StereoDepthConfig.PostProcessing.Filter.SPECKLE, dai.StereoDepthConfig.PostProcessing.Filter.SPATIAL]",
    'cfg.postProcessing.medianFilter.enable': False,
    'stereo.initialConfig.setMedianFilter': "MedianFilter.KERNEL_3x3",
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
    'cfg.postProcessing.thresholdFilter.enable': False,
    'cfg.postProcessing.thresholdFilter.minRange': 0,
    'cfg.postProcessing.thresholdFilter.maxRange': 65535,
    'cfg.postProcessing.decimationFilter.enable': False,
    'cfg.postProcessing.decimationFilter.decimationFactor': 1,
    'cfg.postProcessing.decimationFilter.decimationMode': 'StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.PIXEL_SKIPPING'
}

last_config = None

def warning():
    print("FILTERING ORDER IS NOT VALID")
    show_popup("FILTERING ORDER IS NOT VALID")
    pass

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


def open_replay_settings_screen(config, original_config=None):
    global last_config
    def update_label(var, label, form="int"):
        """Update the label when slider is moved."""
        if form == "int":
            label.config(text=str(int(var.get())))
        elif form == "float":
            label.config(text=str(round(float(var.get()), 1)))
    def on_generate():
        global last_config

        if not custom_settings_val.get():
            config['stereo.setDepthAlign'] = depth_align.get()
            if profile_preset.get() != 'None':
                config['stereo.setDefaultProfilePreset'] = profile_preset.get()

            popup_window.destroy()
            popup_window1.destroy()
            last_config = config
            return config

        if filtering_order_enable.get(): # needs to be the first item for the config to not be initialised in case of wrong filtering order
            if not check_valid_filtering_order([decimation_order.get(), median_order.get(), speckle_order.get(), spatial_order.get(), temporal_order.get()]):
                warning()
                return config
            else:
                config['cfg.postProcessing.filteringOrder'] = get_filter_order(decimation_order.get(), median_order.get(), speckle_order.get(), spatial_order.get(), temporal_order.get())

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
        # Note: temporal filter disabled due to the nature of data (not continuous recording)

        if median_filter_enable.get():
            config['stereo.initialConfig.setMedianFilter'] = median_val.get()

        # Bilateral filter
        if bilateral_filter_enable.get():
            # is this all settings?
            config['cfg.postProcessing.bilateralSigmaValue'] = bilateral_sigma_val.get()

        # Brightness filter
        if brightness_filter_enable.get():
            config['cfg.postProcessing.brightnessFilter.maxBrightness'] = max_brightness_slider.get()
            config['cfg.postProcessing.brightnessFilter.minBrightness'] = min_brightness_slider.get()

        # Speckle filter
        if speckle_filter_enable.get():
            config['cfg.postProcessing.speckleFilter.enable'] = speckle_filter_enable.get()
            config['cfg.postProcessing.speckleFilter.speckleRange'] = speckle_range_slider.get()
            config['cfg.postProcessing.speckleFilter.differenceThreshold'] = speckle_difference_threshold.get()

        # Spacial filter
        if spatial_filter_enable.get():
            config['cfg.postProcessing.spatialFilter.enable'] = spatial_filter_enable.get()
            config['cfg.postProcessing.spatialFilter.holeFillingRadius'] = hole_filling_radius_slider.get()
            config['cfg.postProcessing.spatialFilter.numIterations'] = num_iterations_slider.get()
            config['cfg.postProcessing.spatialFilter.alpha'] = alpha_slider.get()
            config['cfg.postProcessing.spatialFilter.delta'] = delta_slider.get()

        # Threshold filter
        if threshold_filter_enable.get():
            config['cfg.postProcessing.thresholdFilter.minRange'] = min_range_val.get()
            config['cfg.postProcessing.thresholdFilter.maxRange'] = max_range_val.get()

        # Decimation filter
        if decimation_filter_enable.get():
            config['cfg.postProcessing.decimationFilter.decimationFactor'] = decimation_factor_val.get()
            config['cfg.postProcessing.decimationFilter.decimationMode'] = handle_dict(decimation_mode_val.get(), decimation_set_dict)  # leave this is correct


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

        popup_window.destroy()
        popup_window1.destroy()
        last_config = config
        return config


    if last_config is None and original_config is not None:
        current_config = settings2config(original_config)
    elif last_config is not None:
        current_config = last_config
    else:
        current_config = default_config

    # -------------------------------------------------------- INITIAL SETTINGS ------------------------------------------------
    # Create the pop-up window
    popup_window1 = tk.Toplevel()
    popup_window1.title("Replay settings")
    popup_window1.geometry("1000x900")

    # Create a canvas and a scrollbar
    canvas = tk.Canvas(popup_window1)
    scrollbar = ttk.Scrollbar(popup_window1, orient="vertical", command=canvas.yview)
    popup_window = ttk.Frame(canvas)

    # Configure the canvas to update scrollregion when the window is resized
    popup_window.bind(
        "<Configure>",
        lambda e: canvas.configure(
            scrollregion=canvas.bbox("all")
        )
    )

    # Add the frame inside the canvas
    canvas.create_window((0, 0), window=popup_window, anchor="nw")

    # Pack the canvas and scrollbar
    canvas.pack(side="left", fill="both", expand=True)
    scrollbar.pack(side="right", fill="y")
    canvas.configure(yscrollcommand=scrollbar.set)

    # Enable mouse scrolling
    def _on_mouse_wheel(event):
        if canvas.winfo_height() < canvas.bbox("all")[3]:  # Ensure content overflows before scrolling
            canvas.yview_scroll(-1 * int((event.delta / 120)), "units")

    # Bind mouse wheel event only to the canvas area to avoid conflicts with scrollbar
    canvas.bind_all("<MouseWheel>", _on_mouse_wheel)

    # # For Linux systems (usually bound to <Button-4> and <Button-5>)
    canvas.bind_all("<Button-4>", lambda event: canvas.yview_scroll(-1, "units"))
    canvas.bind_all("<Button-5>", lambda event: canvas.yview_scroll(1, "units"))

    # ----------------------------------- Initialize the UI elements with default values from current_config -----------------------------------
    # Initialize tkinter variables, falling back to default_config if a key is missing in current_config
    depth_align = tk.StringVar(value=current_config.get('stereo.setDepthAlign', default_config['stereo.setDepthAlign']))
    profile_preset = tk.StringVar(value=current_config.get('stereo.setDefaultProfilePreset', default_config['stereo.setDefaultProfilePreset']))

    custom_settings_val = tk.BooleanVar(value=False)

    rectificationBox_val = tk.BooleanVar(value=current_config.get('stereo.setRectification', True))  # Default to True if not found
    LRBox_val = tk.BooleanVar(value=current_config.get('stereo.setLeftRightCheck', default_config['stereo.setLeftRightCheck']))
    extendedBox_val = tk.BooleanVar(value=current_config.get('stereo.setExtendedDisparity', default_config['stereo.setExtendedDisparity']))
    subpixelBox_val = tk.BooleanVar(value=current_config.get('stereo.setSubpixel', default_config['stereo.setSubpixel']))
    fractional_bits = tk.IntVar(value=current_config.get('stereo.setSubpixelFractionalBits', default_config['stereo.setSubpixelFractionalBits']))

    # FILTERS -----------------------------------------------------------------------------------
    filtering_order_enable = tk.BooleanVar(value=(True if 'cfg.postProcessing.filteringOrder' in current_config else False))
    if 'cfg.postProcessing.filteringOrder' in current_config:
        initial_filter_order = get_filter_order_back(current_config['cfg.postProcessing.filteringOrder'])
    else:
        initial_filter_order = get_filter_order_back(default_config['cfg.postProcessing.filteringOrder'])

    decimation_order = tk.IntVar(value=initial_filter_order[0])
    median_order = tk.IntVar(value=initial_filter_order[1])
    speckle_order = tk.IntVar(value=initial_filter_order[2])
    spatial_order = tk.IntVar(value=initial_filter_order[3])
    temporal_order = tk.IntVar(value=initial_filter_order[4])

    median_filter_enable = tk.BooleanVar(value=(current_config.get('stereo.initialConfig.setMedianFilter', "MedianFilter.MEDIAN_OFF")!="MedianFilter.MEDIAN_OFF"))
    median_val = tk.StringVar(value=current_config.get('stereo.initialConfig.setMedianFilter',
                                                       default_config['stereo.initialConfig.setMedianFilter']))

    bilateral_filter_enable = tk.BooleanVar(value=current_config.get('cfg.postProcessing.bilateralFilter.enable', False))
    bilateral_sigma_val = tk.IntVar(value=current_config.get('cfg.postProcessing.bilateralSigmaValue',
                                                             default_config['cfg.postProcessing.bilateralSigmaValue']))

    brightness_filter_enable = tk.BooleanVar(value=current_config.get('cfg.postProcessing.brightnessFilter.enable', False))
    min_brightness_slider = tk.IntVar(value=current_config.get('cfg.postProcessing.brightnessFilter.minBrightness',
                                                               default_config['cfg.postProcessing.brightnessFilter.minBrightness']))
    max_brightness_slider = tk.IntVar(value=current_config.get('cfg.postProcessing.brightnessFilter.maxBrightness',
                                                               default_config['cfg.postProcessing.brightnessFilter.maxBrightness']))

    speckle_filter_enable = tk.BooleanVar(value=current_config.get('cfg.postProcessing.speckleFilter.enable', False))
    speckle_range_slider = tk.IntVar(value=current_config.get('cfg.postProcessing.speckleFilter.speckleRange',
                                                              default_config['cfg.postProcessing.speckleFilter.speckleRange']))
    speckle_difference_threshold = tk.IntVar(value=current_config.get('cfg.postProcessing.speckleFilter.differenceThreshold',
                                                              default_config['cfg.postProcessing.speckleFilter.differenceThreshold']))

    spatial_filter_enable = tk.BooleanVar(value=current_config.get('cfg.postProcessing.spatialFilter.enable', False))
    hole_filling_radius_slider = tk.IntVar(value=current_config.get('cfg.postProcessing.spatialFilter.holeFillingRadius',
                                                                    default_config['cfg.postProcessing.spatialFilter.holeFillingRadius']))
    num_iterations_slider = tk.IntVar(value=current_config.get('cfg.postProcessing.spatialFilter.numIterations',
                                                               default_config['cfg.postProcessing.spatialFilter.numIterations']))
    alpha_slider = tk.DoubleVar(value=current_config.get('cfg.postProcessing.spatialFilter.alpha',
                                                      default_config['cfg.postProcessing.spatialFilter.alpha']))
    delta_slider = tk.IntVar(value=current_config.get('cfg.postProcessing.spatialFilter.delta',
                                                      default_config['cfg.postProcessing.spatialFilter.delta']))

    threshold_filter_enable = tk.BooleanVar(value=(True if 'cfg.postProcessing.thresholdFilter.minRange' in current_config else False))
    min_range_val = tk.IntVar(value=current_config.get('cfg.postProcessing.thresholdFilter.minRange',
                                                       default_config['cfg.postProcessing.thresholdFilter.minRange']))
    max_range_val = tk.IntVar(value=current_config.get('cfg.postProcessing.thresholdFilter.maxRange',
                                                       default_config['cfg.postProcessing.thresholdFilter.maxRange']))

    decimation_filter_enable = tk.BooleanVar(value=(True if 'cfg.postProcessing.decimationFilter.decimationFactor' in current_config else False))
    decimation_factor_val = tk.IntVar(value=current_config.get('cfg.postProcessing.decimationFilter.decimationFactor',
                                                               default_config['cfg.postProcessing.decimationFilter.decimationFactor']))
    decimation_mode_val = tk.StringVar(value=handle_dict(current_config.get('cfg.postProcessing.decimationFilter.decimationMode', default_config['cfg.postProcessing.decimationFilter.decimationMode']),
                                                         decimation_set_dict, reverse=True))

    # ----------------------------------------------------------------- BUTTONS -------------------------------------------------------------

    current_row = 0

    # Add radiobuttons for left/right choice with a label
    ttk.Label(popup_window, text="setDepthAlign").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    left_radiobutton = ttk.Radiobutton(popup_window, text="Left", variable=depth_align, value="CameraBoardSocket.LEFT")
    right_radiobutton = ttk.Radiobutton(popup_window, text="Right", variable=depth_align, value="CameraBoardSocket.RIGHT")
    rec_left_radiobutton = ttk.Radiobutton(popup_window, text="Rec Left", variable=depth_align, value='StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT')
    rec_right_radiobutton = ttk.Radiobutton(popup_window, text="Rec Right", variable=depth_align, value='StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT')
    rgb_radiobutton = ttk.Radiobutton(popup_window, text="RGB", variable=depth_align, value="CameraBoardSocket.CAM_A")
    left_radiobutton.grid(row=current_row, column=1, padx=10, pady=5, sticky="w")
    right_radiobutton.grid(row=current_row, column=2, padx=10, pady=5, sticky="w")
    rec_left_radiobutton.grid(row=current_row, column=3, padx=10, pady=5, sticky="w")
    rec_right_radiobutton.grid(row=current_row, column=4, padx=10, pady=5, sticky="w")
    rgb_radiobutton.grid(row=current_row, column=5, padx=10, pady=5, sticky="w")
    current_row += 1

    #
    ttk.Label(popup_window, text="setDefaultProfilePreset").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    Hdef_radiobutton = ttk.Radiobutton(popup_window, text="DEFAULT", variable=profile_preset, value="node.StereoDepth.PresetMode.DEFAULT")
    HA_radiobutton = ttk.Radiobutton(popup_window, text="HIGH_ACCURACY", variable=profile_preset, value="node.StereoDepth.PresetMode.HIGH_ACCURACY")
    HD_radiobutton = ttk.Radiobutton(popup_window, text="HIGH_DENSITY", variable=profile_preset, value="node.StereoDepth.PresetMode.HIGH_DENSITY")
    HR_radiobutton = ttk.Radiobutton(popup_window, text="ROBOTICS", variable=profile_preset, value="node.StereoDepth.PresetMode.ROBOTICS")
    HDE_radiobutton = ttk.Radiobutton(popup_window, text="HIGH_DETAIL", variable=profile_preset, value="node.StereoDepth.PresetMode.HIGH_DETAIL")
    HF_radiobutton = ttk.Radiobutton(popup_window, text="FACE", variable=profile_preset, value="node.StereoDepth.PresetMode.FACE")
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
            toggle_frame_settings(threshold_filter_enable, threshold_frame)
            toggle_frame_settings(bilateral_filter_enable, bilateral_frame)
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
    ttk.Label(popup_window, text="Use Custom Settings").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    cust_box = ttk.Checkbutton(popup_window, variable=custom_settings_val, command=toggle_custom_frame_settings)
    cust_box.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
    current_row += 1

    custom_settings_frame = ttk.LabelFrame(popup_window, text="Custom Settings", padding=(10, 10))
    custom_settings_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

    # # Add checkbuttons for rectification
    ttk.Label(custom_settings_frame, text="setRectification").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    recbox = ttk.Checkbutton(custom_settings_frame, variable=rectificationBox_val)
    recbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
    current_row += 1

    # LR CHECK
    ttk.Label(custom_settings_frame, text="setLRcheck").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    LRbox = ttk.Checkbutton(custom_settings_frame, variable=LRBox_val)
    LRbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
    current_row += 1

    # EXTENDED DISPARITY
    ttk.Label(custom_settings_frame, text="setExtendedDisparity").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    extbox = ttk.Checkbutton(custom_settings_frame, variable=extendedBox_val)
    extbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
    current_row += 1

    # SetSubpixel Label and Checkbox
    ttk.Label(custom_settings_frame, text="setSubpixel").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    subbox = ttk.Checkbutton(custom_settings_frame, variable=subpixelBox_val)
    subbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")

    # Add subpixelFractionalBits combobox
    ttk.Label(custom_settings_frame, text="subpixelFractionalBits").grid(row=current_row, column=2, padx=10, pady=10, sticky="w")
    fractional_bits_combo = ttk.Combobox(custom_settings_frame, textvariable=fractional_bits, values=[3, 4, 5],
                                         state="readonly")
    fractional_bits_combo.grid(row=current_row, column=3, padx=10, pady=10, sticky="e")

    current_row += 1  # Increment the row


    # Decimation Filter Enable
    ttk.Label(custom_settings_frame, text="Decimation Filter Enable").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    decimation_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=decimation_filter_enable, command=lambda: toggle_frame_settings(decimation_filter_enable, decimation_frame))
    decimation_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")

    current_row += 1  # Increment the row

    # Decimation Filter Group (in a LabelFrame with a black border)
    decimation_frame = ttk.LabelFrame(custom_settings_frame, text="Decimation Filter", padding=(10, 10))
    decimation_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

    # Decimation Filter (Factor and Mode on one row)
    ttk.Label(decimation_frame, text="Factor").grid(row=1, column=0, padx=10, pady=10, sticky="w")
    decimation_factor_label = ttk.Label(decimation_frame)
    decimation_factor_label.grid(row=1, column=2, padx=10, pady=10, sticky="w")
    decimation_factor_dropdown = ttk.Combobox(decimation_frame, values=[1, 2, 3, 4], textvariable=decimation_factor_val,
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
    ttk.Label(custom_settings_frame, text="Median Enable").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    median_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=median_filter_enable, command=lambda: toggle_frame_settings(median_filter_enable, median_frame))
    median_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
    current_row += 1

    # Median Filter Frame
    median_frame = ttk.LabelFrame(custom_settings_frame, text="Median Filter", padding=(10, 10))
    median_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

    ttk.Label(median_frame, text="Median").grid(row=0, column=0, padx=10, pady=10, sticky="w")
    # median_off_button = ttk.Radiobutton(median_frame, text="MEDIAN_OFF", variable=median_val, value="MEDIAN_OFF")
    median_3_button = ttk.Radiobutton(median_frame, text="MEDIAN_3x3", variable=median_val, value="MedianFilter.KERNEL_3x3")
    median_5_button = ttk.Radiobutton(median_frame, text="MEDIAN_5x5", variable=median_val, value="MedianFilter.KERNEL_5x5")
    median_7_button = ttk.Radiobutton(median_frame, text="MEDIAN_7x7", variable=median_val, value="MedianFilter.KERNEL_7x7")
    # median_off_button.grid(row=current_row, column=1, padx=10, pady=5, sticky="w")
    median_3_button.grid(row=0, column=1, padx=10, pady=5, sticky="w")
    median_5_button.grid(row=0, column=2, padx=10, pady=5, sticky="w")
    median_7_button.grid(row=0, column=3, padx=10, pady=5, sticky="w")
    current_row += 1


    # Speckle Filter Enable
    ttk.Label(custom_settings_frame, text="Speckle Filter Enable").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    speckle_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=speckle_filter_enable, command=lambda: toggle_frame_settings(speckle_filter_enable, speckle_frame))
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
    ttk.Label(speckle_frame, text="Speckle Difference Threshold").grid(row=2, column=0, padx=10, pady=10, sticky="w")
    speckle_difference_label = ttk.Label(speckle_frame, text=str(speckle_difference_threshold.get()))
    speckle_difference_spinbox = ttk.Spinbox(speckle_frame, from_=0, to=256, textvariable=speckle_difference_threshold,
                                             command=lambda: update_label(speckle_difference_threshold,
                                                                          speckle_difference_label))
    speckle_difference_spinbox.grid(row=2, column=1, padx=10, pady=10, sticky="e")
    ttk.Label(speckle_frame, text="(0, 256)").grid(row=2, column=2, padx=10, pady=10, sticky="w")
    current_row += 1




    # Spatial Filter Enable
    ttk.Label(custom_settings_frame, text="Spatial Filter Enable").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    spatial_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=spatial_filter_enable, command=lambda: toggle_frame_settings(spatial_filter_enable, spatial_frame))
    spatial_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
    current_row += 1

    # Spatial Filter Frame
    spatial_frame = ttk.LabelFrame(custom_settings_frame, text="Spatial Filter", padding=(10, 10))
    spatial_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

    # Spatial Filter Settings
    ttk.Label(spatial_frame, text="Hole Filling Radius").grid(row=1, column=0, padx=10, pady=10, sticky="w")
    hole_filling_radius_label = ttk.Label(spatial_frame, text=str(hole_filling_radius_slider.get()))
    hole_filling_radius_spinbox = ttk.Spinbox(spatial_frame, from_=0, to=255, textvariable=hole_filling_radius_slider,
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
              command=lambda x: update_label(alpha_slider, alpha_label, form="float")).grid(row=2, column=1, padx=10, pady=10,
                                                                              sticky="e")

    ttk.Label(spatial_frame, text="Delta").grid(row=2, column=3, padx=10, pady=10, sticky="w")
    delta_label = ttk.Label(spatial_frame, text=str(delta_slider.get()))
    delta_spinbox = ttk.Spinbox(spatial_frame, from_=0, to=255, textvariable=delta_slider,
                                command=lambda: update_label(delta_slider, delta_label))
    delta_spinbox.grid(row=2, column=4, padx=10, pady=10, sticky="e")
    ttk.Label(spatial_frame, text="(0, 255)").grid(row=2, column=5, padx=10, pady=10, sticky="w")

    current_row += 1


    # Temporal Filter Frame
    temporal_frame = ttk.LabelFrame(custom_settings_frame, text="Temporal Filter", padding=(10, 10))
    temporal_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

    # Temporal Filter Enable
    ttk.Label(temporal_frame, text="Temporal filter not available due to the nature of data").grid(row=0, column=0, padx=10, pady=10, sticky="w")

    current_row += 1



    # Threshold Filter Enable
    ttk.Label(custom_settings_frame, text="Threshold Filter Enable").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    threshold_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=threshold_filter_enable, command=lambda: toggle_frame_settings(threshold_filter_enable, threshold_frame))
    threshold_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
    current_row += 1

    # Threshold Filter Frame
    threshold_frame = ttk.LabelFrame(custom_settings_frame, text="Threshold Filter", padding=(10, 10))
    threshold_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

    # Threshold Filter Min and Max with Spinboxes
    ttk.Label(threshold_frame, text="Min").grid(row=1, column=0, padx=10, pady=10, sticky="w")
    min_range_spinbox = ttk.Spinbox(threshold_frame, from_=0, to=int(max_range_val.get()), textvariable=min_range_val,
                                    width=10)
    min_range_spinbox.grid(row=1, column=1, padx=10, pady=10, sticky="e")
    ttk.Label(threshold_frame, text="(0, " + str(max_range_val.get()) + ")").grid(row=1, column=2, padx=10, pady=10,
                                                                                  sticky="w")

    ttk.Label(threshold_frame, text="Max").grid(row=1, column=3, padx=10, pady=10, sticky="w")
    max_range_spinbox = ttk.Spinbox(threshold_frame, from_=int(min_range_val.get()), to=65535,
                                    textvariable=max_range_val, width=10)
    max_range_spinbox.grid(row=1, column=4, padx=10, pady=10, sticky="e")
    ttk.Label(threshold_frame, text="(" + str(min_range_val.get()) + ", 65535)").grid(row=1, column=5, padx=10, pady=10,
                                                                                      sticky="w")

    current_row += 1

    # Bilateral Filter Enable
    ttk.Label(custom_settings_frame, text="Bilateral Filter Enable (NOT TESTED IN GUI)").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    bilateral_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=bilateral_filter_enable, command=lambda: toggle_frame_settings(bilateral_filter_enable, bilateral_frame))
    bilateral_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
    current_row += 1

    # Bilateral Filter Frame
    bilateral_frame = ttk.LabelFrame(custom_settings_frame, text="Bilateral Filter", padding=(10, 10))
    bilateral_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

    # Bilateral Sigma Value
    ttk.Label(bilateral_frame, text="Bilateral Sigma Value").grid(row=1, column=0, padx=10, pady=10, sticky="w")
    bilateral_sigma_label = ttk.Label(bilateral_frame, text=str(bilateral_sigma_val.get()))
    bilateral_sigma_label.grid(row=1, column=2, padx=10, pady=10, sticky="w")
    bilateral_sigma_slider = ttk.Scale(bilateral_frame, from_=0, to=20, variable=bilateral_sigma_val,
                                       orient="horizontal",
                                       command=lambda x: update_label(bilateral_sigma_val, bilateral_sigma_label))
    bilateral_sigma_slider.grid(row=1, column=1, padx=10, pady=10, sticky="e")
    current_row += 1


    # Brightness Filter Enable
    ttk.Label(custom_settings_frame, text="Brightness Filter Enable (NOT TESTED IN GUI)").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    brightness_filter_checkbox = ttk.Checkbutton(custom_settings_frame, variable=brightness_filter_enable, command=lambda: toggle_frame_settings(brightness_filter_enable, brightness_frame))
    brightness_filter_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")
    current_row += 1

    # Brightness Filter Frame
    brightness_frame = ttk.LabelFrame(custom_settings_frame, text="Brightness Filter", padding=(10, 10))
    brightness_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

    # Brightness Filter Sliders
    ttk.Label(brightness_frame, text="Min Brightness").grid(row=1, column=0, padx=10, pady=10, sticky="w")
    min_brightness_label = ttk.Label(brightness_frame, text=str(min_brightness_slider.get()))
    min_brightness_label.grid(row=1, column=2, padx=10, pady=10, sticky="w")
    ttk.Scale(brightness_frame, from_=0, to=int(max_brightness_slider.get()), variable=min_brightness_slider, orient="horizontal",
              command=lambda x: update_label(min_brightness_slider, min_brightness_label)).grid(row=1, column=1,
                                                                                                padx=10, pady=10,
                                                                                                sticky="e")

    ttk.Label(brightness_frame, text="Max Brightness").grid(row=1, column=3, padx=10, pady=10, sticky="w")
    max_brightness_label = ttk.Label(brightness_frame, text=str(max_brightness_slider.get()))
    max_brightness_label.grid(row=1, column=5, padx=10, pady=10, sticky="w")
    ttk.Scale(brightness_frame, from_=int(min_brightness_slider.get()), to=255, variable=max_brightness_slider, orient="horizontal",
              command=lambda x: update_label(max_brightness_slider, max_brightness_label)).grid(row=1, column=4,
                                                                                                padx=10, pady=10,
                                                                                                sticky="e")
    current_row += 1


    # Filter Order Enable
    ttk.Label(custom_settings_frame, text="Filter Order Selection Enable (NOT TESTED IN GUI)").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    send_filter_order_checkbox = ttk.Checkbutton(custom_settings_frame, variable=filtering_order_enable, command=lambda: toggle_frame_settings(filtering_order_enable, order_frame))
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
    # todo - add default depthai values

    advanced_settings_enable = tk.BooleanVar(value=False)
    mean_mode_enable = tk.BooleanVar(value=False)
    CT_kernel_val = tk.StringVar(value='KERNEL_5x5')
    CT_threshold_val = tk.IntVar(value=0)
    division_factor_val = tk.IntVar(value=1)
    horizontal_penalty_p1_val = tk.IntVar(value=0)
    horizontal_penalty_p2_val = tk.IntVar(value=0)
    vertical_penalty_p1_val = tk.IntVar(value=0)
    vertical_penalty_p2_val = tk.IntVar(value=0)
    confidence_threshold_val = tk.IntVar(value=0)
    CM_alpha_val = tk.IntVar(value=0)
    CM_beta_val = tk.IntVar(value=0)
    matching_threshold_val = tk.IntVar(value=0)

    def toggle_advanced_settings():
        if advanced_settings_enable.get():
            enable_frame_widgets(advanced_stereo_setting_frame, True)
        else:
            enable_frame_widgets(advanced_stereo_setting_frame, False)

    ttk.Label(popup_window, text="Enable Advanced Settings (I know what I'm doing)").grid(row=current_row, column=0, padx=10, pady=10, sticky="w")
    advanced_settings_checkbox = ttk.Checkbutton(popup_window, variable=advanced_settings_enable, command=toggle_advanced_settings)
    advanced_settings_checkbox.grid(row=current_row, column=1, padx=10, pady=10, sticky="e")

    current_row += 1

    advanced_stereo_setting_frame = ttk.LabelFrame(popup_window, text="Advanced Settings", padding=(10, 10))
    advanced_stereo_setting_frame.grid(row=current_row, column=0, columnspan=6, padx=10, pady=10, sticky="ew")

    ttk.Label(advanced_stereo_setting_frame, text="censusTransform.kernelSize").grid(row=1, column=0, padx=10, pady=10, sticky="w")
    CT_kernel_dropdown = ttk.Combobox(advanced_stereo_setting_frame, textvariable=CT_kernel_val, values=[
        'KERNEL_5x5',
        'KERNEL_7x7',
        'KERNEL_7x9'
    ], state="readonly")
    CT_kernel_dropdown.grid(row=1, column=1, padx=10, pady=10, sticky="e")

    ttk.Label(advanced_stereo_setting_frame, text="censusTransform.enableMeanMode").grid(row=2, column=0, padx=10, pady=10,
                                                                                   sticky="w")
    mean_mode_checkbox = ttk.Checkbutton(advanced_stereo_setting_frame, variable=mean_mode_enable)
    mean_mode_checkbox.grid(row=2, column=1, padx=10, pady=10, sticky="e")

    ttk.Label(advanced_stereo_setting_frame, text="censusTransform.threshold").grid(row=3, column=0, padx=10, pady=10,
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
    ttk.Label(advanced_stereo_setting_frame, text="costMatching.linearEquationParameters.alpha").grid(row=10, column=0,
                                                                                                      padx=10, pady=10,
                                                                                                      sticky="w")
    CM_alpha_slider = ttk.Scale(advanced_stereo_setting_frame, from_=0, to=10, variable=CM_alpha_val, orient="horizontal",
                             command=lambda x: update_label(CM_alpha_slider, CM_alpha_label))
    CM_alpha_slider.grid(row=10, column=1, padx=10, pady=10, sticky="ew")
    CM_alpha_label = ttk.Label(advanced_stereo_setting_frame, text=str(int(CM_alpha_val.get())))
    CM_alpha_label.grid(row=10, column=2, padx=10, pady=10, sticky="w")

    # Beta Slider
    ttk.Label(advanced_stereo_setting_frame, text="costMatching.linearEquationParameters.beta").grid(row=11, column=0,
                                                                                                     padx=10, pady=10,
                                                                                                     sticky="w")
    CM_beta_slider = ttk.Scale(advanced_stereo_setting_frame, from_=0, to=10, variable=CM_beta_val, orient="horizontal",
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
                                command=on_generate)
    generate_button.grid(row=current_row, column=0, columnspan=2, pady=20)
    current_row += 1
    # ------------------------------------------------------------------------------------------------------------------------------

    toggle_custom_frame_settings()
    toggle_advanced_settings() # turn off by default

    popup_window.grab_set()  # Make the window modal (disable interaction with the main window)
    popup_window.wait_window()  # Wait for the popup window to be destroyed

    return config


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
