import json

# todo - add filter parameter settings to config2settings

setDepthAlign_dict = {
    "Left": "CameraBoardSocket.LEFT",
    "Right": "CameraBoardSocket.RIGHT",
    "RecLeft": "StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT",
    "RecRight": "StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT",
    "LEFT": "CameraBoardSocket.LEFT",
    "RIGHT": "CameraBoardSocket.RIGHT",
    "REC_LEFT": "StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT",
    "REC_RIGHT": "StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT",
    "RGB": "CameraBoardSocket.CAM_A",
}

profilePreset_dict = {
    "HIGH_ACCURACY": "node.StereoDepth.PresetMode.HIGH_ACCURACY",
    "HIGH_DENSITY": "node.StereoDepth.PresetMode.HIGH_DENSITY"
}

median_dict = {
    "MEDIAN_OFF": "MedianFilter.MEDIAN_OFF",
    "MEDIAN_3x3": "MedianFilter.KERNEL_3x3",
    "MEDIAN_5x5": "MedianFilter.KERNEL_5x5",
    "MEDIAN_7x7": "MedianFilter.KERNEL_7x7",
}

decimation_set_dict = {
    'NON_ZERO_MEAN': 'StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEAN',
    'NON_ZERO_MEDIAN': 'StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEDIAN',
    'PIXEL_SKIPPING': 'StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.PIXEL_SKIPPING'
}

CT_kernel_dict = {
    "KERNEL_5x5": "StereoDepthConfig.CensusTransform.KernelSize.KERNEL_5x5",
    "KERNEL_7x7": "StereoDepthConfig.CensusTransform.KernelSize.KERNEL_7x7",
    "KERNEL_7x9": "StereoDepthConfig.CensusTransform.KernelSize.KERNEL_7x9"
}


# Generic conversion function
def handle_dict(value, key_dict, reverse=False):
    """
    Convert a value using the given dictionary and reverse option.

    :param value: The value to be converted.
    :param key_dict: The dictionary used for conversion.
    :param reverse: If True, reverse the key-value pairs for lookup.
    :return: Converted value or key.
    """
    if reverse:
        # Reverse the dictionary and return the key for the value
        reverse_dict = {v: k for k, v in key_dict.items()}
        return reverse_dict[value]
    return key_dict[value]

# Function to convert settings JSON into the application's config format
def settings2config(settings_json):
    config = {}

    # Parse the settings JSON
    if type(settings_json) is str:
        settings = json.loads(settings_json)
    elif type(settings_json) is dict:
        settings = settings_json
    else:
        raise Exception(f"Invalid settings_json type: {type(settings_json)}")

    # Depth align
    if settings.get('stereoAlign', False):
        config['stereo.setDepthAlign'] = handle_dict(settings['alignSocket'], setDepthAlign_dict)
    else:
        config['stereo.setDepthAlign'] = handle_dict("REC_RIGHT", setDepthAlign_dict)

    if config['stereo.setDepthAlign'] == "StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT" or config[
        'stereo.setDepthAlign'] == "StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT":
        config['stereo.setRectification'] = True
    else:
        config['stereo.setRectification'] = False

    # General settings
    config['stereo.setExtendedDisparity'] = settings['extendedDisparity']
    config['stereo.setSubpixel'] = settings['subpixelDisparity']
    config['stereo.setSubpixelFractionalBits'] = int(settings['subpixelValue'])
    config['stereo.setLeftRightCheck'] = settings.get('LRcheck', True)
    config['stereo.setDefaultProfilePreset'] = handle_dict('HIGH_ACCURACY' if settings['highAccuracy'] else 'HIGH_DENSITY', profilePreset_dict)

    # Filters
    if settings['filters_on']:
        filters = settings['filters']
        # Median filter
        if filters.get('median_filter', False):  # Changed to get with default
            config['stereo.initialConfig.setMedianFilter'] = handle_dict(filters['median_size'], median_dict)
        # Bilateral filter
        if filters.get('bilateral_filter', False):  # default is false
            config['cfg.postProcessing.bilateralSigmaValue'] = filters['bilateral_sigma']
        # Brightness filter
        if filters.get('brightness_filter', False):  # default is false
            config['cfg.postProcessing.brightnessFilter.maxBrightness'] = filters['brightness_filter_max']
            config['cfg.postProcessing.brightnessFilter.minBrightness'] = filters['brightness_filter_min']
        # Speckle filter
        if filters.get('speckle_filter', False):  # default is false
            config['cfg.postProcessing.speckleFilter.enable'] = filters['speckle_filter']
            config['cfg.postProcessing.speckleFilter.speckleRange'] = filters['speckle_range']
        # Spatial filter
        if filters.get('spacial_filter', False):  # default is false
            config['cfg.postProcessing.spatialFilter.enable'] = filters['spacial_filter']
            # config['cfg.postProcessing.spatialFilter.holeFillingRadius'] = hole_filling_radius_slider.get()
            # config['cfg.postProcessing.spatialFilter.numIterations'] = num_iterations_slider.get()
            # config['cfg.postProcessing.spatialFilter.alpha'] = alpha_slider.get()
            # config['cfg.postProcessing.spatialFilter.delta'] = delta_slider.get()
        # Threshold filter
        if filters.get('threshold_filter', False):  # default is false
            config['cfg.postProcessing.thresholdFilter.minRange'] = filters['lower_threshold_filter']
            config['cfg.postProcessing.thresholdFilter.maxRange'] = filters['upper_threshold_filter']
        # Temporal filter
        if filters.get('temporal_filter', False):  # default is false
            config['cfg.postProcessing.temporalFilter.enable'] = filters['temporal_filter']
            # config missing
        # Decimation filter
        if filters.get('decimation_filter', False):  # default is false
            config['cfg.postProcessing.decimationFilter.decimationFactor'] = filters['decimation_factor']
            print(handle_dict(filters.get('decimation_mode', 'PIXEL_SKIPPING'), decimation_set_dict))
            config['cfg.postProcessing.decimationFilter.decimationMode'] = handle_dict(filters.get('decimation_mode', 'PIXEL_SKIPPING'), decimation_set_dict)
    return config

def config2settings(config, capture_data):
    if type(config) == str:
        config = json.loads(config)
    filters_on = ('cfg.postProcessing.decimationFilter.decimationFactor' in config
                  or 'cfg.postProcessing.thresholdFilter.minRange' in config
                  or config.get('cfg.postProcessing.spatialFilter.enable')
                  or config.get('cfg.postProcessing.speckleFilter.enable', False)
                  or 'stereo.initialConfig.setMedianFilter' in config
                  or 'cfg.postProcessing.bilateralSigmaValue' in config
                  or 'cfg.postProcessing.brightnessFilter.maxBrightness' in config)
    settings = {
        "ir": capture_data["ir"],
        "ir_value": capture_data["ir_value"],
        "flood_light": capture_data["flood_light"],
        "flood_light_intensity": capture_data["flood_light_intensity"],
        "stereoAlign": "stereo.setDepthAlign" in config,
        "alignSocket": handle_dict(config.get("stereo.setDepthAlign", "REC_RIGHT"), setDepthAlign_dict, reverse=True),
        "autoexposure": capture_data["autoexposure"],
        "expTime": capture_data["expTime"],
        "sensIso": capture_data["sensIso"],
        "LRcheck": config.get("LRcheck", True),
        "extendedDisparity": config.get('stereo.setExtendedDisparity', False),
        "subpixelDisparity": config.get('stereo.setSubpixel', False),
        "subpixelValue": config.get('stereo.setSubpixelFractionalBits', 3),
        "highAccuracy": config.get('stereo.setDefaultProfilePreset') == handle_dict('HIGH_ACCURACY', profilePreset_dict,
                                                                                    reverse=False),
        "highDensity": config.get('stereo.setDefaultProfilePreset') == handle_dict('HIGH_DENSITY', profilePreset_dict,
                                                                                   reverse=False),
        "filters_on": filters_on,
        "filters": {
            "threshold_filter": True if 'cfg.postProcessing.thresholdFilter.minRange' in config else False,
            "lower_threshold_filter": config.get('cfg.postProcessing.thresholdFilter.minRange', 0),
            "upper_threshold_filter": config.get('cfg.postProcessing.thresholdFilter.maxRange', 15000),

            "decimation_filter": 'cfg.postProcessing.decimationFilter.decimationFactor' in config,
            "decimation_factor": config.get('cfg.postProcessing.decimationFilter.decimationFactor', 2),
            "decimation_mode": handle_dict(config.get('cfg.postProcessing.decimationFilter.decimationMode','StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.PIXEL_SKIPPING'),
                                           decimation_set_dict, reverse=True),

            "spacial_filter": config.get('cfg.postProcessing.spatialFilter.enable', False),

            "temporal_filter": config.get('cfg.postProcessing.temporalFilter.enable', False),

            "speckle_filter": config.get('cfg.postProcessing.speckleFilter.enable', False),
            "speckle_range": config.get('cfg.postProcessing.speckleFilter.speckleRange', 50),

            "median_filter": 'stereo.initialConfig.setMedianFilter' in config,
            "median_size": handle_dict(config.get('stereo.initialConfig.setMedianFilter', "MedianFilter.MEDIAN_OFF"),
                                       median_dict, reverse=True),

            "bilateral_filter": 'cfg.postProcessing.bilateralSigmaValue' in config,
            "bilateral_sigma": config.get('cfg.postProcessing.bilateralSigmaValue', 0),

            "brightness_filter": 'cfg.postProcessing.brightnessFilter.maxBrightness' in config,
            "brightness_filter_max": config.get('cfg.postProcessing.brightnessFilter.maxBrightness', 0),
            "brightness_filter_min": config.get('cfg.postProcessing.brightnessFilter.minBrightness', 0)
        },
        "FPS": capture_data["FPS"],
        "num_captures": capture_data["num_captures"],
        "replay_generated": True,
        "custom_advanced_settings": True if 'cfg.censusTransform.kernelSize' in config else False
    }

    return settings

if __name__ == "__main__":
    # config = "{'stereo.setDepthAlign': 'StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT', 'stereo.setDefaultProfilePreset': 'node.StereoDepth.PresetMode.HIGH_ACCURACY', 'stereo.setRectification': True, 'stereo.setLeftRightCheck': True, 'stereo.setExtendedDisparity': True, 'stereo.setSubpixel': False, 'stereo.initialConfig.setMedianFilter': 'MedianFilter.MEDIAN_OFF', 'cfg.postProcessing.speckleFilter.enable': False, 'cfg.postProcessing.spatialFilter.enable': False}"

    # with open("../settings_jsons/settings_default.json", "r") as settings_json:
    #     settings_original = json.load(settings_json)

    with open("/home/katka/PycharmProjects/capture-viewer/DATA/OAK-D-PRO_20241217140500/metadata.json", "r") as metadata:
        metadata1 = json.load(metadata)
        settings_original = metadata1["settings"]

    config = settings2config(settings_original)
    print(json.dumps(config, indent=4))

    data_capture = {
        "ir": settings_original["ir"],
        "ir_value": settings_original["ir_value"],
        "flood_light": settings_original["flood_light"],
        "flood_light_intensity": settings_original["flood_light_intensity"],
        "autoexposure": settings_original["autoexposure"],
        "expTime": settings_original["expTime"],
        "sensIso": settings_original["sensIso"],
        "FPS": settings_original["FPS"],
        "num_captures": settings_original["num_captures"],
    }

    settings = config2settings(config, data_capture)
    print("ORIGINAL")
    print(json.dumps(settings_original, indent=4))
    print("GENERATED")
    print(json.dumps(settings, indent=4))
    print(json.dumps(settings_original, indent=4) == json.dumps(settings, indent=4))