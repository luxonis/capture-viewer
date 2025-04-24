import json
from .dictionary_tools import *

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