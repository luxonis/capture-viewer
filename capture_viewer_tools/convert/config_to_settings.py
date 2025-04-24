import json
from .dictionary_tools import *

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
            "median_size": handle_dict(config.get('stereo.initialConfig.setMedianFilter', "dai.MedianFilter.MEDIAN_OFF"),
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