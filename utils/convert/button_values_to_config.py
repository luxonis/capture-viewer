from .dictionary_tools import *
from ..popup_info import show_popup

def convert_current_button_values_to_config(button_values, frame):
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