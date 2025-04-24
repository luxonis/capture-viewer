setDepthAlign_dict = {
    "Left": "dai.CameraBoardSocket.LEFT",
    "Right": "dai.CameraBoardSocket.RIGHT",
    "RecLeft": "dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT",
    "RecRight": "dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT",
    "LEFT": "dai.CameraBoardSocket.LEFT",
    "RIGHT": "dai.CameraBoardSocket.RIGHT",
    "REC_LEFT": "dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT",
    "REC_RIGHT": "dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT",
    "RGB": "dai.CameraBoardSocket.CAM_A",
}

profilePreset_dict = {
    "HIGH_ACCURACY": "dai.node.StereoDepth.PresetMode.HIGH_ACCURACY",
    "HIGH_DENSITY": "dai.node.StereoDepth.PresetMode.HIGH_DENSITY"
}

median_dict = {
    "MEDIAN_OFF": "dai.MedianFilter.MEDIAN_OFF",
    "MEDIAN_3x3": "dai.MedianFilter.KERNEL_3x3",
    "MEDIAN_5x5": "dai.MedianFilter.KERNEL_5x5",
    "MEDIAN_7x7": "dai.MedianFilter.KERNEL_7x7",
}

decimation_set_dict = {
    'NON_ZERO_MEAN': 'dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEAN',
    'NON_ZERO_MEDIAN': 'dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEDIAN',
    'PIXEL_SKIPPING': 'dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.PIXEL_SKIPPING'
}

CT_kernel_dict = {
    "KERNEL_AUTO": "dai.StereoDepthConfig.CensusTransform.KernelSize.AUTO",
    "KERNEL_5x5": "dai.StereoDepthConfig.CensusTransform.KernelSize.KERNEL_5x5",
    "KERNEL_7x7": "dai.StereoDepthConfig.CensusTransform.KernelSize.KERNEL_7x7",
    "KERNEL_7x9": "dai.StereoDepthConfig.CensusTransform.KernelSize.KERNEL_7x9"
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