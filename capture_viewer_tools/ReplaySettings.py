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
    'cfg.postProcessing.temporalFilter.enable': False,
    'cfg.postProcessing.temporalFilter.alpha': 0.5,
    'cfg.postProcessing.temporalFilter.delta': 3,
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
    show_popup("Warning", "FILTERING ORDER IS NOT VALID")
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
