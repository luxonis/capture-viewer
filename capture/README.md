# CAPTURE GUIDE

You can now run capture for OAKs stereo and OAK-D-SR-POE using one script.

```bash
python capture/oak_capture.py default my_capture
```

Arguments:
- **settings** - choose `default`, `default_tof` or add path to your own [settings.json](#settings).
- **view_name** - the name of capture that will be saved in metadata file
- **--output** (optional) - path to custom output folder, where the output of capture will be created. By default, it will be saved in DATA folder in capture-viewer
- **--autostart** (optional) - number of seconds after which the capture automatically starts
  - If you did not select **--autostart** you can start the capture by pressing `S`. The script will automatically save predefined number of frames to a folder named `DATA`. 
- **--devices** (optional) - the **MXIDs** or **IPs** of devices to connnect to. If left blank, it will connect to the first available device.

---
## Settings

For capture, you can configure various parameters. Below is an example settings configuration:

```json
{
    "ir": true,
    "ir_value": 1,
    "flood_light": false,
    "flood_light_intensity": 1,
    "stereoResolution": "THE_800_P",
    "rgbResolution": "THE_1080_P",
    "stereoAlign": true,
    "alignSocket": "COLOR",
    "autoexposure": true,
    "expTime": 200,
    "sensIso": 800,
    "output_settings": {
        "left": true,
        "right": true,
        "left_raw": false,
        "right_raw": false,
        "rgb": false,
        "rgb_png": false,
        "depth": true,
        "disparity": true
    },
    "LRcheck": true,
    "extendedDisparity": false,
    "subpixelDisparity": true,
    "subpixelValue": 3,
    "profilePreset": "ROBOTICS",
    "use_filter_settings": false,
    "filters": {
        "threshold_filter": false,
        "lower_threshold_filter": 1000,
        "upper_threshold_filter": 5000,
        "decimation_filter": false,
        "decimation_factor": 1,
        "decimation_mode" : "PIXEL_SKIPPING",
        "spacial_filter": false,
        "spatial_alfa": 0.5,
        "spatial_delta": 3,
        "spatial_num_iterations": 1,
        "spatial_hole_filling_radius": 1,
        "temporal_filter": false,
        "temporal_alfa": 0.5,
        "temporal_delta": 3,
        "speckle_filter": false,
        "speckle_range": 200,
        "speckle_difference_threshold": 2,

        "median_filter": false,
        "median_size": "KERNEL_3x3"
    },
    "FPS": 30,
    "num_captures": 20
}


```

This settings need to be configured before the capture start. To change the settings, simply 
edit parameters in the json and restart the capture script.

- `ir` : IR dot projector, available on PRO models.
- `flood_light` : IR floodlight, available on PRO models.
- If `autoexposure` is set to false, `expTime` and `sensIso` will be used.
- `extendedDisparity` : extendedDisparity
- `subpixelDisparity`: subpixelDisparity with the selected `subpixelValue`
- `profilePreset`: select a Profile Preset such as `DEFAULT`, `HIGH_DETAIL`, `FACE` or `ROBOTICS` based on your needs. 
Profiles `HIGH_ACCURACY` and `HIGH_DENSITY` are also available but will be removed in future versions/
- `output_settings` : select what streams you'd like to visualize. Some stream combinations might not be valid 
(such as choosing depth stream but not selecting left and right)
- `use_filter_settings`: **if set to false, none of the following filters will be used**. Set to true to use custom filter settings.
  - `filters`: self-explanatory
- `FPS`: will be applied to mono camera streams as well as the color camera
- `num_captures` : the number of depth frames which will be saved before the capture ending

These are the basic information about the capture. Settings for the capture can be edited and re-applied using the **Replay** feature 
in the capture viewer. 
---
## RVC4 capture
To make a capture with OAK4 devices, use `rvc4_capture.py` script.

```bash
python capture/rvc4_capture.py rvc4 <view_name> --ip <ip>
```

the available settings for rvc4 are the following:
```json
{
    "ir": true,
    "ir_value": 1,
    "flood_light": false,
    "flood_light_intensity": 1,
    "stereoResolution": {"x": 1280, "y": 800},
    "rgbResolution": {"x": 640, "y": 480},
    "monoSettings": {
        "luma_denoise": 1,  // levels are 0, 1, 2, 3, 4 but the difference is only between 1 (off) and 2 (on)
        "sharpness": 2,
        "contrast": 0
    },

    "output_settings": {
        "left": true,
        "right": true,
        "rgb": true,
        "depth": true,
        "disparity": true,
        "sync": true
    },

    "profilePreset": "DEFAULT",
    "LRcheck": true,
    "extendedDisparity": true,

    "FPS": 5,
    "num_captures": 20
}

```


---

## Multiple TOF capture

For multiple TOF capture you need to configure the appropriate settings json as:
- turn ON RIGHT stream and turn OFF LEFT stream
- turn OFF sync node - that configures software synchronization which might interfere with fsync
- Connect the cameras with FSYNC cables

Appropriate settings are saved in `tof_multiple.json`

If the devices are OAK D SR POE and connected with fsync, the script will automatically
configure the appropriate fsync settings. Trying to run multiple TOFs without fsync cable connected might cause problems.

```bash
python capture/oak_capture.py <appropriate_tof_config> <capture_name> --devices <mxids_of_tofs>
```

---
## Thermal capture

To make a capture with thermal devices, use `thermal_capture.py` script.

```bash
python capture/thermal_capture.py default_thermal <view_name> --device-ip <ip>
```

Most of the settings are not implemented yet.


---

## Depthai 3 Capture (for RVC4)
Capture with RVC4 still has limited settings as not all the functions were implemented and/or are reliably running on all devices.

To change settings for RVC4 capture, edit the `rvc4.json` file.

```bash
python capture/rvc4_capture.py rvc4 <rvc4_capture_name> --ip <camera_ip>
```


# ZED capture
This module allows capturing left, right and neural stereo depth ZED device, saving data in a format compatible with DepthAI pipelines (e.g., ZED-to-OAK mimic format).

OAK-like calibration file is generated. This file includes a dummy rgb camera to allow for replay functions to work normally.


## Requirements
To use this script, install the [ZED SDK](https://www.stereolabs.com/developers/) and its [Python API](https://www.stereolabs.com/docs/api/python/getting-started/). After installing the SDK, run `pip install pyzed` to get the Python bindings.

## Run

```bash
python capture/zed_capture.py my_scene --output DATA --settings settings_jsons/zed_settings.json
```

- Press s to start capturing frames
- Press q to quit
- Use --autostart 5 to begin automatically after 5 seconds

## Settings
You can modify the capture settings in the `settings_jsons/zed_settings.json`


# Intel Realsense capture
This module allows capturing left, right, depth, and optional RGB streams from an Intel RealSense device, saving data in a format compatible with DepthAI pipelines (e.g., Realsense-to-OAK mimic format).

OAK-like calibration file is generated.
## Requirements

```bash
   pip install pyrealsense2
```

## Run
```bash
python capture/realsense_capture.py my_scene --output DATA --settings settings_jsons/rs_settings.json
```

- Press s to start capturing frames
- Press q to quit
- Use --autostart 5 to begin automatically after 5 seconds

## Settings
You can modify the capture settings in the `settings_jsons/rs_settings.json`

