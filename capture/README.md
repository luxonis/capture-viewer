# CAPTURE GUIDE

This is a guide on how to run capture with one device either using Depthai v2 or Depthai v3. For guide on how to capture using multiple devices 
refer to `README_multiple_devices.md`. For RVC2 capture. you can use both Depthai v2 and v3 capture scripts.

## Depthai 2 Capture (for RVC2 Stereo and TOF)

```bash
python capture/dai2_capture.py default <capture_name> --devices <camera_ip>
```

### Arguments:
- **settings_file_path** – path to the settings JSON file. You can choose from `default`, `default_tof`, or choose from others in `settings_jsons` 
or provide a custom path `/path/to/your/settings`. More details can be found in the Settings section.
- **view_name** – name of the capture session; this name will be saved in the metadata file.
- **--output** (optional) – path to a custom output folder where the capture results will be stored. If not provided, output will be saved in the `DATA` folder relative to the script root.
- **--autostart** (optional) – specify the number of seconds to wait before the capture starts automatically. Set to `-1` to disable autostart.
  - If this option is not used, you can manually start capture by pressing `S`. The script will then save a predefined number of frames to the `DATA` folder.
- **--devices** (optional) – list of **MXIDs** or **IP addresses** of devices to connect to. If left empty, the script will attempt to connect to the first available device.
- **--ram** (optional) – maximum RAM (in GB) that the capture process is allowed to use while saving data. Default is `2`.
- **--att_connection** (optional) – if set to `True`, the script will attempt to find all devices on the network before attempting direct connection.
- **--autostart_time** (optional) – specify a fixed system time (in seconds since epoch or formatted string depending on script behavior) when the capture should start.
- **--autostart_end** (optional) – specify a fixed system time when the capture should end automatically.
- **--show_streams** (optional) – set to `True` to show all active streams; if `False`, only the left frame will be shown.

### Settings

For capture, you can configure various parameters. Below is an example settings configuration.

This settings need to be configured before the capture start. To change the settings, simply edit parameters in the json and restart the capture script.
The json settings can be found in the `settings_jsons` folder.

- `ir` : IR dot projector, available on PRO models.
- `flood_light` : IR floodlight, available on PRO models.
- If `autoexposure` is set to false, `expTime` and `sensIso` will be used.
- `extendedDisparity` : extendedDisparity
- `subpixelDisparity`: subpixelDisparity with the selected `subpixelValue`
- `profilePreset` : choose a preset to optimize processing for different use cases. Options include `DEFAULT`, `HIGH_DETAIL`, `FACE`, `ROBOTICS`. (Note: `HIGH_ACCURACY` and `HIGH_DENSITY` may be deprecated in future releases.)
- `output_settings` : select what streams you'd like to visualize. Some stream combinations might not be valid 
(such as choosing depth stream but not selecting left and right)
- `use_filter_settings`: **if set to false, none of the following filters will be used**. Set to true to use custom filter settings.
  - `filters`: self-explanatory
- `FPS`: will be applied to mono camera streams as well as the color camera
- `num_captures` : the number of depth frames which will be saved before the capture ending

These are the basic information about the capture. Settings for the capture can be edited and re-applied using the **Replay** feature 
in the capture viewer. 

## Depthai 3 Capture (for RVC2 & RVC4)

```bash
python capture/dai3_stereo_capture.py dai3 <capture_name> --ip <camera_ip>
```

### Arguments:

- **settings_file_path** – path to the settings JSON file. You can use `dai3`, or choose from others in `settings_jsons`
or provide a custom path `/path/to/your/settings`. More details can be found in the Settings section.
- **view_name** – name of the capture session; this name will be saved in the metadata file.
- **--output** (optional) – path to a custom output folder where the capture results will be stored. If not provided, output will be saved in the `DATA` folder relative to the script root.
- **--autostart** (optional) – specify the number of seconds to wait before the capture starts automatically. Set to `-1` to disable autostart.
  - If this option is not used, you can manually start capture by pressing `S`. The script will then save a predefined number of frames to the `DATA` folder.
- **--ip** (optional) – provide the **IP address** of the device to connect to. If omitted, it will connect to the first available device.
- **--autostart_time** (optional) – specify a fixed system time (in seconds since epoch or formatted string depending on script behavior) when the capture should start.
- **--autostart_end** (optional) – specify a fixed system time when the capture should end automatically.
- **--show_streams** (optional) – set to `True` to show all active streams; if `False`, only the left frame will be shown.
- **--alternating_capture** (optional) – toggles the IR projector on or off for alternating capture behavior.
- **--port** (optional) – ZMQ control port to communicate with the device. Default is `5555`. Only used in multiple device capture

### Settings

For capture, you can configure various parameters. Below is an example settings configuration.

These settings need to be configured before the capture starts. To change them, simply edit the parameters in the JSON file and restart the capture script.
The json settings can be found in the `settings_jsons` folder.

- `ir` : enables the IR dot projector (set to `true` to activate). Available on PRO models.
- `ir_value` : controls the intensity of the IR projector (1 = ON).
- `flood_light` : enables the IR floodlight. Available on PRO models.
- `flood_light_intensity` : intensity of the IR floodlight. Currently set to `1`.
- `stereoResolution` : resolution of the stereo mono cameras. Example: `"x": 1280, "y": 800`.
- `rgbResolution` : resolution of the RGB camera. Example: `"x": 1280, "y": 800`.
- `sync_on_host` : when `true`, all streams will be synchronized on the host.
- `monoSettings` : tuning parameters for the mono (grayscale) streams:
  - `luma_denoise`, `chroma_denoise`, `sharpness`, `contrast` – all set to `0` in this example.
- `exposureSettings` :
  - `autoexposure` – if `true`, the device handles exposure automatically.
  - If `autoexposure` is set to `false`, the script will use `expTime` (exposure time) and `sensIso` (ISO sensitivity) values.
- `output_settings` : defines which streams to visualize and save:
  - `left`, `right`, `rgb`, `depth` – enabled.
  - `left_raw`, `right_raw`, `disparity`, `sync` – disabled.
- `profilePreset`: select a Profile Preset such as `DEFAULT`, `HIGH_DETAIL`, `FACE`, `ROBOTICS`, `FAST_ACCURACY`, `FAST_DENSITY`.
- `LRcheck` : enables left-right consistency check for better depth accuracy.
- `extendedDisparity` : enables extended disparity range, improving depth for closer objects.
- `FPS` : frame rate for mono and RGB streams. Example: `15`.
- `num_captures` : number of frames to save before capture ends. set to "inf" if you don't want to limit your capture.

These are the core parameters for configuring your capture session. You can later re-edit and apply these settings using the **Replay** feature in the Capture Viewer.



## Thermal capture

To make a capture with thermal devices, use `thermal_capture.py` script.

```bash
python capture/dai2_thermal_capture.py default_thermal <view_name> --device-ip <ip>
```

Most of the settings are not implemented yet. 









