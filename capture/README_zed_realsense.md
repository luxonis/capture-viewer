

# ZED capture
This module allows capturing left, right and neural stereo depth ZED device, saving data in a format compatible with DepthAI pipelines (e.g., ZED-to-OAK mimic format).

OAK-like calibration file is generated. This file includes a dummy rgb camera to allow for replay functions to work normally.


## Requirements
To use this script, install the [ZED SDK](https://www.stereolabs.com/developers/) and its [Python API](https://www.stereolabs.com/docs/api/python/getting-started/). After installing the SDK, run `pip install pyzed` to get the Python bindings.

## Run

```bash
python capture/zed_capture.py my_scene --output DATA --settings capture/settings_jsons/zed_settings.json
```

- Press s to start capturing frames
- Press q to quit
- Use --autostart 5 to begin automatically after 5 seconds

## Settings
You can modify the capture settings in the `capture/settings_jsons/zed_settings.json`


# Intel Realsense capture
This module allows capturing left, right, depth, and optional RGB streams from an Intel RealSense device, saving data in a format compatible with DepthAI pipelines (e.g., Realsense-to-OAK mimic format).

OAK-like calibration file is generated.
## Requirements

```bash
   pip install pyrealsense2
```

## Run
```bash
python capture/realsense_capture.py my_scene --output DATA --settings capture/settings_jsons/rs_settings.json
```

- Press s to start capturing frames
- Press q to quit
- Use --autostart 5 to begin automatically after 5 seconds

## Settings
You can modify the capture settings in the `capture/settings_jsons/rs_settings.json`

