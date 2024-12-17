
# Capture Viewer

`capture-viewer` is a Luxonis internal Python tool designed to view and analyze data captured by OAK cameras. 
It provides functionalities to capture, display, and inspect stereo depth captures along with other types of camera data. 
It also allows the use of a REPLAY to regenerate data with different settings.

---

## Requirements

- Python (Tested on Python 3.11)
- DepthAI version 2.29.0 or higher
- Compatible with RVC2 OAK cameras (Tested on OAK-D and OAK-D-POE, compatible with Wide and PRO models)
- To use the REPLAY feature, you need to have a camera connected to your PC

Ensure the required dependencies are installed by running:

```bash
pip install -r requirements.txt
```

## How to Run

### 1. Capturing Data

To capture data from an OAK camera, use the following command:

```bash
python capture_scripts/oak_capture.py /path/to/settings.json capture_name
```

If you'd like to use default settings, run:

```bash
python capture_scripts/oak_capture.py default my_first_capture
```

After pressing `S` the script will automatically save predefined number of frames to a folder named `DATA`. 
You can also specify your own settings JSON file by replacing `/path/to/settings.json`.

### 2. Running the Capture Viewer

To start the capture viewer and view the captured data, run:

```bash
python capture_viewer.py -p DATA
```

Or specify your own capture folder:

```bash
python capture_viewer.py -p /path/to/DATA
```

---

## Settings Options for Captures

During capture, you can configure various parameters. Below is an example settings configuration:

```json
{
    "ir": true,
    "ir_value": 1,
    "flood_light": false,
    "flood_light_intensity": 1,
    "stereoAlign": true,
    "alignSocket": "REC_LEFT",
    "autoexposure": true,
    "expTime": 200,
    "sensIso": 800,
    "LRcheck": true,
    "extendedDisparity": false,
    "subpixelDisparity": true,
    "subpixelValue": 3,
    "highAccuracy": false,
    "highDensity": true,
    "filters_on": true,
    "filters": {
        "threshold_filter": false,
        "lower_threshold_filter": 1000,
        "upper_threshold_filter": 5000,
        "decimation_filter": true,
        "decimation_factor": 2,
        "spacial_filter": false,
        "temporal_filter": false,
        "speckle_filter": false,
        "speckle_range": 50,
        "median_filter": false,
        "median_size": "MEDIAN_OFF"
    },
    "FPS": 30,
    "num_captures": 60
}
```

- `ir` : IR dot projector, available on PRO models.
- `flood_light` : IR floodlight, available on PRO models.
- If `autoexposure` is set to false, `expTime` and `sensIso` will be used.
- `extendedDisparity` : extendedDisparity
- `subpixelDisparity`: subpixelDisparity with the selected `subpixelValue`
- preset mode selection: `highAccuracy` or `highDensity`
- `filters_on`: if set to false, none of the following filters will be used
  - `filters`: self-explanatory
- `FPS`: will be applied to mono camera streams as well as the color camera
- `num_captures` : the number of depth frames which will be saved before the capture ending

These are the basic information about the capture. Settings for the capture can be edited and re-applied using the **Replay** feature 
in the capture viewer. 

**The REPLAY feature offers also additional settings.**

---

## REPLAY Feature

The **REPLAY** feature allows you to regenerate data using different settings or stereo algorithms. To use REPLAY, you need the left, right, and optionally color images from the camera. You can adjust various parameters like disparity algorithms and filter settings.

The REPLAY script can be found in the `depth` module and the user GUI is available in `capture_viewer.py`.

REPLAY features can only be used if you have an OAK camera connected to your PC via USB, or if you specify the IP of a network device.

### REPLAY Output

The output from using **REPLAY** is automatically saved in `path/to/DATA/capture_folder/replay_outputs`  
(e.g., `/home/user/projects/capture-viewer/DATA/OAK-D-PRO_20241217125608/replay_outputs/20241217133208`).

Each folder is named based on the time of generation and contains the following files:
- `config.json` - Configuration used for depth generation.
- `depth_timestamp.npy` and `depth_timestamp.png` - Depth generated with the given config from `left_timestamp.npy` and `right_timestamp.npy`.
- `pcl_timestamp.ply` - Pointcloud generated from the depth. If the depth is aligned to the RGB camera, the pointcloud uses colors from the RGB camera; otherwise, it uses the JET colormap.

---

## Adjusting window size

If the App window is too large/small for your screen, you can adjust window size

For one capture display: `coak_capture_show.py`
    
    # change window size here
    canvas_width = 1620
    canvas_height = 880

For REPLAY: `capture_viewer_tools/ReplayVisualizer.py`

    class ReplayVisualizer:
        def __init__(self, root, view_info, current_view):
            self.window = tk.Toplevel(root)
            self.window.title("Depth Visualization")
            self.window.geometry("2200x1200")  # adjust here
    
            max_image_width = 640  # adjust here
            max_image_height = 400  # adjust here
    ...

---

## Project Structure

Here is an overview of the project structure:

```
capture-viewer/
├── capture_scripts/
├── capture_viewer.py
├── capture_viewer_tools/
├── DATA/
├── depth/
├── oak_capture_show.py
├── README.md
├── requirements.txt
├── settings_jsons/
```

### Description of Key Folders and Files:

- `capture_scripts/`: Scripts for capturing data from OAK cameras.
- `capture_viewer.py`: Main script for launching the capture viewer.
- `DATA/`: Folder where captured data is saved.
- `depth/`: Contains modules for working with replay.
- `oak_capture_show.py`: Script for displaying and handling individual OAK camera captures.
- `settings_jsons/`: Folder containing predefined settings JSON files.

## Example Data Structure

Captured data is organized in folders named with the timestamp of the capture. Each folder contains various `.npy` and `.png` files corresponding to different data types, such as depth, disparity, and raw camera frames.

Example:

```
DATA/OAK-D-PRO_20241217182033/
├── calib.json
├── depth_35748175.npy
├── disparity_35748175.npy
├── left_35748175.npy
├── right_35748175.npy
├── metadata.json
└── replay_outputs/
```
