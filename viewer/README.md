# Capture Viewer - Viewer Module

The viewer module provides the interface for viewing and analyzing data captured by OAK cameras. It includes a GUI application for browsing capture sessions and visualizing stereo depth data.

## Features

- **Session Browser**: Browse and select capture sessions from your data folder
- **Multi-stream Visualization**: View left, right, depth, disparity, RGB, and other camera streams
- **REPLAY Feature**: Regenerate depth data with different stereo algorithms and settings
- **Point Cloud Visualization**: View 3D point clouds generated from depth data
- **Metadata Display**: View capture metadata and device information

## Requirements

- Python (Tested on Python 3.11)
- DepthAI version 2.29.0 or 2.30.0
- Compatible with RVC2 OAK cameras (Tested on OAK-D and OAK-D-POE, compatible with Wide and PRO models)
- To use the REPLAY feature, you need to have a camera connected to your PC

## Installation

Ensure the required dependencies are installed by running:

```bash
pip install -r requirements.txt
```

## How to Run

### Basic Usage

To start the capture viewer and view the captured data, run:

```bash
python viewer/capture_viewer.py -p DATA
```

Or specify your own capture folder:

```bash
python viewer/capture_viewer.py -p /path/to/DATA
```

Where DATA is the folder containing all the capture sessions. It will direct you to a session selection, where you can visualize individual sessions.

### Network Device Support

Optionally, you can add `--ip` to specify the IP of the device you want to run the replay on:

```bash
python viewer/capture_viewer.py -p /path/to/DATA --ip device_ip
```

## REPLAY Feature

The **REPLAY** feature allows you to regenerate data using different settings or stereo algorithms. To use REPLAY, you need the left, right, and optionally color images from the camera. You can adjust various parameters like disparity algorithms and filter settings.

REPLAY features can only be used if you have an OAK camera connected to your PC via USB, or if you specify the IP of a network device.

### REPLAY Output

The output from using **REPLAY** is automatically saved in `path/to/DATA/capture_folder/replay_outputs`  
(e.g., `/home/user/projects/capture-viewer/DATA/OAK-D-PRO_20241217125608/replay_outputs/20241217133208`).

Each folder is named based on the time of generation and contains the following files:
- `config.json` - Configuration used for depth generation.
- `depth_timestamp.npy` and `depth_timestamp.png` - Depth generated with the given config from `left_timestamp.npy` and `right_timestamp.npy`.
- `pcl_timestamp.ply` - Pointcloud generated from the depth. If the depth is aligned to the RGB camera, the pointcloud uses colors from the RGB camera; otherwise, it uses the JET colormap.

## Adjusting Window Size

If the App window is too large/small for your screen, you can adjust window size.

### For one capture display: `oak_capture_show.py`
    
```python
# change window size here
canvas_width = 1620
canvas_height = 880
```

### For REPLAY: `utils/ReplayVisualizer.py`

```python
class ReplayVisualizer:
    def __init__(self, root, view_info, current_view):
        self.window = tk.Toplevel(root)
        self.window.title("Depth Visualization")
        self.window.geometry("2200x1200")  # adjust here

        max_image_width = 640  # adjust here
        max_image_height = 400  # adjust here
```

## Module Structure

- `capture_viewer.py` - Main GUI application for browsing sessions
- `oak_capture_show.py` - Individual session viewer
- `utils/` - Utility modules for various functions
  - `capture_tools.py` - Core capture and visualization tools
  - `button_functions.py` - GUI button functionality
  - `ReplayVisualizer.py` - REPLAY feature interface
  - `ReplaySettings.py` - REPLAY configuration management
  - `ReplayThread.py` - Background processing for REPLAY
  - `pointcloud.py` - Point cloud utilities
  - `visualize_pointcloud.py` - Point cloud visualization
  - `popup_info.py` - Information popup dialogs
  - `alignment.py` - Image alignment utilities
  - `stereo.py` - Stereo processing utilities
  - `convert.py` - Data conversion utilities
- `depth/` - Depth processing modules
- `profile_presets/` - Preset configurations

## Troubleshooting

### Import Errors
If you encounter import errors, ensure you're running the script from the project root directory and that all dependencies are installed.

### REPLAY Not Working
- Ensure you have an OAK camera connected via USB
- Or specify the correct IP address for network devices
- Check that the capture data contains left and right stereo images

### GUI Issues
- Adjust window sizes as described above
- Ensure your display supports the required resolution
- Check that tkinter is properly installed 