
# Capture Viewer

`capture-viewer` is a Luxonis Python tool designed to view and analyze data captured by OAK cameras. 
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

[See GUIDE](capture/README.md)


### 2. Viewing Data - Running the Capture Viewer

To start the capture viewer and view the captured data, run:

```bash
python capture_viewer.py -p DATA
```

Or specify your own capture folder:

```bash
python capture_viewer.py -p /path/to/DATA
```

Where DATA is the folder containing all the capture sessions. It will direct you to a session selection, where you can 
visualize individual sessions.

Optionally, you can add --ip to specify the **IP of the device** you want to run the replay on.

```bash
python capture_viewer.py -p /path/to/DATA --ip device_ip
```

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

If the App window is too large/small for your screen, you can adjust window size. 
This will be improved in future versions of the app.

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
