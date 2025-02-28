import argparse
import re

from capture_viewer_tools.capture_tools import extract_calibration_values
from capture_viewer_tools.button_functions import *
from capture_viewer_tools.popup_info import show_popup

# change window size here
canvas_width = 1920  # 1920
canvas_height = 1000

def load_data(folder, types):
    data = {}
    height, width = 0, 0
    rgb_width, rgb_height = 0, 0
    for filename in os.listdir(folder):
        if filename.endswith(".npy"):
            if "left_raw" in filename:
                type_key = 'left_raw'
            elif "right_raw" in filename:
                type_key = 'right_raw'
            elif "neural_disparity" in filename:
                type_key = 'neural_disparity'
            elif "disparity_rescaled" in filename:
                type_key = 'disparity_rescaled'
            elif "tof_depth" in filename:
                type_key = 'tof_depth'
            else:
                type_key = filename.split('_')[0]
                if type_key == "isp": type_key = "rgb"  # to make compatible with old captures which use different naming
                if (height == 0 or width == 0) and type_key == "left":
                    left = np.load(os.path.join(folder, filename), allow_pickle=True)
                    height, width = left.shape[0], left.shape[1]
                elif (rgb_height == 0 or rgb_width == 0) and type_key == "rgb":
                    color = np.load(os.path.join(folder, filename), allow_pickle=True)
                    rgb_height, rgb_width = color.shape[0], color.shape[1]
            if type_key not in types:
                continue
            filepath = os.path.join(folder, filename)
            key = filename.split('_')[-1]  # Group by timestamp
            if key not in data:
                data[key] = {}
            data[key][type_key] = filepath
    return data, height, width, rgb_width, rgb_height

def check_settings(capture_folder):
    metadata_file = os.path.join(capture_folder, 'metadata.json')
    if not os.path.isfile(metadata_file): raise ValueError(f"Error: {metadata_file} not found.")

    try:
        with open(metadata_file, 'r') as f:
            data = json.load(f)
    except json.JSONDecodeError:
        return ValueError("Error: Failed to parse {metadata_file}.")

    settings = data['settings']
    if 'stereoAlign' in settings and settings['stereoAlign']:
        return True
    else:
        return False


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("capture_folder", nargs='?', help='Path to folder containing captures')
    parser.add_argument("types", nargs='+', help='write types')
    args = parser.parse_args()
    capture_folder = args.capture_folder
    selected_types = args.types
    print("capture folder:", capture_folder)
    if capture_folder is None:
        raise Exception

    if not (os.path.exists(capture_folder) and os.path.isdir(capture_folder)):
        capture_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"DATA/{capture_folder}")
        if not (os.path.exists(capture_folder) and os.path.isdir(capture_folder)):
            raise ValueError("Provided capture path does not exist or is not a directory")
    return capture_folder, selected_types

def extract_timestamps(data):
    def extract_number(file_name):
        match = re.match(r'(\d+)', file_name)
        return int(match.group(0)) if match else float('inf')
    timestamps = sorted(list(data.keys()), key=extract_number)
    return timestamps

class MainApp():
    def __init__(self):
        capture_folder, selected_types = parse_arguments()
        data, height, width, rgb_width, rgb_height = load_data(capture_folder, selected_types)
        timestamps = extract_timestamps(data)

        calib = ''
        if os.path.exists(f'{capture_folder}/calib.json'):
            calib = dai.CalibrationHandler(f'{capture_folder}/calib.json')
            M1, D1, M2, D2, T, R, TARGET_MATRIX, lensPosition = extract_calibration_values(calib, width, height, rgb_width, rgb_height)
            datas = M1, D1, M2, D2, T, R, TARGET_MATRIX, lensPosition
        else:
            datas = None, None, None, None, None, None, None, None

        metadata_file = os.path.join(capture_folder, 'metadata.json')
        if os.path.exists(metadata_file):
            with open(metadata_file, 'r') as f:
                metadata = json.load(f)

        self.view_info = {
            "capture_folder": capture_folder,
            "types": selected_types,
            "timestamps": timestamps,
            "current_index": 0,
            "data": data,
            "metadata": metadata,
            "calib": calib,
            "canvas_width": canvas_width,
            "canvas_height": canvas_height,
            "depth_size": (width, height),
            "rgb_size": (rgb_width, rgb_height),
            "mono_size": (width, height),
            "stereoAlign_used_in_capture": check_settings(capture_folder),
            "calibration_parameters": datas,
            "RGB_SOCKET": dai.CameraBoardSocket.CAM_A,
            "LEFT_SOCKET": dai.CameraBoardSocket.CAM_B,
            "RIGHT_SOCKET": dai.CameraBoardSocket.CAM_C,
            "device_connected": device_connected(),
        }

        self.current_view = {
            "rgb": None,
            "depth": None,
            "disparity": None,
            "left": None,
            "right": None,
            "alignSocket": "COLOR"
        }
    
    def main_layout(self):
        def update_sliders(*args):
            min_val = min_slider.get()
            max_val = max_slider.get()
            if min_val > max_val:
                min_slider.set(max_val)
            elif max_val < min_val:
                max_slider.set(min_val)

        def select_alignment():
            self.current_view["alignSocket"] = align_socket.get()
            
        root = Tk()
        root.title(f"Capture Viewer: {self.view_info['capture_folder']}")
        canvas = Canvas(root, width=canvas_width, height=canvas_height, bg="black")
        canvas.pack()
    
        prev_button = Button(root, text="<-", bg="orange", activebackground="yellow",
                             command=lambda: on_prev(root, canvas, self.view_info, self.current_view))
        next_button = Button(root, text="->", bg="orange", activebackground="yellow",
                             command=lambda: on_next(root, canvas, self.view_info, self.current_view))
        next_button.pack(side="right")
        prev_button.pack(side="right")
    
        pointcloud_button = Button(root, text="OpenCV \nPointCloud", command=lambda: on_pointcloud_rgb(root, self.view_info, self.current_view,
                                                                                                       downsample=False))
        pointcloud_button.pack(side="right")
    
        align_button = Button(root, text="OpenCV\nAlignment\nset",
                              command=select_alignment)
        align_button.pack(side="right")
    
        align_socket = StringVar(value="COLOR")
        radiobutton_color = Radiobutton(root, text="COLOR", variable=align_socket, value="COLOR")
        radiobutton_color.pack(side="right")
        radiobutton_left = Radiobutton(root, text="LEFT", variable=align_socket, value="LEFT")
        radiobutton_left.pack(side="right")

    
        canvas.tooltip = Label(root, text="", background="white", relief="solid", borderwidth=1, padx=2, pady=2)
        canvas.tooltip.place_forget()
    
        canvas.bind("<Motion>", lambda event: on_mouse_move(event, canvas))
    
        replay_button = Button(root, text="REPLAY", bg="#4169E1", activebackground="#63A8FF",
                               command=lambda: on_replay(root, self.view_info, self.current_view))
        replay_button.pack(side="left")
    
        info_button = Button(root, text="INFO", bg="#32CD32", activebackground="#90EE90",
                             command=lambda: show_popup("Metadata", str(self.view_info["metadata"])))
        info_button.pack(side="left")
    
        update_button = Button(root, text="Colorize\nDepth", command=lambda: display_images(root, canvas, self.view_info, self.current_view,
                                                                                            min_slider.get(), max_slider.get()))
        update_button.pack(side="right")
    
        # Sliders for setting range
        if 'disparity' in self.view_info["types"] or 'neural_disparity' in self.view_info["types"]:
            slider_upper_range = 800
        else:
            slider_upper_range = 7000
    
        min_slider = Scale(root, from_=0, to=slider_upper_range, orient=VERTICAL, label="Min Value")  # Adjust range and labels as needed
        max_slider = Scale(root, from_=0, to=slider_upper_range, orient=VERTICAL, label="Max Value")  # Adjust range and labels as needed
        min_slider.pack(side=RIGHT, fill="y")
        max_slider.pack(side=RIGHT, fill="y")
    
        # Link sliders to update their behavior when changed
        min_slider.config(command=update_sliders)
        max_slider.config(command=update_sliders)
    
        display_images(root, canvas, self.view_info, self.current_view)
    
        root.mainloop()


if __name__ == "__main__":
    print(dai.__version__)
    window = MainApp()
    window.main_layout()