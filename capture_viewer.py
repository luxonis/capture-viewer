import os
import argparse
import json
import tkinter as tk
import subprocess
import time


# Function to load session metadata from each session folder
def load_sessions(folder_path):
    sessions = []
    for session_name in os.listdir(folder_path):
        session_path = os.path.join(folder_path, session_name)
        if os.path.isdir(session_path):
            metadata_file = os.path.join(session_path, 'metadata.json')
            if os.path.exists(metadata_file):
                with open(metadata_file, 'r') as f:
                    metadata = json.load(f)
                    sessions.append({
                        'session_name': session_name,
                        'session_path': session_path,
                        'metadata': metadata,
                        'date': metadata.get('date', '00000000000000')  # Default to '00000000000000' if no date
                    })
    sessions.sort(key=lambda s: s['date'], reverse=False)
    return sessions


# Function to handle button click and run the capture script with selected streams
def run_capture_script(session, selected_streams, ip):
    session_path = session['session_path']

    script_directory = os.path.dirname(os.path.abspath(__file__))
    # print("Script directory:", script_directory)
    capture_show_path = os.path.join(script_directory, 'oak_capture_show.py')

    # The command to run the script with the session path and selected streams
    command = [
        'python',
        capture_show_path,
        session_path,
    ] + selected_streams + ['--ip', ip]

    # Run the command as a subprocess
    try:
        subprocess.Popen(command)
        time.sleep(4)
        print(f"Opened capture script for session: {session_path} with streams: {selected_streams}")
    except subprocess.CalledProcessError as e:
        print(f"Error running capture script for session: {session_path}")
        print(e)


# Function to create a Tkinter app with buttons for each session and stream selection
def create_app(sessions, ip):
    root = tk.Tk()
    root.title("Session Viewer")

    # Define number of columns for grid layout
    columns = 4

    # Stream options to be selected
    stream_options = ['left', 'right', 'depth', 'disparity', 'rgb', 'tof_depth', 'neural_disparity', 'disparity_rescaled']  # Add more as needed
    on_off = [1, 1, 1, 0, 1, 0, 0, 0]
    selected_streams_vars = {stream: tk.IntVar(value=on_off[idx]) for idx, stream in enumerate(stream_options)}

    # Create checkboxes for each stream option and display them horizontally
    stream_frame = tk.Frame(root)
    stream_frame.grid(row=0, column=0, columnspan=columns, padx=10, pady=10)

    tk.Label(stream_frame, text="Select Streams to Display:").grid(row=0, column=0, columnspan=len(stream_options),
                                                                   padx=5, pady=5)

    for idx, stream in enumerate(stream_options):
        checkbox = tk.Checkbutton(stream_frame, text=stream, variable=selected_streams_vars[stream])
        checkbox.grid(row=1, column=idx, padx=5, pady=5)  # Place each checkbox in a new column

    # Function to get selected streams as a list
    def get_selected_streams():
        return [stream for stream, var in selected_streams_vars.items() if var.get() == 1]

    # Create buttons for each session
    for idx, session in enumerate(sessions):
        metadata = session['metadata']
        camera = metadata.get('model_name', 'Unknown Camera')
        scene = metadata.get('scene', 'Unknown Scene')
        date = metadata.get('date', 'Unknown Date')

        # Display scene and date on the button
        button_label = f"{camera}\n{scene}\n{date}"

        button = tk.Button(
            root,
            text=button_label,
            command=lambda s=session: run_capture_script(s, get_selected_streams(), ip),
            width=25,
            height=3
        )
        # Arrange buttons in a grid
        row = (idx // columns) + 2  # Start from the next row after the stream options
        col = idx % columns
        button.grid(row=row, column=col, padx=5, pady=5)

    root.mainloop()


# Main function to load sessions and start the Tkinter app
def main():
    """
    example usage - run from command line:
    python path/to/this/script/oak_capture_viewer.py -p path/to/capture/folder
    python capture/capture_and_viewer/oak_capture_viewer.py -p /mnt/nas/calibration/datasets/20240923_gas_boilers/oak-d-pro
    Don't forget to use the -p as an argument!
    """

    parser = argparse.ArgumentParser(description="Process folder path for session loading.")
    parser.add_argument('-p', '--path', type=str, help='Absolute path to the folder', default=None)
    parser.add_argument('--ip', type=str, help='IP adress of device to use for replay', default=None)
    args = parser.parse_args()

    ip = args.ip

    if args.path:
        folder_path = args.path
    else:
        # for running in Pycharm
        script_dir = os.path.dirname(os.path.abspath(__file__))
        folder_path = os.path.join(script_dir, 'DATA')
        folder_path = '/mnt/nas/calibration/datasets/20240905_office4/20240905_scene_lady'
        # folder_path = os.path.join(folder_path, 'tools_ground')  # select subfolder if needed

    sessions = load_sessions(folder_path)

    if not sessions:
        print("No sessions found!")
    else:
        create_app(sessions, ip)


if __name__ == "__main__":
    main()
