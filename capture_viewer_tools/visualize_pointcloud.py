import open3d as o3d
import sys
import json
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from capture_viewer_tools.popup_info import show_popup

# Global variable to store the JSON information
custom_information_json = None

# This function will be called when a key is pressed
def key_callback(vis, action, mods):
    if action == 1:  # Check if the key was pressed
        # We can use a specific key to trigger displaying the custom JSON info

        # Display the custom JSON information in the window
        if custom_information_json:
            info_text = json.dumps(custom_information_json, indent=4)
            print(f"Displaying JSON Info: {info_text}")

            # Show the popup window with the JSON info
            show_popup(info_text)


# Visualize the point cloud and listen for key presses
def visualize_pointcloud(pcl_path, custom_information_json_input):
    # Load the point cloud from the file
    pointcloud = o3d.io.read_point_cloud(pcl_path)

    # Check if the point cloud was loaded successfully
    if pointcloud.is_empty():
        print(f"Failed to load point cloud from {pcl_path}")
        return

    # Set the global JSON information to be displayed
    global custom_information_json
    custom_information_json = custom_information_json_input

    # Visualize the point cloud
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="Point CLoud: press I for INFO")
    vis.add_geometry(pointcloud)

    # Register the key press callback
    vis.register_key_action_callback(ord('I'), key_callback)  # 'I' key to show info

    # Run the visualization
    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python visualize_pointcloud.py <point_cloud_file_path> <custom_information_json>")
        sys.exit(1)

    pcl_path = sys.argv[1]
    custom_information_json = sys.argv[2]  # Parse the JSON string passed as a command-line argument
    visualize_pointcloud(pcl_path, custom_information_json)
