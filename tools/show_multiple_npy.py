#!/usr/bin/env python3
"""
Multi-Device Data Visualizer for 4-device sync node data
Shows all 8 folders simultaneously, displaying the same timestamp across all devices
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button, Slider
import tkinter as tk
from tkinter import filedialog
import glob
from pathlib import Path
import argparse
from collections import defaultdict

class MultiDeviceVisualizer:
    def __init__(self, data_path):
        self.data_path = Path(data_path)
        self.subfolders = []
        self.files_by_folder = {}
        self.device_timestamps = {}  # Timestamps for each device
        self.device_current_idx = {}  # Current index for each device
        self.fig = None
        self.axs = None
        self.data_cache = {}  # Cache for loaded data
        self.cache_size = 50  # Maximum number of cached items per device
        self.current_data_type = 'left'  # Current data type being displayed
        self.fast_mode = False  # Fast navigation mode (less console output)
        self.last_console_update = 0  # Track when we last updated console
        
        self.load_data_structure()
        self.setup_device_timestamps()
        
    def load_data_structure(self):
        """Load the structure of all subfolders and their files"""
        print(f"Loading data from: {self.data_path}")
        
        # Get all subfolders
        self.subfolders = [d for d in self.data_path.iterdir() if d.is_dir()]
        self.subfolders.sort()
        
        print(f"Found {len(self.subfolders)} subfolders:")
        for i, folder in enumerate(self.subfolders):
            print(f"  {i+1}. {folder.name}")
        
        # Load file structure for each subfolder
        for folder in self.subfolders:
            files = list(folder.glob("*.npy"))
            files.sort()
            
            # Group files by timestamp (extract timestamp from filename)
            files_by_timestamp = {}
            for file in files:
                # Extract timestamp from filename (e.g., "left_101809326.npy" -> "101809326")
                parts = file.stem.split('_')
                timestamp = parts[-1]  # Last part is timestamp
                data_type = '_'.join(parts[:-1])  # Everything except last part is data type
                
                if timestamp not in files_by_timestamp:
                    files_by_timestamp[timestamp] = {}
                files_by_timestamp[timestamp][data_type] = file
            
            self.files_by_folder[folder.name] = files_by_timestamp
            
            # Show available data types for this folder
            if files_by_timestamp:
                first_timestamp = list(files_by_timestamp.keys())[0]
                available_types = list(files_by_timestamp[first_timestamp].keys())
                print(f"  {folder.name}: {len(files_by_timestamp)} timestamps, data types: {available_types}")
            else:
                print(f"  {folder.name}: {len(files_by_timestamp)} timestamps")
    
    def setup_device_timestamps(self):
        """Setup timestamps for each device independently"""
        print("\nSetting up timestamps for each device...")
        
        # Get timestamps for each device
        for folder_name, timestamp_data in self.files_by_folder.items():
            self.device_timestamps[folder_name] = sorted(timestamp_data.keys())
            self.device_current_idx[folder_name] = 0
            print(f"  {folder_name}: {len(self.device_timestamps[folder_name])} timestamps")
        
        # Find the device with the most timestamps
        max_timestamps = max(len(timestamps) for timestamps in self.device_timestamps.values())
        print(f"\nMaximum timestamps per device: {max_timestamps}")
        
        # Show first few timestamps for each device
        print("\nFirst 5 timestamps per device:")
        for folder_name, timestamps in self.device_timestamps.items():
            short_name = folder_name.split('_')[0] + '_' + folder_name.split('_')[-1]
            first_five = timestamps[:5]
            print(f"  {short_name}: {first_five}")
    
    def load_data(self, folder_name, timestamp, data_type):
        """Load specific data for a folder and timestamp with caching"""
        if folder_name not in self.files_by_folder:
            return None
            
        timestamp_data = self.files_by_folder[folder_name].get(timestamp, {})
        if data_type not in timestamp_data:
            return None
        
        # Create cache key
        cache_key = f"{folder_name}_{timestamp}_{data_type}"
        
        # Check if data is in cache
        if cache_key in self.data_cache:
            return self.data_cache[cache_key]
        
        # Load data from file
        try:
            data = np.load(timestamp_data[data_type])
            
            # Add to cache
            self.data_cache[cache_key] = data
            
            # Limit cache size per device
            device_cache_keys = [k for k in self.data_cache.keys() if k.startswith(folder_name)]
            if len(device_cache_keys) > self.cache_size:
                # Remove oldest cached item for this device
                oldest_key = device_cache_keys[0]
                del self.data_cache[oldest_key]
            
            return data
        except Exception as e:
            print(f"Error loading {timestamp_data[data_type]}: {e}")
            return None
    
    def setup_plot(self):
        """Setup the matplotlib figure and subplots for all devices"""
        # Create figure with subplots for all 8 devices
        # Layout: 2 rows x 4 columns for 8 devices
        self.fig, self.axs = plt.subplots(2, 4, figsize=(20, 10))
        self.fig.suptitle('Multi-Device Data Visualization', fontsize=16)
        
        # Set titles for each subplot based on folder names
        for i, folder in enumerate(self.subfolders):
            row = i // 4
            col = i % 4
            # Shorten folder name for display
            short_name = folder.name.split('_')[0] + '_' + folder.name.split('_')[-1]
            self.axs[row, col].set_title(short_name, fontsize=10)
        
        # Connect keyboard events
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
    def reconnect_keyboard_events(self):
        """Reconnect keyboard events after figure recreation"""
        if self.fig:
            self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
    def display_data(self, data_type='left'):
        """Display data for all devices at their current timestamps"""
        if not self.fig:
            self.setup_plot()
        else:
            # Clear the entire figure to remove any leftover colorbars
            self.fig.clear()
            # Recreate subplots
            self.axs = self.fig.subplots(2, 4)
            # Reconnect keyboard events
            self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Calculate global frame index (average of all device indices)
        total_frames = 0
        current_frames = 0
        for folder_name, timestamps in self.device_timestamps.items():
            total_frames += len(timestamps)
            current_frames += self.device_current_idx[folder_name]
        
        # Display data for each device at their current timestamp
        for i, folder in enumerate(self.subfolders):
            row = i // 4
            col = i % 4
            
            folder_name = folder.name
            current_idx = self.device_current_idx[folder_name]
            timestamps = self.device_timestamps[folder_name]
            
            # Ensure we don't exceed the subplot grid
            if row >= 2 or col >= 4:
                print(f"Warning: Too many devices ({len(self.subfolders)}), only showing first 8")
                break
            
            if current_idx < len(timestamps):
                current_timestamp = timestamps[current_idx]
                data = self.load_data(folder_name, current_timestamp, data_type)
                
                if data is not None:
                    # Choose colormap based on data type
                    if data_type in ['left', 'left_raw', 'right', 'right_raw']:
                        cmap = 'gray'
                    elif data_type == 'depth':
                        cmap = 'viridis'
                    elif data_type == 'disparity':
                        cmap = 'plasma'
                    else:
                        cmap = 'gray'
                    
                    im = self.axs[row, col].imshow(data, cmap=cmap)
                    
                    # Add colorbar for depth and disparity
                    if data_type in ['depth', 'disparity']:
                        plt.colorbar(im, ax=self.axs[row, col], fraction=0.046, pad=0.04)
                    
                    # Shorten folder name for display
                    short_name = folder_name.split('_')[0] + '_' + folder_name.split('_')[-1]
                    self.axs[row, col].set_title(f"{short_name}\n{data_type.upper()}\nFrame: {current_idx + 1}/{len(timestamps)}\n{current_timestamp}", fontsize=8)
                else:
                    # Check what data types are actually available for this timestamp
                    available_types = list(self.files_by_folder[folder_name].get(current_timestamp, {}).keys())
                    self.axs[row, col].text(0.5, 0.5, f'No {data_type}\ndata\nAvailable: {available_types[:3]}...', 
                                          ha='center', va='center', transform=self.axs[row, col].transAxes, fontsize=6)
                    short_name = folder_name.split('_')[0] + '_' + folder_name.split('_')[-1]
                    self.axs[row, col].set_title(f"{short_name}\n{data_type.upper()} - Missing", fontsize=8)
            else:
                # Device has run out of timestamps
                self.axs[row, col].text(0.5, 0.5, 'No more\ntimestamps', 
                                      ha='center', va='center', transform=self.axs[row, col].transAxes)
                short_name = folder_name.split('_')[0] + '_' + folder_name.split('_')[-1]
                self.axs[row, col].set_title(f"{short_name}\n{data_type.upper()} - End", fontsize=8)
            
            self.axs[row, col].axis('off')
        
        # Handle empty subplots for devices beyond what we have
        for i in range(len(self.subfolders), 8):
            row = i // 4
            col = i % 4
            if row < 2 and col < 4:
                self.axs[row, col].clear()
                self.axs[row, col].text(0.5, 0.5, 'No device', 
                                      ha='center', va='center', transform=self.axs[row, col].transAxes)
                self.axs[row, col].set_title('Empty', fontsize=8)
                self.axs[row, col].axis('off')
        
        # Update main title with current data type, progress info, and global frame counter
        active_devices = sum(1 for idx in self.device_current_idx.values() if idx < len(self.device_timestamps[list(self.device_current_idx.keys())[0]]))
        total_devices = len(self.subfolders)
        
        # Calculate global progress
        avg_frame_idx = current_frames // total_devices if total_devices > 0 else 0
        avg_total_frames = total_frames // total_devices if total_devices > 0 else 0
        
        self.fig.suptitle(f'Multi-Device Visualization | Data Type: {data_type.upper()} | '
                         f'Active Devices: {active_devices}/{total_devices} | '
                         f'Global Frame: ~{avg_frame_idx + 1}/{avg_total_frames}', 
                         fontsize=12)
        
        plt.tight_layout()
        plt.draw()
        
    def on_key_press(self, event):
        """Handle keyboard navigation"""
        update_display = False
        
        if event.key == 'left' or event.key == 'a':
            # Previous timestamp for all devices
            for folder_name in self.device_current_idx:
                self.device_current_idx[folder_name] = max(0, self.device_current_idx[folder_name] - 1)
            update_display = True
        elif event.key == 'right' or event.key == 'd':
            # Next timestamp for all devices
            for folder_name in self.device_current_idx:
                timestamps = self.device_timestamps[folder_name]
                self.device_current_idx[folder_name] = min(len(timestamps) - 1, self.device_current_idx[folder_name] + 1)
            update_display = True
        elif event.key == 'home':
            # First timestamp for all devices
            for folder_name in self.device_current_idx:
                self.device_current_idx[folder_name] = 0
            update_display = True
        elif event.key == 'end':
            # Last timestamp for all devices
            for folder_name in self.device_current_idx:
                timestamps = self.device_timestamps[folder_name]
                self.device_current_idx[folder_name] = len(timestamps) - 1
            update_display = True
        elif event.key == '1':
            # Switch to left camera
            self.current_data_type = 'left'
            self.display_data('left')
            return
        elif event.key == '2':
            # Switch to right camera
            self.current_data_type = 'right'
            self.display_data('right')
            return
        elif event.key == '3':
            # Switch to left_raw camera
            self.current_data_type = 'left_raw'
            self.display_data('left_raw')
            return
        elif event.key == '4':
            # Switch to right_raw camera
            self.current_data_type = 'right_raw'
            self.display_data('right_raw')
            return
        elif event.key == '5':
            # Switch to depth
            self.current_data_type = 'depth'
            self.display_data('depth')
            return
        elif event.key == '6':
            # Switch to disparity
            self.current_data_type = 'disparity'
            self.display_data('disparity')
            return
        elif event.key == 'f':
            # Toggle fast mode
            self.fast_mode = not self.fast_mode
            print(f"\nFast mode: {'ON' if self.fast_mode else 'OFF'}")
            return
        elif event.key == 'p':
            # Jump to specific percentage
            try:
                percentage = int(input("Enter percentage (0-100): "))
                if 0 <= percentage <= 100:
                    for folder_name, timestamps in self.device_timestamps.items():
                        target_frame = int(len(timestamps) * percentage / 100)
                        self.device_current_idx[folder_name] = min(len(timestamps) - 1, target_frame)
                    update_display = True
            except (ValueError, KeyboardInterrupt):
                print("Invalid input or cancelled")
            return
        elif event.key == 'q':
            # Quit
            plt.close()
            return
        
        # Update display only if needed
        if update_display:
            self.display_data(self.current_data_type)
            
        # Print navigation info (only if not in fast mode or every 10 frames)
        if not self.fast_mode or self.last_console_update % 10 == 0:
            active_devices = sum(1 for idx in self.device_current_idx.values() if idx < len(self.device_timestamps[list(self.device_current_idx.keys())[0]]))
            total_devices = len(self.subfolders)
            
            # Calculate global frame progress
            total_frames = 0
            current_frames = 0
            for folder_name, timestamps in self.device_timestamps.items():
                total_frames += len(timestamps)
                current_frames += self.device_current_idx[folder_name]
            
            avg_frame_idx = current_frames // total_devices if total_devices > 0 else 0
            avg_total_frames = total_frames // total_devices if total_devices > 0 else 0
            
            if not self.fast_mode:
                print(f"\n=== FRAME PROGRESS ===")
                print(f"Global Frame: ~{avg_frame_idx + 1}/{avg_total_frames} ({(avg_frame_idx + 1) * 100 // avg_total_frames}%)")
                print(f"Active devices: {active_devices}/{total_devices}")
                print("\nCurrent timestamps per device:")
                for folder_name, current_idx in self.device_current_idx.items():
                    timestamps = self.device_timestamps[folder_name]
                    if current_idx < len(timestamps):
                        short_name = folder_name.split('_')[0] + '_' + folder_name.split('_')[-1]
                        progress_pct = (current_idx + 1) * 100 // len(timestamps)
                        print(f"  {short_name}: Frame {current_idx + 1}/{len(timestamps)} ({progress_pct}%) - {timestamps[current_idx]}")
                    else:
                        short_name = folder_name.split('_')[0] + '_' + folder_name.split('_')[-1]
                        print(f"  {short_name}: COMPLETED ({len(timestamps)} frames)")
            else:
                print(f"Frame: ~{avg_frame_idx + 1}/{avg_total_frames} ({(avg_frame_idx + 1) * 100 // avg_total_frames}%) - {self.current_data_type.upper()}")
            
            print("\nControls: A/D to navigate timestamps, 1-6 to switch data types, F for fast mode, P for percentage jump, Q to quit")
            print("Data types: 1=Left, 2=Right, 3=Left_Raw, 4=Right_Raw, 5=Depth, 6=Disparity")
        
        self.last_console_update += 1
        
    def run(self):
        """Start the visualization"""
        if not self.device_timestamps:
            print("No timestamps found in any folder!")
            return
            
        # Display first data with current data type
        self.display_data(self.current_data_type)
        
        print(f"\nStarting multi-device visualization...")
        total_timestamps = sum(len(timestamps) for timestamps in self.device_timestamps.values())
        print(f"Total timestamps across all devices: {total_timestamps}")
        print("Note: Each device shows its own timestamp progression")
        print("Controls: A/D to navigate timestamps, 1-6 to switch data types, Q to quit")
        print("Data types: 1=Left, 2=Right, 3=Left_Raw, 4=Right_Raw, 5=Depth, 6=Disparity")
        plt.show()

def select_data_directory():
    """Open a dialog to select the data directory"""
    root = tk.Tk()
    root.withdraw()  # Hide the main window
    
    data_dir = filedialog.askdirectory(
        title="Select 4-device sync node data directory",
        initialdir="/home/katka/PycharmProjects/capture-viewer/DATA/4_device_sync_node"
    )
    
    root.destroy()
    return data_dir

def main():
    parser = argparse.ArgumentParser(
        description='Multi-device visualizer for 4-device sync node data',
        epilog='''
Examples:
  python show_multiple_npy.py /path/to/your/data
  python show_multiple_npy.py /path/to/your/data --data-type depth
  python show_multiple_npy.py --select
  python show_multiple_npy.py  # uses default path
        '''
    )
    parser.add_argument('data_path', type=str, nargs='?',
                       default='/home/katka/PycharmProjects/capture-viewer/DATA/4_device_sync_node',
                       help='Path to the data directory (optional, defaults to capture-viewer path)')
    parser.add_argument('--select', action='store_true',
                       help='Open file dialog to select data directory')
    parser.add_argument('--data-type', type=str, choices=['left', 'left_raw', 'right', 'right_raw', 'depth', 'disparity'],
                       default='left', help='Initial data type to display (default: left)')
    
    args = parser.parse_args()
    
    if args.select:
        data_path = select_data_directory()
        if not data_path:
            print("No directory selected. Exiting.")
            return
    else:
        data_path = args.data_path
    
    if not os.path.exists(data_path):
        print(f"Data path does not exist: {data_path}")
        print("Usage examples:")
        print("  python show_multiple_npy.py /path/to/your/data")
        print("  python show_multiple_npy.py /path/to/your/data --data-type left_raw")
        print("  python show_multiple_npy.py --select")
        print("  python show_multiple_npy.py  # uses default path")
        return
        
    # Create and run visualizer
    visualizer = MultiDeviceVisualizer(data_path)
    
    # Set initial data type if specified
    if args.data_type != 'left':
        visualizer.current_data_type = args.data_type
        print(f"Starting with data type: {args.data_type}")
    
    visualizer.run()

if __name__ == "__main__":
    main() 