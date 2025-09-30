#!/usr/bin/env python3
"""
Multi-Device Data Visualizer for 4-device sync node data
Shows all 8 folders simultaneously, displaying the same timestamp across all devices
Supports both .npy files and RGB images (.png, .jpg, .jpeg)
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button, Slider
import tkinter as tk
from tkinter import filedialog, messagebox
import glob
from pathlib import Path
import argparse
from collections import defaultdict
from PIL import Image

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
        self.convert_bgr_to_rgb = True  # Convert BGR images to RGB
        
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
            # Support both .npy files and image files
            npy_files = list(folder.glob("*.npy"))
            image_files = list(folder.glob("*.png")) + list(folder.glob("*.jpg")) + list(folder.glob("*.jpeg"))
            files = npy_files + image_files
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
            file_path = timestamp_data[data_type]
            
            # Check if it's an image file or .npy file
            if file_path.suffix.lower() in ['.png', '.jpg', '.jpeg']:
                # Load image file
                image = Image.open(file_path)
                
                # Convert to RGB mode to ensure consistent color format
                if image.mode != 'RGB':
                    image = image.convert('RGB')
                
                data = np.array(image)
                
                # Convert BGR to RGB if needed (for OpenCV-saved images)
                if self.convert_bgr_to_rgb and len(data.shape) == 3 and data.shape[2] == 3:
                    data = data[:, :, ::-1]  # Reverse the color channels
            else:
                # Load .npy file
                data = np.load(file_path)
            
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
        
        # Connect mouse click events for delete buttons
        self.fig.canvas.mpl_connect('pick_event', self.on_pick)
        
    def reconnect_keyboard_events(self):
        """Reconnect keyboard events after figure recreation"""
        if self.fig:
            self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
            self.fig.canvas.mpl_connect('pick_event', self.on_pick)
        
    def display_data(self, data_type='left'):
        """Display data for all devices at their current timestamps"""
        if not self.fig:
            self.setup_plot()
        else:
            # Clear the entire figure to remove any leftover colorbars
            self.fig.clear()
            # Recreate subplots
            self.axs = self.fig.subplots(2, 4)
            # Reconnect events
            self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
            self.fig.canvas.mpl_connect('pick_event', self.on_pick)
        
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
            
            # Check if device has any timestamps left
            if len(timestamps) == 0:
                # Device has no timestamps left (all deleted)
                self.axs[row, col].text(0.5, 0.5, 'All frames\ndeleted', 
                                      ha='center', va='center', transform=self.axs[row, col].transAxes,
                                      fontsize=10, color='red')
                short_name = folder_name.split('_')[0] + '_' + folder_name.split('_')[-1]
                self.axs[row, col].set_title(f"{short_name}\n{data_type.upper()} - Empty", fontsize=8)
            elif current_idx < len(timestamps):
                current_timestamp = timestamps[current_idx]
                data = self.load_data(folder_name, current_timestamp, data_type)
                
                if data is not None:
                    # Check if data is RGB (3D array with 3 channels)
                    is_rgb = len(data.shape) == 3 and data.shape[2] == 3
                    
                    if is_rgb:
                        # Display RGB image without colormap
                        im = self.axs[row, col].imshow(data)
                    else:
                        # Choose colormap based on data type for grayscale/depth data
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
                    
                    # Add delete button (small X) in the top-right corner
                    self.add_delete_button(row, col, folder_name, current_timestamp, data_type)
                    
                    # Add move button (small M) in the top-left corner
                    self.add_move_button(row, col, folder_name, current_timestamp, data_type)
                    
                    # Add single delete button (small x) in the bottom-right corner
                    self.add_single_delete_button(row, col, folder_name, current_timestamp, data_type)
                    
                    # Add single move button (small m) in the bottom-left corner
                    self.add_single_move_button(row, col, folder_name, current_timestamp, data_type)
                    
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
        active_devices = sum(1 for timestamps in self.device_timestamps.values() if len(timestamps) > 0)
        total_devices = len(self.subfolders)
        
        # Calculate global progress
        avg_frame_idx = current_frames // active_devices if active_devices > 0 else 0
        avg_total_frames = total_frames // active_devices if active_devices > 0 else 0
        
        self.fig.suptitle(f'Multi-Device Visualization | Data Type: {data_type.upper()} | '
                         f'Active Devices: {active_devices}/{total_devices} | '
                         f'Global Frame: ~{avg_frame_idx + 1}/{avg_total_frames}', 
                         fontsize=12)
        
        plt.tight_layout()
        plt.draw()
    
    def add_delete_button(self, row, col, folder_name, timestamp, data_type):
        """Add a small X button in the top-right corner of the subplot"""
        if row >= 2 or col >= 4:
            return
            
        # Get the subplot position
        ax = self.axs[row, col]
        
        # Add a small X button in the top-right corner
        # Position: (0.9, 0.9) in axes coordinates (top-right)
        delete_text = ax.text(0.9, 0.9, '✕', fontsize=12, color='red', 
                             ha='center', va='center', transform=ax.transAxes,
                             bbox=dict(boxstyle='circle', facecolor='white', 
                                     edgecolor='red', linewidth=1),
                             picker=5)  # Set picker to 5 pixels tolerance
        
        # Store metadata for the delete button
        delete_text._delete_info = {
            'folder_name': folder_name,
            'timestamp': timestamp,
            'data_type': data_type,
            'row': row,
            'col': col
        }
    
    def add_move_button(self, row, col, folder_name, timestamp, data_type):
        """Add a small M button in the top-left corner of the subplot"""
        if row >= 2 or col >= 4:
            return
            
        # Get the subplot position
        ax = self.axs[row, col]
        
        # Add a small M button in the top-left corner
        # Position: (0.1, 0.9) in axes coordinates (top-left)
        move_text = ax.text(0.1, 0.9, 'M', fontsize=12, color='blue', 
                           ha='center', va='center', transform=ax.transAxes,
                           bbox=dict(boxstyle='circle', facecolor='white', 
                                   edgecolor='blue', linewidth=1),
                           picker=5)  # Set picker to 5 pixels tolerance
        
        # Store metadata for the move button
        move_text._move_info = {
            'folder_name': folder_name,
            'timestamp': timestamp,
            'data_type': data_type,
            'row': row,
            'col': col
        }
    
    def add_single_delete_button(self, row, col, folder_name, timestamp, data_type):
        """Add a small x button in the bottom-right corner of the subplot for single file deletion"""
        if row >= 2 or col >= 4:
            return
            
        # Get the subplot position
        ax = self.axs[row, col]
        
        # Add a small x button in the bottom-right corner
        # Position: (0.9, 0.1) in axes coordinates (bottom-right)
        single_delete_text = ax.text(0.9, 0.1, 'x', fontsize=10, color='darkred', 
                                   ha='center', va='center', transform=ax.transAxes,
                                   bbox=dict(boxstyle='circle', facecolor='lightgray', 
                                           edgecolor='darkred', linewidth=1),
                                   picker=5)  # Set picker to 5 pixels tolerance
        
        # Store metadata for the single delete button
        single_delete_text._single_delete_info = {
            'folder_name': folder_name,
            'timestamp': timestamp,
            'data_type': data_type,
            'row': row,
            'col': col
        }
    
    def add_single_move_button(self, row, col, folder_name, timestamp, data_type):
        """Add a small m button in the bottom-left corner of the subplot for single file moving"""
        if row >= 2 or col >= 4:
            return
            
        # Get the subplot position
        ax = self.axs[row, col]
        
        # Add a small m button in the bottom-left corner
        # Position: (0.1, 0.1) in axes coordinates (bottom-left)
        single_move_text = ax.text(0.1, 0.1, 'm', fontsize=10, color='darkblue', 
                                 ha='center', va='center', transform=ax.transAxes,
                                 bbox=dict(boxstyle='circle', facecolor='lightblue', 
                                         edgecolor='darkblue', linewidth=1),
                                 picker=5)  # Set picker to 5 pixels tolerance
        
        # Store metadata for the single move button
        single_move_text._single_move_info = {
            'folder_name': folder_name,
            'timestamp': timestamp,
            'data_type': data_type,
            'row': row,
            'col': col
        }
    
    def on_pick(self, event):
        """Handle pick events for delete and move buttons"""
        # Check if we picked a delete button
        if hasattr(event.artist, '_delete_info'):
            delete_info = event.artist._delete_info
            self.confirm_and_delete(delete_info)
        # Check if we picked a move button
        elif hasattr(event.artist, '_move_info'):
            move_info = event.artist._move_info
            self.confirm_and_move(move_info)
        # Check if we picked a single delete button
        elif hasattr(event.artist, '_single_delete_info'):
            single_delete_info = event.artist._single_delete_info
            self.confirm_and_single_delete(single_delete_info)
        # Check if we picked a single move button
        elif hasattr(event.artist, '_single_move_info'):
            single_move_info = event.artist._single_move_info
            self.confirm_and_single_move(single_move_info)
    
    def confirm_and_delete(self, delete_info):
        """Show confirmation dialog and delete the file if confirmed"""
        folder_name = delete_info['folder_name']
        timestamp = delete_info['timestamp']
        data_type = delete_info['data_type']
        
        # Get the file path
        if folder_name not in self.files_by_folder:
            return
            
        timestamp_data = self.files_by_folder[folder_name].get(timestamp, {})
        if data_type not in timestamp_data:
            return
            
        file_path = timestamp_data[data_type]
        
        # Determine related files to delete
        related_types = self.get_related_data_types(data_type)
        related_files = []
        
        for related_type in related_types:
            if related_type in timestamp_data:
                related_files.append((related_type, timestamp_data[related_type]))
        
        # Show confirmation dialog with information about related files
        message = f"Are you sure you want to delete this file?\n\n"
        message += f"File: {file_path.name}\n"
        message += f"Device: {folder_name}\n"
        message += f"Timestamp: {timestamp}\n"
        message += f"Data Type: {data_type}\n\n"
        
        if related_files:
            message += f"This will also delete {len(related_files)} related files:\n"
            for related_type, related_path in related_files:
                message += f"  - {related_path.name}\n"
        
        root = tk.Tk()
        root.withdraw()  # Hide the main window
        
        result = messagebox.askyesno(
            "Confirm Delete",
            message,
            icon='warning'
        )
        
        root.destroy()
        
        if result:
            try:
                # Delete the main file
                file_path.unlink()
                print(f"Deleted: {file_path}")
                
                # Delete related files
                for related_type, related_path in related_files:
                    try:
                        related_path.unlink()
                        print(f"Deleted related: {related_path}")
                    except Exception as e:
                        print(f"Error deleting related file {related_path}: {e}")
                
                # Remove from data structure
                if timestamp in self.files_by_folder[folder_name]:
                    # Remove the main data type
                    del self.files_by_folder[folder_name][timestamp][data_type]
                    
                    # Remove related data types
                    for related_type, _ in related_files:
                        if related_type in self.files_by_folder[folder_name][timestamp]:
                            del self.files_by_folder[folder_name][timestamp][related_type]
                    
                    # If no more data types for this timestamp, remove the timestamp
                    if not self.files_by_folder[folder_name][timestamp]:
                        del self.files_by_folder[folder_name][timestamp]
                        
                        # Update device timestamps
                        if timestamp in self.device_timestamps[folder_name]:
                            self.device_timestamps[folder_name].remove(timestamp)
                            
                            # Adjust current index if necessary
                            if self.device_current_idx[folder_name] >= len(self.device_timestamps[folder_name]):
                                self.device_current_idx[folder_name] = max(0, len(self.device_timestamps[folder_name]) - 1)
                
                # Clear cache for this file and related files
                cache_key = f"{folder_name}_{timestamp}_{data_type}"
                if cache_key in self.data_cache:
                    del self.data_cache[cache_key]
                
                for related_type, _ in related_files:
                    related_cache_key = f"{folder_name}_{timestamp}_{related_type}"
                    if related_cache_key in self.data_cache:
                        del self.data_cache[related_cache_key]
                
                # Refresh the display
                self.display_data(self.current_data_type)
                
            except Exception as e:
                print(f"Error deleting file {file_path}: {e}")
                # Show error dialog
                root = tk.Tk()
                root.withdraw()
                messagebox.showerror("Delete Error", f"Failed to delete file:\n{file_path}\n\nError: {e}")
                root.destroy()
    
    def get_related_data_types(self, data_type):
        """Get related data types that should be deleted together"""
        if data_type == 'left':
            return ['right', 'left_raw', 'right_raw']
        elif data_type == 'right':
            return ['left', 'left_raw', 'right_raw']
        elif data_type == 'left_raw':
            return ['left', 'right', 'right_raw']
        elif data_type == 'right_raw':
            return ['left', 'right', 'left_raw']
        elif data_type == 'depth':
            return ['disparity']  # Depth and disparity are related
        elif data_type == 'disparity':
            return ['depth']  # Depth and disparity are related
        elif data_type == 'rgb':
            return []  # RGB images are standalone
        else:
            return []  # No related types for other data types
    
    def confirm_and_single_delete(self, delete_info):
        """Show confirmation dialog and delete only the single file if confirmed"""
        folder_name = delete_info['folder_name']
        timestamp = delete_info['timestamp']
        data_type = delete_info['data_type']
        
        # Get the file path
        if folder_name not in self.files_by_folder:
            return
            
        timestamp_data = self.files_by_folder[folder_name].get(timestamp, {})
        if data_type not in timestamp_data:
            return
            
        file_path = timestamp_data[data_type]
        
        # Show confirmation dialog for single file deletion
        message = f"Are you sure you want to delete ONLY this file?\n\n"
        message += f"File: {file_path.name}\n"
        message += f"Device: {folder_name}\n"
        message += f"Timestamp: {timestamp}\n"
        message += f"Data Type: {data_type}\n\n"
        message += f"Note: This will NOT delete related files (left/right/raw files)"
        
        root = tk.Tk()
        root.withdraw()  # Hide the main window
        
        result = messagebox.askyesno(
            "Confirm Single Delete",
            message,
            icon='warning'
        )
        
        root.destroy()
        
        if result:
            try:
                # Delete only the main file
                file_path.unlink()
                print(f"Deleted single file: {file_path}")
                
                # Remove from data structure
                if timestamp in self.files_by_folder[folder_name]:
                    # Remove only the specific data type
                    del self.files_by_folder[folder_name][timestamp][data_type]
                    
                    # If no more data types for this timestamp, remove the timestamp
                    if not self.files_by_folder[folder_name][timestamp]:
                        del self.files_by_folder[folder_name][timestamp]
                        
                        # Update device timestamps
                        if timestamp in self.device_timestamps[folder_name]:
                            self.device_timestamps[folder_name].remove(timestamp)
                            
                            # Adjust current index if necessary
                            if self.device_current_idx[folder_name] >= len(self.device_timestamps[folder_name]):
                                self.device_current_idx[folder_name] = max(0, len(self.device_timestamps[folder_name]) - 1)
                
                # Clear cache for this file only
                cache_key = f"{folder_name}_{timestamp}_{data_type}"
                if cache_key in self.data_cache:
                    del self.data_cache[cache_key]
                
                # Refresh the display
                self.display_data(self.current_data_type)
                
            except Exception as e:
                print(f"Error deleting file {file_path}: {e}")
                # Show error dialog
                root = tk.Tk()
                root.withdraw()
                messagebox.showerror("Delete Error", f"Failed to delete file:\n{file_path}\n\nError: {e}")
                root.destroy()
    
    def confirm_and_single_move(self, move_info):
        """Show confirmation dialog and move only the single file if confirmed"""
        folder_name = move_info['folder_name']
        timestamp = move_info['timestamp']
        data_type = move_info['data_type']
        
        # Get the file path
        if folder_name not in self.files_by_folder:
            return
            
        timestamp_data = self.files_by_folder[folder_name].get(timestamp, {})
        if data_type not in timestamp_data:
            return
            
        file_path = timestamp_data[data_type]
        
        # Find compatible folders (same model)
        compatible_folders = self.get_compatible_folders(folder_name)
        
        if not compatible_folders:
            root = tk.Tk()
            root.withdraw()
            messagebox.showinfo("No Compatible Folders", 
                              f"No compatible folders found for {folder_name}")
            root.destroy()
            return
        
        # Show folder selection dialog
        root = tk.Tk()
        root.withdraw()
        
        # Create a simple dialog for folder selection
        from tkinter import simpledialog
        
        folder_list = "\n".join([f"{i+1}. {folder}" for i, folder in enumerate(compatible_folders)])
        message = f"Select destination folder for moving SINGLE file:\n\n{folder_list}\n\nEnter folder number (1-{len(compatible_folders)}):"
        
        try:
            choice = simpledialog.askinteger("Move Single File", message, minvalue=1, maxvalue=len(compatible_folders))
            root.destroy()
            
            if choice is None:  # User cancelled
                return
                
            dest_folder = compatible_folders[choice - 1]
            
            # Confirm the move operation
            result = messagebox.askyesno(
                "Confirm Single Move",
                f"Are you sure you want to move ONLY this file?\n\n"
                f"File: {file_path.name}\n"
                f"From: {folder_name}\n"
                f"To: {dest_folder}\n"
                f"Timestamp: {timestamp}\n"
                f"Data Type: {data_type}\n\n"
                f"Note: This will NOT move related files (left/right/raw files)",
                icon='question'
            )
            
            if result:
                self.move_single_file(folder_name, dest_folder, timestamp, data_type)
                
        except Exception as e:
            root.destroy()
            messagebox.showerror("Error", f"Error in folder selection: {e}")
    
    def move_single_file(self, source_folder, dest_folder, timestamp, data_type):
        """Move only a single file from source to destination folder"""
        try:
            # Get the specific file for this timestamp and data type
            source_timestamp_data = self.files_by_folder[source_folder].get(timestamp, {})
            
            if data_type not in source_timestamp_data:
                return
            
            source_path = source_timestamp_data[data_type]
            dest_path = self.data_path / dest_folder / source_path.name
            
            # Create destination directory if it doesn't exist
            dest_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Move the file
            source_path.rename(dest_path)
            print(f"Moved single file: {source_path} -> {dest_path}")
            
            # Update data structures
            self.update_data_structures_after_single_move(source_folder, dest_folder, timestamp, data_type)
            
            # Show success message
            root = tk.Tk()
            root.withdraw()
            messagebox.showinfo("Move Successful", 
                              f"Successfully moved single file:\n{data_type}: {source_path.name}")
            root.destroy()
            
            # Refresh the display
            self.display_data(self.current_data_type)
            
        except Exception as e:
            print(f"Error moving single file: {e}")
            root = tk.Tk()
            root.withdraw()
            messagebox.showerror("Move Error", f"Failed to move single file:\n{str(e)}")
            root.destroy()
    
    def update_data_structures_after_single_move(self, source_folder, dest_folder, timestamp, data_type):
        """Update internal data structures after moving a single file"""
        # Remove from source folder
        if timestamp in self.files_by_folder[source_folder]:
            if data_type in self.files_by_folder[source_folder][timestamp]:
                del self.files_by_folder[source_folder][timestamp][data_type]
            
            # If no more data types for this timestamp, remove the timestamp
            if not self.files_by_folder[source_folder][timestamp]:
                del self.files_by_folder[source_folder][timestamp]
                
                # Update device timestamps
                if timestamp in self.device_timestamps[source_folder]:
                    self.device_timestamps[source_folder].remove(timestamp)
                    
                    # Adjust current index if necessary
                    if self.device_current_idx[source_folder] >= len(self.device_timestamps[source_folder]):
                        self.device_current_idx[source_folder] = max(0, len(self.device_timestamps[source_folder]) - 1)
        
        # Add to destination folder
        if dest_folder not in self.files_by_folder:
            self.files_by_folder[dest_folder] = {}
        
        if timestamp not in self.files_by_folder[dest_folder]:
            self.files_by_folder[dest_folder][timestamp] = {}
        
        # Add moved file to destination data structure
        dest_path = self.data_path / dest_folder / f"{data_type}_{timestamp}.npy"
        if dest_path.exists():
            self.files_by_folder[dest_folder][timestamp][data_type] = dest_path
        
        # Update device timestamps for destination
        if dest_folder not in self.device_timestamps:
            self.device_timestamps[dest_folder] = []
        
        if timestamp not in self.device_timestamps[dest_folder]:
            self.device_timestamps[dest_folder].append(timestamp)
            self.device_timestamps[dest_folder].sort()
        
        # Clear cache for moved file
        source_cache_key = f"{source_folder}_{timestamp}_{data_type}"
        dest_cache_key = f"{dest_folder}_{timestamp}_{data_type}"
        
        if source_cache_key in self.data_cache:
            del self.data_cache[source_cache_key]
        if dest_cache_key in self.data_cache:
            del self.data_cache[dest_cache_key]
    
    def confirm_and_move(self, move_info):
        """Show confirmation dialog and move the files if confirmed"""
        folder_name = move_info['folder_name']
        timestamp = move_info['timestamp']
        data_type = move_info['data_type']
        
        # Get the file path
        if folder_name not in self.files_by_folder:
            return
            
        timestamp_data = self.files_by_folder[folder_name].get(timestamp, {})
        if data_type not in timestamp_data:
            return
            
        file_path = timestamp_data[data_type]
        
        # Find compatible folders (same model)
        compatible_folders = self.get_compatible_folders(folder_name)
        
        if not compatible_folders:
            root = tk.Tk()
            root.withdraw()
            messagebox.showinfo("No Compatible Folders", 
                              f"No compatible folders found for {folder_name}")
            root.destroy()
            return
        
        # Show folder selection dialog
        root = tk.Tk()
        root.withdraw()
        
        # Create a simple dialog for folder selection
        from tkinter import simpledialog
        
        folder_list = "\n".join([f"{i+1}. {folder}" for i, folder in enumerate(compatible_folders)])
        message = f"Select destination folder for moving:\n\n{folder_list}\n\nEnter folder number (1-{len(compatible_folders)}):"
        
        try:
            choice = simpledialog.askinteger("Move Files", message, minvalue=1, maxvalue=len(compatible_folders))
            root.destroy()
            
            if choice is None:  # User cancelled
                return
                
            dest_folder = compatible_folders[choice - 1]
            
            # Confirm the move operation
            result = messagebox.askyesno(
                "Confirm Move",
                f"Are you sure you want to move this file?\n\n"
                f"File: {file_path.name}\n"
                f"From: {folder_name}\n"
                f"To: {dest_folder}\n"
                f"Timestamp: {timestamp}\n"
                f"Data Type: {data_type}",
                icon='question'
            )
            
            if result:
                self.move_files(folder_name, dest_folder, timestamp, data_type)
                
        except Exception as e:
            root.destroy()
            messagebox.showerror("Error", f"Error in folder selection: {e}")
    
    def get_compatible_folders(self, source_folder):
        """Get folders that are compatible for moving (same model)"""
        # Extract model name by removing the timestamp and True/False suffix
        # Format: MODEL_SERIAL_TIMESTAMP_True/False
        parts = source_folder.split('_')
        
        # Find the model name by looking for patterns
        if len(parts) >= 4:
            # For OAK4-D-PRO_2460444260_20250710164544_True format
            # Take everything except the last two parts (timestamp and True/False)
            model_name = '_'.join(parts[:-2])
        elif len(parts) >= 3:
            # For OAK-D-PRO-POE_18443010111E970F00_20250710164551_True format
            # Take everything except the last two parts (timestamp and True/False)
            model_name = '_'.join(parts[:-2])
        else:
            # Fallback: remove only the last part
            model_name = '_'.join(parts[:-1])
        
        compatible_folders = []
        for folder in self.subfolders:
            folder_name = folder.name
            # Check if it's the same model but different folder
            if folder_name.startswith(model_name) and folder_name != source_folder:
                compatible_folders.append(folder_name)
        
        return compatible_folders
    
    def move_files(self, source_folder, dest_folder, timestamp, data_type):
        """Move files from source to destination folder"""
        try:
            # Get all files for this timestamp and data type
            source_timestamp_data = self.files_by_folder[source_folder].get(timestamp, {})
            
            if data_type not in source_timestamp_data:
                return
            
            # Get related data types
            related_types = self.get_related_data_types(data_type)
            all_types = [data_type] + related_types
            
            moved_files = []
            
            # Move each file type
            for file_type in all_types:
                if file_type in source_timestamp_data:
                    source_path = source_timestamp_data[file_type]
                    dest_path = self.data_path / dest_folder / source_path.name
                    
                    # Create destination directory if it doesn't exist
                    dest_path.parent.mkdir(parents=True, exist_ok=True)
                    
                    # Move the file
                    source_path.rename(dest_path)
                    moved_files.append((file_type, source_path.name))
                    print(f"Moved: {source_path} -> {dest_path}")
            
            # Update data structures
            self.update_data_structures_after_move(source_folder, dest_folder, timestamp, all_types)
            
            # Show success message
            root = tk.Tk()
            root.withdraw()
            messagebox.showinfo("Move Successful", 
                              f"Successfully moved {len(moved_files)} files:\n" + 
                              "\n".join([f"  - {file_type}: {filename}" for file_type, filename in moved_files]))
            root.destroy()
            
            # Refresh the display
            self.display_data(self.current_data_type)
            
        except Exception as e:
            print(f"Error moving files: {e}")
            root = tk.Tk()
            root.withdraw()
            messagebox.showerror("Move Error", f"Failed to move files:\n{str(e)}")
            root.destroy()
    
    def update_data_structures_after_move(self, source_folder, dest_folder, timestamp, data_types):
        """Update internal data structures after moving files"""
        # Remove from source folder
        if timestamp in self.files_by_folder[source_folder]:
            for data_type in data_types:
                if data_type in self.files_by_folder[source_folder][timestamp]:
                    del self.files_by_folder[source_folder][timestamp][data_type]
            
            # If no more data types for this timestamp, remove the timestamp
            if not self.files_by_folder[source_folder][timestamp]:
                del self.files_by_folder[source_folder][timestamp]
                
                # Update device timestamps
                if timestamp in self.device_timestamps[source_folder]:
                    self.device_timestamps[source_folder].remove(timestamp)
                    
                    # Adjust current index if necessary
                    if self.device_current_idx[source_folder] >= len(self.device_timestamps[source_folder]):
                        self.device_current_idx[source_folder] = max(0, len(self.device_timestamps[source_folder]) - 1)
        
        # Add to destination folder
        if dest_folder not in self.files_by_folder:
            self.files_by_folder[dest_folder] = {}
        
        if timestamp not in self.files_by_folder[dest_folder]:
            self.files_by_folder[dest_folder][timestamp] = {}
        
        # Add moved files to destination data structure
        for data_type in data_types:
            dest_path = self.data_path / dest_folder / f"{data_type}_{timestamp}.npy"
            if dest_path.exists():
                self.files_by_folder[dest_folder][timestamp][data_type] = dest_path
        
        # Update device timestamps for destination
        if dest_folder not in self.device_timestamps:
            self.device_timestamps[dest_folder] = []
        
        if timestamp not in self.device_timestamps[dest_folder]:
            self.device_timestamps[dest_folder].append(timestamp)
            self.device_timestamps[dest_folder].sort()
        
        # Clear cache for moved files
        for data_type in data_types:
            source_cache_key = f"{source_folder}_{timestamp}_{data_type}"
            dest_cache_key = f"{dest_folder}_{timestamp}_{data_type}"
            
            if source_cache_key in self.data_cache:
                del self.data_cache[source_cache_key]
            if dest_cache_key in self.data_cache:
                del self.data_cache[dest_cache_key]
        
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
                if len(timestamps) > 0:  # Only navigate if there are timestamps
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
                if len(timestamps) > 0:  # Only navigate if there are timestamps
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
        elif event.key == '7':
            # Switch to RGB images
            self.current_data_type = 'rgb'
            self.display_data('rgb')
            return
        elif event.key == 'f':
            # Toggle fast mode
            self.fast_mode = not self.fast_mode
            print(f"\nFast mode: {'ON' if self.fast_mode else 'OFF'}")
            return
        elif event.key == 'c':
            # Toggle BGR to RGB conversion
            self.convert_bgr_to_rgb = not self.convert_bgr_to_rgb
            print(f"\nBGR to RGB conversion: {'ON' if self.convert_bgr_to_rgb else 'OFF'}")
            # Clear cache to reload images with new setting
            self.data_cache.clear()
            # Refresh display
            self.display_data(self.current_data_type)
            return
        elif event.key == 'p':
            # Jump to specific percentage
            try:
                percentage = int(input("Enter percentage (0-100): "))
                if 0 <= percentage <= 100:
                    for folder_name, timestamps in self.device_timestamps.items():
                        if len(timestamps) > 0:  # Only navigate if there are timestamps
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
            # Count active devices (devices with timestamps)
            active_devices = sum(1 for timestamps in self.device_timestamps.values() if len(timestamps) > 0)
            total_devices = len(self.subfolders)
            
            # Calculate global frame progress
            total_frames = 0
            current_frames = 0
            for folder_name, timestamps in self.device_timestamps.items():
                if len(timestamps) > 0:  # Only count devices with timestamps
                    total_frames += len(timestamps)
                    current_frames += self.device_current_idx[folder_name]
            
            avg_frame_idx = current_frames // active_devices if active_devices > 0 else 0
            avg_total_frames = total_frames // active_devices if active_devices > 0 else 0
            
            if not self.fast_mode:
                print(f"\n=== FRAME PROGRESS ===")
                print(f"Global Frame: ~{avg_frame_idx + 1}/{avg_total_frames} ({(avg_frame_idx + 1) * 100 // avg_total_frames}%)")
                print(f"Active devices: {active_devices}/{total_devices}")
                print("\nCurrent timestamps per device:")
                for folder_name, current_idx in self.device_current_idx.items():
                    timestamps = self.device_timestamps[folder_name]
                    short_name = folder_name.split('_')[0] + '_' + folder_name.split('_')[-1]
                    
                    if len(timestamps) == 0:
                        print(f"  {short_name}: EMPTY (all frames deleted)")
                    elif current_idx < len(timestamps):
                        progress_pct = (current_idx + 1) * 100 // len(timestamps)
                        print(f"  {short_name}: Frame {current_idx + 1}/{len(timestamps)} ({progress_pct}%) - {timestamps[current_idx]}")
                    else:
                        print(f"  {short_name}: COMPLETED ({len(timestamps)} frames)")
            else:
                print(f"Frame: ~{avg_frame_idx + 1}/{avg_total_frames} ({(avg_frame_idx + 1) * 100 // avg_total_frames}%) - {self.current_data_type.upper()}")
            
            print("\nControls: A/D to navigate timestamps, 1-7 to switch data types, F for fast mode, C to toggle BGR/RGB, P for percentage jump, Q to quit")
            print("Data types: 1=Left, 2=Right, 3=Left_Raw, 4=Right_Raw, 5=Depth, 6=Disparity, 7=RGB")
            print("Batch operations: X (red) to delete all related files, M (blue) to move all related files")
            print("Single operations: x (dark red) to delete only this file, m (dark blue) to move only this file")
        
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
        print("Controls: A/D to navigate timestamps, 1-7 to switch data types, C to toggle BGR/RGB, Q to quit")
        print("Data types: 1=Left, 2=Right, 3=Left_Raw, 4=Right_Raw, 5=Depth, 6=Disparity, 7=RGB")
        print("Batch operations: X (red) to delete all related files, M (blue) to move all related files")
        print("Single operations: x (dark red) to delete only this file, m (dark blue) to move only this file")
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
  python show_multiple_npy.py /path/to/your/data --data-type rgb
  python show_multiple_npy.py --select
  python show_multiple_npy.py  # uses default path
        '''
    )
    parser.add_argument('data_path', type=str, nargs='?',
                       default='/home/katka/PycharmProjects/capture-viewer/DATA/4_device_sync_node',
                       help='Path to the data directory (optional, defaults to capture-viewer path)')
    parser.add_argument('--select', action='store_true',
                       help='Open file dialog to select data directory')
    parser.add_argument('--data-type', type=str, choices=['left', 'left_raw', 'right', 'right_raw', 'depth', 'disparity', 'rgb'],
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