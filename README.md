
# Capture Viewer

`capture-viewer` is a Luxonis Python tool designed to view and analyze data captured by OAK cameras. 
It provides functionalities to capture, display, and inspect stereo depth captures along with other types of camera data. 
It also allows the use of a REPLAY to regenerate data with different settings.

## Project Structure

This project is organized into two main modules:

### 📷 [Capture Module](capture/README.md)
The capture module contains all the tools and scripts for capturing data from various camera types:
- OAK-D cameras (Dai2 and Dai3)
- ZED cameras
- RealSense cameras
- Multiple device capture support
- Various capture modes and configurations

**📖 [Read the Capture Documentation →](capture/README.md)**

### 🖥️ [Viewer Module](viewer/README.md)
The viewer module provides the interface for viewing and analyzing captured data:
- Session browser and selection
- Multi-stream visualization (left, right, depth, RGB, etc.)
- REPLAY feature for regenerating depth data
- Point cloud visualization
- Metadata display

**📖 [Read the Viewer Documentation →](viewer/README.md)**

## Documentation

- **[Capture Module Documentation](capture/README.md)** - Complete guide for capturing data from various camera types
- **[Viewer Module Documentation](viewer/README.md)** - Complete guide for viewing and analyzing captured data
- **[Multiple Device Capture Guide](capture/README_multiple_device.md)** - Guide for capturing from multiple devices
- **[ZED and RealSense Integration](capture/README_zed_realsense.md)** - Guide for using ZED and RealSense cameras
