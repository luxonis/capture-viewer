#!/usr/bin/env python3

import depthai as dai
import numpy as np
import time
import json
import cv2
import os

print(dai.__version__)

if str(dai.__version__)[0] != "3":
    print("Go to depthai-core branch v3-develop")
    print("and run this: depthai-core/examples/python/install_requirements.py")
    exit("U dont have depthai 3")

script_dir = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.join(os.path.dirname(script_dir), 'DATA')

from utils.show_frames import visualize_frame, visualize_frame_info
from utils.capture_universal import initialize_capture, finalise_capture, count_output_streams
from utils.parse_arguments import parseArguments, process_argument_logic
from utils.isp_control import initialize_mono_control, controlQueueSend

from pipelines.dai3_tof_pipeline import initialize_pipeline

def colorizeToFDepth(frameDepth, phaseUnwrappingLevel=4):
    """Colorize ToF depth frame for visualization based on phase unwrapping level"""
    # Handle invalid/zero depth values
    invalidMask = (frameDepth == 0) | (frameDepth < 0) | np.isnan(frameDepth)
    
    # ToF depth data is in 16-bit format where max value (65535) represents max distance
    # Calculate actual max distance in mm based on phase unwrapping level
    # For 100MHz: max_distance_mm = (phaseUnwrappingLevel + 1) * 1498
    maxDistanceMm = (phaseUnwr0appingLevel + 1) * 1498
    
    # Convert 16-bit depth values to actual distances in mm
    # Scale from [0, 65535] to [0, maxDistanceMm]
    actualDepthMm = (frameDepth / 65535.0) * maxDistanceMm
    
    # Get valid depth values for better range estimation
    validDepths = actualDepthMm[~invalidMask]
    if len(validDepths) > 0:
        # Use 95th percentile for better visualization (avoids outliers)
        actualMaxDepth = np.percentile(validDepths, 95)
        # Use the smaller of calculated max or actual data max
        maxDepth = min(maxDistanceMm, actualMaxDepth)
    else:
        maxDepth = maxDistanceMm
    
    # Ensure we have a reasonable range
    if maxDepth <= 0:
        maxDepth = 1000  # Default fallback
    
    # Normalize actual depth to 0-255 range
    depthNormalized = np.clip(actualDepthMm / maxDepth, 0, 1)
    depthFrameColor = (depthNormalized * 255).astype(np.uint8)
    
    # Apply colormap
    depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)
    
    # Set invalid pixels to black
    depthFrameColor[invalidMask] = [0, 0, 0]
    
    return depthFrameColor

def main(args):
    settings_path, view_name, ip, autostart, autostart_time, wait_end, show_streams, _ = process_argument_logic(args)
    print(f"connecting to device... IP: {ip}")

    if ip is not None: 
        device = dai.Device(ip)
    else: 
        device = dai.Device()
    mxid = device.getDeviceId()

    device_name = device.getDeviceName()
    print(f"Device connected! Device Name: {device_name}")

    with open(settings_path) as settings_file:
        settings = json.load(settings_file)

    output_folders = {}
    mxids = [mxid]
    num_captures = {mxid: 0}
    streams = [None]

    save = False
    capture_ended = False

    streams = count_output_streams(settings['output_settings'])
    if settings['num_captures'] == 'inf' or settings['num_captures'] == 'INF': 
        settings['num_captures'] = float('inf')
    final_num_captures = settings['num_captures'] * len(streams)
    print(f"Streams: {streams}")
    print(f"Number of streams: {len(streams)}")
    print(f"Will capture max frames ({settings['num_captures']}) * number of streams ({len(streams)}) = {final_num_captures}")

    initial_time = time.time()
    if autostart_time:
        print("waiting till:", autostart_time)
        initialize_capture_time = autostart_time.timestamp()
    else:
        initialize_capture_time = initial_time + autostart

    with dai.Pipeline(device) as pipeline:
        pipeline, q, input_queues = initialize_pipeline(pipeline, settings)
        pipeline.start()

        platform = pipeline.getDefaultDevice().getPlatform()
        print(platform)
        if platform == dai.Platform.RVC4:
            control = initialize_mono_control(settings)
            controlQueueSend(input_queues, control)

        if settings['ir']: 
            pipeline.getDefaultDevice().setIrLaserDotProjectorIntensity(settings['ir_value'])
        if settings['flood_light']: 
            pipeline.getDefaultDevice().setIrFloodLightIntensity(settings['flood_light_intensity'])

        print("Starting loop...")
        while pipeline.isRunning():
            if not save and autostart > -1 and time.time() > initialize_capture_time:
                out_dir = initialize_capture(root_path, device, settings_path, view_name)
                output_folders[mxid] = out_dir

                save = True
                print("Starting capture via autosave")
                start_time = time.time()

            if settings["output_settings"]["sync"]:
                if not q['sync'].has():
                    continue
                msgGrp = q['sync'].get()
                for name, msg in msgGrp:
                    timestamp = int(msg.getTimestamp().total_seconds() * 1000)

                    cvFrame = msg.getCvFrame()

                    if save:
                        if name in ['left', 'right']:
                            if len(cvFrame.shape) == 3:
                                cvFrame = cv2.cvtColor(cvFrame, cv2.COLOR_BGR2GRAY)
                        np.save(f'{output_folders[mxid]}/{name}_{timestamp}.npy', cvFrame)
                        num_captures[mxid] += 1
                    if show_streams:
                        # Special visualization for ToF depth
                        if name == 'tof_depth':
                            phaseUnwrappingLevel = settings.get("tofConfig", {}).get("phaseUnwrappingLevel", 4)
                            # Debug: print depth statistics for each frame
                            h, w = cvFrame.shape[:2]
                            center_h, center_w = h // 2, w // 2
                            center_region = cvFrame[center_h-5:center_h+5, center_w-5:center_w+5]
                            center_mean = np.mean(center_region) if center_region.size > 0 else 0
                            
                            # Convert to actual distances for better understanding
                            maxDistanceMm = (phaseUnwrappingLevel + 1) * 1498
                            actualMinMm = (np.min(cvFrame) / 65535.0) * maxDistanceMm
                            actualMaxMm = (np.max(cvFrame) / 65535.0) * maxDistanceMm
                            actualMeanMm = (np.mean(cvFrame) / 65535.0) * maxDistanceMm
                            actualCenterMm = (center_mean / 65535.0) * maxDistanceMm
                            
                            print(f"Frame {timestamp}: Raw[Min: {np.min(cvFrame)}, Max: {np.max(cvFrame)}, Mean: {np.mean(cvFrame):.1f}] -> Actual[Min: {actualMinMm:.1f}mm, Max: {actualMaxMm:.1f}mm, Mean: {actualMeanMm:.1f}mm, Center: {actualCenterMm:.1f}mm], Phase Level: {phaseUnwrappingLevel}")
                            visualized_depth = colorizeToFDepth(cvFrame, phaseUnwrappingLevel)
                            visualize_frame(name, visualized_depth, timestamp, mxid)
                        else:
                            visualize_frame(name, cvFrame, timestamp, mxid)
                    elif not show_streams and name == 'left':
                        visualize_frame_info(name, cvFrame, timestamp, mxid, streams, save)
            else:
                for name in q.keys():
                    frame = q[name].get()
                    cvFrame = frame.getCvFrame()
                    timestamp = int(frame.getTimestamp().total_seconds() * 1000)
                    if save:
                        if name in ['left', 'right']:
                            if len(cvFrame.shape) == 3:
                                cvFrame = cv2.cvtColor(cvFrame, cv2.COLOR_BGR2GRAY)
                        np.save(f'{output_folders[mxid]}/{name}_{timestamp}.npy', cvFrame)
                        num_captures[mxid] += 1
                    if show_streams:
                        # Special visualization for ToF depth
                        if name == 'tof_depth':
                            phaseUnwrappingLevel = settings.get("tofConfig", {}).get("phaseUnwrappingLevel", 4)
                            # Debug: print depth statistics for each frame
                            h, w = cvFrame.shape[:2]
                            center_h, center_w = h // 2, w // 2
                            center_region = cvFrame[center_h-5:center_h+5, center_w-5:center_w+5]
                            center_mean = np.mean(center_region) if center_region.size > 0 else 0
                            
                            # Convert to actual distances for better understanding
                            maxDistanceMm = (phaseUnwrappingLevel + 1) * 1498
                            actualMinMm = (np.min(cvFrame) / 65535.0) * maxDistanceMm
                            actualMaxMm = (np.max(cvFrame) / 65535.0) * maxDistanceMm
                            actualMeanMm = (np.mean(cvFrame) / 65535.0) * maxDistanceMm
                            actualCenterMm = (center_mean / 65535.0) * maxDistanceMm
                            
                            print(f"Frame {timestamp}: Raw[Min: {np.min(cvFrame)}, Max: {np.max(cvFrame)}, Mean: {np.mean(cvFrame):.1f}] -> Actual[Min: {actualMinMm:.1f}mm, Max: {actualMaxMm:.1f}mm, Mean: {actualMeanMm:.1f}mm, Center: {actualCenterMm:.1f}mm], Phase Level: {phaseUnwrappingLevel}")
                            visualized_depth = colorizeToFDepth(cvFrame, phaseUnwrappingLevel)
                            visualize_frame(name, visualized_depth, timestamp, mxid)
                        else:
                            visualize_frame(name, cvFrame, timestamp, mxid)
                    elif not show_streams and name == 'left':
                        visualize_frame_info(name, cvFrame, timestamp, mxid, streams, save)

            key = cv2.waitKey(1)
            if key == ord('q'):
                pipeline.stop()
                break
            elif key == ord("s"):
                save = not save
                if save:
                    out_dir = initialize_capture(root_path, device, settings_path, view_name)
                    output_folders[mxid] = out_dir
                    save = True
                    print("Starting capture")
                    start_time = time.time()
                else:
                    end_time = time.time()
                    print(mxid, end=' ')
                    finalise_capture(start_time, end_time, num_captures[mxid], streams)
                    capture_ended, save = True, False
                    pipeline.getDefaultDevice().setIrLaserDotProjectorIntensity(0)
                    pipeline.stop()

            now = time.time()
            if (not wait_end and num_captures[mxid] >= final_num_captures) or (wait_end and now >= wait_end.timestamp()):
                end_time = time.time()
                print(mxid, end=' ')
                finalise_capture(start_time, end_time, num_captures[mxid], streams)
                capture_ended, save = True, False
                pipeline.getDefaultDevice().setIrLaserDotProjectorIntensity(0)
                pipeline.stop()

if __name__ == "__main__":
    args = parseArguments(root_path)
    main(args)
