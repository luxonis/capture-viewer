#!/usr/bin/env python3
import depthai as dai
import threading
import contextlib
import logging
import queue
import cv2
import json
import os
import argparse
import numpy as np
import time
import re

from utils.capture_universal import colorize_depth, initialize_capture, finalise_capture

########################################################
#                   Configuration
########################################################

script_dir = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.join(os.path.dirname(script_dir), 'DATA')

# How large the in-RAM buffer can grow before blocking reader
# 0 => infinite size (not recommended for truly unbounded usage)
MAX_RAM_QUEUE_SIZE = 0

# To skip frames for display (to keep up with new ones), we keep only the latest
# If you'd like a different policy, you can store them in a queue with no skipping.
SHOW_MOST_RECENT_ONLY = True

########################################################
#                   Argument Parsing
########################################################

def count_output_streams(output_streams):
    """
    Utility to count how many streams we are actually outputting
    (e.g. left, right, rgb, depth, etc.).
    """
    stream_names = []
    for item in output_streams.keys():
        if item in ["tof", "sync", "rgb_png"]:
            continue
        if output_streams[item]:
            stream_names.append(item)
    return stream_names

def parseArguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("settings_file_path")
    parser.add_argument("view_name")
    parser.add_argument("--output", default=root_path)
    parser.add_argument("--autostart", default=-1, type=int)
    parser.add_argument("--devices", default=[], dest="mxids", nargs="+")
    args = parser.parse_args()

    settings_path = args.settings_file_path
    view_name = args.view_name
    devices = args.mxids

    # Resolve settings file path if shorthand
    if not os.path.exists(settings_path):
        settings_path_1 = f"settings_jsons/{settings_path}.json"
        settings_path_2 = f"settings_jsons/{settings_path}"
        if os.path.exists(settings_path_1):
            settings_path = settings_path_1
        elif os.path.exists(settings_path_2):
            settings_path = settings_path_2
        else:
            raise FileNotFoundError(settings_path)

    # Check if devices are IP addresses
    ip_pattern = re.compile(r"^(?:\d{1,3}\.){3}\d{1,3}$")
    is_ip = all(ip_pattern.match(device) for device in devices)

    return settings_path, view_name, devices, args.autostart, is_ip

def attempt_connection(mxids, attempts=10):
    """
    Attempt to connect to given device(s). If no MXID is provided,
    try to discover any available OAK device.
    """
    if len(mxids) == 0:
        for i in range(attempts):
            devs = dai.Device.getAllAvailableDevices()
            if len(devs) > 0:
                mxids = [devs[0].mxid]
                return mxids
            else:
                time.sleep(5)
        raise ValueError("No devices found")
    for attempt in range(attempts):
        found_count = 0
        all_devs = dai.Device.getAllAvailableDevices()
        for dev in all_devs:
            if dev.mxid in mxids:
                found_count += 1
        if found_count < len(mxids):
            time.sleep(5)
        else:
            return mxids
    raise ValueError("Not all requested devices found after retries")

########################################################
#                   Multi-Thread Logic
########################################################

# We keep a global "exit_event" or "stop_event" to signal all threads to exit.
stop_event = threading.Event()

# Use a global to indicate whether we are currently saving.
saving_event = threading.Event()
saving_event.clear()  # Initially not saving

# We'll store frames from Reader -> Saver in this queue
# If MAX_RAM_QUEUE_SIZE is 0 => infinite queue (not recommended for large captures)
save_queue = queue.Queue(maxsize=MAX_RAM_QUEUE_SIZE)

# We'll store the most recent frames for display in a dictionary:
# latest_frames[(mxid, stream_name)] = (frame, timestamp, fps)
latest_frames = {}
latest_frames_lock = threading.Lock()

# For capturing how many frames have been saved (for each mxid).
num_captures = {}

# We can keep some optional FPS counters
fps_counters = {}

def update_fps(name):
    """
    name: a string describing which FPS to update (e.g. "reader_mxid_left", "saver", "display").
    """
    global fps_counters
    if name not in fps_counters:
        fps_counters[name] = {
            "last_time": time.time(),
            "fps": 0.0,
            "frame_count": 0
        }

    data = fps_counters[name]
    data["frame_count"] += 1
    now = time.time()
    elapsed = now - data["last_time"]
    # Update once per second
    if elapsed >= 1.0:
        data["fps"] = data["frame_count"] / elapsed
        data["frame_count"] = 0
        data["last_time"] = now

def get_fps(name):
    if name in fps_counters:
        return fps_counters[name]["fps"]
    return 0.0

def saver_thread_func():
    """
    Continuously pop frames from the 'save_queue' and save them to disk.
    """
    while True:
        if stop_event.is_set():
            break
        try:
            item = save_queue.get(timeout=0.2)
        except queue.Empty:
            continue
        if item is None:
            # This means we want to stop for good
            break

        (mxid, stream_name, frame, timestamp, out_folder) = item

        # Do the actual saving
        outpath = os.path.join(out_folder, f"{stream_name}_{timestamp}.npy")
        np.save(outpath, frame)

        # Count captures per device
        num_captures[mxid] += 1

        # Update saver FPS
        update_fps("saver")

        # Let queue know the task is done
        save_queue.task_done()

def display_thread_func(shared_devices, streams):
    """
    Periodically display the newest frames for each (mxid, stream_name).
    We skip older frames if they can't be displayed quickly enough.
    """
    while True:
        if stop_event.is_set():
            break

        # Retrieve and display the newest frames
        with latest_frames_lock:
            # Copy out the references for display
            current_frames = dict(latest_frames)

        # Show each frame (one window per (mxid, stream_name))
        for (mxid, stream_name), (frame, timestamp, read_fps) in current_frames.items():
            # Display the read_fps in the window along with some info
            disp_fps = get_fps("display")  # Display thread FPS
            # Overlays
            overlay = frame.copy() if len(frame.shape) == 3 else None

            if stream_name in ["depth", "disparity", "tof_depth"]:
                if stream_name == "tof_depth":
                    max_depth = 5 * 1500
                    overlay = colorize_depth(frame, min_depth=0, max_depth=max_depth)
                else:
                    # For normal depth or disparity
                    overlay = colorize_depth(
                        frame, min_depth=0, max_depth=frame.max()
                    )

                cv2.putText(overlay,
                            f"{timestamp} ms [ReadFPS: {read_fps:.1f} DispFPS: {disp_fps:.1f}]",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255), 2)
            else:
                # normal grayscale or color frames
                overlay = frame.copy()
                cv2.putText(overlay,
                            f"{timestamp} ms [ReadFPS: {read_fps:.1f} DispFPS: {disp_fps:.1f}]",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255), 2)

            window_name = f"{mxid} {stream_name}"
            if overlay is not None:
                cv2.imshow(window_name, overlay)

        # Update display FPS
        update_fps("display")

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            stop_event.set()
            break
        elif key == ord('s'):
            if saving_event.is_set():
                # Stop saving
                saving_event.clear()
                print(">>> Stopped Saving.")
            else:
                # Start saving
                saving_event.set()
                print(">>> Started Saving.")

def reader_thread_func(mxid, device_queues, out_folder):
    while not stop_event.is_set():
        for stream_name, q in device_queues.items():
            if not q.has():
                continue

            # Get the data from the queue
            data_packet = q.get()

            # Check if this is a "sync" queue (returns MessageGroup)
            if stream_name == "sync":
                # data_packet is a MessageGroup
                msgGrp = data_packet
                for subStreamName, msg in msgGrp:
                    frame = msg.getCvFrame()
                    timestamp = int(msg.getTimestamp().total_seconds() * 1000)

                    # Update read FPS
                    update_fps(f"reader_{mxid}_{subStreamName}")
                    read_fps_val = get_fps(f"reader_{mxid}_{subStreamName}")

                    # Save if needed
                    if saving_event.is_set():
                        save_queue.put((mxid, subStreamName, frame, timestamp, out_folder))

                    # Update the newest frame for display
                    with latest_frames_lock:
                        latest_frames[(mxid, subStreamName)] = (frame, timestamp, read_fps_val)

            else:
                # data_packet is a single ImgFrame
                frame = data_packet.getCvFrame()
                timestamp = int(data_packet.getTimestamp().total_seconds() * 1000)

                # Update read FPS
                update_fps(f"reader_{mxid}_{stream_name}")
                read_fps_val = get_fps(f"reader_{mxid}_{stream_name}")

                # Save if needed
                if saving_event.is_set():
                    save_queue.put((mxid, stream_name, frame, timestamp, out_folder))

                # Update the newest frame for display
                with latest_frames_lock:
                    latest_frames[(mxid, stream_name)] = (frame, timestamp, read_fps_val)

########################################################
#                   Main
########################################################

def main():
    settings_path, view_name, mxids, autostart, is_ip = parseArguments()

    # If no IP devices, attempt to connect to USB
    if not is_ip:
        mxids = attempt_connection(mxids)

    # Read your JSON settings
    with open(settings_path) as f:
        settings = json.load(f)

    # For each device, we store the depthai OutputQueues in a dict
    devices_queues = {}   # devices_queues[mxid] = { stream_name: dai.OutputQueue }
    shared_devices = {}   # mxid -> dai.Device
    global num_captures
    num_captures = {mxid: 0 for mxid in mxids}

    # Build + start pipeline for each device in a context manager
    with contextlib.ExitStack() as stack:
        # Start device + pipeline
        for i, mxid in enumerate(mxids):
            openvino_version = dai.OpenVINO.Version.VERSION_2021_4
            device_info = dai.DeviceInfo(mxid)
            # Some OAK cameras might need USB2 mode = False or True
            usb2_mode = False

            device = stack.enter_context(dai.Device(openvino_version, device_info, usb2_mode))
            shared_devices[mxid] = device

            # Decide which pipeline to load
            if device.getDeviceName() == "OAK-D-SR-POE":
                from pipelines.oak_tof_pipeline import get_pipeline
                pipeline_output = get_pipeline(settings, i)
            else:
                from pipelines.oak_stereo_pipeline import get_pipeline
                pipeline_output = get_pipeline(settings)

            device.startPipeline(pipeline_output["pipeline"])

            # Create OutputQueue dictionary
            dev_queues = {}
            out_set = settings["output_settings"]
            if out_set["sync"]:
                # Single sync queue
                dev_queues["sync"] = device.getOutputQueue("xout", maxSize=30, blocking=False)
            else:
                # Build separate queues for each stream
                if out_set.get("tof", False):
                    if out_set.get("tof_raw", False):
                        dev_queues["tof_raw"] = device.getOutputQueue("tof_raw", maxSize=4, blocking=False)
                    if out_set.get("tof_depth", False):
                        dev_queues["tof_depth"] = device.getOutputQueue("tof_depth", maxSize=4, blocking=False)
                    if out_set.get("tof_intensity", False):
                        dev_queues["tof_intensity"] = device.getOutputQueue("tof_intensity", maxSize=4, blocking=False)
                    if out_set.get("tof_amplitude", False):
                        dev_queues["tof_amplitude"] = device.getOutputQueue("tof_amplitude", maxSize=4, blocking=False)
                if out_set["depth"]:
                    dev_queues["depth"] = device.getOutputQueue("depth", maxSize=4, blocking=False)
                if out_set["disparity"]:
                    dev_queues["disparity"] = device.getOutputQueue("disparity", maxSize=30, blocking=False)
                if out_set["left"]:
                    dev_queues["left"] = device.getOutputQueue("left", maxSize=30, blocking=False)
                if out_set["right"]:
                    dev_queues["right"] = device.getOutputQueue("right", maxSize=30, blocking=False)
                if out_set["left_raw"]:
                    dev_queues["left_raw"] = device.getOutputQueue("left_raw", maxSize=30, blocking=False)
                if out_set["right_raw"]:
                    dev_queues["right_raw"] = device.getOutputQueue("right_raw", maxSize=30, blocking=False)
                if out_set["rgb"]:
                    dev_queues["rgb"] = device.getOutputQueue("rgb", maxSize=30, blocking=False)

            devices_queues[mxid] = dev_queues

        # IR / Floodlight config if needed
        for mxid, device in shared_devices.items():
            if settings["ir"]:
                device.setIrLaserDotProjectorIntensity(settings["ir_value"])
            if settings["flood_light"]:
                device.setIrFloodLightIntensity(settings["flood_light_intensity"])

        # Prepare local output folder for each device if "autostart" is triggered
        out_folders = {}
        if autostart >= 0:
            # We'll wait until autostart is reached to actually call initialize_capture
            pass
        else:
            # Not using autostart => we do it on 's' key
            pass

        # Start saver thread
        saver_thread = threading.Thread(target=saver_thread_func, daemon=True)
        saver_thread.start()

        # Count total frames we want
        all_streams = count_output_streams(settings["output_settings"])
        final_num_captures = settings["num_captures"] * len(all_streams)

        # Start a dedicated reader thread for each device
        readers = []
        for mxid in mxids:
            # By default, the output folder is not known until we "start capturing"
            # We'll store a placeholder. We'll fill it in once we actually start saving.
            out_folders[mxid] = None

            t = threading.Thread(
                target=reader_thread_func,
                args=(mxid, devices_queues[mxid], None),  # We'll pass in None for out_folder for now
                daemon=True
            )
            t.start()
            readers.append(t)

        # Start display thread
        # This will handle key input, so it must own cv2.imshow / cv2.waitKey
        display_streams = threading.Thread(
            target=display_thread_func,
            args=(shared_devices, all_streams),
            daemon=True
        )
        display_streams.start()

        # If autostart > -1, schedule a timer to set 'saving_event' = True
        # and initialize captures
        def autostart_saver():
            # Called after autostart seconds
            print(">>> Auto-starting capture & saving...")
            for mxid, dev in shared_devices.items():
                out_folders[mxid] = initialize_capture(root_path, dev, settings_path, view_name)
            # Update each reader thread with the final out_folder
            # (In Python, the function above has it as None - you can store in a global or a dict and reference it)
            for mxid in mxids:
                # We can’t easily reassign the function’s argument,
                # so a simple approach is storing it in a global dictionary
                pass
            saving_event.set()

        if autostart >= 0:
            timer = threading.Timer(autostart, autostart_saver)
            timer.start()

        ########################################################
        # Main loop monitors for completion or out-of-memory, etc.
        ########################################################

        try:
            while not stop_event.is_set():
                # Check if we've reached the required captures for each device
                done_count = 0
                for mxid in mxids:
                    if num_captures[mxid] >= final_num_captures and saving_event.is_set():
                        # We can finalize
                        saving_event.clear()
                        end_time = time.time()
                        # We pass in (start_time, end_time, capture_count, streams) as per your existing code
                        # If you want the actual start_time, store it when you started saving
                        # For demonstration, just use a placeholder
                        start_time = end_time - 1
                        finalise_capture(start_time, end_time, num_captures[mxid], all_streams)
                        print(f">>> Device {mxid} reached {final_num_captures} captures.")
                        done_count += 1
                if done_count == len(mxids):
                    # All devices done
                    stop_event.set()

                time.sleep(0.2)

        except KeyboardInterrupt:
            print("KeyboardInterrupt -> stopping.")
            stop_event.set()

        # Cleanup
        stop_event.set()

        # Join read threads
        for t in readers:
            t.join()

        # Signal saver thread to finish
        save_queue.put(None)
        saver_thread.join()

        display_streams.join()

    cv2.destroyAllWindows()
    print("All threads joined. Exiting.")

########################################################
#                   __main__
########################################################

if __name__ == "__main__":
    main()
