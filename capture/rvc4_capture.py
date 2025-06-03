#!/usr/bin/env python3
import queue

import cv2
import os
import depthai as dai
import numpy as np
import argparse
import time
import json
import datetime

print(dai.__version__)

if str(dai.__version__)[0] != "3":
    print("Go to depthai-core branch v3-develop")
    print("and run this: depthai-core/examples/python/install_requirements.py")
    exit("U dont have depthai 3")

script_dir = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.join(os.path.dirname(script_dir), 'DATA')

from utils.capture_universal import initialize_capture, finalise_capture
from utils.raw_data_utils import unpackRaw10
from oak_capture import visualize_frame, visualize_frame_info, count_output_streams

def set_stereo_node(pipeline, settings):
    stereo = pipeline.create(dai.node.StereoDepth)

    stereo.setRectification(True)


    stereo.setDefaultProfilePreset(eval(f"dai.node.StereoDepth.PresetMode.{settings['profilePreset']}"))

    stereo.setLeftRightCheck(settings["LRcheck"])
    if settings["extendedDisparity"]: stereo.setExtendedDisparity(True)
    else: stereo.setExtendedDisparity(False)
    # if settings["subpixelDisparity"]: stereo.setSubpixel(True)
    # if settings["subpixelValue"]: stereo.setSubpixelFractionalBits(settings["subpixelValue"])

    return stereo

def initialize_pipeline(pipeline, settings):
    def configure_cam(cam, size_x:int, size_y:int, fps:float):
        cap = dai.ImgFrameCapability()
        cap.size.fixed((size_x, size_y))

        cap.fps.fixed(fps)
        return cam.requestOutput(cap, True)
    queues = {}
    input_queues = {}
    output_settings = settings["output_settings"]

    if output_settings["left"]:
        monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoLeftOut = configure_cam(monoLeft, settings["stereoResolution"]["x"], settings["stereoResolution"]["y"], settings["FPS"])

    if output_settings["right"]:
        monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        monoRightOut = configure_cam(monoRight, settings["stereoResolution"]["x"], settings["stereoResolution"]["y"], settings["FPS"])

    if output_settings["rgb"]:
        color = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        colorOut = configure_cam(color, settings["rgbResolution"]["x"], settings["rgbResolution"]["y"], settings["FPS"])



    if output_settings["left"] or output_settings["left_raw"]: input_queues["left_input_control"] = monoLeft.inputControl.createInputQueue()
    if output_settings["right"] or output_settings["right_raw"]: input_queues["right_input_control"] = monoRight.inputControl.createInputQueue()



    if output_settings["depth"] or output_settings["disparity"]:
        stereo = set_stereo_node(pipeline, settings)
        monoLeftOut.link(stereo.left)
        monoRightOut.link(stereo.right)



    if output_settings["sync"]:
        sync = pipeline.create(dai.node.Sync)
        sync.setRunOnHost(settings['sync_on_host'])  # Can also run on device

        if output_settings["depth"]:
            stereo.syncedLeft.link(sync.inputs["left"])
            stereo.syncedRight.link(sync.inputs["right"])
            stereo.disparity.link(sync.inputs["disparity"])
            stereo.depth.link(sync.inputs["depth"])
        else:
            monoLeftOut.link(sync.inputs["left"])
            monoRightOut.link(sync.inputs["right"])

        if output_settings["rgb"]:
            colorOut.link(sync.inputs["rgb"])

        if output_settings["left_raw"]: monoLeft.raw.link(sync.inputs["left_raw"])
        if output_settings["right_raw"]: monoRight.raw.link(sync.inputs["right_raw"])

        if output_settings.get("rgb_raw", False):
            raise NotImplementedError("RGB raw frames not implemented")
            # if output_settings["rgb_raw"]: color.raw.link(sync.inputs["rgb_raw"])

        queues["sync"] = sync.out.createOutputQueue()

    elif not output_settings["sync"]:
        if output_settings["depth"]:
            syncedLeftQueue = stereo.syncedLeft.createOutputQueue()
            syncedRightQueue = stereo.syncedRight.createOutputQueue()
            disparityQueue = stereo.disparity.createOutputQueue()
            depthQueue = stereo.depth.createOutputQueue()

            queues['left'] = syncedLeftQueue
            queues['right'] = syncedRightQueue
            queues['disparity'] = disparityQueue
            queues['depth'] = depthQueue

        else:
            queues['left'] = monoLeftOut.createOutputQueue()
            queues['right'] = monoRightOut.createOutputQueue()

        if output_settings["rgb"]:
            queues["rgb"] = colorOut.createOutputQueue()

        if output_settings["left_raw"]: queues['left_raw'] = monoLeft.raw.createOutputQueue()
        if output_settings["right_raw"]: queues['right_raw'] = monoRight.raw.createOutputQueue()
        if output_settings.get("rgb_raw", False):
            # queues['rgb_raw'] = color.raw.createOutputQueue()
            raise NotImplementedError("RGB raw frames not implemented")


    return pipeline, queues, input_queues

def parseArguments():
    # PARSE ARGUMENTS
    parser = argparse.ArgumentParser()
    # Mandatory arguments
    parser.add_argument("settings_file_path", help="Path to settings JSON")
    parser.add_argument("view_name", help="Name of the capture")
    parser.add_argument("--output", default=root_path, help="Custom output folder")
    parser.add_argument("--autostart", default=-1, type=int, help='Automatically start capturing after given number of seconds (-1 to disable)')
    # parser.add_argument("--devices", default=[], dest="mxids", nargs="+", help="MXIDS of devices to connect to")
    parser.add_argument("--ip", default=None, dest="ip", help="IP to connect to")
    parser.add_argument("--autostart_time", default=0, help="Select a fixed time when the script is supposed to start")
    parser.add_argument("--show_streams", default=False, help="Show all the running streams. If false, only shows the left frame")

    args = parser.parse_args()
    settings_path = args.settings_file_path
    view_name = args.view_name
    ip = args.ip

    # SETTINGS loading
    if not os.path.exists(settings_path):
        settings_path_1 = f"settings_jsons/{settings_path}.json"
        settings_path_2 = f"settings_jsons/{settings_path}"
        if os.path.exists(settings_path_1):
            settings_path = settings_path_1
        elif os.path.exists(settings_path_2):
            settings_path = settings_path_2
        else: raise FileNotFoundError(f"Settings file '{settings_path}' does not exist.")

    if args.autostart_time:
        today = datetime.date.today()
        time_part = datetime.time.fromisoformat(args.autostart_time)
        wait = datetime.datetime.combine(today, time_part)
    else:
        wait = 0

    if args.autostart_time: args.autostart = 0

    return settings_path, view_name, ip, args.autostart, wait, args.show_streams

def controlQueueSend(input_queues, ctrl):
    for queue in input_queues.values():
        queue.send(ctrl)

def initialize_mono_control(settings):
    ctrl = dai.CameraControl()

    mono_settings = settings["monoSettings"]
    ctrl.setLumaDenoise(mono_settings["luma_denoise"])
    ctrl.setChromaDenoise(mono_settings["chroma_denoise"])
    ctrl.setSharpness(mono_settings["sharpness"])
    ctrl.setContrast(mono_settings["contrast"])

    exposure_settings = settings["exposureSettings"]
    if not exposure_settings["autoexposure"]:
        ctrl.setManualExposure(exposure_settings["expTime"], exposure_settings["sensIso"])

    return ctrl

if __name__ == "__main__":
    settings_path, view_name, ip, autostart, autostart_time, show_streams = parseArguments()

    print(f"connecting to device... IP: {ip}")

    if ip is not None: device = dai.Device(ip)
    else: device = dai.Device()
    mxid = device.getDeviceId()

    device_name = device.getDeviceName()
    print(f"Device connected! Device Name: {device_name}")

    with open(settings_path) as settings_file:
        settings = json.load(settings_file)

    # devices, shared_devices = {}, {}
    output_folders = {}
    mxids = [mxid]
    num_captures = {mxid: 0}
    streams = [None]

    save = False
    capture_ended = False

    streams = count_output_streams(settings['output_settings'])
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

        control = initialize_mono_control(settings)
        controlQueueSend(input_queues, control)

        if settings['ir']: pipeline.getDefaultDevice().setIrLaserDotProjectorIntensity(settings['ir_value'])
        if settings['flood_light']: pipeline.getDefaultDevice().setIrFloodLightIntensity(settings['flood_light_intensity'])

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

                    if 'raw' in name:
                        dataRaw = msg.getData()
                        cvFrame = unpackRaw10(dataRaw, msg.getWidth(), msg.getHeight(), msg.getStride())
                    else: cvFrame = msg.getCvFrame()

                    if save:
                        if name in ['left', 'right']:
                            if len(cvFrame.shape) == 3:
                                cvFrame = cv2.cvtColor(cvFrame, cv2.COLOR_BGR2GRAY)
                        np.save(f'{output_folders[mxid]}/{name}_{timestamp}.npy', cvFrame)
                        num_captures[mxid] += 1
                    if show_streams:
                        visualize_frame(name, cvFrame, timestamp, mxid)
                    elif not show_streams and name == 'left':
                        visualize_frame_info(name, cvFrame, timestamp, mxid, streams)
            else:
                for name in q.keys():
                    frame = q[name].get()
                    if 'raw' in name:
                        dataRaw = frame.getData()
                        cvFrame = unpackRaw10(dataRaw, frame.getWidth(), frame.getHeight(), frame.getStride())
                    else: cvFrame = frame.getCvFrame()
                    timestamp = int(frame.getTimestamp().total_seconds() * 1000)
                    if save:
                        if name in ['left', 'right']:
                            if len(cvFrame.shape) == 3:
                                cvFrame = cv2.cvtColor(cvFrame, cv2.COLOR_BGR2GRAY)
                        np.save(f'{output_folders[mxid]}/{name}_{timestamp}.npy', cvFrame)
                        num_captures[mxid] += 1
                    if show_streams:
                        visualize_frame(name, cvFrame, timestamp, mxid)
                    elif not show_streams and name == 'left':
                        visualize_frame_info(name, cvFrame, timestamp, mxid, streams)

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
                    pipeline.stop()

            if num_captures[mxid] >= final_num_captures:
                end_time = time.time()
                print(mxid, end=' ')
                finalise_capture(start_time, end_time, num_captures[mxid], streams)
                capture_ended, save = True, False
                pipeline.stop()