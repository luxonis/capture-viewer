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
import zmq


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
    parser.add_argument("--autostart_end", default=0, help="Select a fixed time for capture to end")
    parser.add_argument("--show_streams", default=False, help="Show all the running streams. If false, only shows the left frame")
    parser.add_argument("--alternating_capture", default=False, help="Turn ON/OFF IR projector")
    parser.add_argument("--port", type=int, default=5555, help="ZMQ control port for this device")

    return parser.parse_args()

def process_argument_logic(args):
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

    today = datetime.date.today()

    if args.autostart_time:
        wait = datetime.datetime.combine(today, datetime.time.fromisoformat(args.autostart_time))
    else:
        wait = 0

    if args.autostart_end:
        wait_end = datetime.datetime.combine(today, datetime.time.fromisoformat(args.autostart_end))
    else:
        wait_end = 0

    if args.autostart_time: args.autostart = 0

    return settings_path, view_name, ip, args.autostart, wait, wait_end, args.show_streams, args.alternating_capture

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

def save_frame(cvFrame, projector_on):
    if name in ['left', 'right']:
        if len(cvFrame.shape) == 3:
            cvFrame = cv2.cvtColor(cvFrame, cv2.COLOR_BGR2GRAY)
    np.save(f'{output_folders[mxid][projector_on]}/{name}_{timestamp}.npy', cvFrame)

if __name__ == "__main__":
    args = parseArguments()
    settings_path, view_name, ip, autostart, autostart_time, wait_end, show_streams, alternating = process_argument_logic(args)

    # ZMQ setup
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind(f"tcp://*:{args.port}")
    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    print("Starting CONTROLLED CAPTURE script with live streaming")
    print(f"Connecting to device... IP: {ip}")

    if ip is not None:
        device = dai.Device(ip)
    else:
        device = dai.Device()

    mxid = device.getDeviceId()
    device_name = device.getDeviceName()
    print(f"Device connected! Device Name: {device_name} | MXID: {mxid}")

    with open(settings_path) as settings_file:
        settings = json.load(settings_file)

    output_folders = {}
    output_folders[mxid] = {}
    num_captures = {mxid: 0}
    frame_num = 0
    projector_on = False

    save = True
    capture_ended = False

    streams = count_output_streams(settings['output_settings'])
    print(f"Streams: {streams}")
    print(f"Number of streams: {len(streams)}")

    with dai.Pipeline(device) as pipeline:
        pipeline, q, input_queues = initialize_pipeline(pipeline, settings)
        pipeline.start()

        platform = pipeline.getDefaultDevice().getPlatform()
        if platform == dai.Platform.RVC4:
            control = initialize_mono_control(settings)
            controlQueueSend(input_queues, control)

        # IR & Floodlight setup
        if settings['ir']:
            device.setIrLaserDotProjectorIntensity(0)
        if settings['flood_light']:
            device.setIrFloodLightIntensity(settings['flood_light_intensity'])

        # Initialize output folders
        out_dir_on = initialize_capture(root_path, device, settings_path, view_name, projector=True)
        out_dir_off = initialize_capture(root_path, device, settings_path, view_name, projector=False)
        output_folders[mxid][True] = out_dir_on
        output_folders[mxid][False] = out_dir_off

        print(f"Listening for commands on port {args.port}...")
        status = "ready"

        try:
            while pipeline.isRunning():
                # --- Frame streaming ---
                for name in q.keys():
                    if q[name].has():
                        frame = q[name].get()
                        if 'raw' in name:
                            dataRaw = frame.getData()
                            cvFrame = unpackRaw10(dataRaw, frame.getWidth(), frame.getHeight(), frame.getStride())
                        else:
                            cvFrame = frame.getCvFrame()

                        if show_streams:
                            visualize_frame(name+" "+str(args.port), cvFrame, int(time.time() * 1000), mxid)
                        elif name == 'left':
                            visualize_frame_info(name+" "+str(args.port), cvFrame, int(time.time() * 1000), mxid, streams)

                # --- Command handling ---
                socks = dict(poller.poll(timeout=1))
                if socket in socks and socks[socket] == zmq.POLLIN:
                    try:
                        msg = socket.recv_json()
                        cmd = msg.get("cmd", "")

                        if cmd == "projector_on":
                            device.setIrLaserDotProjectorIntensity(settings['ir_value'])
                            projector_on = True
                            print("Projector ON")
                            socket.send_json({"status": "ok"})
                            status = "projector on"

                        elif cmd == "projector_off":
                            device.setIrLaserDotProjectorIntensity(0)
                            projector_on = False
                            print("Projector OFF")
                            socket.send_json({"status": "ok"})
                            status = "projector off"


                        elif cmd == "capture_frame":
                            status = "capturing"
                            timestamp = int(time.time() * 1000)
                            captured = 0
                            timeout_ms = 500  # max wait for frame availability
                            if settings["output_settings"]["sync"]:
                                start_wait = time.time()
                                while not q['sync'].has():
                                    if (time.time() - start_wait) > (timeout_ms / 1000):
                                        break
                                if q['sync'].has():
                                    msgGrp = q['sync'].get()
                                    for name, msg in msgGrp:
                                        if 'raw' in name:
                                            dataRaw = msg.getData()
                                            cvFrame = unpackRaw10(dataRaw, msg.getWidth(), msg.getHeight(), msg.getStride())
                                        else:
                                            cvFrame = msg.getCvFrame()
                                        save_frame(cvFrame, projector_on)
                                        captured += 1
                                else:
                                    print("Timeout waiting for sync queue.")
                            else:
                                for name, queue in q.items():
                                    start_wait = time.time()
                                    while not queue.has():
                                        if (time.time() - start_wait) > (timeout_ms / 1000):
                                            print(f"Timeout waiting for frame: {name}")
                                            break
                                    if queue.has():
                                        frame = queue.get()
                                        if 'raw' in name:
                                            dataRaw = frame.getData()
                                            cvFrame = unpackRaw10(dataRaw, frame.getWidth(), frame.getHeight(),
                                                                  frame.getStride())
                                        else:
                                            cvFrame = frame.getCvFrame()
                                        save_frame(cvFrame, projector_on)
                                        captured += 1
                            num_captures[mxid] += captured
                            print(f"Captured {captured} frames")
                            socket.send_json({"status": "ok", "frames": captured})
                            status = "ready"

                        elif cmd == "status":
                            socket.send_json({"status": status})


                        elif cmd == "exit":
                            print("Exit received.")
                            socket.send_json({"status": "shutting_down"})
                            break

                        else:
                            socket.send_json({"status": "unknown_command", "cmd": cmd})

                    except Exception as e:
                        print("Exception during command handling:", e)
                        socket.send_json({"status": "error", "detail": str(e)})

                key = cv2.waitKey(1)
                if key == ord('q'):
                    print("Quit pressed. Exiting.")
                    socket.send_json({"status": "Quit"})
                    break

            # Finalize
            end_time = time.time()
            finalise_capture(end_time - 5, end_time, num_captures[mxid], streams)
            pipeline.stop()
            print("Pipeline stopped.")

        except KeyboardInterrupt:
            status = "interrupted"
            print("KeyboardInterrupt: interrupted by user")
            try:
                socket.send_json({"status": "interrupted"})
            except:
                pass