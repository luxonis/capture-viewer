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

from utils.raw_data_utils import unpackRaw10
from utils.show_frames import visualize_frame, visualize_frame_info
from utils.capture_universal import initialize_capture, finalise_capture, count_output_streams
from utils.parse_arguments import parseArguments, process_argument_logic
from utils.isp_control import initialize_mono_control, controlQueueSend

from pipelines.dai3_stereo_pipeline import initialize_pipeline

def save_frame(cvFrame, projector_on):
    if name in ['left', 'right']:
        if len(cvFrame.shape) == 3:
            cvFrame = cv2.cvtColor(cvFrame, cv2.COLOR_BGR2GRAY)
    np.save(f'{output_folders[mxid][projector_on]}/{name}_{timestamp}.npy', cvFrame)

def main(args):
    settings_path, view_name, ip, autostart, autostart_time, wait_end, show_streams, alternating = process_argument_logic(args)

    print("Starting ALTERNATING CAPTURE script")
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
    output_folders[mxid] = {}
    mxids = [mxid]
    num_captures = {mxid: 0}

    frame_num = 0
    projector_on = settings['ir_value']

    save = False
    capture_ended = False

    if alternating:
        settings['num_captures'] *= 2

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

        platform = pipeline.getDefaultDevice().getPlatform()
        print(platform)
        if platform == dai.Platform.RVC4:
            control = initialize_mono_control(settings)
            controlQueueSend(input_queues, control)

        if settings['ir']: pipeline.getDefaultDevice().setIrLaserDotProjectorIntensity(settings['ir_value'])
        if settings['flood_light']: pipeline.getDefaultDevice().setIrFloodLightIntensity(settings['flood_light_intensity'])

        print("Starting loop...")
        while pipeline.isRunning():
            if not save and autostart > -1 and time.time() > initialize_capture_time:
                out_dir = initialize_capture(root_path, device, settings_path, view_name, projector=True)
                output_folders[mxid][True] = out_dir

                out_dir2 = initialize_capture(root_path, device, settings_path, view_name, projector=False)
                output_folders[mxid][False] = out_dir2

                save = True
                print("Starting capture via autosave")
                start_time = time.time()

            # turning projector on/off logic
            if int(frame_num/len(streams)) % (settings["FPS"] * 4) == 0:
                turn_on, turn_off = True, False
            elif int(frame_num/len(streams)) % (settings["FPS"] * 2) == 0:
                turn_on, turn_off = False, True
            else: turn_on, turn_off = False, False

            if not alternating: capture_frame = True
            else:
                if turn_on or turn_off: capture_frame = True
                else: capture_frame = False

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

                    if save and capture_frame:
                        save_frame(cvFrame, projector_on)
                        num_captures[mxid] += 1
                    if show_streams:
                        visualize_frame(name, cvFrame, timestamp, mxid)
                    elif not show_streams and name == 'left':
                        visualize_frame_info(name, cvFrame, timestamp, mxid, streams, save)

                    frame_num += 1
            else:
                for name in q.keys():
                    frame = q[name].get()
                    if 'raw' in name:
                        dataRaw = frame.getData()
                        cvFrame = unpackRaw10(dataRaw, frame.getWidth(), frame.getHeight(), frame.getStride())
                    else: cvFrame = frame.getCvFrame()
                    timestamp = int(frame.getTimestamp().total_seconds() * 1000)
                    if save and capture_frame:
                        save_frame(cvFrame, projector_on)
                        num_captures[mxid] += 1
                    if show_streams:
                        visualize_frame(name, cvFrame, timestamp, mxid)
                    elif not show_streams and name == 'left':
                        visualize_frame_info(name, cvFrame, timestamp, mxid, streams, save)
                    frame_num += 1

            if alternating:
                if turn_on:
                    pipeline.getDefaultDevice().setIrLaserDotProjectorIntensity(settings['ir_value'])
                    projector_on = True
                    print("Projector ON")
                elif turn_off:
                    pipeline.getDefaultDevice().setIrLaserDotProjectorIntensity(0)
                    projector_on = False
                    print("Projector OFF")
                else: pass



            key = cv2.waitKey(1)
            if key == ord('q'):
                pipeline.stop()
                break
            elif key == ord("s"):
                save = not save
                if save:
                    out_dir = initialize_capture(root_path, device, settings_path, view_name, projector=True)
                    output_folders[mxid][True] = out_dir

                    out_dir2 = initialize_capture(root_path, device, settings_path, view_name, projector=False)
                    output_folders[mxid][False] = out_dir2

                    save = True
                    print("Starting capture")
                    start_time = time.time()
                else:
                    end_time = time.time()
                    print(mxid, end=' ')
                    finalise_capture(start_time, end_time, num_captures[mxid], streams)
                    capture_ended, save = True, False
                    pipeline.stop()

            now = time.time()
            if (not wait_end and num_captures[mxid] >= final_num_captures) or (wait_end and now >= wait_end.timestamp()):
                end_time = time.time()
                print(mxid, end=' ')
                finalise_capture(start_time, end_time, num_captures[mxid], streams)
                capture_ended, save = True, False
                pipeline.stop()

if __name__ == "__main__":
    args = parseArguments(root_path)
    main(args)