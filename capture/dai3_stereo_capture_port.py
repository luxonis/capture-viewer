#!/usr/bin/env python3

import depthai as dai
import numpy as np
import time
import json
import cv2
import zmq
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

def cleanup_empty_folders(folder_list):
    whitelist = {"metadata.json", "calib.json"}
    for folder in folder_list:
        if not os.path.isdir(folder):
            continue
        contents = set(os.listdir(folder))
        print(contents)
        if contents.issubset(whitelist) or len(contents) == 0:
            print(f"[Cleanup] Removing unused folder: {folder}")
            try:
                for f in contents:
                    os.remove(os.path.join(folder, f))
                os.rmdir(folder)
            except Exception as e:
                print(f"[Cleanup Error] Could not remove {folder}: {e}")


if __name__ == "__main__":
    args = parseArguments(root_path)
    settings_path, view_name, ip, autostart, autostart_time, wait_end, show_streams, alternating = process_argument_logic(args)

    root_path = args.output

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
                            visualize_frame(device_name + " " + name+" "+str(args.port), cvFrame, int(time.time() * 1000), mxid)
                        elif name == 'left':
                            visualize_frame_info(device_name + " " + name+" "+str(args.port), cvFrame, int(time.time() * 1000), mxid, streams, None)

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

                        elif cmd == "status":
                            socket.send_json({"status": status})

                        elif cmd == "cleanup":
                            cleanup_empty_folders([...])
                            socket.send_json({"status": "cleaned"})

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

        except KeyboardInterrupt:
            status = "interrupted"
            print("KeyboardInterrupt: interrupted by user")
            try:
                socket.send_json({"status": "interrupted"})
            except:
                pass

        finally:
            # Finalize
            end_time = time.time()
            finalise_capture(end_time - 5, end_time, num_captures[mxid], streams)
            pipeline.stop()
            print("Pipeline stopped.")
            cleanup_empty_folders([
                output_folders[mxid][True],
                output_folders[mxid][False]
            ])