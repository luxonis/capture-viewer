import depthai as dai
import numpy as np
import json
import time
import cv2
import os

from utils.capture_universal import initialize_capture
from utils.parse_arguments import parseArguments

# Get the directory where the script is located and choose it as the destination for DATA folder
script_dir = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.join(os.path.dirname(script_dir), 'DATA')

def get_pipeline(settings):
    # todo configurable output streams
    # todo set resolution
    # todo se FPS
    queues = {}
    pipeline_output = {}
    pipeline = dai.Pipeline()

    cam_thermal = pipeline.createCamera()
    cam_thermal.setBoardSocket(dai.CameraBoardSocket.CAM_E)

    cam_rgb = pipeline.createColorCamera()
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

    # create output
    video_queue = pipeline.create(dai.node.XLinkOut)
    video_queue.setStreamName('thermal_video')

    thermal_queue = pipeline.create(dai.node.XLinkOut)
    thermal_queue.setStreamName('thermal_raw')

    rgb_queue = pipeline.create(dai.node.XLinkOut)
    rgb_queue.setStreamName('rgb')

    # Link camera nodes to output queues
    cam_thermal.video.link(video_queue.input)
    cam_thermal.raw.link(thermal_queue.input)

    cam_rgb.isp.link(rgb_queue.input)

    queues['thermal_video'] = video_queue
    queues['thermal_raw'] = thermal_queue
    queues['rgb'] = rgb_queue

    pipeline_output["pipeline"] = pipeline
    pipeline_output["queues"] = queues

    return pipeline_output


if __name__ == "__main__":
    print("Running OAK capture script")
    settings_path, view_name, device_info, autostart = parseArguments()

    with open(settings_path, 'r') as file:
        settings = json.load(file)

    print("\nUSAGE:")
    print("Press S to start capture")
    print("Press Q to quit")
    print(f"The capture will finish automatically after {settings['num_captures']} captures. This number may be specified in the selected settings json.")
    print("Or press S again to end the capture early.")

    print("\nConnecting device...")
    pipeline_output = get_pipeline(settings)
    with (dai.Device(pipeline_output["pipeline"], device_info) as device):
        print("Device Connected!")
        out_dir = None

        queues = {}
        for queue in pipeline_output["queues"].keys():
            queues[queue] = device.getOutputQueue(name=queue, maxSize=4, blocking=False)

        save = False
        num_captures = 0

        start_time = time.time()
        while True:
            if autostart >= -1:
                autostart -= 1
            if autostart == -1:
                out_dir = initialize_capture(root_path, device, settings_path, view_name)
                save = True
                print("Starting capture via autosave")
                start_time = time.time()

            for name, q in queues.items():
                msg = q.get()
                timestamp = msg.getTimestamp().total_seconds() * 1000

                if name == "thermal_raw":
                    frame = msg.getCvFrame().astype(np.float32)
                    thermal_frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                    thermal_frame = cv2.applyColorMap(thermal_frame, cv2.COLORMAP_MAGMA)
                    cv2.imshow(name, thermal_frame)
                elif name == "thermal_video":
                    frame = msg.getCvFrame().astype(np.float32)
                    thermal_frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                    thermal_frame = cv2.applyColorMap(thermal_frame, cv2.COLORMAP_MAGMA)
                    cv2.imshow(name, thermal_frame)
                else:
                    frame = msg.getCvFrame()
                    cv2.imshow(name, frame)

                if save:
                    if name == "termal_raw":
                        data = msg.getData()
                        np.save(f'{out_dir}/{name}_{timestamp}', data)
                    elif name == 'thermal_video':
                        np.save(f'{out_dir}/{name}_{timestamp}', frame)
                        pass
                    elif name == 'rgb':
                        if settings["output_settings"]["rgb"]: np.save(f'{out_dir}/{name}_{timestamp}', frame)
                        if settings["output_settings"]["rgb_png"]: cv2.imwrite(f'{out_dir}/{name}_{timestamp}.png', frame)
                        num_captures += 1
                    else:
                        print(name)


            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            elif key == ord("s"):
                save = not save

                if save:
                    print("CAPTURING...")
                else:
                    print(f"capture finished with: {num_captures} captures")
                    exit(0)

                out_dir = initialize_capture(root_path, device, settings_path, view_name)
                start_time = time.time()

            if num_captures == settings["num_captures"]:
                end_time = time.time()
                print("Capture took " + str(end_time - start_time) + " seconds.")
                save = False
                print(f"CAPTURE FINISHED with: {num_captures} captures")
                print(f"Capture was {round(num_captures / (end_time - start_time), 2)} FPS")
                exit(0)
