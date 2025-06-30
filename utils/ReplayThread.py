import threading
import queue
import numpy as np
import tempfile
import open3d as o3d
import cv2

from utils.capture_tools import process_pointcloud

from depth.replay_depth import Replay
from depth.stereo_config import StereoConfig

class ReplayRequest:
    def __init__(self, left: np.ndarray, right: np.ndarray, calibration, config: dict, section: int, parent_frame = None, color: np.ndarray = None):
        self.left = left
        self.right = right
        self.color = color
        self.calib = calibration
        self.config = config
        self.section = section
        self.parent_frame = parent_frame

        self.replayer = None
        self.replay_outputs = None

class ReplayThread(threading.Thread):
    def __init__(self, input_queue: queue.Queue, output_queue: queue.Queue, replay_outputs, device_info):
        super().__init__(daemon=True)
        self.input_queue = input_queue
        self.output_queue = output_queue
        self.running = True
        self.replay_outputs = replay_outputs

        self.initialize_replayer(device_info)

    def run(self):
        if not self.replayer:
            raise ValueError("Replayer not initialized")
        while self.running:
            try:
                frame_data = self.input_queue.get(timeout=1)
                self.process_frame(frame_data)
                self.input_queue.task_done()
            except queue.Empty:
                continue

    def initialize_replayer(self, device_info):
        self.replayer = Replay(
            device_info=device_info,
            outputs=self.replay_outputs,
            stereo_config=StereoConfig({
                'stereo.setDepthAlign': 'dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT'
            })
        )

    def process_image(self, img):
        MAX_WIDTH = 1280
        if len(img.shape) == 3 and img.shape[2] == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img.shape[1] > MAX_WIDTH:
            scale = MAX_WIDTH / img.shape[1]
            img = cv2.resize(img, (MAX_WIDTH, int(img.shape[0] * scale)), interpolation=cv2.INTER_AREA)
        return img

    def process_frame(self, frame_data):
        left = frame_data.left
        right = frame_data.right
        color = frame_data.color
        calib = frame_data.calib
        config = frame_data.config
        section = frame_data.section

        left = self.process_image(left)
        right = self.process_image(right)

        replayed = tuple(self.replayer.replay(
            ((left, right, color), (left, right, color)),
            calib=calib,
            stereo_config=StereoConfig(config)
        ))
        depth = replayed[1]['depth']

        last_generated_pcl_path = None
        if 'pcl' in self.replay_outputs:
            pcl = replayed[1]['pcl']
            aligned_to_rgb = config['stereo.setDepthAlign'] == "dai.CameraBoardSocket.CAM_A"
            pcl = process_pointcloud(pcl, depth, color, aligned_to_rgb)
            with tempfile.NamedTemporaryFile(delete=False, suffix='.ply') as tmp_file:
                o3d.io.write_point_cloud(tmp_file.name, pcl)
            last_generated_pcl_path = tmp_file.name

        self.output_queue.put((section, depth, last_generated_pcl_path))

    def stop(self):
        self.running = False