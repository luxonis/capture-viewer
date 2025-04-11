"""
Replay on RVC2 and RVC4 devices
"""
import datetime
import more_itertools
import json
import time
from typing import Set
import cv2
import depthai as dai
import numpy as np
import tenacity
from .timeout import timeout
from depth.stereo_config import StereoConfig

__is_dai3 = dai.__version__.startswith('3.')

@tenacity.retry(
        stop=tenacity.stop_after_attempt(7),
        wait=tenacity.wait_exponential(multiplier=2, min=4, max=20),
        retry=tenacity.retry_if_exception_type((
            RuntimeError,
            ValueError  # Device not in Gate state
        )),
        reraise=True)
def get_device(device_info=None):
    """Get dai.Device() based on optional device_info, retry 5 times and wait between each retry."""
    try:
        if device_info is None:
            return dai.Device()
        else:
            return dai.Device(device_info)
    except RuntimeError as e:
        raise RuntimeError(f'Failed to get device ({device_info}) {e}')

@timeout(8)
def _send_images(input_queues, frames, *, ts=None):
    """
    Send left, right and rgb images to appropriate pipeline input queues.

    Args:
     - input_queues: mapping from name to input queue, where name is one of 'left', 'right', or 'rgb'
     - frame: mapping from name to image, where name is one of 'left', 'right', or 'rgb'
     - ts: optional timestamp to assign to the sent data
    """
    if ts is None:
        ts = dai.Clock.now()

    for name, frame in frames.items():
        h, w, *_ = frame.shape
        number = {'rgb': dai.CameraBoardSocket.CAM_A,
                  'left': dai.CameraBoardSocket.CAM_B,
                  'right': dai.CameraBoardSocket.CAM_C}[name]
        img = dai.ImgFrame()
        img.setData(frame.flatten())
        img.setTimestamp(ts)
        img.setWidth(w)
        try:
            img.setStride(w)
        except AttributeError:
            pass
        img.setHeight(h)
        img.setInstanceNum(number)
        img.setType({
            'rgb': dai.ImgFrame.Type.RGB888i,
            'left': dai.ImgFrame.Type.RAW8,
            'right': dai.ImgFrame.Type.RAW8,
        }[name])
        input_queues[name].send(img)

def _convert(name, data):
    """Convert data from xlink to numpy based on name."""
    if name == 'pcl':
        return data.getPoints().astype(np.float64)
    if name == 'disparity_cost_dump':
        return np.array(data.getData(), dtype=np.uint8)
    return data.getCvFrame().astype(float)

@timeout(8)
def _get_data(output_queues):
    return {name: _convert(name, queue.get()) for name, queue in output_queues.items()}

def replay_depth(depth_on, *args, **kwargs):
    if depth_on == 'device':
        yield from replay(*args, outputs={'depth',}, **kwargs)
    elif depth_on == 'host':
        config = {}
        if kwargs['stereo_config']:
            config_update = json.loads(kwargs['stereo_config'])
            config.update(config_update)

        stereo = getattr(cv2, config['stereo'])
        del config['stereo']  # remove class config, so that I can pass the rest of the dictionary into the constructor of the class
        stereo = stereo.create(**config)
        baseline = kwargs['calib'].getBaselineDistance(useSpecTranslation=False) * 10  # mm

        for frame in replay(*args, outputs={'rectified_left', 'rectified_right'}, **kwargs):
            height, width = frame['rectified_right'].shape
            M2 = np.array(kwargs['calib'].getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, width, height))
            focal_length = M2[0][0]
            disparity = stereo.compute(frame['rectified_left'].astype(np.uint8), frame['rectified_right'].astype(np.uint8)).astype(float)
            with np.errstate(divide='ignore'):
                depth_map = focal_length * baseline / (disparity / 2**4)  # 4 fractional bits
            depth_map[disparity == 0] = 0
            yield {
                'disparity': disparity,
                'depth': depth_map,
            }


def replay(frames, *, calib=None, stereo_config=StereoConfig({}), outputs:Set[str]={'depth',}, device_info=None):
    replayer = Replay(calib=calib, stereo_config=stereo_config, outputs=outputs)
    yield from replayer.replay(frames, device_info=device_info)


class ReplayV2:
    def __init__(self, calib=None, stereo_config=None, outputs:Set[str]={'depth',}):
        """
        Args:
        - frames: iterable of (left, right[, rgb]) images as numpy arrays
        - calib: device calibration. If None the currently attached device' calibration is used.
        - stereo_config: device pipeline configuration
        - outputs: choose which outputs you want: 'depth', 'rectified_left', 'rectified_right', 'pcl'
        - timeout_s: raise TimeoutError if getting a single queue item from pipeline takes longer than given time in seconds
        - device_info: optional depthai.DeviceInfo to use specified device
        """
        if not outputs:
            raise ValueError('No output specified')

        self._pipeline = dai.Pipeline()

        # all dependencies of given output
        dependencies = {
            'depth': {},
            'disparity': {'depth',},
            'rectified_left': {'depth',},
            'rectified_right': {'depth',},
            'pcl': {'depth',},
            'disparity_cost_dump': {'depth',},
        }
        all_outputs = set(outputs)
        for out in outputs:
            all_outputs.update(dependencies[out])
        # print(f'Pipeline dependencies {outputs} -> {all_outputs}')

        # camRgb = self._pipeline.create(dai.node.ColorCamera)
        # camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        # camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        # camRgb.setIspScale(1, 3) # (1, 3) => (427, 267), (1, 2) => (640, 400)
        # print(camRgb.getIspSize())
        # print(camRgb.getPreviewSize())
        # print(camRgb.getVideoSize())
        # camRgb.setFps(fps)

        # Pipeline inputs
        xin_left = self._pipeline.create(dai.node.XLinkIn)
        xin_left.setStreamName("left")
        xin_right = self._pipeline.create(dai.node.XLinkIn)
        xin_right.setStreamName("right")
        # xin_rgb = self._pipeline.create(dai.node.XLinkIn)
        # xin_rgb.setStreamName("rgb")
        # xin_rgb.setMaxDataSize(1920*1080*3)

        # Workaround RGB camera to fix depth alignment
        if not stereo_config is None and stereo_config.get('stereo.setDepthAlign', '') == 'dai.CameraBoardSocket.CAM_A':
            cam_rgb = self._pipeline.create(dai.node.ColorCamera)
            cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        # cam_rgb.setIspScale(1, 3)

        self._xOut = {}
        self._xOut['depth'] = self._pipeline.create(dai.node.XLinkOut)
        self._xOut['depth'].setStreamName("depth")
        stereo = self._pipeline.create(dai.node.StereoDepth)

        xin_left.out.link(stereo.left)
        xin_right.out.link(stereo.right)
        stereo.depth.link(self._xOut['depth'].input)

        if not stereo_config is None:
            stereo_config.configure_stereo_node(stereo)

        # TODO remove hardcoded postprocessing hardware resources
        stereo.setPostProcessingHardwareResources(5, 5)

        if 'disparity_cost_dump' in all_outputs:
            self._xOut['disparity_cost_dump'] = self._pipeline.create(dai.node.XLinkOut)
            self._xOut['disparity_cost_dump'].setStreamName("disparity_cost_dump")
            stereo.debugDispCostDump.link(self._xOut['disparity_cost_dump'].input)

        if 'rectified_left' in all_outputs:
            self._xOut['rectified_left'] = self._pipeline.create(dai.node.XLinkOut)
            self._xOut['rectified_left'].setStreamName('rectified_left')
            stereo.rectifiedLeft.link(self._xOut['rectified_left'].input)

        if 'rectified_right' in all_outputs:
            self._xOut['rectified_right'] = self._pipeline.create(dai.node.XLinkOut)
            self._xOut['rectified_right'].setStreamName('rectified_right')
            stereo.rectifiedRight.link(self._xOut['rectified_right'].input)

        if 'pcl' in all_outputs:
            self._xOut['pcl'] = self._pipeline.create(dai.node.XLinkOut)
            self._xOut['pcl'].setStreamName('pcl')
            pcl = self._pipeline.create(dai.node.PointCloud)
            stereo.depth.link(pcl.inputDepth)
            pcl.outputPointCloud.link(self._xOut['pcl'].input)

        if 'disparity' in all_outputs:
            self._xOut['disparity'] = self._pipeline.create(dai.node.XLinkOut)
            self._xOut['disparity'].setStreamName('disparity')
            stereo.disparity.link(self._xOut['disparity'].input)

        if calib is None:
            print('Warning: using device calibration!')
            # calib = device.readCalibration()
        else:
            self._pipeline.setCalibrationData(calib)

        self._pipeline.setXLinkChunkSize(0)  # Increase throughput for large data transfers. Effectively disables xlink chunking - increasing the max fps

        self._runtime_statistics = {}


    def runtime_stats(self, key=None):
        if not self._runtime_statistics:
            raise RuntimeError('No statistics, run `replay()` first.')
        if key is None:
            return self._runtime_statistics
        else:
            return self._runtime_statistics[key]
        

    def replay(self, frames, device_info=None, *, workaround_decimation_filter=True):
        with get_device(device_info=device_info) as device:
            device.startPipeline(self._pipeline)

            input_queues = {
                "left": device.getInputQueue("left"),
                "right": device.getInputQueue("right"),
                # 'rgb': device.getInputQueue('rgb'),
            }
            output_queues = {name: device.getOutputQueue(name, maxSize=4, blocking=False) for name in self._xOut.keys()}

            # workaround for postProcessing.decimationFilter.factor > 1, where the first returned frame is garbage
            # TODO is this workaround still needed?
            if workaround_decimation_filter:
                frames = more_itertools.peekable(frames)
                frame = frames.peek()
                left_frame, right_frame, *_ = frame
                _send_images(input_queues, {
                    "left": left_frame,
                    "right": right_frame,
                    # "rgb": np.stack((left_frame, left_frame, np.zeros_like(left_frame)), axis=-1).astype(np.uint8)
                })
                _ = [queue.get() for _, queue in output_queues.items()]

            start = time.perf_counter()
            sent_frames = 0
            frames_count = 0
            wait_offset = 3  # send N frames before expecting results from the first
            for idx, frame in enumerate(frames):
                frames_count += 1
                if len(frame) == 2:
                    left_frame, right_frame = frame
                    # rgb_frame = np.stack((left_frame, left_frame, np.zeros_like(left_frame)), axis=-1).astype(np.uint8)
                elif len(frame) == 3:
                    left_frame, right_frame, rgb_frame = frame
                else:
                    raise ValueError(f'Unexpected length of frame {len(frame)}. Expected 2 or 3.')
                _send_images(input_queues, {"left": left_frame, "right": right_frame})  # , 'rgb': rgb_frame})
                sent_frames += 1
                if idx >= wait_offset:
                    yield _get_data(output_queues)
                    sent_frames -= 1
            for _ in range(sent_frames):
                yield _get_data(output_queues)
            end = time.perf_counter()
        self._runtime_statistics = {
            'spf': (end - start) / frames_count,  # seconds per frame
            'total_time': end - start,
        }


class ReplayV3:
    def __init__(self, calib=None, stereo_config={}, outputs:Set[str]={'depth',}):
        self._calib = calib
        self._stereo_config = stereo_config
        self._outputs = outputs
        self._runtime_statistics = {}

    def runtime_stats(self, key=None):
        if not self._runtime_statistics:
            raise RuntimeError('No statistics, run `replay()` first.')
        if key is None:
            return self._runtime_statistics
        else:
            return self._runtime_statistics[key]

    def replay(self, frames, device_info=None):
        """
        Args:
        - frames: iterable of (left, right[, rgb]) images as numpy arrays
        - calib: device calibration. If None the currently attached device calibration is used.
        - stereo_config: device pipeline configuration
        - fps: device frames per second
        - timeout_s: raise TimeoutError if getting a single queue item from pipeline takes longer than given time in seconds
        - outputs: choose which outputs you want: 'depth', 'disparity', 'rectified_left', 'rectified_right'
        """
        if not self._outputs:
            raise ValueError('No output specified')

        with dai.Pipeline(get_device(device_info)) as pipeline:
            stereo = pipeline.create(dai.node.StereoDepth)
            mono_left = stereo.left.createInputQueue()
            mono_right = stereo.right.createInputQueue()
            out_depth = stereo.depth.createOutputQueue()
            out_depth.setName('depth')
            out_q = [out_depth, ]

            # Workaround RGB camera to fix depth alignment
            if not self._stereo_config is None and self._stereo_config.get('stereo.setDepthAlign', '') == 'dai.CameraBoardSocket.CAM_A':
                cam_rgb = pipeline.create(dai.node.ColorCamera)
                cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
                cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
                # cam_rgb.setIspScale(1, 3)

            if 'disparity' in self._outputs:
                out_disparity = stereo.disparity.createOutputQueue()
                out_disparity.setName('disparity')
                out_q.append(out_disparity)

            if 'rectified_left' in self._outputs:
                out_rectified_left = stereo.rectifiedLeft.createOutputQueue()
                out_rectified_left.setName('rectified_left')
                out_q.append(out_rectified_left)

            if 'rectified_right' in self._outputs:
                out_rectified_right = stereo.rectifiedRight.createOutputQueue()
                out_rectified_right.setName('rectified_right')
                out_q.append(out_rectified_right)

            if 'pcl' in self._outputs:
                raise NotImplementedError('Generating a pointcloud on the device is not implemented')

            if not self._stereo_config is None:
                self._stereo_config.configure_stereo_node(stereo)

            if self._calib is None:
                print('Warning: using device calibration!')
            else:
                pipeline.setCalibrationData(self._calib)

            pipeline.start()
            in_q = {
                'left': mono_left,
                'right': mono_right
            }

            output_queues = {q.getName(): q for q in out_q}

            timestamp_ms = 0  # needed for sync stage in Stereo node
            frame_interval_ms = 50
            start = time.perf_counter()
            frames_count = 0
            sent_frames = 0
            wait_offset = 3  # send N frames before expecting results from the first
            for idx, frame in enumerate(frames):
                frames_count += 1
                if len(frame) == 2:
                    left_frame, right_frame = frame
                    rgb_frame = np.stack((left_frame, left_frame, np.zeros_like(left_frame)), axis=-1).astype(np.uint8)
                elif len(frame) == 3:
                    left_frame, right_frame, rgb_frame = frame
                else:
                    raise ValueError(f'Unexpected length of frame {len(frame)}. Expected 2 or 3.')
                tstamp = datetime.timedelta(seconds = timestamp_ms // 1000,
                            milliseconds = timestamp_ms % 1000)
                # TODO send RGB as well (, 'rgb': rgb_frame)
                _send_images(in_q, {'left': left_frame.astype(np.uint8), 'right': right_frame.astype(np.uint8)}, ts=tstamp)
                sent_frames += 1
                timestamp_ms += frame_interval_ms

                if not pipeline.isRunning():
                    raise RuntimeError('Pipeline stopped before all frames were processed')
                if idx >= wait_offset:
                    yield _get_data(output_queues)
                    sent_frames -= 1
            for _ in range(sent_frames):
                yield _get_data(output_queues)
            end = time.perf_counter()

        self._runtime_statistics = {
            'spf': (end - start) / frames_count,  # seconds per frame
            'total_time': end - start,
        }


if __is_dai3:
    Replay = ReplayV3
else:
    Replay = ReplayV2
