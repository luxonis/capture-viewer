"""
Replay on RVC2 devices
"""
import copy
import json
import signal
from typing import Set
import cv2
import depthai as dai
import numpy as np
import tenacity


# Custom Stereo pipeline configurations
STEREO_PRESETS = {
    # https://github.com/luxonis/depthai-core/blob/main/src/pipeline/node/StereoDepth.cpp
    'HIGH_DENSITY': {
        'stereo.setConfidenceThreshold': 245,
        'stereo.setLeftRightCheck': True,
        'cfg.algorithmControl.leftRightCheckThreshold': 10,
    },
    'HIGH_ACCURACY': {
        'stereo.setConfidenceThreshold': 200,
        'stereo.setLeftRightCheck': True,
        'cfg.algorithmControl.leftRightCheckThreshold': 5,
    },
    'default': {
        'stereo.setLeftRightCheck': True,
        'stereo.setExtendedDisparity': False,
        'stereo.setSubpixel': True,
        'stereo.setSubpixelFractionalBits': 3,
        'stereo.initialConfig.setMedianFilter': 'dai.MedianFilter.KERNEL_7x7',
        # HIGH DENSITY preset
        'stereo.setConfidenceThreshold': 245,
        'stereo.setLeftRightCheck': True,
        'cfg.algorithmControl.leftRightCheckThreshold': 10,
        # end of HIGH DENSITY preset
        'cfg.postProcessing.filteringOrder': 'dai.StereoDepthConfig.PostProcessing.Filter.DECIMATION,dai.StereoDepthConfig.PostProcessing.Filter.MEDIAN,dai.StereoDepthConfig.PostProcessing.Filter.SPECKLE,dai.StereoDepthConfig.PostProcessing.Filter.SPATIAL,dai.StereoDepthConfig.PostProcessing.Filter.TEMPORAL',
        'cfg.postProcessing.decimationFilter.decimationFactor': 2,
        'cfg.postProcessing.decimationFilter.decimationMode': 'dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEAN',
        'cfg.postProcessing.spatialFilter.enable': True,
        'cfg.postProcessing.spatialFilter.holeFillingRadius': 1,
        'cfg.postProcessing.spatialFilter.numIterations': 2,
        'cfg.postProcessing.spatialFilter.alpha': 0.5,
        'cfg.postProcessing.spatialFilter.delta': 3,
        'cfg.postProcessing.temporalFilter.enable': True,
        'cfg.postProcessing.temporalFilter.alpha': 0.5,
        'cfg.postProcessing.temporalFilter.delta': 3,
        'cfg.postProcessing.speckleFilter.enable': True,
        'cfg.postProcessing.speckleFilter.speckleRange': 200,
        'cfg.postProcessing.speckleFilter.differenceThreshold': 2,
        'cfg.postProcessing.thresholdFilter.minRange': 0,
        'cfg.postProcessing.thresholdFilter.maxRange': 15000,
    },
    'face': {
        'stereo.setLeftRightCheck': True,
        'stereo.setExtendedDisparity': True,
        'stereo.setSubpixel': True,
        'stereo.setSubpixelFractionalBits': 5,
        'stereo.initialConfig.setMedianFilter': 'dai.MedianFilter.MEDIAN_OFF',
        # HIGH DENSITY preset
        'stereo.setConfidenceThreshold': 245,
        'stereo.setLeftRightCheck': True,
        'cfg.algorithmControl.leftRightCheckThreshold': 10,
        # end of HIGH DENSITY preset
        'cfg.postProcessing.filteringOrder': 'dai.StereoDepthConfig.PostProcessing.Filter.DECIMATION,dai.StereoDepthConfig.PostProcessing.Filter.MEDIAN,dai.StereoDepthConfig.PostProcessing.Filter.SPECKLE,dai.StereoDepthConfig.PostProcessing.Filter.SPATIAL,dai.StereoDepthConfig.PostProcessing.Filter.TEMPORAL',
        'cfg.postProcessing.decimationFilter.decimationFactor': 2,
        'cfg.postProcessing.decimationFilter.decimationMode': 'dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEAN',
        'cfg.postProcessing.spatialFilter.enable': True,
        'cfg.postProcessing.spatialFilter.holeFillingRadius': 1,
        'cfg.postProcessing.spatialFilter.numIterations': 1,
        'cfg.postProcessing.spatialFilter.alpha': 0.5,
        'cfg.postProcessing.spatialFilter.delta': 3,
        'cfg.postProcessing.temporalFilter.enable': True,
        'cfg.postProcessing.temporalFilter.alpha': 0.5,
        'cfg.postProcessing.temporalFilter.delta': 3,
        'cfg.postProcessing.speckleFilter.enable': True,
        'cfg.postProcessing.speckleFilter.speckleRange': 200,
        'cfg.postProcessing.speckleFilter.differenceThreshold': 2,
        'cfg.postProcessing.thresholdFilter.minRange': 30,
        'cfg.postProcessing.thresholdFilter.maxRange': 3000,
    },
    'high_detail': {
        'stereo.setLeftRightCheck': True,
        'stereo.setExtendedDisparity': True,
        'stereo.setSubpixel': True,
        'stereo.setSubpixelFractionalBits': 5,
        'stereo.initialConfig.setMedianFilter': 'dai.MedianFilter.MEDIAN_OFF',
        # HIGH ACCURACY preset
        'stereo.setConfidenceThreshold': 200,
        'stereo.setLeftRightCheck': True,
        'cfg.algorithmControl.leftRightCheckThreshold': 5,
        # end of HIGH ACCURACY preset
        'cfg.postProcessing.filteringOrder': 'dai.StereoDepthConfig.PostProcessing.Filter.DECIMATION,dai.StereoDepthConfig.PostProcessing.Filter.MEDIAN,dai.StereoDepthConfig.PostProcessing.Filter.SPECKLE,dai.StereoDepthConfig.PostProcessing.Filter.SPATIAL,dai.StereoDepthConfig.PostProcessing.Filter.TEMPORAL',
        'cfg.postProcessing.decimationFilter.decimationFactor': 1,
        'cfg.postProcessing.decimationFilter.decimationMode': 'dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEAN',
        'cfg.postProcessing.spatialFilter.enable': True,
        'cfg.postProcessing.spatialFilter.holeFillingRadius': 1,
        'cfg.postProcessing.spatialFilter.numIterations': 3,
        'cfg.postProcessing.spatialFilter.alpha': 0.7,
        'cfg.postProcessing.spatialFilter.delta': 3,
        'cfg.postProcessing.temporalFilter.enable': True,
        'cfg.postProcessing.temporalFilter.alpha': 0.5,
        'cfg.postProcessing.temporalFilter.delta': 3,
        'cfg.postProcessing.speckleFilter.enable': True,
        'cfg.postProcessing.speckleFilter.speckleRange': 200,
        'cfg.postProcessing.speckleFilter.differenceThreshold': 2,
        'cfg.postProcessing.thresholdFilter.minRange': 0,
        'cfg.postProcessing.thresholdFilter.maxRange': 15000,
    },
    'high_fps': {
        'stereo.setLeftRightCheck': True,
        'stereo.setExtendedDisparity': False,
        'stereo.setSubpixel': True,
        'stereo.setSubpixelFractionalBits': 3,
        'stereo.initialConfig.setMedianFilter': 'dai.MedianFilter.KERNEL_3x3',
        # HIGH DENSITY preset
        'stereo.setConfidenceThreshold': 245,
        'stereo.setLeftRightCheck': True,
        'cfg.algorithmControl.leftRightCheckThreshold': 10,
        # end of HIGH DENSITY preset
        'cfg.postProcessing.filteringOrder': 'dai.StereoDepthConfig.PostProcessing.Filter.DECIMATION,dai.StereoDepthConfig.PostProcessing.Filter.MEDIAN,dai.StereoDepthConfig.PostProcessing.Filter.SPECKLE,dai.StereoDepthConfig.PostProcessing.Filter.SPATIAL,dai.StereoDepthConfig.PostProcessing.Filter.TEMPORAL',
        'cfg.postProcessing.decimationFilter.decimationFactor': 2,
        'cfg.postProcessing.decimationFilter.decimationMode': 'dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEAN',
        'cfg.postProcessing.spatialFilter.enable': True,
        'cfg.postProcessing.spatialFilter.holeFillingRadius': 1,
        'cfg.postProcessing.spatialFilter.numIterations': 2,
        'cfg.postProcessing.spatialFilter.alpha': 0.5,
        'cfg.postProcessing.spatialFilter.delta': 3,
        'cfg.postProcessing.temporalFilter.enable': False,
        'cfg.postProcessing.temporalFilter.alpha': 0.5,
        'cfg.postProcessing.temporalFilter.delta': 3,
        'cfg.postProcessing.speckleFilter.enable': True,
        'cfg.postProcessing.speckleFilter.speckleRange': 50,
        'cfg.postProcessing.speckleFilter.differenceThreshold': 2,
        'cfg.postProcessing.thresholdFilter.minRange': 0,
        'cfg.postProcessing.thresholdFilter.maxRange': 15000,
    },
    'high_accuracy': {
        'stereo.setLeftRightCheck': True,
        'stereo.setExtendedDisparity': True,
        'stereo.setSubpixel': True,
        'stereo.setSubpixelFractionalBits': 5,
        'stereo.initialConfig.setMedianFilter': 'dai.MedianFilter.MEDIAN_OFF',
        # HIGH ACCURACY preset
        'stereo.setConfidenceThreshold': 200,
        'stereo.setLeftRightCheck': True,
        'cfg.algorithmControl.leftRightCheckThreshold': 5,
        # end of HIGH ACCURACY preset
        'cfg.postProcessing.filteringOrder': 'dai.StereoDepthConfig.PostProcessing.Filter.DECIMATION,dai.StereoDepthConfig.PostProcessing.Filter.MEDIAN,dai.StereoDepthConfig.PostProcessing.Filter.SPECKLE,dai.StereoDepthConfig.PostProcessing.Filter.SPATIAL,dai.StereoDepthConfig.PostProcessing.Filter.TEMPORAL',
        'cfg.postProcessing.decimationFilter.decimationFactor': 2,
        'cfg.postProcessing.decimationFilter.decimationMode': 'dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEAN',
        'cfg.postProcessing.spatialFilter.enable': True,
        'cfg.postProcessing.spatialFilter.holeFillingRadius': 1,
        'cfg.postProcessing.spatialFilter.numIterations': 3,
        'cfg.postProcessing.spatialFilter.alpha': 0.5,
        'cfg.postProcessing.spatialFilter.delta': 3,
        'cfg.postProcessing.temporalFilter.enable': True,
        'cfg.postProcessing.temporalFilter.alpha': 0.5,
        'cfg.postProcessing.temporalFilter.delta': 3,
        'cfg.postProcessing.speckleFilter.enable': True,
        'cfg.postProcessing.speckleFilter.speckleRange': 200,
        'cfg.postProcessing.speckleFilter.differenceThreshold': 2,
        'cfg.postProcessing.thresholdFilter.minRange': 0,
        'cfg.postProcessing.thresholdFilter.maxRange': 15000,
    },
    'robotics': {
        'stereo.setLeftRightCheck': True,
        'stereo.setExtendedDisparity': False,
        'stereo.setSubpixel': True,
        'stereo.setSubpixelFractionalBits': 3,
        'stereo.initialConfig.setMedianFilter': 'dai.MedianFilter.KERNEL_7x7',
        # HIGH DENSITY preset
        'stereo.setConfidenceThreshold': 245,
        'stereo.setLeftRightCheck': True,
        'cfg.algorithmControl.leftRightCheckThreshold': 10,
        # end of HIGH DENSITY preset
        'cfg.postProcessing.filteringOrder': 'dai.StereoDepthConfig.PostProcessing.Filter.DECIMATION,dai.StereoDepthConfig.PostProcessing.Filter.MEDIAN,dai.StereoDepthConfig.PostProcessing.Filter.SPECKLE,dai.StereoDepthConfig.PostProcessing.Filter.SPATIAL,dai.StereoDepthConfig.PostProcessing.Filter.TEMPORAL',
        'cfg.postProcessing.decimationFilter.decimationFactor': 2,
        'cfg.postProcessing.decimationFilter.decimationMode': 'dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEAN',
        'cfg.postProcessing.spatialFilter.enable': True,
        'cfg.postProcessing.spatialFilter.holeFillingRadius': 2,
        'cfg.postProcessing.spatialFilter.numIterations': 1,
        'cfg.postProcessing.spatialFilter.alpha': 0.5,
        'cfg.postProcessing.spatialFilter.delta': 20,
        'cfg.postProcessing.temporalFilter.enable': False,
        'cfg.postProcessing.temporalFilter.alpha': 0.5,
        'cfg.postProcessing.temporalFilter.delta': 3,
        'cfg.postProcessing.speckleFilter.enable': True,
        'cfg.postProcessing.speckleFilter.speckleRange': 200,
        'cfg.postProcessing.speckleFilter.differenceThreshold': 2,
        'cfg.postProcessing.thresholdFilter.minRange': 0,
        'cfg.postProcessing.thresholdFilter.maxRange': 10000,
    }
}


def _configure_stereo(stereo: dai.Node, config_json: str):
    """
    Given a configuration `config_json` as a json string, configure `dai.node.StereoDepth` by calling its functions and settings its properties.
    The keys starting with 'stereo.' are applied first, the keys starting with 'cfg.' are applied second.
    Thus the 'cfg.' keys override the 'stereo.'.
    """
    config = json.loads(config_json)

    try:
        # Split filtering cfg.postProcessing.filteringOrder if composed of a single string
        config['cfg.postProcessing.filteringOrder'] = config['cfg.postProcessing.filteringOrder'].split(',')
    except (KeyError, AttributeError):
        pass

    if 'setDefaultProfilePreset' in config:
        default_config = copy.deepcopy(STEREO_PRESETS[config['setDefaultProfilePreset']])
        del config['setDefaultProfilePreset']
        default_config.update(config)
        config = default_config
        print('INFO: Applying preset and possibly overriding it with other specific settings.')

    cfg = None
    # We need to set the stereo params first, then initial config (cfg)
    for key_prefix in ('stereo.', 'cfg.'):
        for k, v in config.items():
            if not k.startswith(key_prefix):
                continue
            # print(f'{k} -> {v}')
            k0, _, kN = k.partition('.')
            if k0 == 'cfg' and cfg is None:
                cfg = stereo.initialConfig.get()
                # workaround for versions 2.28.0.0 and lower
                if not hasattr(cfg.postProcessing, 'filteringOrder') and 'cfg.postProcessing.filteringOrder' in config:
                    print('WARNING: Ignoring cfg.postProcessing.filteringOrder settings, since it is not implemented in this version of depthai')
                    del config['cfg.postProcessing.filteringOrder']

                # print(' - stereo.initialConfig.get()')
            if isinstance(v, (list, tuple)):
                v = ','.join(v)
            try:
                # print(f' - trying calling {k} as a function')
                eval(f'{k}({v})')
                # fn(v)
            except (TypeError,SyntaxError):  # not callable, try setting it directly
                # print(f' - trying setting {k} as a variable')
                # rsetattr(locals()[k0], kN, v)
                # print(f' - {k}={v}')
                exec(f'{k}={v}')
                # print('  => ', rgetattr(locals()[k0], kN))
    if not cfg is None:
        stereo.initialConfig.set(cfg)

def _send_images(device, frames):
    input_queues = {
        "left": device.getInputQueue("left"),
        "right": device.getInputQueue("right"),
        'rgb': device.getInputQueue('rgb'),
    }
    ts = dai.Clock.now()
    for name, frame in frames.items():
        h, w, *_ = frame.shape
        number = {'rgb': int(0), 'left': int(1), 'right': int(2)}[name]
        img = dai.ImgFrame()
        img.setData(frame.flatten())
        img.setTimestamp(ts)
        img.setWidth(w)
        img.setHeight(h)
        img.setInstanceNum(number)
        img.setType({
            'rgb': dai.ImgFrame.Type.RGB888i,
            'left': dai.ImgFrame.Type.RAW8,
            'right': dai.ImgFrame.Type.RAW8,
        }[name])
        input_queues[name].send(img)


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

def replay(frames, *, calib=None, stereo_config:str='', fps:int=30, outputs:Set[str]={'depth',}, timeout_s:int = 2, device_info=None):
    """
    Args:
    - frames: iterable of (left, right[, rgb]) images as numpy arrays
    - calib: device calibration. If None the currently attached device' calibration is used.
    - stereo_config: device pipeline configuration
    - fps: device frames per second
    - outputs: choose which outputs you want: 'depth', 'rectified_left', 'rectified_right', 'pcl'
    - timeout_s: raise TimeoutError if getting a single queue item from pipeline takes longer than given time in seconds
    - device_info: optional depthai.DeviceInfo to use specified device
    """
    @tenacity.retry(
            stop=tenacity.stop_after_attempt(5),
            wait=tenacity.wait_exponential(multiplier=1, min=4, max=10),
            retry=tenacity.retry_if_exception_type(RuntimeError),
            reraise=True)
    def get_device(device_info=None):
        """Get any dai.Device(), retry 5 times and wait between each retry."""
        if device_info is None:
            return dai.Device()
        else:
            return dai.Device(device_info)

    if not outputs:
        raise ValueError('No output specified')

    with get_device(device_info) as device:
        pipeline = dai.Pipeline()

        # all dependencies of given output
        dependencies = {
            'depth': {},
            'rectified_left': {'depth',},
            'rectified_right': {'depth',},
            'pcl': {'depth',},
            'disparity_cost_dump': {'depth',},
        }
        all_outputs = set(outputs)
        for out in outputs:
            all_outputs.update(dependencies[out])
        # print(f'Pipeline dependencies {outputs} -> {all_outputs}')

        # camRgb = pipeline.create(dai.node.ColorCamera)
        # camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        # camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        # camRgb.setIspScale(1, 3) # (1, 3) => (427, 267), (1, 2) => (640, 400)
        # print(camRgb.getIspSize())
        # print(camRgb.getPreviewSize())
        # print(camRgb.getVideoSize())
        # camRgb.setFps(fps)

        # Pipeline inputs
        xin_left = pipeline.create(dai.node.XLinkIn)
        xin_left.setStreamName("left")
        xin_right = pipeline.create(dai.node.XLinkIn)
        xin_right.setStreamName("right")
        xin_rgb = pipeline.create(dai.node.XLinkIn)
        xin_rgb.setStreamName("rgb")
        xin_rgb.setMaxDataSize(1920*1080*3)

        # Workaround RGB camera to fix depth alignment
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

        xOut = {}
        xOut['depth'] = pipeline.create(dai.node.XLinkOut)
        xOut['depth'].setStreamName("depth")
        stereo = pipeline.create(dai.node.StereoDepth)

        xin_left.out.link(stereo.left)
        xin_right.out.link(stereo.right)
        stereo.depth.link(xOut['depth'].input)

        if stereo_config:
            _configure_stereo(stereo, stereo_config)

        # TODO remove hardcoded postprocessing hardware resources
        stereo.setPostProcessingHardwareResources(3, 3)

        if 'disparity_cost_dump' in all_outputs:
            xOut['disparity_cost_dump'] = pipeline.create(dai.node.XLinkOut)
            xOut['disparity_cost_dump'].setStreamName("disparity_cost_dump")
            stereo.debugDispCostDump.link(xOut['disparity_cost_dump'].input)

        if 'rectified_left' in all_outputs:
            xOut['rectified_left'] = pipeline.create(dai.node.XLinkOut)
            xOut['rectified_left'].setStreamName('rectified_left')
            stereo.rectifiedLeft.link(xOut['rectified_left'].input)

        if 'rectified_right' in all_outputs:
            xOut['rectified_right'] = pipeline.create(dai.node.XLinkOut)
            xOut['rectified_right'].setStreamName('rectified_right')
            stereo.rectifiedRight.link(xOut['rectified_right'].input)

        if 'pcl' in outputs:
            xOut['pcl'] = pipeline.create(dai.node.XLinkOut)
            xOut['pcl'].setStreamName('pcl')
            pcl = pipeline.create(dai.node.PointCloud)
            stereo.depth.link(pcl.inputDepth)
            pcl.outputPointCloud.link(xOut['pcl'].input)

        if calib is None:
            print('Warning: using device calibration!')
            # calib = device.readCalibration()
        else:
            pipeline.setCalibrationData(calib)

        device.startPipeline(pipeline)
        queues = {name: device.getOutputQueue(name, maxSize=4, blocking=False) for name in xOut.keys()}

        def _convert(name, data):
            """Convert data from xlink to numpy based on name."""
            if name == 'pcl':
                return data.getPoints().astype(np.float64)
            if name == 'disparity_cost_dump':
                return np.array(data.getData(), dtype=np.uint8)
            return data.getCvFrame().astype(float)
        
        # workaround for postProcessing.decimationFilter.factor > 1, where the first returned frame is garbage
        frame = frames[0]
        left_frame, right_frame, *_ = frame
        _send_images(device, {
            "left": left_frame,
            "right": right_frame,
            "rgb": np.stack((left_frame, left_frame, np.zeros_like(left_frame)), axis=-1).astype(np.uint8)
        })
        _ = [queue.get() for _, queue in queues.items()]

        # timeout on pipeline not returning results
        def handler(signum, frame):
            signame = signal.Signals(signum).name
            print(f"Timeout error. signal: {signame} ({signum}), frame: {frame}")
            raise TimeoutError()

        # signal.signal(signal.SIGALRM, handler)

        for frame in frames:
            if len(frame) == 2:
                left_frame, right_frame = frame
                rgb_frame = np.stack((left_frame, left_frame, np.zeros_like(left_frame)), axis=-1).astype(np.uint8)
            elif len(frame) == 3:
                left_frame, right_frame, rgb_frame = frame
            else:
                raise ValueError(f'Unexpected length of frame {len(frame)}. Expected 2 or 3.')
            _send_images(device, {"left": left_frame, "right": right_frame, 'rgb': rgb_frame})
            # signal.alarm(timeout_s)
            data = {name: queue.get() for name, queue in queues.items()}
            # signal.alarm(0)
            yield {name: _convert(name, datum) for name, datum in data.items()}
