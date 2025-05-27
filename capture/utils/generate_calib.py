import json
import time

def generate_depthai_calib_from_zed(calib_params, width, height):
    """
    Generates a 3-camera DepthAI-compatible calib.json using ZED stereo calibration data
    """
    baseline = calib_params.get_camera_baseline()

    def camera_block(fx, fy, cx, cy, disto, K, translation, socket):
        return {
            "cameraType": 0,
            "distortionCoeff": disto,
            "extrinsics": {
                "rotationMatrix": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0]
                ],
                "specTranslation": {"x": translation, "y": 0.0, "z": 0.0},
                "toCameraSocket": socket,
                "translation": {"x": translation, "y": 0.0, "z": 0.0}
            },
            "height": height,
            "intrinsicMatrix": K,
            "lensPosition": 0,
            "specHfovDeg": 71.86,
            "width": width
        }

    def dummy_rgb_block():
        return {
            "cameraType": 0,
            "distortionCoeff": [0.0] * 14,
            "extrinsics": {
                "rotationMatrix": [[0.0]*3]*3,
                "specTranslation": {"x": 0.0, "y": 0.0, "z": 0.0},
                "toCameraSocket": -1,
                "translation": {"x": 0.0, "y": 0.0, "z": 0.0}
            },
            "height": height * 2,
            "intrinsicMatrix": [
                [fx * 4, 0.0, width],
                [0.0, fy * 4, height],
                [0.0, 0.0, 1.0]
            ],
            "lensPosition": 128,
            "specHfovDeg": 68.79,
            "width": width * 3
        }

    fx = calib_params.left_cam.fx
    fy = calib_params.left_cam.fy
    cx = calib_params.left_cam.cx
    cy = calib_params.left_cam.cy
    K_left = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]

    fx_r = calib_params.right_cam.fx
    fy_r = calib_params.right_cam.fy
    cx_r = calib_params.right_cam.cx
    cy_r = calib_params.right_cam.cy
    K_right = [[fx_r, 0, cx_r], [0, fy_r, cy_r], [0, 0, 1]]

    left_translation = -baseline / 2
    right_translation = baseline / 2

    calib = {
        "batchName": "",
        "batchTime": int(time.time()),
        "boardConf": "ZED-to-OAK",
        "boardCustom": "",
        "boardName": "ZED-Mimic",
        "boardOptions": 0,
        "boardRev": "R1",
        "cameraData": [
            [0, dummy_rgb_block()],
            [1, camera_block(fx, fy, cx, cy, calib_params.left_cam.disto.tolist(), K_left, left_translation, 2)],
            [2, camera_block(fx_r, fy_r, cx_r, cy_r, calib_params.right_cam.disto.tolist(), K_right, right_translation, 0)]
        ],
        "deviceName": "",
        "hardwareConf": "F0-FV00-ZED",
        "housingExtrinsics": {
            "rotationMatrix": [],
            "specTranslation": {"x": 0.0, "y": 0.0, "z": 0.0},
            "toCameraSocket": -1,
            "translation": {"x": 0.0, "y": 0.0, "z": 0.0}
        },
        "imuExtrinsics": {
            "rotationMatrix": [[0.0]*3]*3,
            "specTranslation": {"x": 0.0, "y": 0.0, "z": 0.0},
            "toCameraSocket": -1,
            "translation": {"x": 0.0, "y": 0.0, "z": 0.0}
        },
        "miscellaneousData": [],
        "productName": "ZED-to-OAK",
        "stereoEnableDistortionCorrection": False,
        "stereoRectificationData": {
            "leftCameraSocket": 1,
            "rectifiedRotationLeft": [[1.0, 0.0, 0.0],
                                      [0.0, 1.0, 0.0],
                                      [0.0, 0.0, 1.0]],
            "rectifiedRotationRight": [[1.0, 0.0, 0.0],
                                       [0.0, 1.0, 0.0],
                                       [0.0, 0.0, 1.0]],
            "rightCameraSocket": 2
        },
        "stereoUseSpecTranslation": True,
        "version": 7,
        "verticalCameraSocket": -1
    }

    return calib
