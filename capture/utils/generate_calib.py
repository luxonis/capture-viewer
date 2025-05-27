import json
import time

def generate_depthai_calib_from_zed(calib_params, width, height):
    """
    Generates a DepthAI-compatible calib.json dictionary using ZED calibration data
    """
    baseline = calib_params.get_camera_baseline()

    def build_camera_block(fx, fy, cx, cy, disto, K, translation_to_main):
        return {
            "cameraType": 0,
            "distortionCoeff": disto,
            "extrinsics": {
                "rotationMatrix": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0]
                ],
                "specTranslation": {"x": translation_to_main, "y": 0.0, "z": 0.0},
                "toCameraSocket": 2 if translation_to_main < 0 else 0,
                "translation": {"x": translation_to_main, "y": 0.0, "z": 0.0}
            },
            "height": height,
            "intrinsicMatrix": K,
            "lensPosition": 0,
            "specHfovDeg": 70.0,
            "width": width
        }

    left_translation = baseline / 2
    right_translation = -baseline / 2

    calib = {
        "batchName": "",
        "batchTime": int(time.time()),
        "boardConf": "ZED-to-OAK",
        "boardCustom": "",
        "boardName": "ZED-Emulated",
        "boardOptions": 0,
        "boardRev": "R1",
        "cameraData": [
            [1, build_camera_block(
                calib_params.left_cam.fx,
                calib_params.left_cam.fy,
                calib_params.left_cam.cx,
                calib_params.left_cam.cy,
                calib_params.left_cam.disto.tolist(),
                [[calib_params.left_cam.fx, 0, calib_params.left_cam.cx],
                 [0, calib_params.left_cam.fy, calib_params.left_cam.cy],
                 [0, 0, 1]],
                left_translation
            )],
            [2, build_camera_block(
                calib_params.right_cam.fx,
                calib_params.right_cam.fy,
                calib_params.right_cam.cx,
                calib_params.right_cam.cy,
                calib_params.right_cam.disto.tolist(),
                [[calib_params.right_cam.fx, 0, calib_params.right_cam.cx],
                 [0, calib_params.right_cam.fy, calib_params.right_cam.cy],
                 [0, 0, 1]],
                right_translation
            )]
        ],
        "deviceName": "ZED-to-OAK",
        "hardwareConf": "F0-ZED-EMU",
        "housingExtrinsics": {
            "rotationMatrix": [],
            "specTranslation": {"x": 0.0, "y": 0.0, "z": 0.0},
            "toCameraSocket": -1,
            "translation": {"x": 0.0, "y": 0.0, "z": 0.0}
        },
        "imuExtrinsics": {
            "rotationMatrix": [
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0]
            ],
            "specTranslation": {"x": 0.0, "y": 0.0, "z": 0.0},
            "toCameraSocket": -1,
            "translation": {"x": 0.0, "y": 0.0, "z": 0.0}
        },
        "miscellaneousData": [],
        "productName": "ZED-to-OAK",
        "stereoEnableDistortionCorrection": False,
        "stereoUseSpecTranslation": True,
        "version": 7,
        "verticalCameraSocket": -1
    }

    return calib

# Usage:
# calib_json = generate_depthai_calib_from_zed(calib_params, 1280, 720)
# with open("zed_depthai_calib.json", "w") as f:
#     json.dump(calib_json, f, indent=4)
