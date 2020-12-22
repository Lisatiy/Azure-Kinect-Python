
"""
This is a python port of the simple-sample project from.
    https://github.com/microsoft/Azure-Kinect-Samples/blob/bf2f8cf95d969dcc7842c4c450052fe5a943c756/body-tracking-samples/simple_sample/main.c
"""

import traceback
import sys
import ctypes
import os
import numpy as np

# Add .. to the import path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import k4a

def VERIFY(result, error):
    if result != k4a.K4A_RESULT_SUCCEEDED:
        print(error)
        traceback.print_stack()
        sys.exit(1)


if __name__ == "__main__":
    # device_config = k4a.K4A_DEVICE_CONFIG_INIT_DISABLE_ALL
    # device_config.depth_mode = k4a.K4A_DEPTH_MODE_NFOV_UNBINNED

    # device = k4a.k4a_device_t()
    # VERIFY(k4a.k4a_device_open(0, ctypes.byref(device)), "Open K4A Device failed!")
    # VERIFY(k4a.k4a_device_start_cameras(device, ctypes.byref(device_config)), "Start K4A cameras failed!")

    # sensor_calibration = k4a.k4a_calibration_t()
    # VERIFY(k4a.k4a_device_get_calibration(device, device_config.depth_mode, k4a.K4A_COLOR_RESOLUTION_OFF, ctypes.byref(sensor_calibration)), "Get depth camera calibration failed!")
    # print(sensor_calibration.depth_camera_calibration.intrinsics.parameters.param.cx)

    # # import pdb; pdb.set_trace()
    # k4a.k4a_device_stop_cameras(device)
    # k4a.k4a_device_close(device)

    device_config = k4a.K4A_DEVICE_CONFIG_INIT_DISABLE_ALL
    device_config.depth_mode = k4a.K4A_DEPTH_MODE_NFOV_UNBINNED

    # open file
    playback_handle = k4a.k4a_playback_t()
    mkv_path = 'G:/dataset/installing_1.mkv'
    VERIFY(k4a.k4a_playback_open(ctypes.c_char_p(bytes(mkv_path, encoding='utf8')), ctypes.byref(playback_handle)), "Cannot open recording {}!".format(mkv_path))

    sensor_calibration = k4a.k4a_calibration_t()
    VERIFY(k4a.k4a_playback_get_calibration(playback_handle, ctypes.byref(sensor_calibration)), "Get depth camera calibration failed!")
    
    dep_in_cx = sensor_calibration.depth_camera_calibration.intrinsics.parameters.param.cx
    dep_in_cy = sensor_calibration.depth_camera_calibration.intrinsics.parameters.param.cy
    dep_in_fx = sensor_calibration.depth_camera_calibration.intrinsics.parameters.param.fx
    dep_in_fy = sensor_calibration.depth_camera_calibration.intrinsics.parameters.param.fy
    dep_in_mat = np.mat([[dep_in_fx, 0, dep_in_cx],
                    [0, dep_in_fy, dep_in_cy],
                    [0, 0, 1]])

    col_in_cx = sensor_calibration.color_camera_calibration.intrinsics.parameters.param.cx
    col_in_cy = sensor_calibration.color_camera_calibration.intrinsics.parameters.param.cy
    col_in_fx = sensor_calibration.color_camera_calibration.intrinsics.parameters.param.fx
    col_in_fy = sensor_calibration.color_camera_calibration.intrinsics.parameters.param.fy
    col_in_mat = np.mat([[col_in_fx, 0, col_in_cx],
                    [0, col_in_fy, col_in_cy],
                    [0, 0, 1]])

    dep_ex_r = sensor_calibration.depth_camera_calibration.extrinsics.rotation
    dep_ex_t = sensor_calibration.depth_camera_calibration.extrinsics.translation
    dep_ex_mat = np.mat([[dep_ex_r[0], dep_ex_r[1], dep_ex_r[2], dep_ex_t[0]],
                    [dep_ex_r[3], dep_ex_r[4], dep_ex_r[5], dep_ex_t[1]],
                    [dep_ex_r[6], dep_ex_r[7], dep_ex_r[8], dep_ex_t[2]],
                    [0, 0, 0, 1]])
    dep_ex_mat = np.concatenate(dep_ex_mat).astype(None)
    
    col_ex_r = sensor_calibration.color_camera_calibration.extrinsics.rotation
    col_ex_t = sensor_calibration.color_camera_calibration.extrinsics.translation
    col_ex_mat = np.mat([[col_ex_r[0], col_ex_r[1], col_ex_r[2], col_ex_t[0]],
                    [col_ex_r[3], col_ex_r[4], col_ex_r[5], col_ex_t[1]],
                    [col_ex_r[6], col_ex_r[7], col_ex_r[8], col_ex_t[2]],
                    [0, 0, 0, 1]])
    col_ex_mat = np.concatenate(col_ex_mat).astype(None)
    
    trans_mat = col_ex_mat * dep_ex_mat.I
    

    k4a.k4a_playback_close(playback_handle)
