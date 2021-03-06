
"""
Visualize body pose with .mkv videos captured by Azure Kinect and skeleton data without normalization (nn)
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import cv2
import traceback
import sys
import ctypes
import os

# Add .. to the import path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import k4a

max_body = 2
num_joint = 25
max_frame = 300

# img in azure kinect
img_w = 1920
img_h = 1080
frame_id = 0

file_json = 'G:/dataset/skeleton_rgbd_h_nn/skeleton_data/picking_v0_p1_h1.json'
file_mkv = 'G:/dataset/tmp_h/picking_v0_p1_h1.mkv'

azure_kinect_id_list = np.array([1, 2, 4, 27, 6, 7, 8, 9, 13, 14, 15, 16, 
19, 20, 21, 22, 23, 24, 25, 26, 3, 10, 11, 17, 18]) - 1

edge_group = np.array([[1, 2], [2, 21], [21, 3], [3, 4], [21, 5], [5, 6], [6, 7], [7, 8], [8, 22],
[7, 23], [21, 9], [9, 10], [10, 11], [11, 12], [12, 24], [11, 25], [1, 13], [13, 14], [14, 15],
[15, 16], [1, 17], [17, 18], [18, 19], [19, 20]]) - 1


cap = cv2.VideoCapture(file_mkv)


def VERIFY(result, error):
    if result != k4a.K4A_RESULT_SUCCEEDED:
        print(error)
        traceback.print_stack()
        sys.exit(1)

if __name__ == "__main__":
    device_config = k4a.K4A_DEVICE_CONFIG_INIT_DISABLE_ALL
    device_config.depth_mode = k4a.K4A_DEPTH_MODE_NFOV_UNBINNED

    # open file
    playback_handle = k4a.k4a_playback_t()
    VERIFY(k4a.k4a_playback_open(ctypes.c_char_p(bytes(file_mkv, encoding='utf8')), ctypes.byref(playback_handle)), "Cannot open recording {}!".format(file_mkv))

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
    # dep_ex_mat = np.mat([[dep_ex_r[0], dep_ex_r[3], dep_ex_r[6], dep_ex_t[0]],
    #                 [dep_ex_r[1], dep_ex_r[4], dep_ex_r[7], dep_ex_t[1]],
    #                 [dep_ex_r[2], dep_ex_r[5], dep_ex_r[8], dep_ex_t[2]],
    #                 [0, 0, 0, 1]])
    dep_ex_mat = np.concatenate(dep_ex_mat).astype(None)
    
    col_ex_r = sensor_calibration.color_camera_calibration.extrinsics.rotation
    col_ex_t = sensor_calibration.color_camera_calibration.extrinsics.translation
    col_ex_mat = np.mat([[col_ex_r[0], col_ex_r[1], col_ex_r[2], col_ex_t[0]],
                    [col_ex_r[3], col_ex_r[4], col_ex_r[5], col_ex_t[1]],
                    [col_ex_r[6], col_ex_r[7], col_ex_r[8], col_ex_t[2]],
                    [0, 0, 0, 1]])
    # col_ex_mat = np.mat([[col_ex_r[0], col_ex_r[3], col_ex_r[6], col_ex_t[0]],
    #                 [col_ex_r[1], col_ex_r[4], col_ex_r[7], col_ex_t[1]],
    #                 [col_ex_r[2], col_ex_r[5], col_ex_r[8], col_ex_t[2]],
    #                 [0, 0, 0, 1]])
    col_ex_mat = np.concatenate(col_ex_mat).astype(None)
    trans_mat = col_ex_mat * dep_ex_mat.I
    
    k4a.k4a_playback_close(playback_handle)

    with open(file_json, 'r') as f:
        
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_id)
        ret, frame = cap.read()
        frame = cv2.resize(frame, (img_w, img_h))
        frame = frame[:,:,(2,1,0)]


        video_info = json.load(f)

        # fill data_numpy
        data_numpy = np.zeros((3, max_frame, num_joint, max_body))
        label = video_info['label_index']
        for frame_info in video_info['data']:

            frame_index = frame_info['frame_index']
            if frame_index == frame_id + 1:
                pose = np.array(frame_info["skeleton"]['pose']).reshape(32, 3)
                new_pose = pose[azure_kinect_id_list]

                data_numpy[0, frame_index - 1, :, 0] = new_pose[:, 0]
                data_numpy[1, frame_index - 1, :, 0] = new_pose[:, 1]
                data_numpy[2, frame_index - 1, :, 0] = new_pose[:, 2]


        pw = new_pose.T
        pw_p = np.zeros((pw.shape[0] + 1, pw.shape[1]))
        pw_p[:pw.shape[0], :pw.shape[1]] = pw
        pd = trans_mat * pw_p
        pd_p = pd[:pd.shape[0]-1, :pd.shape[1]]
        pc = (col_in_mat * pd_p).T
        # import pdb; pdb.set_trace()

        for i in range(25):
            pc[i, :] = pc[i, :] / pc[i, 2]
    
        x_list = np.array(pc[:, 0])
        y_list = np.array(pc[:, 1])

        #创建图并命名
        plt.figure('Scatter fig')
        ax = plt.gca()
        plt.axis([0, img_w, 0, img_h])
        #设置x轴、y轴名称
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.invert_yaxis()  #y轴反向
        
        #画散点图，以x_list中的值为横坐标，以y_list中的值为纵坐标
        #参数c指定点的颜色，s指定点的大小,alpha指定点的透明度
        ax.scatter(x_list, y_list, c='r', s=20, alpha=0.5)
        
        for edge in edge_group:
            plt.plot(x_list[edge], y_list[edge])

        plt.imshow(frame)
        plt.show()

        