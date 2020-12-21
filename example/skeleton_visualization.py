import json
import numpy as np
import matplotlib.pyplot as plt
import cv2

max_body = 2
num_joint = 25
max_frame = 300

# img in azure kinect
img_w = 1920
img_h = 1080
frame_id = 10

file_json = 'G:/dataset/action_skeleton_rgbd/skeleton_data/installing_1.json'
file_mkv = 'G:/dataset/tmp/installing_1.mkv'

azure_kinect_id_list = np.array([1, 2, 4, 27, 6, 7, 8, 9, 13, 14, 15, 16, 
19, 20, 21, 22, 23, 24, 25, 26, 3, 10, 11, 17, 18]) - 1

edge_group = np.array([[1, 2], [2, 21], [21, 3], [3, 4], [21, 5], [5, 6], [6, 7], [7, 8], [8, 22],
[7, 23], [21, 9], [9, 10], [10, 11], [11, 12], [12, 24], [11, 25], [1, 13], [13, 14], [14, 15],
[15, 16], [1, 17], [17, 18], [18, 19], [19, 20]]) - 1


cap = cv2.VideoCapture(file_mkv)

with open(file_json, 'r') as f:
    
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_id)
    ret, frame = cap.read()
    frame = frame[:,:,(2,1,0)]
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)

    video_info = json.load(f)

    # fill data_numpy
    data_numpy = np.zeros((3, max_frame, num_joint, max_body))
    label = video_info['label_index']
    for frame_info in video_info['data']:

        frame_index = frame_info['frame_index']
        if frame_index == frame_id:
            pose = np.array(frame_info["skeleton"]['pose']).reshape(32, 3)
            new_pose = pose[azure_kinect_id_list]

            data_numpy[0, frame_index, :, 0] = new_pose[:, 0]
            data_numpy[1, frame_index, :, 0] = new_pose[:, 1]
            data_numpy[2, frame_index, :, 0] = new_pose[:, 2]
    
    x_list = ((new_pose[:,0] - 0.5) * img_w + img_w / 2) / 2 + 200
    y_list = ((new_pose[:,1] - 0.5) * img_h + img_h / 2) / 2 + 200
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

    