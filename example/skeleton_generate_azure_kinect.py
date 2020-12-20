
"""
This is a python port of the simple-sample project from.
    https://github.com/microsoft/Azure-Kinect-Samples/blob/bf2f8cf95d969dcc7842c4c450052fe5a943c756/body-tracking-samples/simple_sample/main.c
"""

import traceback
import sys
import ctypes
import os
import time
import json

# Add .. to the import path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import k4a
import numpy as np

# img in azure kinect
img_w = 1920
img_h = 1080


toolbar_width = 20

# get video list
video_root = 'G:/dataset/tmp/'
output_root = 'G:/dataset/skeleton_rgbd/'
video_list = []
label_index_map = {'walking':0, 'installing':1, 'waving':2, 'stopping':3, 'picking':4}
js_label_dict = {}

def VERIFY(result, error):
    if result != k4a.K4A_RESULT_SUCCEEDED:
        print(error)
        traceback.print_stack()
        sys.exit(1)

def body_information(body):
    # print("Body ID: {}".format(body.id))
    multi_pose = []
    for i in range(k4a.K4ABT_JOINT_COUNT):
        position = body.skeleton.joints[i].position
        # orientation = body.skeleton.joints[i].orientation
        # confidence_level = body.skeleton.joints[i].confidence_level
        multi_pose.append([position.v[0], position.v[1], position.v[2]])
        # print("Joint[{}]: Position[mm] ( {}, {}, {} ); Orientation ( {}, {}, {}, {}); Confidence Level ({})".format(
        #     i, position.v[0], position.v[1], position.v[2], orientation.v[0], orientation.v[1], orientation.v[2], orientation.v[3], confidence_level))
    return multi_pose


def print_body_index_map_middle_line(body_index_map):
    print("print_body_index_map_middle_line not implemented")
    """
    uint8_t* body_index_map_buffer = k4a_image_get_buffer(body_index_map);

    // Given body_index_map pixel type should be uint8, the stride_byte should be the same as width
    // TODO: Since there is no API to query the byte-per-pixel information, we have to compare the width and stride to
    // know the information. We should replace this assert with proper byte-per-pixel query once the API is provided by
    // K4A SDK.
    assert(k4a_image_get_stride_bytes(body_index_map) == k4a_image_get_width_pixels(body_index_map));

    int middle_line_num = k4a_image_get_height_pixels(body_index_map) / 2;
    body_index_map_buffer = body_index_map_buffer + middle_line_num * k4a_image_get_width_pixels(body_index_map);

    printf("BodyIndexMap at Line %d:\n", middle_line_num);
    for (int i = 0; i < k4a_image_get_width_pixels(body_index_map); i++)
    {
        printf("%u, ", *body_index_map_buffer);
        body_index_map_buffer++;
    }
    printf("\n");
    """


def print_toolbar(rate, annotation=''):
    # setup toolbar
    sys.stdout.write("{}[".format(annotation))
    for i in range(toolbar_width):
        if i * 1.0 / toolbar_width > rate:
            sys.stdout.write(' ')
        else:
            sys.stdout.write('-')
        sys.stdout.flush()
    sys.stdout.write(']\r')

def end_toolbar():
    sys.stdout.write("\n")

if __name__ == "__main__":
    for home, dirs, files in os.walk(video_root):
        for filename in files:
            video_list.append(os.path.join(home, filename))
    
    playback_handle = k4a.k4a_playback_t()
    for v_i, video_dir in enumerate(video_list):
        
        print_toolbar(v_i * 1.0 / len(video_list),
                      '({:>5}/{:<5}) Processing data: '.format(
                          v_i + 1, len(video_list)))

        video_name = video_dir.split('/')[-1].split('.')[0]
        label_name = video_dir.split('/')[-1].split('_')[0]

        # open file
        VERIFY(k4a.k4a_playback_open(ctypes.c_char_p(bytes(video_dir, encoding='utf8')), ctypes.byref(playback_handle)), "Cannot open recording {}!".format(video_dir))

        sensor_calibration = k4a.k4a_calibration_t()
        VERIFY(k4a.k4a_playback_get_calibration(playback_handle, ctypes.byref(sensor_calibration)), "Get depth camera calibration failed!")
        
        tracker = k4a.k4abt_tracker_t()
        tracker_config = k4a.K4ABT_TRACKER_CONFIG_DEFAULT
        VERIFY(k4a.k4abt_tracker_create(ctypes.byref(sensor_calibration), tracker_config, ctypes.byref(tracker)), "Body tracker initialization failed!")

        js_dict = {'data':[], 'label': label_name,'label_index':label_index_map[label_name]}
        if video_name not in js_label_dict.keys():
            js_label_dict[video_name] = {"has_skeleton": True, "label": label_name, "label_index": label_index_map[label_name]}


        frame_count = 0
        while frame_count < 300:
            sensor_capture = k4a.k4a_capture_t()
            stream_result = k4a.k4a_playback_get_next_capture(playback_handle,ctypes.byref(sensor_capture))
            if stream_result == k4a.K4A_STREAM_RESULT_EOF:
                print("Error! Get depth frame time out!")
                break
            if stream_result == k4a.K4A_STREAM_RESULT_SUCCEEDED:
                frame_count+=1
                # print("Start processing frame {}".format(frame_count))

                depth = k4a.k4a_capture_get_depth_image(sensor_capture)
                if depth is not None:
                    k4a.k4a_image_release(depth)
                    
                    queue_capture_result = k4a.k4abt_tracker_enqueue_capture(tracker, sensor_capture, k4a.K4A_WAIT_INFINITE)

                    k4a.k4a_capture_release(sensor_capture)

                    if queue_capture_result == k4a.K4A_WAIT_RESULT_TIMEOUT:
                        # It should never hit timeout when K4A_WAIT_INFINITE is set.
                        print("Error! Add capture to tracker process queue timeout!")
                        break
                    elif queue_capture_result == k4a.K4A_WAIT_RESULT_FAILED:
                        print("Error! Add capture to tracker process queue failed!")
                        break

                    body_frame = k4a.k4abt_frame_t()
                    pop_frame_result = k4a.k4abt_tracker_pop_result(tracker, ctypes.byref(body_frame), k4a.K4A_WAIT_INFINITE)
                    if pop_frame_result == k4a.K4A_WAIT_RESULT_SUCCEEDED:
                        num_bodies = k4a.k4abt_frame_get_num_bodies(body_frame)
                        # print("{} bodies are detected!".format(num_bodies))
                        timestamp = k4a.k4abt_frame_get_device_timestamp_usec(body_frame)
                        # print("{} timestamp are detected!".format(timestamp))
                        if num_bodies:
                            multi_pose = np.zeros((num_bodies, k4a.K4ABT_JOINT_COUNT, 3))
                            for i in range(num_bodies):
                                body = k4a.k4abt_body_t()
                                VERIFY(k4a.k4abt_frame_get_body_skeleton(body_frame, i, ctypes.byref(body.skeleton)), "Get body from body frame failed!")
                                body.id = k4a.k4abt_frame_get_body_id(body_frame, i)

                                multi_pose[i, :, :] = body_information(body)

                            # normalization
                            center_pelvis = multi_pose[0,0,:]
                            multi_pose[0, :, :] = multi_pose[0, :, :] - center_pelvis
                            # print(np.shape(center_pelvis))
                            multi_pose[:, :, 0] = multi_pose[:, :, 0] / img_w + 0.5
                            multi_pose[:, :, 1] = multi_pose[:, :, 1] / img_h + 0.5
                            multi_pose[:, :, 2] = multi_pose[:, :, 2] / img_h + 0.5
                        else:
                            multi_pose = np.zeros((1, k4a.K4ABT_JOINT_COUNT, 3))

                        # import pdb; pdb.set_trace()
                        pose_list = [round(x, 3) for x in multi_pose[0, :, :].reshape(-1).tolist()]
                        js_item = {'frame_index':frame_count, 'skeleton':{'pose':pose_list}}
                        js_dict['data'].append(js_item)
                        
                        # body_index_map = k4a.k4abt_frame_get_body_index_map(body_frame)
                        # if body_index_map:
                        #     print_body_index_map_middle_line(body_index_map)
                        #     k4a.k4a_image_release(body_index_map)
                        # else:
                        #     print("Error: Fail to generate bodyindex map!")

                        k4a.k4abt_frame_release(body_frame)
                    elif pop_frame_result == k4a.K4A_WAIT_RESULT_TIMEOUT:
                        # It should never hit timeout when K4A_WAIT_INFINITE is set.
                        print("Error! Pop body frame result timeout!")
                        break
                    else:
                        print("Pop body frame result failed!")
                        break
                else:
                    print("Get depth capture returned error!")
            else:
                print("Get offline next capture returned error: {}".format(stream_result))
        k4a.k4abt_tracker_shutdown(tracker)
        k4a.k4abt_tracker_destroy(tracker)
        k4a.k4a_playback_close(playback_handle)
        # print("There are {} frames in video {}".format(frame_count + 1, video_dir))
        save_path = os.path.join(output_root, 'skeleton_data')
        if not os.path.exists(save_path):
            os.makedirs(save_path)
        with open(os.path.join(save_path, video_name+'.json'), 'w') as f:
            json.dump(js_dict, f)
        # print('{} done. {}|{}'.format(video_name, v_i, len(video_list)))
        # print("Finished body tracking processing!")
        

    with open(os.path.join(output_root, 'skeleton_label.json'), 'w') as f:
        json.dump(js_label_dict, f)

    end_toolbar()

                        
