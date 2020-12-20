import os
import sys
import argparse
import cv2
import numpy as np


img_color=3
img_height=180 
img_weidth=320

toolbar_width = 30

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

def split_video(datapath, filename, outpath, img_color=3, img_height=180, img_weidth=320):
    cap = cv2.VideoCapture(os.path.join(datapath, filename))

    img_path = os.path.join(out_path, filename[:-4])
    if not os.path.exists(img_path):
        os.makedirs(img_path)

    cnt = 0
    while True:
        success, data = cap.read()
        print(np.shape(data))
        if not success:
            break
        data = cv2.resize(data, (img_weidth, img_height))
        cv2.imwrite(os.path.join(img_path, filename[:-4]+'_frame_'+str(cnt)+'.jpg'), data)
        # print(os.path.join(img_path, filename[:-4]+'_frame_'+str(cnt)+'.jpg'))
        cnt += 1



def split(rgb_path,
            out_path):

    sample_name = []
    for filename in os.listdir(rgb_path):
        sample_name.append(filename)

    for i, s in enumerate(sample_name):
        print_toolbar(i * 1.0 / len(sample_name),
                      '({:>5}/{:<5}) Processing data: '.format(
                          i + 1, len(sample_name)))
        split_video(
            datapath=rgb_path, filename=s, outpath=out_path, 
            img_color=img_color, img_height=img_height, img_weidth=img_weidth)
    end_toolbar()


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Azure Kinect Data Converter.')
    parser.add_argument(
    '--rgb_path', default='G:/dataset/test/')
    parser.add_argument('--out_folder', default='G:/dataset/test/rgb_imgs')

    arg = parser.parse_args()

    out_path = arg.out_folder
    if not os.path.exists(out_path):
        os.makedirs(out_path)

    split(
        arg.rgb_path,
        out_path)


