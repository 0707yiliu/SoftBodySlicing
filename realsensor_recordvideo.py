'''
Using realsense camera to record video
'''

import time
import pyrealsense2 as rs
import numpy as np
import cv2


import yaml
with open('config.yml', 'r', encoding="utf-8") as f:
    config = yaml.safe_load(f) # import config from yaml

class Camera(object):
    '''
    realsense class
    '''

    def __init__(self, width=1280, height=720, fps=30):
        self.width = width
        self.height = height
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, fps)

        self.pipeline.start(self.config)  # connect camera

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()  # get frame(RGB and depth)
        # get align
        align_to = rs.stream.color  # rs.align
        align = rs.align(align_to)  # “align_to”
        aligned_frames = align.process(frames)
        # get aligned frame

        color_frame = aligned_frames.get_color_frame()
        colorizer = rs.colorizer()

        color_image = np.asanyarray(color_frame.get_data())
        return color_image

    def release(self):
        self.pipeline.stop()


if __name__ == '__main__':
    traj_name = config['traj_name']
    current_time = time.strftime('%Y%m%d%H%M%S', time.localtime())
    # recording path
    video_path = './' + current_time + traj_name + 'video.mp4'
    # init parameters
    fps, w, h = 30, 1920, 1080
    mp4 = cv2.VideoWriter_fourcc(*'mp4v')
    wr = cv2.VideoWriter(video_path, mp4, fps, (w, h), isColor=True)  #

    cam = Camera(w, h, fps)
    print('recording the video press: s, save or out recording press: q')
    flag_V = 0
    while True:
        color_image = cam.get_frame()  # get the frame (RGB and depth)
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', colorizer_depth)
        cv2.imshow('RealSense', color_image)

        # print('ll')
        key = cv2.waitKey(1)
        if key & 0xFF == ord('s'):
            flag_V = 1
        if flag_V == 1:
            wr.write(color_image)  # save RGB frame
            print('recording...')
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            print('Exit...')
            wr.release()
            cam.release()
            break



