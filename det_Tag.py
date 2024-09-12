# MODEL: AprilTag markers detection and camera calibration (ZED Python API with cv2 lib)
# AUTHOR: Yi Liu @AiRO
# UNIVERSITY: UGent-imec
# DEPARTMENT: Faculty of Engineering and Architecture
# Control Engineering / Automation Engineering

import cv2
import apriltag
import numpy as np
import matplotlib.pyplot as plt
import sys, os, math, time
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
import json


def mkdir(path):
    folder = os.path.exists(path)
    if not folder:
        os.makedirs(path)
    else:
        pass

class AprilTagDet:
    def __init__(self, rootid=9, objid=10, enable_recording=False, path=None, render=False) -> None:
        self.pipeline = rs.pipeline() # define the pipeline
        self.config = rs.config() # define the configuration
        self.config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30) # config the color stream
        self.profile = self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        self.render = render

        # self.FHD_fx = 914.676513671875 # 1280,720
        # self.FHD_fy = 912.8101196289062
        # self.FHD_cx = 645.1201171875
        # self.FHD_cy = 372.2779541015625

        self.FHD_fx = 1372.0147705078125 # 1920, 1080
        self.FHD_fy = 1369.2152099609375
        self.FHD_cx = 967.6801147460938
        self.FHD_cy = 558.4169311523438
        self.enable_record = enable_recording
        self.recording_count = 0
        self.recording_path = path

        if self.enable_record is True:
            fps, w, h = 30, 1920, 1080
            mp4 = cv2.VideoWriter_fourcc(*'mp4v')
            self.wr = cv2.VideoWriter(self.recording_path, mp4, fps, (w, h), isColor=True)

        print("complete the RealSense initialization.")

        self.K = np.array([[self.FHD_fx, 0., self.FHD_cx],
                           [0., self.FHD_fy, self.FHD_cy],
                           [0., 0., 1.]])
        self.K1 = np.array([self.FHD_fx, self.FHD_fy, self.FHD_cx, self.FHD_cy])

        self.id_root = rootid
        self.id_object = objid
        self.tag_len = 4.15
        self.tag_outer_side = 1.02
        obj_offset_x = 0
        obj_offset_y = 7.66
        obj_offset_z = 0
        root_z_offset = 2.03
        root_base_x = 13.1
        root_base_y = 13.1

        self.rootTobj = np.identity(4)
        self.rootTrootside = np.identity(4)
        self.rootsideTcam = np.identity(4)
        self.camTobjside = np.identity(4)
        self.objsideTobj = np.identity(4)

        self.rootTrootside[0, 3] = ((root_base_x / 2) - (self.tag_len / 2 + self.tag_outer_side))
        self.rootTrootside[1, 3] = (self.tag_len / 2 + self.tag_outer_side + root_base_y / 2)
        self.rootTrootside[2, 3] = root_z_offset
        self.objsideTobj[0, 3] = -obj_offset_x
        self.objsideTobj[1, 3] = -obj_offset_y
        self.objsideTobj[2, 3] = -obj_offset_z

        self.x = 0
        self.y = 0
        self.z = 0

        self.output = np.zeros(6)

    def get_aligned_images(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        # intr = color_frame.profile.as_video_stream_profile().intrinsics
        # camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
        #                      'ppx': intr.ppx, 'ppy': intr.ppy,
        #                      'height': intr.height, 'width': intr.width,
        #                      'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
        #                      }
        color_image = np.asanyarray(color_frame.get_data())
        return color_image

    def robot2tag(self) -> np.ndarray:
        img = self.get_aligned_images()
        cv2.waitKey(1)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        at_detactor = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
        tags = at_detactor.detect(gray)

        for tag in tags:
            if self.render is True:
                H = tag.homography
                num, Rs, Ts, Ns = cv2.decomposeHomographyMat(H, self.K)
                r = R.from_matrix(Rs[3].T)
                for i in range(4):
                    cv2.circle(img, tuple(tag.corners[i].astype(int)), 4, (255, 0, 0), 2)

            M, e1, e2 = at_detactor.detection_pose(tag, self.K1)
            P = M[:3, :4]
            _t = M[:3, 3]
            t = self.tag_len * _t
            P = np.matmul(self.K, P)
            self.z = np.matmul(P, np.array([[0], [0], [-1], [1]]))
            # print(z)
            self.z = self.z / self.z[2]
            self.x = np.matmul(P, np.array([[1], [0], [0], [1]]))
            self.x = self.x / self.x[2]
            self.y = np.matmul(P, np.array([[0], [-1], [0], [1]]))
            self.y = self.y / self.y[2]
            if self.render is True:
                cv2.line(img, tuple(tag.center.astype(int)), tuple(np.squeeze(self.x[:2].T, axis=0).astype(int)), (0, 0, 255), 2)
                cv2.line(img, tuple(tag.center.astype(int)), tuple(np.squeeze(self.y[:2].T, axis=0).astype(int)), (0, 255, 0), 2)
                cv2.line(img, tuple(tag.center.astype(int)), tuple(np.squeeze(self.z[:2].T, axis=0).astype(int)), (255, 0, 0), 2)

            M[:3, 3] = t
            if tag.tag_id == self.id_root:
                self.rootsideTcam = np.linalg.inv(M)
            elif tag.tag_id == self.id_object:
                self.camTobjside = np.copy(M)
            rootsideTobjside = np.matmul(self.rootsideTcam, self.camTobjside)
            rootTobjside = np.matmul(self.rootTrootside, rootsideTobjside)
            self.rootTobj = np.matmul(rootTobjside, self.objsideTobj)
            # now we just output x, y, z (unit: meter)
            self.x = self.rootTobj[0, 3] / 100
            self.y = -self.rootTobj[1, 3] / 100
            self.z = -self.rootTobj[2, 3] / 100
            # print(self.rootTobj[:3, :3])
            # [[ 0.99989305 -0.00309032 -0.01429449]
            #  [ 0.00284012  0.999843   -0.01749017]
            #  [ 0.0143463   0.0174477   0.99974485]]
            r = R.from_matrix(self.rootTobj[:3, :3])
            rot = r.as_rotvec()
            self.output = np.hstack([np.array([self.x, self.y, self.z]), rot])

        if self.enable_record is True:
            self.wr.write(img)
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('RealSense', colorizer_depth)
            cv2.imshow('RealSense', img)
            print('detTag recording...')
        # if self.render is True:
        #     cv2.imshow("camera-image", img)
        #     if cv2.waitKey(1) & 0xFF == ord("j"):
        #         i += 1
        #         n = str(i)
        #         filename = str("./image" + n + ".jpg")
        #         cv2.imwrite(filename, img)
        return self.output



