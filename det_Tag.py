import cv2
import apriltag
import numpy as np
import matplotlib.pyplot as plt
import sys, os, math, time
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
import json

pipeline = rs.pipeline()  #定义流程pipeline
config = rs.config()   #定义配置config
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)  #配置depth流
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)   #配置color流
profile = pipeline.start(config)  #流程开始
align_to = rs.stream.color  #与color流对齐
align = rs.align(align_to)

def get_aligned_images():
    frames = pipeline.wait_for_frames()  # 等待获取图像帧
    aligned_frames = align.process(frames)  # 获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的color帧

    ############### 相机参数的获取 #######################
    intr = color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
    camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                         'ppx': intr.ppx, 'ppy': intr.ppy,
                         'height': intr.height, 'width': intr.width,
                         'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                         }
    # 保存内参到本地
    with open('./intrinsics.json', 'w') as fp:
        json.dump(camera_parameters, fp)
    #######################################################

    depth_image = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 深度图（8位）
    depth_image_3d = np.dstack((depth_image_8bit, depth_image_8bit, depth_image_8bit))  # 3通道深度图
    color_image = np.asanyarray(color_frame.get_data())  # RGB图

    # 返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame

def det_apriltag():
    FHD_fx = 914.676513671875
    FHD_fy = 912.8101196289062
    FHD_cx = 645.1201171875
    FHD_cy = 372.2779541015625
    K = np.array([[FHD_fx, 0., FHD_cx],
                  [0., FHD_fy, FHD_cy],
                  [0., 0., 1.]])
    K1 = np.array([FHD_fx, FHD_fy, FHD_cx, FHD_cy])
    id_root = 9
    id_object = 10
    tag_len = 4.15  # centmeters (smaller size)
    rootTrootside = np.identity(4)
    rootsideTcam = np.identity(4)
    camTobjside = np.identity(4)
    objsideTobj = np.identity(4)
    while True:
        intr, depth_intrin, img, depth, aligned_depth_frame = get_aligned_images()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        at_detactor = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
        tags = at_detactor.detect(gray)
        # print(tags)
        for tag in tags:
            H = tag.homography
            # print(H)
            num, Rs, Ts, Ns = cv2.decomposeHomographyMat(H, K)
            r = R.from_matrix(Rs[3].T)
            # print(Ts[0])
            eulerangle = r.as_euler("xyz").T * 180 / math.pi
            # print(eulerangle)
            # print(Ts)
            # print("num: {}".format(num), "Rs:{}".format(Rs), "Ts:{}".format(Ts), "Ns:{}".format(Ns))
            for i in range(4):
                cv2.circle(img, tuple(tag.corners[i].astype(int)), 4, (255, 0, 0), 2)
                # print("corner:{}".format(i), tag.corners[i])
            # dis02 = np.linalg.norm(tag.corners[0] - tag.corners[2])
            # real_z = (zed_focal * tag_len) / dis02
            # print(dis03)
            # cv2.circle(img, tuple(tag.center.astype(int)), 4, (2, 180, 200), 4)
            # if tag.tag_id == id_root:
            #     root_real_z = np.copy(real_z)
            #     root_center_x = np.copy(np.linalg.norm(tag.center[0] - zed_center_x))
            #     root_center_y = np.copy(np.linalg.norm(tag.center[1] - zed_center_y))
            #     # print("center", tag.center)
            #     root_real_x = (root_center_x * real_z) / zed_focal
            #     root_real_y = (root_center_y * real_z) / zed_focal
            # elif tag.tag_id == id_object:
            #     obj_real_z = np.copy(real_z)
            #     obj_center_x = np.copy(np.linalg.norm(tag.center[0] - zed_center_x))
            #     obj_center_y = np.copy(np.linalg.norm(tag.center[1] - zed_center_y))
            #     obj_real_x = (obj_center_x * real_z) / zed_focal
            #     obj_real_y = (obj_center_y * real_z) / zed_focal
            # print(tag.tag_id)
            # dis_root_obj = np.linalg.norm(np.array([obj_real_x, obj_real_y, obj_real_z]) - np.array([root_real_x, root_real_y, root_real_z]))
            # print(dis_root_obj)

            # dis03 = np.linalg.norm(tag.center[0] - tag.center[0])
            # ---------------------------------------------------
            M, e1, e2 = at_detactor.detection_pose(tag, K1)
            # print(M)
            P = M[:3, :4]
            _t = M[:3, 3]
            t = tag_len * _t
            P = np.matmul(K, P)
            # print(P)
            z = np.matmul(P, np.array([[0], [0], [-1], [1]]))
            # print(z)
            z = z / z[2]
            x = np.matmul(P, np.array([[1], [0], [0], [1]]))
            x = x / x[2]
            y = np.matmul(P, np.array([[0], [-1], [0], [1]]))
            y = y / y[2]
            # y = x[:2].T
            # print(y)
            # print("center", tag.center)
            cv2.line(img, tuple(tag.center.astype(int)), tuple(np.squeeze(x[:2].T, axis=0).astype(int)), (0, 0, 255), 2)
            cv2.line(img, tuple(tag.center.astype(int)), tuple(np.squeeze(y[:2].T, axis=0).astype(int)), (0, 255, 0), 2)
            cv2.line(img, tuple(tag.center.astype(int)), tuple(np.squeeze(z[:2].T, axis=0).astype(int)), (255, 0, 0), 2)
            # -----------------------------------------------------
            # angle_z = eulerangle[2] * math.pi / 180
            # angle_y = eulerangle[1] * math.pi / 180
            # angle_x = eulerangle[0] * math.pi / 180
            # deltax = -math.sin(angle_y) * ARROW_LENGTH
            # deltay = ARROW_LENGTH * math.sin(angle_x)
            # center_z = tag.center + np.array([deltax, deltay])
            M[:3, 3] = t
            if tag.tag_id == id_root:
                root_pos = np.copy(np.squeeze(t.T))
                rootsideTcam = np.linalg.inv(M)
                # print(M)
            elif tag.tag_id == id_object:
                obj_pos = np.copy(np.squeeze(t.T))
                camTobjside = np.copy(M)
            # dis_root_obj = np.linalg.norm(root_pos - obj_pos)
            rootsideTobjside = np.matmul(rootsideTcam, camTobjside)
            rootTobjside = np.matmul(rootTrootside, rootsideTobjside)
            rootTobj = np.matmul(rootTobjside, objsideTobj)
            x = rootsideTobjside[0, 3] / 100
            y = -rootsideTobjside[1, 3] / 100
            z = -rootsideTobjside[2, 3] / 100
            print(x, y, z)
            # print("dis:", dis_root_obj)
            # print(root_pos - obj_pos)

            # # print(newcenter, tag.center)
            # cv2.arrowedLine(img, tuple(tag.center.astype(int)), tuple(center_z.astype(int)), (0, 0, 255), 5)
            # -----------------------------------------------------
        cv2.imshow("camera-image", img)
        if cv2.waitKey(1) & 0xFF == ord("j"):
            i += 1
            n = str(i)
            filename = str("./image" + n + ".jpg")
            cv2.imwrite(filename, img)

        if cv2.waitKey(1) & 0xFF == 27:
            break
    pipeline.stop()
    cv2.destroyAllWindows()
# det_apriltag()



