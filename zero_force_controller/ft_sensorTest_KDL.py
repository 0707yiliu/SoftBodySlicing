import sys
import rtde_receive
import rtde_control
import time

from matplotlib import pyplot as plt
import numpy as np

from zero_force_controller.core import ZeroForceController
from scipy.spatial.transform import Rotation as R

from kdl_func import KDL_ROBOT

import os

sample_time = []
force_xyz = []
force_rxyz = []
i = 0

rtde_c = rtde_control.RTDEControlInterface("10.42.0.163")
rtde_r = rtde_receive.RTDEReceiveInterface("10.42.0.163")

# ------------- zero force controller -------------
rtde_c.zeroFtSensor()
# print('zero force:', rtde_r.getActualTCPForce())
k_xyz = np.array([0.015, 0.015, 0.015])
# k_xyz = np.array([0, 0, 0])
kr_xyz = np.array([0.5, 0.5, 3])
force_controller = ZeroForceController(k_xyz, kr_xyz, 0.01)
# -------------------------------------------------
# ------- KDL ------------
ur3e_kdl = KDL_ROBOT('../ur3e.urdf', "base_link", "tool0")
# ----------------------
# ! testing part ---------------------------
# joint_angles_curr = rtde_r.getActualQ()
# # print(joint_angles_curr)
# match_angle = np.array([0, 3.14, -0.000550976546946913, -0.01395626485858159, -0.007717911397115529, 0.006361254956573248])
# match_angle = rtde_r.getActualQ()
# # ------------- KDL test ---------------
# f_pos, kdl_f_qua = ur3e_kdl.forward(match_angle)
# print('KDL pos and qua:', f_pos, kdl_f_qua)
# print('ee pos', rtde_r.getActualTCPPose()[:3])
# r_target_rot = R.from_rotvec(rtde_r.getActualTCPPose()[3:], degrees=False)
# r_target_qua = r_target_rot.as_quat()
# print('ee qua:', r_target_qua)
# print('ee q:', joint_angles_curr)
# # f_pos[2] -= 0.01
# ik_q = ur3e_kdl.inverse(rtde_r.getActualQ(), f_pos, kdl_f_qua)
# print('ik q:', ik_q)
# rtde_c.servoJ(ik_q, 0.001, 0.001, 0.5, 0.03, 500)
# ----------------------
# testq = np.array([-0.18713871, -1.54668583,  2.21557594, -1.78615128, -0.38870288,  0.43094718])
# f_pos, kdl_f_qua = ur3e_kdl.forward(testq)
# print('test:', f_pos, kdl_f_qua)
# -----------------------------------------
ratio = 0
last_ft = np.zeros(6)
last_pos = np.array(rtde_r.getActualTCPPose())
def lowpass_filter(last, cur, ratio):
    new = ratio * last + (1 - ratio) * cur
    return new
init_q = np.array([0.50788814, -1.45690663,  1.38477117, -1.71768059, -1.50159198,  2.11026955])
rtde_c.servoJ(init_q, 0.001, 0.001, 3, 0.05, 500)
print('waiting and enter.')
input()
des_pos = np.array(rtde_r.getActualTCPPose())[:3]
des_rot = R.from_rotvec(rtde_r.getActualTCPPose()[3:], degrees=False)
des_euler = des_rot.as_euler('xyz', degrees=False)
truncation_num = 6
des_pos = np.around(des_pos, truncation_num) # truncation for float number
des_euler = np.around(des_euler, truncation_num)
ft_rot_th = 0.4
ft_pos_th = 2
_init_delay = True
init_delay_item = 30
control_speed = 1
rtde_c.zeroFtSensor()
while True:
    # TODO: data cutting for ik with truncation_num (have done a part of it)
    time.sleep(0.01)
    # curr_ft = np.array(rtde_r.getActualTCPForce())
    des_ft = np.zeros(6)
    # # des_ft: zero force control, the desired force/torque are zero, which can be changed when using bi-tele control
    curr_ft = np.around(rtde_r.getActualTCPForce(), truncation_num)
    last_ft = np.around(lowpass_filter(last_ft, curr_ft, ratio), truncation_num)
    err_ft = last_ft - des_ft
    position_d, rotation_d = force_controller.zeroforce_control(
                                                                ft=err_ft,
                                                                desired_position=des_pos,
                                                                desired_rotation=des_euler,
                                                            )
    position_d = np.around(position_d, truncation_num)
    rotation_d = np.around(rotation_d, truncation_num)
    r_target_rot = R.from_euler('xyz', rotation_d, degrees=False)
    r_target_qua = r_target_rot.as_quat()
    cur_q = np.around(rtde_r.getActualQ(), truncation_num)
    ik_q = ur3e_kdl.inverse(cur_q, position_d, r_target_qua)
    ik_q = np.around(ik_q, truncation_num)
    # --------- zero force control (separate as one func)------------
    if np.any(abs(err_ft[3:]) > ft_rot_th) or np.any(abs(err_ft[:3]) > ft_pos_th):
        # for recording the desired posture but not set to des_pos and des_euler
        des_pos = position_d
        des_euler = rotation_d
    # ---------------------------------------------
    if _init_delay is True:
        init_delay_item -= 1
        control_speed = 0.8
        if init_delay_item < 0:
            control_speed = 0.01
            _init_delay = False
    # print('r qua:', r_target_qua)
    # print(init_delay_item)
    # print('ee q:', rtde_r.getActualQ())
    # print('ik q:', ik_q)
    e_q = ik_q - rtde_r.getActualQ()
    if np.any(abs(e_q) > 0.5):
        print(abs(e_q))
        print('move too much!!!!!!!!!!!!! IK problem')
    else:
        # print('moving ?????????')
        rtde_c.servoJ(ik_q, 0.001, 0.001, control_speed, 0.03, 500) # moving func !!!!!!!!!!!!!!!!!!!!!!!!
    # ---------------------------------------------
    try:
        i += 1
        sample_time.append(i)
        # force_xyz.append(np.hstack((position_d, rotation_d)))
        # force_xyz.append(position_d)
        force_xyz.append(last_ft[:3])
        plt.clf()  # 清除之前画的图
        plt.plot(sample_time, force_xyz)  # 画出当前x列表和y列表中的值的图形
        plt.pause(0.001)  # 暂停一段时间，不然画的太快会卡住显示不出来
        plt.ioff()  # 关闭画图窗口

    except KeyboardInterrupt:
        time.sleep(1)
        print('over')
        plt.clf()
        plt.close()
        sys.exit(0)
