import sys

import rtde_receive
import rtde_control
import time

from matplotlib import pyplot as plt
import numpy as np

from admittance_controller.core import FT_controller as AdmController
from ur_ikfast import ur_kinematics
from scipy.spatial.transform import Rotation as R

ur_arm = ur_kinematics.URKinematics('ur3e')

sample_time = []
force_xyz = []
force_rxyz = []
i = 0

rtde_c = rtde_control.RTDEControlInterface("10.42.0.162")
rtde_r = rtde_receive.RTDEReceiveInterface("10.42.0.162")
actual_q = rtde_r.getActualQ()
# print('q:', actual_q)
rtde_c.zeroFtSensor()
# print('zero force:', rtde_r.getActualTCPForce())
force_controller = AdmController(0.1, 1, 100, 0.01)
admittance_params = np.zeros((3, 3)) # contains acc, vel and pos in xyz derictions
admittance_paramsT = np.zeros((3, 3))
# [0.000947074091527611, -0.012041763668396044, -0.0005254745483398438, -0.013991789226867724, -0.007697884236470998, 0.006365747656673193]
# ikfast pos: [0.45735733 0.37369277 0.07264649]
# ikfast qua: [ 0.70711017  0.70702009  0.00444449 -0.00990112]
# ikfast euler: [-179.55771415   -1.16238521   89.98821334]
# ee pos [-0.45680789512679115, -0.3750043626147314, 0.07476524229379196]
# ee qua: [ 0.70454861  0.01078779 -0.00346276  0.70956528]
# ee euler: [89.5994319   1.1568022   0.58952851]
joint_angles_curr = rtde_r.getActualQ()
print(joint_angles_curr)
match_angle = np.array([0, 3.14, -0.000550976546946913, -0.01395626485858159, -0.007717911397115529, 0.006361254956573248])
target_pos = ur_arm.forward(match_angle, ee_vec=np.array([0, 0, 0.1507]))
target_qua = np.roll(target_pos[3:], -1)
r_target_qua = R.from_quat(target_qua)
r_target_euler = r_target_qua.as_euler('xyz', degrees=True)
r_target_rot = R.from_rotvec(rtde_r.getActualTCPPose()[3:], degrees=False)
r_target_qua = r_target_rot.as_quat()
r_ee_euler = r_target_rot.as_euler('xyz', degrees=True)
# print('q:', joint_angles_curr)
print('ikfast pos:', target_pos[:3])
print('ikfast qua:', target_qua)
print('ikfast euler:', r_target_euler)
print('ee pos', rtde_r.getActualTCPPose()[:3])
print('ee qua:', r_target_qua)
print('ee euler:', r_ee_euler)

# joint_angles_curr = rtde_r.getActualQ()
# target_pos = ur_arm.forward(joint_angles_curr, ee_vec=np.array([0, 0, 0.1507]))
# ori_zpos = np.copy(target_pos[2])
# target_ideal_rot = [-89.5, 0.1, 0.1]
# r_target_rot = R.from_euler('xyz', target_ideal_rot, degrees=True)
# r_target_qua = r_target_rot.as_quat()
# print('test qua:', r_target_qua)
# target_eepos = np.concatenate((target_pos[:3], np.roll(r_target_qua, 1)))
# target_q = ur_arm.inverse(ee_pose=target_eepos, ee_vec=np.array([0, 0, 0.1507]),
#                           all_solutions=False, q_guess=joint_angles_curr).tolist()
# print(target_q)
# print(joint_angles_curr)
#
# # grasp_q = [0.05079088360071182, -1.1178493958762665, 1.5329473654376429, -1.984063287774557, -1.5724676291095179,
# #            0.04206418991088867]
# # rtde_c.servoJ(grasp_q, 0.001, 0.001, 20, 0.03, 500)
#
# # target_pos[2] -= 0.01
# target_eepos = np.concatenate((target_pos[:3], np.roll(r_target_qua, 1)))
# target_q = ur_arm.inverse(ee_pose=target_eepos, ee_vec=np.array([0, 0, 0.1507]),
#                         all_solutions=False, q_guess=joint_angles_curr).tolist()
# print('ee q:', joint_angles_curr)
# print('ik q:', target_q)
# print(ur_arm.forward(target_q, ee_vec=np.array([0, 0, 0.1507])))
# # rtde_c.servoJ(target_q, 0.001, 0.001, 100, 0.03, 500)
# ratio = 0.7
# last_ft = np.zeros(6)
# last_pos = np.array(rtde_r.getActualTCPPose())
# def lowpass_filter(last, cur, ratio):
#     new = ratio * last + (1 - ratio) * cur
#     return new

# while True:
#     time.sleep(0.01)
#     curr_ft = np.array(rtde_r.getActualTCPForce())
#     curr_pos = np.array(rtde_r.getActualTCPPose())
#     # print(last_ft, curr_ft)
#     last_ft = lowpass_filter(last_ft, curr_ft, ratio)
#     last_pos = lowpass_filter(last_pos, curr_pos, ratio)
#     position_d, rotation_d, admittance_params, admittance_paramsT = force_controller.admittance_control(desired_position=last_pos[:3],
#                                               desired_rotation=last_pos[3:],
#                                               FT_data=last_ft,
#                                               params_mat=admittance_params,
#                                               paramsT_mat=admittance_paramsT)
#     r_target_rot = R.from_euler('xyz', rotation_d, degrees=False)
#     r_target_qua = r_target_rot.as_quat()
#     # print(position_d, rotation_d, admittance_params, admittance_paramsT)
#     # print(rtde_r.getActualQ())
#     try:
#         i += 1
#         sample_time.append(i)
#         force_xyz.append(np.hstack((position_d,rotation_d)))
#         plt.clf()  # 清除之前画的图
#         plt.plot(sample_time, force_xyz)  # 画出当前x列表和y列表中的值的图形
#         plt.pause(0.001)  # 暂停一段时间，不然画的太快会卡住显示不出来
#         plt.ioff()  # 关闭画图窗口
#
#     except KeyboardInterrupt:
#         time.sleep(1)
#         print('over')
#         plt.clf()
#         plt.close()
#         sys.exit(0)

# while True:
#     rtde_c.teachMode()

