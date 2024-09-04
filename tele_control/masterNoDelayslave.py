import pickle
import sys
import time
from matplotlib import pyplot as plt
import numpy as np
from zero_force_controller.core import ZeroForceController
from admittance_controller.core import FT_controller as AdmController
from register_robot import RegisterRobot
import socket

import threading

import yaml
with open('/home/yi/robotic_manipulation/SoftBodySlicing/config.yml', 'r', encoding="utf-8") as f:
    config = yaml.safe_load(f)  # import config from yaml
traj_name = config['traj_name']

m_robot = RegisterRobot("10.42.0.163")
s_robot = RegisterRobot("10.42.0.162")

current_time = time.strftime('%Y%m%d%H%M%S', time.localtime())

# the configuration of zeroforce control for master robot ------------
k_xyz = np.array([0.02, 0.02, 0.02])
# k_xyz = np.array([0, 0, 0])
kr_xyz = np.array([3, 3, 3])
control_time = 0.01
m_zfcontroller = ZeroForceController(k_xyz, kr_xyz, control_time)
# --------------

# the configuration of admittance control for slave robot --------------
s_admcontroller = AdmController(0.5, 1000, 8, 0.01)
admittance_params = np.zeros((3, 3)) # contains acc, vel and pos in xyz derictions
admittance_paramsT = np.zeros((3, 3))
# --------------

def lowpass_filter(last, cur, ratio):
    new = ratio * last + (1 - ratio) * cur
    return new
# --------- init pos -------------
init_time = 4
init_q = np.array([0.50788814, -1.45690663,  1.38477117, -1.71768059, -1.50159198,  2.11026955])
m_robot.servoJ(init_q, init_time, 0.05, 500)
s_robot.servoJ(init_q, init_time, 0.05, 500)
print('waiting and enter.')
input()
# ---------------------------
# ---------- params init ------------
m_des_pos, m_des_euler = m_robot.getToolPos()
s_des_pos, s_des_euler = s_robot.getToolPos()
ft_rot_th = 0.4
ft_pos_th = 2
_init_delay = True
init_delay_item = 30
control_speed = 1
m_robot.zeroforce()
s_robot.zeroforce()
m_last_ft = m_robot.getTCPFT()
s_last_ft = s_robot.getTCPFT()
ratio = 0.6
strength = 3

sample_time = []
force_xyz = []
force_rxyz = []
i = 0

# for record
pos_record = np.zeros(6)
force_record = np.zeros(6)
s_env_ft = np.zeros(6) # it would be changed by slave side, to env_ft

def slaveside():
    global m_des_pos, m_des_euler, s_env_ft
    pos_record = np.zeros(6)
    force_record = np.zeros(6)
    while True:
        s_env_ft = s_robot.getTCPFT()
        last_ft = lowpass_filter(s_last_ft, s_env_ft, ratio)
        # s_position_d = m_des_pos
        # s_rotation_d = m_des_euler
        ik_q = s_robot.IK(m_des_pos, m_des_euler)
        s_robot.servoJ(ik_q, 0.08, 0.03, 500)
        p_rp, p_rr = s_robot.getToolPos()
        p_h = np.hstack([p_rp, p_rr])
        pos_record = np.vstack([pos_record, p_h])
        force_record = np.vstack([force_record, last_ft])
        np.save('data/' + current_time + 'slavepos' + traj_name, pos_record)
        np.save('data/' + current_time + 'slaveforce' + traj_name, force_record)
        time.sleep(control_time)


while True:
    last_time = time.time()
    time.sleep(control_time/2)
    # get the info from slave side (force and pos) ---------------
    # try:
    #     recv_data, server = sendtoslave.recvfrom(1024)
    #     recv_data = pickle.loads(recv_data)
    #     if recv_data.shape[0] == 12:
    #         env_ft = recv_data[:6] / 2.5
    #         robot_pos = recv_data[6:9]
    #         robot_euler = recv_data[9:]
    # except socket.timeout:
    #     print('master request timed out, check the server in the slave side')
    # ------------------------------------
    curr_ft = m_robot.getTCPFT()
    last_ft = lowpass_filter(m_last_ft, curr_ft, ratio)
    err_ft = last_ft + s_env_ft / strength
    position_d, rotation_d, dp, dr = m_zfcontroller.zeroforce_control(
        ft=err_ft,
        desired_position=m_des_pos,
        desired_rotation=m_des_euler,
    )
    ik_q = m_robot.IK(position_d, rotation_d)
    if _init_delay is True:
        init_delay_item -= 1
        control_speed = 0.8
        if init_delay_item < 0:
            control_speed = 0.005
            _init_delay = False
            thread = threading.Thread(target=slaveside)
            thread.start()
            print('slave side is activated!')
    m_robot.servoJ(ik_q, control_speed, 0.05, 200)
    # ---------------------------------------------
    time.sleep(control_time/2)
    m_des_pos, m_des_euler = m_robot.getToolPos()
    # print('time:', time.time() - last_time)
