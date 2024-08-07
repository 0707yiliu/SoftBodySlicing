import pickle
import sys
import time
from matplotlib import pyplot as plt
import numpy as np
from zero_force_controller.core import ZeroForceController
from register_robot import RegisterRobot
import socket

m_robot = RegisterRobot("10.42.0.163")

current_time = time.strftime('%Y%m%d%H%M%S', time.localtime())

# the configuration of zeroforce control for master robot ------------
k_xyz = np.array([0.008, 0.008, 0.008])
# k_xyz = np.array([0, 0, 0])
kr_xyz = np.array([0.5, 0.5, 3])
m_zfcontroller = ZeroForceController(k_xyz, kr_xyz, 0.01)
# --------------

def lowpass_filter(last, cur, ratio):
    new = ratio * last + (1 - ratio) * cur
    return new
# --------- init pos -------------
init_time = 4
init_q = np.array([0.50788814, -1.45690663,  1.38477117, -1.71768059, -1.50159198,  2.11026955])
m_robot.servoJ(init_q, init_time, 0.05, 500)

print('waiting and enter.')
input()
# ---------------------------
# ---------- params init ------------
des_pos, des_euler = m_robot.getToolPos()
ft_rot_th = 0.4
ft_pos_th = 2
_init_delay = True
init_delay_item = 30
control_speed = 1
m_robot.zeroforce()
last_ft = m_robot.getTCPFT()
ratio = 0.8

sample_time = []
force_xyz = []
force_rxyz = []
i = 0

# for record
pos_record = np.zeros(6)
force_record = np.zeros(6)

env_ft = np.zeros(6) # it would be changed by slave side, to env_ft
# ----------------------------
# ------------ socket (send) init ---------------
# only sending message, TODO: we set the recv func
sendtoslave = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP socket
send_port = 12000
addr = ("127.0.0.1", send_port)
sendtoslave.settimeout(0.1)
# ---------------------------
print('master running')
while True:
    time.sleep(0.01)
    # get the info from slave side (force and pos) ---------------
    try:
        recv_data, server = sendtoslave.recvfrom(1024)
        recv_data = pickle.loads(recv_data)
        if recv_data.shape[0] == 12:
            env_ft = recv_data[:6] / 2
            robot_pos = recv_data[6:9]
            robot_euler = recv_data[9:]
    except socket.timeout:
        print('master request timed out, check the server in the slave side')
    # ------------------------------------
    curr_ft = m_robot.getTCPFT()
    last_ft = lowpass_filter(last_ft, curr_ft, ratio)
    err_ft = last_ft + env_ft
    position_d, rotation_d = m_zfcontroller.zeroforce_control(
        ft=err_ft,
        desired_position=des_pos,
        desired_rotation=des_euler,
    )
    ik_q = m_robot.IK(position_d, rotation_d)
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
    m_robot.servoJ(ik_q, control_speed, 0.03, 500)
    # collect data for sending to slave side --------------
    f_h = m_robot.getTCPFT()
    p_hp, p_hr = m_robot.getToolPos()
    msg_to_slave = np.hstack((f_h, p_hp, p_hr))
    message = pickle.dumps(msg_to_slave)
    sendtoslave.sendto(message, addr)
    # ---------------------------------
    p_h = np.hstack([p_hp, p_hr])
    pos_record = np.vstack([pos_record, p_h])
    force_record = np.vstack([force_record, f_h])
    np.save('data/' + current_time + 'masterpos', pos_record)
    np.save('data/' + current_time + 'masterforce', force_record)
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

