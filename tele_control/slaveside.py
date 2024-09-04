import pickle
import socket
import sys
import time
from matplotlib import pyplot as plt
import numpy as np
from admittance_controller.core import FT_controller as AdmController
from register_robot import RegisterRobot

import yaml
with open('/home/yi/robotic_manipulation/SoftBodySlicing/config.yml', 'r', encoding="utf-8") as f:
    config = yaml.safe_load(f)  # import config from yaml

traj_name = config['traj_name']

def lowpass_filter(last, cur, ratio):
    new = ratio * last + (1 - ratio) * cur
    return new

s_robot = RegisterRobot("10.42.0.162")

current_time = time.strftime('%Y%m%d%H%M%S', time.localtime())

# the configuration of admittance control for slave robot --------------
s_admcontroller = AdmController(0.5, 1000, 8, 0.01)
admittance_params = np.zeros((3, 3)) # contains acc, vel and pos in xyz derictions
admittance_paramsT = np.zeros((3, 3))
# --------------
init_time = 4
init_q = np.array([0.50788814, -1.45690663,  1.38477117, -1.71768059, -1.50159198,  2.11026955])
s_robot.servoJ(init_q, init_time, 0.05, 500)
print('waiting and enter.')
input()
last_ft = s_robot.getTCPFT()


des_pos, des_euler = s_robot.getToolPos()
ft_rot_th = 0.4
ft_pos_th = 2
_init_delay = True
init_delay_item = 30
control_speed = 1
s_robot.zeroforce()
last_ft = s_robot.getTCPFT()
ratio = 0.8

# slave side socket for recv func -------------
slavercv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
slaveserver_port = 12000
slavercv.bind(('', slaveserver_port))
udp_act = False
# ---------------------------------------------
recv_data = None
print('slave running')

sample_time = []
force_xyz = []
force_rxyz = []
i = 0

# for record
pos_record = np.zeros(6)
force_record = np.zeros(6)

while True:
    time.sleep(0.01)
    # environment force ------------
    env_ft = s_robot.getTCPFT()
    last_ft = lowpass_filter(last_ft, env_ft, ratio)
    # ------------------------------
    # master side info from socket -------------
    try:
        recv_data, serveraddr = slavercv.recvfrom(1024)
        recv_data = pickle.loads(recv_data)
        udp_act = True
        if recv_data.shape[0] == 12: # get force + pos + euler
            human_ft = recv_data[:6]
            des_pos = recv_data[6:9] # depend on what you zip in master side
            des_euler = recv_data[9:]
    except socket.timeout:
        udp_act = False
        print('slave request timed out. check the server in master side')
    # ------------------------------
    # print(last_ft)
    # position_d, rotation_d, admittance_params, admittance_paramsT = s_admcontroller.admittance_control(
    #     desired_position=des_pos,
    #     desired_rotation=des_euler,
    #     FT_data=last_ft,
    #     params_mat=admittance_params,
    #     paramsT_mat=admittance_paramsT)
    position_d = des_pos
    rotation_d = des_euler
    ik_q = s_robot.IK(position_d, rotation_d)
    # print(ik_q, s_robot.getQ())
    s_robot.servoJ(ik_q, 0.08, 0.03, 500)
    # collect data for sending to master side --------------------
    if udp_act is True:
        p_rp, p_rr = s_robot.getToolPos()
        msg_to_master = np.hstack((last_ft, p_rp, p_rr))
        message = pickle.dumps(msg_to_master)
        slavercv.sendto(message, serveraddr)

    # --------------------------------------
    p_h = np.hstack([p_rp, p_rr])
    pos_record = np.vstack([pos_record, p_h])
    force_record = np.vstack([force_record, last_ft])
    np.save('data/' + current_time + 'slavepos' + traj_name, pos_record)
    np.save('data/' + current_time + 'slaveforce' + traj_name, force_record)
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






