# UNIVERSITY: UGent-imec
# AUTHOR: YiLiu
# DEPARTMENT: Faculty of Engineering and Architecture
# Control Engineering / Automation Engineering

import numpy as np
from movement_primitives.dmp import DMP

from tele_control.register_robot import RegisterRobot
from admittance_controller.core import FT_controller as AdmController

import time

import matplotlib.pyplot as plt
import yaml
with open('config.yml', 'r', encoding="utf-8") as f:
    config = yaml.safe_load(f) # import config from yaml

def forgetting(history_memory, memory_strength):
    history_num = history_memory.shape[0]
    history_linspace = np.linspace(0, 1, history_num)
    # _remember_curve = np.e ** (history_linspace / memory_strength)  # from small to large
    _remember_curve = np.e ** (-history_linspace / memory_strength)  # from large to small
    # remember_curve = np.flip(_remember_curve)
    mul_output = np.dot(_remember_curve, history_memory)
    return mul_output

def r_traj_pos(curr_pos, dmp_pos):
    # input:
    #        curr_pos: current ee-pos (numpy array)
    #        dmp_pos: DMP predicted pos (numpy array)
    # output:
    #        r: ratio for pos (numpy array), which is used to by traj_force for admittance controller
    r = curr_pos - dmp_pos # test, fake
    # print(curr_pos.shape, dmp_pos.shape)
    return r

def traj_load(slaveforce, slavepos):
    # loading tele-trajectory from slave datasets for DMPs
    traj_force = np.load('tele_control/data/' + slaveforce + '.npy')
    traj_pos = np.load('tele_control/data/' + slavepos + '.npy')
    traj_pos = np.delete(traj_pos, 0, 0)
    traj_force = np.delete(traj_force, 0, 0)
    return traj_force, traj_pos

def DMP_traj(origin_traj, new_start, new_goal):
    # dynamic movement primitives for force trajectory
    # input:
    #   origin_traj: original trajectory from slave robot
    #   new_start: the new start of the trajectory
    #   new_goal: the new goal of the trajectory
    # output:
    #   Y_dmp
    traj = origin_traj[10:, :]
    Y = traj
    T = np.linspace(0, 10, traj.shape[0])
    dmp = DMP(
        n_dims=traj.shape[1],
        execution_time=traj.shape[0] / 100,
        dt=config['DMPs']['dt'],
        n_weights_per_dim=config['DMPs']['n_weights_per_dim'],
        smooth_scaling=config['DMPs']['smooth_scaling'],
    )
    dmp.imitate(T, Y)
    dmp.configure(start_y=new_start, goal_y=new_goal)
    _, Y_dmp = dmp.open_loop()
    return Y_dmp

current_time = time.strftime('%Y%m%d%H%M%S', time.localtime())
# register the robot
myrobot = RegisterRobot(config['controlled_robot_ip'])
# init robot
init_time = 4
init_q = np.array([0.50788814, -1.45690663,  1.38477117, -1.71768059, -1.50159198,  2.11026955])
myrobot.servoJ(init_q, init_time, 0.05, 500)
print('waiting and enter for cutting automatically.')
input()
myrobot.zeroforce()
# define new_start and new_goal for DMP_traj func
new_start_pos = np.array(config['DMPs']['new_start_pos'])
new_goal_pos = np.array(config['DMPs']['new_goal_pos'])
new_start_force = np.array(config['DMPs']['new_start_force'])
new_goal_force = np.array(config['DMPs']['new_goal_force'])
# get demonstrated traj from traj_load func
slave_force = config['slave_file_date'] + 'slaveforce'
slave_pos = config['slave_file_date'] + 'slavepos'
traj_force, traj_pos = traj_load(slave_force, slave_pos)
# define admittance control for auto-robot
admcontroller = AdmController(m=config['admittance_controller']['mass'],
                              k=config['admittance_controller']['k_pos'],
                              kr=config['admittance_controller']['k_rot'],
                              dt=config['admittance_controller']['dt']) # TODO: TBD
admittance_params = np.zeros((3, 3)) # contains acc, vel and pos in xyz derictions
admittance_paramsT = np.zeros((3, 3))
# get DMP traj
dmp_traj_force = DMP_traj(traj_force, new_start_force, new_goal_force)
dmp_traj_pos = DMP_traj(traj_pos, new_start_pos, new_goal_pos)
# get the ratio from pos diff
curr_pos, curr_euler = myrobot.getToolPos()
# curr_pos = [0, 0, 0, 0, 0, 0]

# drive robot, TODO: the time need to be considered
pos_record = np.array([0,0,0,0,0,0])
desired_force = np.copy(dmp_traj_force[0, :])
force_record = np.array([0, 0, 0, 0, 0, 0])
curr_force_record = np.array([0, 0, 0, 0, 0, 0])
arm_pos_record = np.array([0,0,0, 0,0,0])
memory_num = config['forgetting_err']['memory_number']
memory_strength = config['forgetting_err']['memory_strength'] # the smaller, the stronger nonlinear
memory_strength_rot = config['forgetting_err']['memory_strength_rot']
e_force_mass_coefficient = config['forgetting_err']['to_force_coefficient']
e_torque_mass_coefficient = config['forgetting_err']['to_torque_coefficient']
pos_memory = np.zeros((memory_num, 3))
rot_memory = np.zeros((memory_num, 3))
pos_r_t_1 = np.zeros(3)
pos_r_t_2 = np.zeros(3)
t_sample = config['time_sample']
truncation_num = 4
for i in range(dmp_traj_force.shape[0]):
    time.sleep(t_sample)
    curr_ft = np.around(myrobot.getTCPFT(), truncation_num)
    adm_force = desired_force + curr_ft
    position_d, rotation_d, admittance_params, admittance_paramsT = admcontroller.admittance_control(
            desired_position=dmp_traj_pos[i, :3],# TODO: TBD
            desired_rotation=dmp_traj_pos[i, 3:],# TODO: TBD
            FT_data=adm_force,
            params_mat=admittance_params,
            paramsT_mat=admittance_paramsT)
    # ratio for changing ft_data ---------------
    # r_p = r_traj_pos(position_d, dmp_traj_pos[i, :3])
    # r_r = r_traj_pos(rotation_d, dmp_traj_pos[i, 3:])
    # r = np.concatenate((r_p, r_r))
    # desired_force = (r + 1) * dmp_traj_force[i, :]
    # ------------------------------------------
    # --------- forgetting mem ------------------
    pos_memory = np.roll(pos_memory, 1, axis=0)  # roll one line
    pos_memory[0, :] = r_traj_pos(position_d, dmp_traj_pos[i, :3]) # pos err
    pos_r_t = forgetting(history_memory=pos_memory, memory_strength=memory_strength)
    rot_memory = np.roll(rot_memory, 1, axis=0)
    rot_memory[0, :] = r_traj_pos(rotation_d, dmp_traj_pos[i, 3:])  # pos err
    rot_r_t = forgetting(history_memory=rot_memory, memory_strength=memory_strength_rot)
    ## -------- calculate force by (Ft = mv' - mv) -----
    # e_force = (pos_r_t - 2 * pos_r_t_1 + pos_r_t_2) / (t_sample )
    e_force = e_force_mass_coefficient * pos_r_t / (t_sample ** 2) # TODO: mass = 0.001, TBD
    e_torque = e_torque_mass_coefficient * rot_r_t / (t_sample ** 2)
    pos_r_t_2 = np.copy(pos_r_t_1)
    pos_r_t_1 = np.copy(pos_r_t)
    desired_force[:3] = e_force - dmp_traj_force[i, :3]
    desired_force[3:] = e_torque - dmp_traj_force[i, 3:]
    # print(e_force, desired_force)
    # -------------------------------------------
    # print(position_d, rotation_d)
    ik_q = myrobot.IK(position_d, rotation_d)
    # print(position_d, rotation_d)
    # print(ik_q)
    # print(myrobot.getQ())
    # print(myrobot.getToolPos())
    # print(myrobot.getTCPPos())
    # print('---------')
    myrobot.servoJ(ik_q, 0.09, 0.03, 500) # !!!!!! dangerous moving function
    arm_pos, arm_rot = myrobot.getToolPos()
    arm_d = np.hstack([arm_pos, arm_rot])
    pos_d = np.hstack([position_d, rotation_d])
    pos_record = np.vstack([pos_record, pos_d])
    arm_pos_record = np.vstack([arm_pos_record, arm_d])
    force_record = np.vstack([force_record, desired_force])
    curr_force_record = np.vstack([curr_force_record, curr_ft])
    np.save('cut_data/' + current_time + 'pos', pos_record)
    np.save('cut_data/' + current_time + 'force', force_record)
    np.save('cut_data/' + current_time + 'armpos', arm_pos_record)
    np.save('cut_data/' + current_time + 'armforce', curr_force_record)
# plt.figure(1)
# plt.plot(traj_force[:, 2], label=r"Demonstration, $g \approx y_0$", ls="--")
# plt.plot(dmp_traj_force[:, 2], label="DMP with new goal", lw=5, alpha=0.5)
# plt.plot(force_record[:, 2])
# plt.figure(2)
# plt.plot(traj_pos[:, 2], label=r"Demonstration, $g \approx y_0$", ls="--")
# plt.plot(dmp_traj_pos[:, 2], label="DMP with new goal", lw=5, alpha=0.5)
# plt.plot(pos_record[:, 2])
# # print(traj_pos[:, 2].shape, dmp_traj_pos[:, 2].shape, pos_record[:, 2].shape)
# plt.show()
#
