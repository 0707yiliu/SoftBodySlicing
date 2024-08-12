# UNIVERSITY: UGent-imec
# AUTHOR: YiLiu
# DEPARTMENT: Faculty of Engineering and Architecture
# Control Engineering / Automation Engineering
# re-plan the DMPs' trajectory when contacting the stuff

import numpy as np
from movement_primitives.dmp import DMP

from tele_control.register_robot import RegisterRobot
from admittance_controller.core import FT_controller as AdmController

import time

import matplotlib.pyplot as plt
import yaml
import det_Tag
import threading
import time




with open('config.yml', 'r', encoding="utf-8") as f:
    config = yaml.safe_load(f)  # import config from yaml


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
    r = curr_pos - dmp_pos  # test, fake
    # print(curr_pos.shape, dmp_pos.shape)
    return r


def traj_load(slaveforce, slavepos):
    # loading tele-trajectory from slave datasets for DMPs
    traj_force = np.load('tele_control/data/' + slaveforce + '.npy')
    traj_pos = np.load('tele_control/data/' + slavepos + '.npy')
    traj_pos = np.delete(traj_pos, [0,1,2,3], 0)
    traj_force = np.delete(traj_force, [0,1,2,3], 0)
    return traj_force, traj_pos


def DMP_traj(origin_traj, new_start, new_goal):
    # dynamic movement primitives for force trajectory
    # input:
    #   origin_traj: original trajectory from slave robot
    #   new_start: the new start of the trajectory
    #   new_goal: the new goal of the trajectory
    # output:
    #   Y_dmp
    # traj = origin_traj[2:, :]
    Y = origin_traj
    T = np.linspace(0, 10, origin_traj.shape[0])
    dmp = DMP(
        n_dims=origin_traj.shape[1],
        execution_time=origin_traj.shape[0] / 100,
        dt=config['DMPs']['dt'],
        n_weights_per_dim=config['DMPs']['n_weights_per_dim'],
        smooth_scaling=config['DMPs']['smooth_scaling'],
    )
    dmp.imitate(T, Y)
    dmp.configure(start_y=new_start, goal_y=new_goal)
    _, Y_dmp = dmp.open_loop()
    return Y_dmp

def re_planning(index, traj_p, traj_f):
    pass


current_time = time.strftime('%Y%m%d%H%M%S', time.localtime())
# register the robot
myrobot = RegisterRobot(config['controlled_robot_ip'])
# init robot
init_time = 4
init_q = np.array([0.50788814, -1.45690663, 1.38477117, -1.71768059, -1.50159198, 2.11026955])
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
                              dt=config['admittance_controller']['dt'])  # TODO: TBD
admittance_params = np.zeros((3, 3))  # contains acc, vel and pos in xyz derictions
admittance_paramsT = np.zeros((3, 3))

# sub-thread for get dynamic target pose
target_curr = np.zeros(6)
def det_tag():
    global target_curr
    while True:
        target_curr = det_Tag.det_apriltag()
        # TODO: change the func in det_Tag as RL_env,
        #  get the target pos and rot, need testing.

det_thread = threading.Thread(target=det_tag)
det_thread.start()

# get DMP traj
dmp_traj_force = DMP_traj(traj_force, new_start_force, new_goal_force)
dmp_traj_pos = DMP_traj(traj_pos, new_start_pos, new_goal_pos)
# get the ratio from pos diff
curr_pos, curr_euler = myrobot.getToolPos()
# curr_pos = [0, 0, 0, 0, 0, 0]

# drive robot, TODO: the time need to be considered
pos_record = np.array([0, 0, 0, 0, 0, 0])
desired_force = np.copy(dmp_traj_force[0, :])
force_record = np.array([0, 0, 0, 0, 0, 0])
curr_force_record = np.array([0, 0, 0, 0, 0, 0])
arm_pos_record = np.array([0, 0, 0, 0, 0, 0])
memory_num = config['forgetting_err']['memory_number']
memory_strength = config['forgetting_err']['memory_strength']  # the smaller, the stronger nonlinear
memory_strength_rot = config['forgetting_err']['memory_strength_rot']
e_force_mass_coefficient = config['forgetting_err']['to_force_coefficient']
e_torque_mass_coefficient = config['forgetting_err']['to_torque_coefficient']
pos_memory = np.zeros((memory_num, 3))
rot_memory = np.zeros((memory_num, 3))
pos_r_t_1 = np.zeros(3)
pos_r_t_2 = np.zeros(3)
t_sample = config['time_sample']
truncation_num = 4
replan_flag = config['replan']['contact']
ft_th = config['replan']['ft_th']
o_pz = config['replan']['o_pz']
cut_class = config['replan']['cut_class']
# get the t1, t2 index from DMPs traj, which used for later contact
t1_index = 0
t2_index = np.argmin(dmp_traj_pos[:, 2])
for i in range(dmp_traj_force.shape[0]):
    if dmp_traj_force[i, 2] > ft_th: # get the first time of contacting
        t1_index = i
        break
# depart the trajectory by t1 and t2
if t1_index >= t2_index:
    print('wrong time step index, quit and check')
    input()
t0_t1_dmps_traj_pos = dmp_traj_pos[:t1_index, :]
t0_t1_dmps_traj_force = dmp_traj_force[:t1_index, :]
t1_t2_dmps_traj_pos = dmp_traj_pos[t1_index:t2_index, :]
t1_t2_dmps_traj_force = dmp_traj_force[t1_index:t2_index, :]
t2_back_dmps_traj_pos = dmp_traj_pos[t2_index:, :]
t2_back_dmps_traj_force = dmp_traj_force[t2_index:, :]
get_t2_start_from_t1end = False
get_back_from_t2end = False
touch_stuff = False # contact the stuff first time on t2 loop
_index = 0 # for re-planning loop
i = 0 # for the whole loop
delta_z_times = 0 # the amount of using o_pz at t2 stage

while True:
    i += 1
    time.sleep(t_sample)
    curr_ft = np.around(myrobot.getTCPFT(), truncation_num)
    # --------------- replanning the dmps traj -------------
    if i < t1_index: # t0-t1 arriving
        # later contact / contact on time ---------------------
        t1_new_goal_pos = np.copy(target_curr)
        t1_new_goal_force = t0_t1_dmps_traj_force[-1, :]
        # TODO: change to the AprilTag position (dynamic), get the target pos from "det_tag" thread
        replan_dmp_traj_pos = DMP_traj(t0_t1_dmps_traj_pos, new_start_pos, t1_new_goal_pos)
        replan_dmp_traj_force = DMP_traj(t0_t1_dmps_traj_force, new_start_force, t1_new_goal_force)
        if _index < replan_dmp_traj_pos.shape[0] - 1:
            _index += 1
        # advance contact -------------------------------------
        # the knife contact the stuff in advance, change the t1_index and add to t2_index
        if curr_ft[2] >= ft_th:
            t1_index = _index - 1
            # re-change the t1 and t2 index
            t0_t1_dmps_traj_pos = dmp_traj_pos[:t1_index, :]
            t0_t1_dmps_traj_force = dmp_traj_force[:t1_index, :]
            t1_t2_dmps_traj_pos = dmp_traj_pos[t1_index:t2_index, :]
            t1_t2_dmps_traj_force = dmp_traj_force[t1_index:t2_index, :]
            # replan the t1 traj for t2 using
            pos, rot = myrobot.getToolPos()
            current_pos = np.hstack([pos, rot])
            replan_dmp_traj_pos = DMP_traj(t0_t1_dmps_traj_pos, new_start_pos, current_pos)
            replan_dmp_traj_force = DMP_traj(t0_t1_dmps_traj_force, new_start_force, curr_ft)

    elif i <= t2_index: # t1-t2 cutting
        if touch_stuff is False:
            if curr_ft[2] >= ft_th: # get the first time of contacting
                touch_stuff = True
            else:
                replan_dmp_traj_pos[_index, 2] -= o_pz
                delta_z_times += 1
                i -= 1 # fix the while loop index i
                # _index = replan_dmp_traj_pos.shape[0] - 1 # fix the index for controlling
        if touch_stuff is True:
            _index += 1
            if get_t2_start_from_t1end is False: # get the start of t2 one time
                get_t2_start_from_t1end = True
                t2_new_start_pos = replan_dmp_traj_pos[-1, :]
                t2_new_start_force = replan_dmp_traj_force[-1, :]
                _index = 0
            # this goal position need to be classified
            # TODO: now is class1 for cut-in and cut-off, we need slice on surface as class2
            if cut_class == 1:
                t2_new_goal_pos = t1_t2_dmps_traj_pos[-1, :] # TODO: use delta to change the traj
                t2_new_goal_force = t1_t2_dmps_traj_force[-1, :]
                # TODO: change to the end of dmp traj
                t2_new_goal_pos[2] = t1_t2_dmps_traj_pos[-1, :]
            elif cut_class == 2:
                t2_new_goal_pos = t1_t2_dmps_traj_pos[-1, :]
                t2_new_goal_force = t1_t2_dmps_traj_force[-1, :]
            replan_dmp_traj_pos = DMP_traj(t1_t2_dmps_traj_pos, t2_new_start_pos, t2_new_goal_pos)
            replan_dmp_traj_force = DMP_traj(t1_t2_dmps_traj_force, t2_new_start_force, t2_new_goal_force)
    else: # t2-t3 back
        _index += 1
        if get_back_from_t2end is False:  # get the start of back one time
            get_back_from_t2end = True
            back_new_start_pos = replan_dmp_traj_pos[-1, :]
            back_new_start_force = replan_dmp_traj_force[-1, :]
            _index = 0
        replan_dmp_traj_pos = DMP_traj(t2_back_dmps_traj_pos, back_new_start_pos, t2_back_dmps_traj_pos[-1, :])
        replan_dmp_traj_force = DMP_traj(t2_back_dmps_traj_force, back_new_start_force, t2_back_dmps_traj_force[-1, :])
        if _index == replan_dmp_traj_pos.shape[0]:
            break # get the end pos, break the while loop and quit the whole system
    # -----------------------------------------------------------------
    adm_force = desired_force + curr_ft
    position_d, rotation_d, admittance_params, admittance_paramsT = admcontroller.admittance_control(
        desired_position=replan_dmp_traj_pos[_index, :3],  # TODO: TBD
        desired_rotation=replan_dmp_traj_pos[_index, 3:],  # TODO: TBD
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
    pos_memory[0, :] = r_traj_pos(position_d, replan_dmp_traj_pos[_index, :3])  # pos err
    pos_r_t = forgetting(history_memory=pos_memory, memory_strength=memory_strength)
    rot_memory = np.roll(rot_memory, 1, axis=0)
    rot_memory[0, :] = r_traj_pos(rotation_d, replan_dmp_traj_pos[_index, 3:])  # pos err
    rot_r_t = forgetting(history_memory=rot_memory, memory_strength=memory_strength_rot)
    ## -------- calculate force by (Ft = mv' - mv) -----
    # e_force = (pos_r_t - 2 * pos_r_t_1 + pos_r_t_2) / (t_sample )
    e_force = e_force_mass_coefficient * pos_r_t / (t_sample ** 2)  # TODO: mass = 0.001, TBD
    e_torque = e_torque_mass_coefficient * rot_r_t / (t_sample ** 2)
    pos_r_t_2 = np.copy(pos_r_t_1)
    pos_r_t_1 = np.copy(pos_r_t)
    desired_force[:3] = e_force - replan_dmp_traj_force[_index, :3]
    desired_force[3:] = e_torque - replan_dmp_traj_force[_index, 3:]
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
    myrobot.servoJ(ik_q, 0.09, 0.03, 500)  # !!!!!! dangerous moving function
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
