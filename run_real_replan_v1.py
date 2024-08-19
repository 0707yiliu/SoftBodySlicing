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
from det_Tag import AprilTagDet
import threading
import time


with open('config.yml', 'r', encoding="utf-8") as f:
    config = yaml.safe_load(f)  # import config from yaml

def rotver_dis(rot_err):
    len = rot_err.shape[0]
    sum = 0
    for i in range(len):
        sum = sum + (rot_err[i] ** 2)
    dis = np.sqrt(sum)
    return dis

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
target_curr = np.zeros(6) # TODO: add rot as zeros(6)
def det_tag():
    global target_curr
    tag_det = AprilTagDet(rootid=9, objid=10)
    while True:
        target_curr = tag_det.robot2tag()
        # print(target_curr)
        # print(x,y,z)
        # TODO: change the func in det_Tag as RL_env,
        #  get the target pos and rot, need testing.

det_thread = threading.Thread(target=det_tag)
det_thread.daemon = True
det_thread.start()

# get the ratio from pos diff
curr_pos, curr_euler = myrobot.getToolPos()
# curr_pos = [0, 0, 0, 0, 0, 0]

# drive robot, TODO: the time need to be considered
pos_record = np.array([0, 0, 0, 0, 0, 0])
desired_force = np.array([0, 0, 0, 0, 0, 0])
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

# # get DMP traj
# dmp_traj_force = DMP_traj(traj_force, new_start_force, new_goal_force)
# dmp_traj_pos = DMP_traj(traj_pos, new_start_pos, new_goal_pos)
# get the t1, t2 index from raw traj, which used for later contact
t1_index = 0
t2_index = np.argmin(traj_pos[:, 2])
min_t2_pos_z = traj_pos[t2_index, 2] # get the minimum value
for i in range(traj_force.shape[0]):
    if traj_force[i, 2] > ft_th: # get the first time of contacting
        t1_index = i
        break
# depart the trajectory by t1 and t2
if t1_index >= t2_index:
    print('wrong time step index, quit and check')
    input()
# the new start pos is fixed, the new goal between t0-t2 is replaced by current marker pose
dmp_traj_force = DMP_traj(traj_force, new_start_force, new_goal_force)
dmp_traj_pos = DMP_traj(traj_pos, new_start_pos, new_goal_pos)
# separate the trajectories
t0_t1_dmps_traj_pos = dmp_traj_pos[:t1_index, :]
t0_t1_dmps_traj_force = dmp_traj_force[:t1_index, :]
t1_t2_dmps_traj_pos = dmp_traj_pos[t1_index:t2_index, :]
t1_t2_dmps_traj_force = dmp_traj_force[t1_index:t2_index, :]
t2_back_dmps_traj_pos = dmp_traj_pos[t2_index:, :]
t2_back_dmps_traj_force = dmp_traj_force[t2_index:, :]
t0_t1_goal_pos_last = np.copy(t0_t1_dmps_traj_pos[-1, :])
t0_t1_goal_pos_curr = np.copy(t0_t1_dmps_traj_pos[-1, :])
# record the changing of curr target rot
curr_rot = np.zeros(3) # current rotation
# last_rot = np.zeros(3) # last time of rotation, (useless 'cause init rot is assumed as zero)
# delta_rot = np.zeros(3) # the changing rotation in the whole loop (useless 'cause init rot is assumed as zero)
t0_t2_goal_rot = np.copy(t1_t2_dmps_traj_pos[-1, 3:])
t0_t2_goal_rot_last = np.copy(t1_t2_dmps_traj_pos[-1, 3:])
t0_t1_goal_rot = np.copy(t0_t1_dmps_traj_pos[-1, 3:])
t0_t1_goal_rot_last = np.copy(t0_t1_dmps_traj_pos[-1, 3:])

run_mode = config['run_mode']

o_pos_dis = config['replan']['o_pos_dis']
o_rot_dis = config['replan']['o_rot_dis']
z_base = config['replan']['z_base']
z_diff_th = config['replan']['z_diff_th']
get_t2_start_from_t1end = False
get_back_from_t2end = False
touch_stuff = False # contact the stuff first time on t2 loop
t1_modified = False # t1 traj is modified or not, generate the new demo traj and keep it
t2_modified = False
t2_deeper = False # o_pz change and replan the traj once time
back_replan = False # get the new start for back phase, replan once time
_index = 0 # for re-planning loop
i = 0 # for the whole loop
delta_z_times = 0 # the amount of using o_pz at t2 stage
z_t2_dyn = 0 # for update the z-base in t2
z_delta = 0
z_t2_dyn_c2 = 0

t1_p_record = np.array([])
t2_p_record = np.array([])
t3_p_record = np.array([])

time.sleep(1) # refresh the marker position

for lenloop in range(dmp_traj_force.shape[0]):
    i += 1
    if run_mode is True:
        time.sleep(t_sample)
    # print(i, lenloop)
    curr_ft = np.around(myrobot.getTCPFT(), truncation_num)
    # --------------- replanning the dmps traj -------------
    # get the current goal from sub-thread det_Tag
    current_goal = np.copy(target_curr[:3])
    # change the goal x and y for t0-t1, do not need to change z-axis because the delta-z can find the Z-deep
    t0_t1_goal_pos_curr[0] = np.copy(current_goal[0]) # x
    t0_t1_goal_pos_curr[1] = np.copy(current_goal[1]) # y
    # calculate the changing rotation for the whole loop, to change the dmp traj of rot
    curr_rot = np.copy(target_curr[3:])
    t0_t2_goal_pos = np.copy(t0_t1_goal_pos_curr)
    t0_t2_goal_pos[3:] = t0_t2_goal_rot - curr_rot
    t0_t1_goal_pos = np.copy(t0_t2_goal_pos)
    t0_t1_goal_pos[3:] = t0_t1_goal_rot - curr_rot

    # DMP need too much time, hence, re-plan the traj when target moving
    # print(target_curr, t0_t1_goal_pos_last, t1_t2_dmps_traj_pos[-1, :])
    if i < t1_index: # t0-t1 arriving
        # print(i, t1_index)
        # later contact / contact on time ---------------------
        # calculate the changing distance
        change_pos_dis = np.linalg.norm(t0_t1_goal_pos_curr[:2] - t0_t1_goal_pos_last[:2])
        change_rot_dis = rotver_dis(t0_t2_goal_pos[3:] - t0_t2_goal_rot_last)
        # print('target xy:', current_goal)
        if change_pos_dis > o_pos_dis or change_rot_dis > o_rot_dis:
            t1_modified = True # t1 traj has been replanned
            # print('---------------', change_pos_dis)
            # print(change_rot_dis, t0_t2_goal_pos[3:], t0_t2_goal_rot_last, t0_t2_goal_pos[3:] - t0_t2_goal_rot_last)
            # generate traj between t0-t2, because t2 is the lowest position, we use t0-t2 to genreate t0-t1
            t0_t1_dmp_traj_pos = DMP_traj(dmp_traj_pos[:t1_index, :], new_start_pos, t0_t1_goal_pos)
            # regenerate the dmp traj for t0-t1
            replan_dmp_traj_pos = t0_t1_dmp_traj_pos[:t1_index, :]
            # follow the raw force traj
            t1_new_goal_force = t0_t1_dmps_traj_force[-1, :]
            replan_dmp_traj_force = DMP_traj(t0_t1_dmps_traj_force, new_start_force, t1_new_goal_force)
        else:
            if t1_modified is False:
                replan_dmp_traj_pos = np.copy(t0_t1_dmps_traj_pos) # the t1_modified gate is used to keep the new traj
                replan_dmp_traj_force = np.copy(t0_t1_dmps_traj_force)
            else:
                pass
        if _index < replan_dmp_traj_pos.shape[0] - 1:
            _index += 1
        t0_t1_goal_pos_last = np.copy(t0_t1_goal_pos_curr)
        t0_t2_goal_rot_last = np.copy(t0_t2_goal_pos[3:])
        if run_mode is False:
            t1_p_record = np.copy(replan_dmp_traj_pos)
        # print(t1_p_record.shape)
        # advance contact -------------------------------------
        # the knife contact the stuff in advance, change the t1_index and add to t2_index
        if curr_ft[2] >= ft_th:
            t1_index = _index + 1 # cut the t1_index
            pos, rot = myrobot.getToolPos()
            current_pos = np.hstack([pos, rot])
            if cut_class == 2: # cut on the surface, need to record the current delta z-position
                z_t2_dyn_c2 = pos[-1] - dmp_traj_pos[t1_index-1, 2]
                t2_deeper = True
            # re-change the t1 and t2 index
            t0_t1_dmps_traj_pos = dmp_traj_pos[:t1_index, :]
            t0_t1_dmps_traj_force = dmp_traj_force[:t1_index, :]
            t1_t2_dmps_traj_pos = dmp_traj_pos[t1_index:t2_index, :]
            t1_t2_dmps_traj_force = dmp_traj_force[t1_index:t2_index, :]
            # replan the t1 traj for t2 using
            replan_dmp_traj_pos = DMP_traj(t0_t1_dmps_traj_pos, new_start_pos, current_pos)
            replan_dmp_traj_force = DMP_traj(t0_t1_dmps_traj_force, new_start_force, curr_ft)

# ------------ plot replan traj -----------------------
# print(_index)
# print(t0_t1_dmps_traj_pos.shape, replan_dmp_traj_pos.shape)
# plt.figure(1)
# for i in range(6):
#     plt.subplot(2, 3, i+1)
#     plt.plot(t0_t1_dmps_traj_pos[:, i], label=r"Demonstration, $g \approx y_0$", ls="--")
#     plt.plot(replan_dmp_traj_pos[:, i], label="DMP with new goal", lw=5, alpha=0.5)
# plt.legend()
# plt.show()
# # ------------------------------------------------------
# #
    elif i <= t2_index: # t1-t2 cutting
        change_pos_dis = np.linalg.norm(t0_t1_goal_pos_curr[:2] - t0_t1_goal_pos_last[:2])
        change_rot_dis = rotver_dis(t0_t2_goal_pos[3:] - t0_t2_goal_rot_last)
        if touch_stuff is False:
            if curr_ft[2] >= ft_th: # get the first time of contacting
                touch_stuff = True
            else:
                replan_dmp_traj_pos[_index, 2] -= o_pz
                delta_z_times += 1
                i -= 1 # fix the while loop index i
                t2_deeper = True
                # print('index test:', _index, replan_dmp_traj_pos.shape)
                # _index = replan_dmp_traj_pos.shape[0] - 1 # fix the index for controlling
        else:
            if get_t2_start_from_t1end is False: # get the start of t2 one time
                get_t2_start_from_t1end = True
                t2_new_start_pos = np.copy(replan_dmp_traj_pos[-1, :])
                t2_new_start_force = np.copy(replan_dmp_traj_force[-1, :])
                _index = 0
            # this goal position need to be classified
            # TODO: now is class1 for cut-in and cut-off, we need slice on surface as class2
            z_diff = abs(current_goal[2] - z_base)
            # the current goal's z is the bottom of the stuff, z_base is the that of testing
            # print('z-diff:', z_diff, current_goal[2])
            if z_diff > z_diff_th:
                z_t2_dyn = current_goal[2] - z_base
                z_base = current_goal[2]
            else:
                z_t2_dyn = 0
            z_delta = z_delta + z_t2_dyn
            if cut_class == 1:
                t2_new_goal_z_pos = min_t2_pos_z + z_delta
                # because cut-in and cut-off need to make the knife close to the base,
                # so just use z-base and curr z-base to modify the target
            elif cut_class == 2:
                t2_new_goal_z_pos = min_t2_pos_z - o_pz * delta_z_times + z_delta + z_t2_dyn_c2
                # TODO: more thinking here for delta z
            t0_t2_goal_pos[2] = np.copy(t2_new_goal_z_pos)
            if change_pos_dis > o_pos_dis or change_rot_dis > o_rot_dis or z_t2_dyn != 0:
                replan_dmp_traj_pos = DMP_traj(t1_t2_dmps_traj_pos, t2_new_start_pos, t0_t2_goal_pos)
                t2_new_goal_force = t1_t2_dmps_traj_force[-1, :]
                replan_dmp_traj_force = DMP_traj(t1_t2_dmps_traj_force, t2_new_start_force, t2_new_goal_force)
                t2_modified = True
            else:
                if t2_modified is False:
                    if t2_deeper is True:
                        replan_dmp_traj_pos = DMP_traj(t1_t2_dmps_traj_pos, t2_new_start_pos, t0_t2_goal_pos)
                        t2_new_goal_force = t1_t2_dmps_traj_force[-1, :]
                        replan_dmp_traj_force = DMP_traj(t1_t2_dmps_traj_force, t2_new_start_force, t2_new_goal_force)
                        t2_modified = True
                        t2_deeper = False
                    # replan_dmp_traj_pos = np.copy(
                    #     t0_t1_dmps_traj_pos)  # the t1_modified gate is used to keep the new traj
                    # replan_dmp_traj_force = np.copy(t0_t1_dmps_traj_force)
                else:
                    pass
            if _index < replan_dmp_traj_pos.shape[0] - 1:
                _index += 1
        t0_t1_goal_pos_last = np.copy(t0_t1_goal_pos_curr)
        t0_t2_goal_rot_last = np.copy(t0_t2_goal_pos[3:])
    else: # t2-t3 back
        _index += 1
        if get_back_from_t2end is False:  # get the start of back one time
            get_back_from_t2end = True
            back_new_start_pos = np.copy(replan_dmp_traj_pos[-1, :])
            back_new_start_force = np.copy(replan_dmp_traj_force[-1, :])
            _index = 0
        if back_replan is False:
            replan_dmp_traj_pos = DMP_traj(t2_back_dmps_traj_pos, back_new_start_pos, t2_back_dmps_traj_pos[-1, :])
            replan_dmp_traj_force = DMP_traj(t2_back_dmps_traj_force, back_new_start_force, t2_back_dmps_traj_force[-1, :])
            back_replan = True
        else:
            pass
        if _index == replan_dmp_traj_pos.shape[0]:
            break # get the end pos, break the while loop and quit the whole system
    # -----------------------------------------------------------------
    if run_mode is True:
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
if run_mode is False:
    plt.figure(1)
    for i in range(6):
        plt.subplot(2, 3, i+1)
        plt.plot(t0_t1_dmps_traj_pos[:, i], label=r"Demonstration, $g \approx y_0$", ls="--")
        plt.plot(t1_p_record[:, i], label="DMP with new goal, t0-t1", lw=5, alpha=0.5)
    plt.legend()
    plt.show()
        #
