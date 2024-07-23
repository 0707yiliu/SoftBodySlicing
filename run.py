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

def r_traj_pos(curr_pos, dmp_pos):
    # input:
    #        curr_pos: current ee-pos (numpy array)
    #        dmp_pos: DMP predicted pos (numpy array)
    # output:
    #        r: ratio for pos (numpy array), which is used to by traj_force for admittance controller
    r = curr_pos - dmp_pos # test, fake
    return r

def traj_load(slaveforce, slavepos):
    # loading tele-trajectory from slave datasets for DMPs
    traj_force = np.load('tele_control/data/' + slaveforce + '.npy')
    traj_pos = np.load('tele_control/data/' + slavepos + '.npy')
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
        dt=0.01,
        n_weights_per_dim=50,
        smooth_scaling=True,
    )
    dmp.imitate(T, Y)
    dmp.configure(start_y=new_start, goal_y=new_goal)
    _, Y_dmp = dmp.open_loop()
    return Y_dmp

current_time = time.strftime('%Y%m%d%H%M%S', time.localtime())
# register the robot
# myrobot = RegisterRobot("10.42.0.162")
# define new_start and new_goal for DMP_traj func
new_start = np.array([-0.2, 0.5, 0, 1, 0, 0])
new_goal = np.array([0.2, 0.5, 0, 1, 0, 0]) # TODO:TBD, for the force, the array is 6-dims
# get demonstrated traj from traj_load func
slave_force = '20240718173024slaveforce'
slave_pos = '20240718173024slavepos'
traj_force, traj_pos = traj_load(slave_force, slave_pos)
# define admittance control for auto-robot
admcontroller = AdmController(0.5, 1000, 8, 0.01) # TODO: TBD
admittance_params = np.zeros((3, 3)) # contains acc, vel and pos in xyz derictions
admittance_paramsT = np.zeros((3, 3))
# get DMP traj
dmp_traj_force = DMP_traj(traj_force, new_start, new_goal)
dmp_traj_pos = DMP_traj(traj_pos, new_start, new_goal)
# get the ratio from pos diff
# curr_pos, curr_euler = myrobot.getToolPos()
curr_pos = [0,0,0,0,0,0]
r = r_traj_pos(curr_pos, dmp_traj_pos)

# drive robot, TODO: the time need to be considered
pos_record = np.array([0,0,0])
for i in range(dmp_traj_force.shape[0]):
    position_d, rotation_d, admittance_params, admittance_paramsT = admcontroller.admittance_control(
            desired_position=dmp_traj_pos[i, :3],# TODO: TBD
            desired_rotation=dmp_traj_pos[i, 3:],# TODO: TBD
            FT_data=dmp_traj_force[i, :],
            params_mat=admittance_params,
            paramsT_mat=admittance_paramsT)
    # ik_q = myrobot.IK(position_d, rotation_d)
    # myrobot.servoJ(ik_q, 0.08, 0.03, 500) # !!!!!! dangerous moving function
    pos_record = np.vstack([pos_record, position_d])
plt.plot(traj_pos[:, 2], label=r"Demonstration, $g \approx y_0$", ls="--")
plt.plot(dmp_traj_pos[:, 2], label="DMP with new goal", lw=3, alpha=0.5)
plt.plot(pos_record[:, 2])
plt.show()

