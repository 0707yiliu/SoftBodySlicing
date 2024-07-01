import sys
import time
from matplotlib import pyplot as plt
import numpy as np
from admittance_controller.core import FT_controller as AdmController
from register_robot import RegisterRobot

s_robot = RegisterRobot("10.42.0.162")
# the configuration of admittance control for slave robot --------------
s_admcontroller = AdmController(0.5, 500, 5, 0.01)
admittance_params = np.zeros((3, 3)) # contains acc, vel and pos in xyz derictions
admittance_paramsT = np.zeros((3, 3))
# --------------
init_time = 4
init_q = np.array([0.50788814, -1.45690663,  1.38477117, -1.71768059, -1.50159198,  2.11026955])
s_robot.servoJ(init_q, init_time, 0.05, 500)
print('waiting and enter.')
input()
last_ft = s_robot.getTCPFT()

while True:
    time.sleep(0.01)
    curr_ft = s_robot.getTCPFT()
    last_ft = lowpass_filter(last_ft, curr_ft, ratio)
    position_d, rotation_d, admittance_params, admittance_paramsT = force_controller.admittance_control(
        desired_position=des_pos,
        desired_rotation=des_euler,
        FT_data=last_ft,
        params_mat=admittance_params,
        paramsT_mat=admittance_paramsT)
    ik_q = s_robot.IK(position_d, rotation_d)



