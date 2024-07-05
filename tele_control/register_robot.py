# MODEL: Teleoperation Register - ZeroForceControl for MasterSide, AdmittanceControl for SlaveSide
# AUTHOR: Yi Liu @AiRO
# UNIVERSITY: UGent-imec
# DEPARTMENT: Faculty of Engineering and Architecture
# Control Engineering / Automation Engineering

import rtde_receive
import rtde_control
import numpy as np
from scipy.spatial.transform import Rotation as R
from kdl_func import KDL_ROBOT

class RegisterRobot():
    def __init__(self, robot_ip):
        self.rtde_c = rtde_control.RTDEControlInterface(robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        self.rtde_c.zeroFtSensor()
        self.ur3e_kdl = KDL_ROBOT('../ur3e.urdf', "base_link", "tool0")
        self.truncation_num = 6

    def servoJ(self, q, time, controltime, gain):
        self.rtde_c.servoJ(q, 0.001, 0.001, time, controltime, gain)

    def getTCPFT(self):
        return np.around(self.rtde_r.getActualTCPForce(), self.truncation_num)

    def getTCPPos(self, rot='euler'):
        ee_pos = self.rtde_r.getActualTCPPose()
        if rot == 'rot':
            return ee_pos[:3], ee_pos[3:]
        else:
            des_pos = np.array(ee_pos)[:3]
            des_rot = R.from_rotvec(ee_pos[3:], degrees=False)
            if rot == 'euler':
                des_euler = des_rot.as_euler('xyz', degrees=False)
                des_pos = np.around(des_pos, self.truncation_num)  # truncation for float number
                des_euler = np.around(des_euler, self.truncation_num)
                return des_pos, des_euler
            elif rot == 'quaternion':
                des_qua = des_rot.as_quat()
                des_pos = np.around(des_pos, self.truncation_num)  # truncation for float number
                des_qua = np.around(des_qua, self.truncation_num)
                return des_pos, des_qua
            else:
                print('please give the right rot')
                return False

    def IK(self, pos_d, r_euler):
        position_d = np.around(pos_d, self.truncation_num)
        rotation_d = np.around(r_euler, self.truncation_num)
        r_target_rot = R.from_euler('xyz', rotation_d, degrees=False)
        r_target_qua = r_target_rot.as_quat()
        cur_q = np.around(self.rtde_r.getActualQ(), self.truncation_num)
        ik_q = self.ur3e_kdl.inverse(cur_q, position_d, r_target_qua)
        ik_q = np.around(ik_q, self.truncation_num)
        e_q = ik_q - self.rtde_r.getActualQ()
        if np.any(abs(e_q) > 0.5):
            print('move too much!!!!!!!!!!!!! IK problem')
            print(ik_q)
            return self.rtde_r.getActualQ()
        else:
            return ik_q

    def zeroforce(self):
        self.rtde_c.zeroFtSensor()

    def getQ(self):
        return self.rtde_r.getActualQ()

    def getToolPos(self, rot='euler'):
        curr_q = self.getQ()
        tool_pos, tool_qua = self.ur3e_kdl.forward(curr_q)
        tool_qua_R = R.from_quat(tool_qua)
        if rot == 'euler':
            tool_euler = tool_qua_R.as_euler('xyz', degrees=False)
            return tool_pos, tool_euler
        elif rot == 'quaternion':
            return tool_pos, tool_qua
        elif rot == 'rot':
            tool_rot = tool_qua_R.as_rotvec()
            return tool_pos, tool_rot
        else:
            print('please give the right rot')
            return False



