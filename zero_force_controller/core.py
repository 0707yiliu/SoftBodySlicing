# MODEL: Zero Force controller
# AUTHOR: Yi Liu @AiRO
# UNIVERSITY: UGent-imec
# DEPARTMENT: Faculty of Engineering and Architecture
# Control Engineering / Automation Engineering

import numpy as np

class ZeroForceController():
    def __init__(self, k, kr, dt) -> None:
        self.K = k
        self.Kr = kr
        self.dt = dt

    def zeroforce_control(self, ft, desired_position, desired_rotation):
        d_p = ft[:3] * self.K
        d_r = ft[3:] * self.Kr
        p = d_p * self.dt
        r = d_r * self.dt
        desired_position = desired_position + p
        desired_rotation = desired_rotation + r
        # desired_rotation = desired_rotation + r
        return desired_position, desired_rotation



