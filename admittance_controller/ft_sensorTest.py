import rtde_receive
import rtde_control
import time
rtde_c = rtde_control.RTDEControlInterface("10.42.0.162")
rtde_r = rtde_receive.RTDEReceiveInterface("10.42.0.162")
actual_q = rtde_r.getActualQ()
print(actual_q)
rtde_c.zeroFtSensor()
print(rtde_r.getActualTCPForce())
while True:
    rtde_c.teachMode()