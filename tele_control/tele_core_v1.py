import sys
import time
from matplotlib import pyplot as plt
import numpy as np
from zero_force_controller.core import ZeroForceController
from admittance_controller.core import FT_controller as AdmController
from register_robot import RegisterRobot
import socket

soc = socket.socket()
soc.connect(("localhost",8888))

while True:
    msg = input("请输入发送给服务端的消息：")
    if "exit" == msg:
        break
    soc.send(msg.encode("UTF-8"))
    data = soc.recv(1024).decode("UTF-8")
    print(f"服务端发来的消息：{data}")
soc.close()