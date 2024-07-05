import pickle
import time
import socket
import numpy as np


client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.settimeout(1)
a = np.array([1,2,3,45])
b = np.array([1,2,3,1])
c = np.hstack((a,b))
message = pickle.dumps(c)
addr = ("127.0.0.1", 12000)

while True:
    start = time.time()
    client_socket.sendto(message, addr)
    try:
        data, server = client_socket.recvfrom(1024)
        end = time.time()
        data = pickle.loads(data)
        elapsed = end - start
        print(f'{data}')
    except socket.timeout:
        print('REQUEST TIMED OUT')