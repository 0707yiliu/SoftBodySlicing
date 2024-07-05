import pickle
import random
import socket

server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind(('', 12000))


while True:
    rand = random.randint(0, 10)
    try:
        message, address = server_socket.recvfrom(1024)
        print(pickle.loads(message))
    except socket.timeout:
        print('slave request timed out. check the server in master side')
    server_socket.sendto(message, address)


