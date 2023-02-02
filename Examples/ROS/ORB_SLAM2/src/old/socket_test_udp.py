# test code for sockets, doesn't do anything
import socket

HOST = "192.168.4.1"
PORT = 3333

NETWORK_ADDRESS = (HOST, PORT)

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    print("Made socket")
    s.connect((HOST, PORT))
    print("Connected to host")
    s.sendall(b"Hello from ROS!")
    s.bind(NETWORK_ADDRESS)
    while True:
        data = s.recv(1024)
        print(data)