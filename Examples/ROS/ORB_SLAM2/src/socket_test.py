# import socket

# HOST = "192.168.4.1"
# PORT = 80

# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#     print("Made socket")
#     s.connect((HOST, PORT))
#     print("Connected to host")
#     s.sendall(b"Hello from ROS!")
#     while True:
#         data = s.recv(1024)
#         print(data)

import websocket
import time

# Connect to WebSocket server
ws = websocket.WebSocket()
ws.connect("ws://192.168.4.1")
print("Connected to WebSocket server")

ws.send("ROS says Hello!")
count = 0
oldTime = time.time()

# Wait for server to respond and print it
while True:
    result = ws.recv()
    print("Message num: {} ||| {}".format(count, result))
    count += 1
    print("Time taken: {}ms".format(time.time() - oldTime))
    oldTime = time.time()
