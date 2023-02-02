# test code for sockets, doesn't do anything
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
oldTime = time.time()*1000
maxDelays = []
maxDelays.append(0)
maxDelays.append(0)
maxDelays.append(0)
maxDelays.append(0)
maxDelays.append(0)
avgDelay = 0

# Wait for server to respond and print it
while count < 10000:
    result = ws.recv()
    # print("Message num: {} ||| {}".format(count, result))
    count += 1
    newTime = time.time()*1000
    timediff = newTime - oldTime
    avgDelay *= count
    avgDelay += timediff
    avgDelay /= count
    for i, x in enumerate(maxDelays):
        if timediff > x:
            maxDelays[i] = timediff
            break
    # print("Time taken: {}ms".format(timediff))
    oldTime = time.time()*1000
    # time.sleep(0.2)

for x in maxDelays:
    print(x)

print("Avg delay ", avgDelay)