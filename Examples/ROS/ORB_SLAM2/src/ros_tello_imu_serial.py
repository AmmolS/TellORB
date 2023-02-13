# esp32-cam IMU data published to ROS

import os
import rospy
import time
#import websocket
import socket
import math
import serial
from threading import Thread, Event

from sensor_msgs.msg import _Imu

# from cv_bridge import CvBridge, CvBridgeError

import cv2
from urllib.request import urlopen
import numpy as np

#ws = websocket.WebSocket()
#ws.connect("ws://192.168.4.1")


# HOST = "192.168.4.1"
# PORT = 63000

# avg = []

# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect((HOST, PORT))




# for i in range(1000):
#     lastTime = time.time()
#     s.sendall(b"p")
#     data = s.recv(128)
#     newTime = time.time()
#     diff = (newTime - lastTime) * 1000
#     avg.append(diff)
#     #print(f"{diff} ms\n")
#     #print(f"{data.decode('utf-8')}\n")
#     time.sleep(0.001)

# print(f"avg delay: {sum(avg) / 1000}")
# print(sorted(avg))




# bridge = CvBridge()
ser = serial.Serial('COM3', 115200, timeout=1)
pub_imu = rospy.Publisher("/imu0", _Imu.Imu, queue_size=10)
result = "0;0;0;0;0;0"
time.sleep(2)

# def readImu():
#     global result
#     while True:
#         s.sendall(b"p")
#         result = s.recv(128)
#         result = f"{result.decode('utf-8')}"

# def pubImuData():
#     seq = 10001
#     oldTime = 0
#     while True:
#         if(time.time() - oldTime >= 1/100):
#             dataCopy = result
#             # print(time.time()-oldTime)
#             imu_msg = _Imu.Imu()
#             imu_msg.header.seq = seq
#             imu_msg.header.stamp.set(math.floor(time.time()), time.time_ns() % 1000000000)
#             imu_msg.header.frame_id = "imu4"
#             imu_msg.orientation.z = 1
#             imu_msg.orientation_covariance[0] = 99999.9
#             imu_msg.orientation_covariance[4] = 99999.9
#             imu_msg.orientation_covariance[8] = 99999.9

#             imu_msg.linear_acceleration.x = float(dataCopy.split(";")[0])
#             imu_msg.linear_acceleration.y = float(dataCopy.split(";")[1])
#             imu_msg.linear_acceleration.z = float(dataCopy.split(";")[2])
#             imu_msg.angular_velocity.x = float(dataCopy.split(";")[3])
#             imu_msg.angular_velocity.y = float(dataCopy.split(";")[4])
#             imu_msg.angular_velocity.z = float(dataCopy.split(";")[5])
#             # print(imu_msg)

#             pub_imu.publish(imu_msg)
#             seq += 1
#             oldTime = time.time()
#             # time.sleep(fps)
#             # print(result)

def main():
    print('ROS IMU: Initializing ros_tello node...')
    rospy.init_node('ros_imu')
    print('IMU data is being published on /imu0')
    while True:
        if(time.time() - oldTime >= 1/500):
            line = ser.readline()   # read a byte
            if(line):
                dataCopy = line.decode()  # convert the byte string to a unicode string
                # num = int(string) # convert the unicode string to an int
            print(dataCopy)
            # dataCopy = string
            # print(time.time()-oldTime)
            imu_msg = _Imu.Imu()
            imu_msg.header.seq = seq
            imu_msg.header.stamp.set(math.floor(time.time()), time.time_ns() % 1000000000)
            imu_msg.header.frame_id = "imu4"
            imu_msg.orientation.z = 1
            imu_msg.orientation_covariance[0] = 99999.9
            imu_msg.orientation_covariance[4] = 99999.9
            imu_msg.orientation_covariance[8] = 99999.9

            imu_msg.linear_acceleration.x = float(dataCopy.split(";")[0])
            imu_msg.linear_acceleration.y = float(dataCopy.split(";")[1])
            imu_msg.linear_acceleration.z = float(dataCopy.split(";")[2])
            imu_msg.angular_velocity.x = float(dataCopy.split(";")[3])
            imu_msg.angular_velocity.y = float(dataCopy.split(";")[4])
            imu_msg.angular_velocity.z = float(dataCopy.split(";")[5])
            # print(imu_msg)

            pub_imu.publish(imu_msg)
            seq += 1
            oldTime = time.time()
            # time.sleep(fps)
            # print(result)

    # readImuThread = Thread(target=readImu)
    # readImuThread.start()
    # publishThread = Thread(target=pubImuData)
    # publishThread.start()

    

    # readImuThread.join()
    # publishThread.join()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
