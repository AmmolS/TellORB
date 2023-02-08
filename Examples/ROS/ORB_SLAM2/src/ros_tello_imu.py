# esp32-cam IMU data published to ROS

import os
import rospy
import time
#import websocket
import socket
import math
from threading import Thread, Event

from sensor_msgs.msg import _Imu

# from cv_bridge import CvBridge, CvBridgeError

import cv2
from urllib.request import urlopen
import numpy as np

#ws = websocket.WebSocket()
#ws.connect("ws://192.168.4.1")


HOST = "192.168.4.1"
PORT = 63000

avg = []

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

for i in range(1000):
    lastTime = time.time()
    s.sendall(b"p")
    data = s.recv(128)
    newTime = time.time()
    diff = (newTime - lastTime) * 1000
    avg.append(diff)
    #print(f"{diff} ms\n")
    #print(f"{data.decode('utf-8')}\n")
    time.sleep(0.001)

print(f"avg delay: {sum(avg) / 1000}")
print(sorted(avg))




# bridge = CvBridge()
pub_imu = rospy.Publisher("/imu0", _Imu.Imu, queue_size=10)
result = ""

def readImu():
    while True:
        s.sendall(b"p")
        result = s.recv(128)
        result = f"{result.decode('utf-8')}"

def pubImuData():
    seq = 10001
    oldTime = 0
    while True:
        if(time.time() - oldTime >= 1/100):
            dataCopy = result
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

def main():
    print('ROS IMU: Initializing ros_tello node...')
    rospy.init_node('ros_imu')

    readImuThread = Thread(target=readImu)
    readImuThread.start()
    publishThread = Thread(target=pubImuData)
    publishThread.start()

    print('IMU data is being published on /imu0')

    readImuThread.join()
    publishThread.join()


    # seq = 10001
    # oldTime = 0
    # while True:
    #     if(time.time() - oldTime >= 1/100):
    #         s.sendall(b"p")
    #         result = s.recv(128)
    #         result = f"{result.decode('utf-8')}"
    #         # print(time.time()-oldTime)
    #         imu_msg = _Imu.Imu()
    #         imu_msg.header.seq = seq
    #         imu_msg.header.stamp.set(math.floor(time.time()), time.time_ns() % 1000000000)
    #         imu_msg.header.frame_id = "imu4"
    #         imu_msg.orientation.z = 1
    #         imu_msg.orientation_covariance[0] = 99999.9
    #         imu_msg.orientation_covariance[4] = 99999.9
    #         imu_msg.orientation_covariance[8] = 99999.9

    #         imu_msg.linear_acceleration.x = float(result.split(";")[0])
    #         imu_msg.linear_acceleration.y = float(result.split(";")[1])
    #         imu_msg.linear_acceleration.z = float(result.split(";")[2])
    #         imu_msg.angular_velocity.x = float(result.split(";")[3])
    #         imu_msg.angular_velocity.y = float(result.split(";")[4])
    #         imu_msg.angular_velocity.z = float(result.split(";")[5])
    #         # print(imu_msg)

    #         pub_imu.publish(imu_msg)
    #         seq += 1
    #         oldTime = time.time()
    #         # time.sleep(fps)
    #         # print(result)

    # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #     s.connect((HOST, PORT))
    #     # s.sendall(b"Hello, world")
    #     while True:
    #         data = s.recv(128)
    #         # print(f"Received {data!r}")
    #         print("{}: ".format(count) + data.decode("utf-8"))
    #         count += 1
    #         time.sleep(1/100)


    # url = r'http://192.168.4.1/imu.txt'
    # while True:
    #     imu_resp = urlopen(url)
    #     print(imu_resp.read())
    #     # imgnp = np.asarray(bytearray(img_resp.read()), dtype="uint8")
    #     # img = cv2.imdecode(imgnp, -1)
    #     # img_msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    #     # pub_img.publish(img_msg)
    #     # cv2.imshow("Camera", img)
    #     # if cv2.waitKey(1) == 113:
    #         # break
    #     # time.sleep(0.5)

    # print("Exiting")
    # cv2.destroyAllWindows()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
