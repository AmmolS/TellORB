import rospy

from std_msgs.msg import Empty, UInt8, Bool
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import Image, _Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
#from h264_image_transport.msg import H264Packet

from cv_bridge import CvBridge, CvBridgeError

#import av
import math
import numpy as np
import time

from djitellopy import Tello
from threading import Thread, Event
import time, cv2
import signal
import logging
import readchar
import websocket

ws = websocket.WebSocket()
ws.connect("ws://192.168.4.1")


seq = 10001
rate = 1.0/20
oldTime = 0

while True:
    result = ws.recv()
    if(time.time() - oldTime >= rate):
        imu_msg = _Imu.Imu()
        imu_msg.header.seq = seq
        imu_msg.header.stamp.set(math.floor(time.time()), time.time_ns() % 1000000000)
        imu_msg.header.frame_id = "imu4"
        imu_msg.orientation.z = 1
        imu_msg.orientation_covariance[0] = 99999.9
        imu_msg.orientation_covariance[4] = 99999.9
        imu_msg.orientation_covariance[8] = 99999.9

        imu_msg.linear_acceleration.x = float(result.split('Ax: ')[1][:result.split('Ax: ')[1].find(',')])
        imu_msg.linear_acceleration.y = float(result.split('Ay: ')[1][:result.split('Ay: ')[1].find(',')])
        imu_msg.linear_acceleration.z = float(result.split('Az: ')[1][:result.split('Az: ')[1].find(' ')])
        imu_msg.angular_velocity.x = float(result.split('Rx: ')[1][:result.split('Rx: ')[1].find(',')])
        imu_msg.angular_velocity.y = float(result.split('Ry: ')[1][:result.split('Ry: ')[1].find(',')])
        imu_msg.angular_velocity.z = float(result.split('Rz: ')[1][:result.split('Rz: ')[1].find(' ')])

        pub_imu.publish(imu_msg)
        seq += 1
        oldTime = time.time()