import os
import rospy
import time
import websocket
import math


from sensor_msgs.msg import _Imu

# from cv_bridge import CvBridge, CvBridgeError

import cv2
from urllib.request import urlopen
import numpy as np

ws = websocket.WebSocket()
ws.connect("ws://192.168.4.1")

# bridge = CvBridge()
pub_imu = rospy.Publisher("/imu0", _Imu.Imu, queue_size=10)

def main():
    print('ROS IMU: Initializing ros_tello node...')
    rospy.init_node('ros_imu')

    seq = 0
    oldTime = 0
    while True:
        result = ws.recv()
        if(time.time() - oldTime >= 1/200):
            # print(time.time()-oldTime)
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
            print(imu_msg)

            pub_imu.publish(imu_msg)
            seq += 1
            oldTime = time.time()
            # time.sleep(fps)
            # print(result)

    # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #     s.connect((HOST, PORT))
    #     # s.sendall(b"Hello, world")
    #     while True:
    #         data = s.recv(128)
    #         # print(f"Received {data!r}")
    #         print("{}: ".format(count) + data.decode("utf-8"))
    #         count += 1
    #         time.sleep(1/100)

    
    
    print('ROS Tello Camera is recording, press any key to terminate recording thread.')

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
