import os
import rospy
import time
import socket 

from sensor_msgs.msg import _Imu

# from cv_bridge import CvBridge, CvBridgeError

import cv2
from urllib.request import urlopen
import numpy as np

HOST = "192.168.4.1" # http://192.168.4.1/imu.txt
PORT = 63000

# bridge = CvBridge()
pub_img = rospy.Publisher("/imu0", _Imu.Imu, queue_size=10)

def main():
    print('ROS IMU: Initializing ros_tello node...')
    rospy.init_node('ros_imu')
    count = 0

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        # s.sendall(b"Hello, world")
        while True:
            data = s.recv(128)
            # print(f"Received {data!r}")
            print("{}: ".format(count) + data.decode("utf-8"))
            count += 1
            time.sleep(1/100)

    
    
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
