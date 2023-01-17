import os
import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import cv2
from urllib.request import urlopen
import numpy as np


bridge = CvBridge()
pub_img = rospy.Publisher("camera/image_raw", Image, queue_size=10)

def main():
    print('ROS Tello Camera: Initializing ros_tello node...')
    rospy.init_node('ros_tello')
    
    print('ROS Tello Camera is recording, press any key to terminate recording thread.')

    url = r'http://192.168.4.1/cam-hi.jpg'
    while True:
        img_resp = urlopen(url)
        imgnp = np.asarray(bytearray(img_resp.read()), dtype="uint8")
        img = cv2.imdecode(imgnp, -1)
        img_msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
        pub_img.publish(img_msg)
        cv2.imshow("Camera", img)
        if cv2.waitKey(1) == 113:
            break

    print("Exiting")
    cv2.destroyAllWindows()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass