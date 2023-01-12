import os
import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import cv2
import signal



bridge = CvBridge()
pub_img = rospy.Publisher("camera/image_raw", Image, queue_size=10)

def main():
    print('ROS Tello Camera: Initializing ros_tello node...')
    rospy.init_node('ros_tello')
    
    print('ROS Tello Camera is recording, press any key to terminate recording thread.')

    cap = cv2.VideoCapture("http://192.168.2.26")

    fps_period = round(1000*(1/30)) # Write to video at around 60fps - this is experimental

    print("Got video capture")

    if cap.isOpened() is not True:
        print("Opening")
        cap.open("http://192.168.2.26")

    if cap.isOpened() is not True:
        print("Failed to open capture")

    while cap.isOpened():
        print("Getting frame")
        ret, frame = cap.read()

        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Retrying ...")
            #break
        img_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        pub_img.publish(img_msg)
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        cv2.imshow('frame', grey)
        
        if cv2.waitKey(fps_period) == ord('q'):
            break

    print("Exiting")
    cap.release()
    cv2.destroyAllWindows()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass