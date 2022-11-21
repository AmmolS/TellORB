import os
import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import math

from djitellopy import Tello
from threading import Thread, Event
import cv2
import signal
import logging

tello = Tello(skipAddressCheck=True)
tello.LOGGER.setLevel(logging.INFO)
tello.connect()
keepRecording = Event()
keepRecording.set()

tello.streamon()
frame_read = tello.get_frame_read()
bridge = CvBridge()
pub_img = rospy.Publisher("camera/image_raw", Image, queue_size=10)

print("Tello Battery Level = {}%".format(tello.get_battery()))

def videoRecorder():
    # create a VideoWrite object, recoring to ./video.avi
    height, width, _ = frame_read.frame.shape
    
    print("Start video recording")
    video = cv2.VideoWriter('video4.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (width, height))
    fps_period = round(1000*(1/60)) # Write to video at around 60fps - this is experimental

    while keepRecording.is_set():
        img_msg = bridge.cv2_to_imgmsg(frame_read.frame, encoding='bgr8')
        pub_img.publish(img_msg)
        cv2.imshow("Tello Camera", frame_read.frame) # Display to window
        video.write(frame_read.frame)
        cv2.waitKey(fps_period)

    cv2.destroyAllWindows()
    print("Stop video recording. Video saved to " + os.getcwd() + "/video.avi")

    video.release()


            

def main():
    print('ROS Tello Camera: Initializing ros_tello node...')
    rospy.init_node('ros_tello')

    recorder = Thread(target=videoRecorder, name="tello camera recorder")
    recorder.start()
    def exit_handler(signum, frame):
        msg = "Stopping program..."
        print(msg, flush=True)
        keepRecording.clear()
        recorder.join()
        tello.streamoff()
        # print("Terminating Tello instance. This may take a few seconds...")
        # tello.end()
        print("Recording thread and Tello instance were terminated. Press Ctrl+C to kill program now...")
    
    signal.signal(signal.SIGINT, exit_handler)

    
    print('ROS Tello Camera is recording, press Ctrl+C to terminate recording thread.')
    recorder.join()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass