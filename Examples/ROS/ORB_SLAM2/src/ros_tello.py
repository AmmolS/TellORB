import time, cv2
from threading import Thread
from djitellopy import Tello


import rospy
from std_msgs.msg import Empty, UInt8, Bool
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
#from h264_image_transport.msg import H264Packet

from cv_bridge import CvBridge, CvBridgeError

#import av
import math
import numpy as np
import threading
import time


#tello = Tello()
#Step 1 connect to Tello
#tello.connect()

#Step 2 publish images to node /camera/image_raw
#To do 



#Step 3 subscribe to commands published by mono_sub.cc
#code borrowed from : https://www.geeksforgeeks.org/ros-subscribers-using-python/ 

class command_subscriber:
  
    def __init__(self):
        # initialize the subscriber node
        # here we deal with messages of type String which are commands coming from subscriber.
        self.image_sub = rospy.Subscriber("tello/command", 
                                          String, self.process_command)
        print("Initializing the command subscriber!")
  
    def process_command(self, String):
        
        # now print what mono sub has sent.
        #rospy.get_caller_id must return command_subscriber....
        #http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
        rospy.loginfo(rospy.get_caller_id() + "The commands in coming are %s",
                      String)
  
  
def main():
    # create a  command subscriber instance
    sub = command_subscriber()
      
    print('Currently in the main function...')
      
    # initializing the subscriber node
    rospy.init_node('command_subscriber', anonymous=True)
    rospy.spin()
  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass