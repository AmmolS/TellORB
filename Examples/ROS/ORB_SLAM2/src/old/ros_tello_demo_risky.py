import rospy

from json import load
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
import time

from djitellopy import Tello
from threading import Thread, Event
import time, cv2
import signal
import logging
import readchar
MAX_ANGLE = 90



tello = Tello(skipAddressCheck=True)
tello.LOGGER.setLevel(logging.INFO)
tello.connect()
keepRecording = Event()
keepRecording.set()
keepAlive = Event()
keepAlive.set()

tello.streamon()
frame_read = tello.get_frame_read()
bridge = CvBridge()
pub_img = rospy.Publisher("tello/image_raw", Image, queue_size=10)

# load the config.json file
def loadConfig():
    with open('drone_config.json') as f:
        return load(f)
data = loadConfig()
rotationAngle = int(data["rotationAngle"])
height = int(data["height"])
sleepTime = int(data["sleep"])
tello.set_speed(int(data["speed"]))


print("Tello Battery Level = {}%".format(tello.get_battery()))

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
        
        # To handle commands
        # check if tello is busy (as a backup, in case for some reason ros_mono_sub/pub doesn't realize it is busy)
        # Use switch case to determine which command it is
        # Call the revelant DJITelloPy API function to have the command work
        # Set flag to indicate tello is busy
        # 

#to convert from commands of the form move up 20 cm to rc command with velocity
def distanceToRC(distance,speed):
    sleepTime = abs(distance/speed)
    print("moving up by %d in time %d\n",distance,sleepTime)
    tello.send_rc_control(0, 0, speed, 0)
    time.sleep(sleepTime)#time to travel this distance
    #send another rc to stop moving-hopefully it stops when desired distance is reached
    tello.send_rc_control(0, 0, 0, 0)

#to convert from commands of the form rotate x degress to rc command with angular velocity
def angleToRC(degree,angularSpeed):
    sleepTime = abs(degree/angularSpeed)
    print("rotating by %d in time %d\n",degree,sleepTime)
    tello.send_rc_control(0, 0, 0, angularSpeed)
    time.sleep(sleepTime)#time to rotate this angle
    #send another rc to stop moving-hopefully it stops when desired angle is reached
    tello.send_rc_control(0, 0, 0, 0)

def videoRecorder():
    # create a VideoWrite object, recoring to ./video.avi
    height, width, _ = frame_read.frame.shape
    
    print("Start recording")
    video = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (width, height))

    while keepRecording.is_set():
        img_msg = bridge.cv2_to_imgmsg(frame_read.frame, encoding='bgr8')
        pub_img.publish(img_msg)
        video.write(frame_read.frame)
        time.sleep(1 / 60)

    print("Stop recording")

    video.release()

def telloControl(recorder):

    angle = 0
    time.sleep(sleepTime)#from the config file
    #360 degree turn
    flightSpeed = 50
    while True:
        updownV = 0
        leftRightV = 0
        forwardBackwardV = 0
        yawV = 0
        #intial scan manouver complete, pass control to keyboard
        # if(numTimesExecute==0):
        inputChar = readchar.readchar()
        if inputChar == 'w':
            forwardBackwardV = flightSpeed
        elif inputChar == 's':
            forwardBackwardV = -flightSpeed
        elif inputChar == 'a':
            leftRightV = -flightSpeed
        elif inputChar == 'd':
            leftRightV = flightSpeed
        elif inputChar == 'z':
            updownV = 70
        elif inputChar == 'x':
            updownV = -70
        elif inputChar == 'q':
            yawV = -20
        elif inputChar == 'e':
            yawV = 20
        elif inputChar == 'o':
            flightSpeed += 10
        elif inputChar == 'p':
            flightSpeed -= 10
        elif inputChar == readchar.key.ENTER:
            tello.land()
            keepAlive.clear()
            keepRecording.clear()
            recorder.join()
            exit(1)
            # rospy.spin()
        elif inputChar == readchar.key.ESC:          # THIS IS A DANGEROUS COMMAND. ONLY USE WHEN DRONE HAS LANDED ALREADY FOR WHATEVER REASON (auto landing, crash landing, flew down too much, etc.) TO EXIT THE PROGRAM.
            keepAlive.clear()
            keepRecording.clear()
            recorder.join()
            exit(1)
            # rospy.spin()
        elif inputChar == 'n':
            angle = 0
            # tello.move("up", int(height - tello.get_height()))
            if(tello.get_height() < height):
                tello.move("up", int(height - tello.get_height()))
            time.sleep(sleepTime)#from the config file
            while angle < (MAX_ANGLE): #not sure if this will over rotate
                #angleToRC(rotationAngle,50)#got this from the code above
                #distanceToRC(20,speed) #move up
                #distanceToRC(20,(-speed))#move down
                print("Rotating: " + str(rotationAngle))

                tello.set_speed(20)
                tello.rotate_counter_clockwise(rotationAngle)
                tello.set_speed(flightSpeed)

                time.sleep(sleepTime)
                # print("Moving up:" + str(20))
                tello.move("up", 50)
                time.sleep(sleepTime)
                # print("Moving down: " + str(20))
                tello.move("down", 50)
                angle += rotationAngle
                print("Rotation angle is now " + str(angle))
                time.sleep(sleepTime)#from config file
        elif inputChar == 'm':
            angle = 0
            # tello.move("up", int(height - tello.get_height()))
            if(tello.get_height() < height):
                tello.move("up", int(height - tello.get_height()))
            time.sleep(sleepTime)#from the config file
            while angle < (MAX_ANGLE): #not sure if this will over rotate
                #angleToRC(rotationAngle,50)#got this from the code above
                #distanceToRC(20,speed) #move up
                #distanceToRC(20,(-speed))#move down
                print("Rotating: " + str(rotationAngle))
                tello.rotate_clockwise(rotationAngle)
                time.sleep(sleepTime)
                # print("Moving up:" + str(20))
                tello.move("up", 50)
                time.sleep(sleepTime)
                # print("Moving down: " + str(20))
                tello.move("down", 50)
                angle += rotationAngle
                print("Rotation angle is now " + str(angle))
                time.sleep(sleepTime)#from config file
        else:
            print("Tello Battery Level = {}%".format(tello.get_battery()))
        # PRESS SPACEBAR TO HOVER-old script execution here
        tello.send_rc_control(leftRightV, forwardBackwardV, updownV, yawV)
  
def main():
    # create a  command subscriber instance
    sub = command_subscriber()
      
    print('Currently in the main function...')
      
    # initializing the subscriber node
    rospy.init_node('ros_tello', anonymous=True)



    print("Press ENTER to initiate flight.", flush=True)
    inputChar = readchar.readchar()

    if inputChar == readchar.key.ENTER:
        tello.takeoff()
        # time.sleep(sleepTime)
        
        time.sleep(sleepTime)
        if(tello.get_height() < height):
                tello.move("up", int(height - tello.get_height()))
                time.sleep(sleepTime)
        
  
        recorder = Thread(target=videoRecorder)
        recorder.start()

        try:
            telloControl(recorder)
        except KeyboardInterrupt:
            telloControl(recorder)

    rospy.spin()
  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
