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
MAX_ANGLE = 360



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

# load the config.json file
def loadConfig():
    with open('drone_config.json') as f:
        return load(f)

#to convert from commands of the form move up 20 cm to rc command with velocity
def distanceToRC(distance,speed):
    time = distance/speed
    print("moving up by %d in time %d\n",distance,time)
    tello.send_rc_control(0, 0, speed, 0)
    time.sleep(time)#time to travel this distance
    #send another rc to stop moving-hopefully it stops when desired distance is reached
    tello.send_rc_control(0, 0, 0, 0)

#to convert from commands of the form rotate x degress to rc command with angular velocity
def angleToRC(degree,angularSpeed):
    time = degree/angularSpeed
    print("rotating by %d in time %d\n",degree,time)
    tello.send_rc_control(0, 0, 0, angularSpeed)
    time.sleep(time)#time to rotate this angle
    #send another rc to stop moving-hopefully it stops when desired angle is reached
    tello.send_rc_control(0, 0, 0, 0)
    

  
def main():
    # create a  command subscriber instance
    sub = command_subscriber()
      
    print('Currently in the main function...')
    # pub = rospy.Publisher("test/raw", String, queue_size=10)
    pub_img = rospy.Publisher("tello/image_raw", Image, queue_size=10)
      
    # initializing the subscriber node
    rospy.init_node('command_subscriber', anonymous=True)
    
    # the following commented code is for debugging rospy
    # rospy.loginfo("Sending \"hello world!\"")
    # image = Image()
    # image.height = 100
    # image.width = 300
    # for i in range(10):
    #     time.sleep(0.5)
    #     pub.publish("hello world at {} times".format(i))
    #     pub_img.publish(image)

    #Adding the code for drone rotation control
    #step one: load the json file with ideal drone parameters
    data = loadConfig()
    #step two get the drone parameter data into variables to be put into command
    speed = int(data["speed"])
    rotationAngle = int(data["rotationAngle"])
    height = int(data["height"])
    sleepTime = int(data["sleep"])
    numTimesExecute = 1 #number of times to execute the loop

    def videoRecorder():
        # create a VideoWrite object, recoring to ./video.avi
        height, width, _ = frame_read.frame.shape
        
        print("Start recording")
        video = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (width, height))

        while keepRecording.is_set():
            img_msg = bridge.cv2_to_imgmsg(frame_read.frame, encoding='rgb8')
            pub_img.publish(img_msg)
            video.write(frame_read.frame)
            time.sleep(1 / 60)

        print("Stop recording")

        video.release()

    # we need to run the recorder in a seperate thread, otherwise blocking options
    #  would prevent frames from getting added to the video

    # this delay is for legacy testing purposes. It may be removed if it causes issues or if it doesn't cause issues to remove it down the road

    time.sleep(2)

    recorder = Thread(target=videoRecorder)
    recorder.start()

    def exitCatcher():
        while keepAlive.is_set():
            try:
                tello.send_keepalive()
            except Exception:
                print("Keeping alive")
            time.sleep(10)


    def exit_handler(signum, frame):
        msg = "Stopping drone. Drone will now hover.\n W = forward\nS = backwards\nA = left\nD = right\nQ = turn left\nE = turn right\nZ = ascend\nX = descend\nENTER = land\nSPACEBAR = hover\nPlease shutdown manually by pressing the button on the drone or press ENTER to land the drone."
        print(msg, flush=True)
        keepingAlive = Thread(target=exitCatcher)
        keepingAlive.start()

        try:
            while True:
                updownV = 0
                leftRightV = 0
                forwardBackwardV = 0
                yawV = 0
                #intial scan manouver complete, pass control to keyboard
                if(numTimesExecute==0):
                    inputChar = readchar.readchar()
                    if inputChar == 'w':
                        forwardBackwardV = 100
                    elif inputChar == 's':
                        forwardBackwardV = -100
                    elif inputChar == 'a':
                        leftRightV = -100
                    elif inputChar == 'd':
                        leftRightV = 100
                    elif inputChar == 'z':
                        updownV = 70
                    elif inputChar == 'x':
                        updownV = -70
                    elif inputChar == 'q':
                        yawV = -50
                    elif inputChar == 'e':
                        yawV = 50
                    elif inputChar == readchar.key.ENTER:
                        tello.land()
                        keepAlive.clear()
                        keepRecording.clear()
                        recorder.join()
                        rospy.spin()
                    elif inputChar == readchar.key.ESC:          # THIS IS A DANGEROUS COMMAND. ONLY USE WHEN DRONE HAS LANDED ALREADY FOR WHATEVER REASON (auto landing, crash landing, flew down too much, etc.) TO EXIT THE PROGRAM.
                        keepAlive.clear()
                        keepRecording.clear()
                        recorder.join()
                        rospy.spin()
                    else:
                        print("Tello Battery Level = {}%".format(tello.get_battery()))
                    # PRESS SPACEBAR TO HOVER-old script execution here
                    tello.send_rc_control(leftRightV, forwardBackwardV, updownV, yawV)
                
                else:
                    #initial hard coded movements
                    #changing the command here to enable drone movement as per config file-converting to velocity from distance 
                    #for rc commands to work
                    distanceUp = int(height - drone.get_height())
                    distanceToRC(distanceUp,speed) #function to issue appropriate rc command, speed comes from config file
                    #next set of manouvers
                    angle = 0
                    sleep(sleepTime)#from the config file
                    #360 degree turn
                    while angle <= (MAX_ANGLE + rotationAngle): #not sure if this will over rotate
                        angleToRC(rotationAngle,50)#got this from the code above
                        distanceToRC(20,updownV) #move up
                        distanceToRC(20,(-updownV))#move down
                        angle += rotationAngle
                        time.sleep(sleepTime)#from config file
                    numTimesExecute = numTimesExecute - 1 #one round of manouver complete

        except KeyboardInterrupt:
            print("Exiting keepAlive")
            keepAlive.clear()
            keepRecording.clear()
            recorder.join()
            print("Killing program")
            rospy.spin()

    signal.signal(signal.SIGINT, exit_handler)



    tello.takeoff()
    cv2.imwrite("picture.png", frame_read.frame)

    time.sleep(3)

    # You can add forced commands here if you want it to run by script (though it may fail in bad lighting or other issues)
    # Most reliable way to control drone is with `send_rc` commands rather then `move` commands

    # cv2.imwrite("picture2.png", frame_read.frame)

    # tello.move("up", 80)
    # cv2.imwrite("picture3.png", frame_read.frame)

    # tello.move("forward", 40)
    # cv2.imwrite("picture4.png", frame_read.frame)

    # tello.move("left", 20)
    # cv2.imwrite("picture5.png", frame_read.frame)

    # tello.move("right", 40)
    # cv2.imwrite("picture6.png", frame_read.frame)

    # tello.move("left", 20)
    # cv2.imwrite("picture7.png", frame_read.frame)

    # tello.move("up", 20)
    # cv2.imwrite("picture8.png", frame_read.frame)

    # tello.move("back", 30)
    # cv2.imwrite("picture9.png", frame_read.frame)

    # print("Landing drone!")
    # tello.land()


    while recorder.is_alive():
        time.sleep(0.2)

        
    if (keepRecording.is_set() == True):
        keepRecording.clear()

    recorder.join()


    rospy.spin()
  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass