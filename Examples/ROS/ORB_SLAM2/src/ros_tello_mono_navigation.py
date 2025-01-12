# Tello - with flight controls - camera published - has turn maneuver 

import rospy

from json import load
from std_msgs.msg import Empty, UInt8, Bool, UInt32, String
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
from enum import Enum
class Tello_Modes(Enum):
    scale_computing = 0
    dfs = 1
    moving = 2
    
MAX_ANGLE = 54


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
pub_move_complete = rospy.Publisher("tello/move_complete",UInt32,queue_size=1)
commands_array=[]
all_commands_received = False


print("Tello Battery Level = {}%".format(tello.get_battery()))

#Step 3 subscribe to commands published by mono_sub.cc
#code borrowed from : https://www.geeksforgeeks.org/ros-subscribers-using-python/ 

class command_subscriber:
  
    def __init__(self):
        # initialize the subscriber node
        # here we deal with messages of type String which are commands coming from subscriber.
        self.command_sub = rospy.Subscriber("tello/command", 
                                          String, self.process_command)
        print("Initializing the command subscriber!")
  
    def process_command(self, data):
        
        # now print what mono sub has sent.
        #rospy.get_caller_id must return command_subscriber....
        #http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
        rospy.loginfo(rospy.get_caller_id() + "Received command %s",
                      data.data)
        #buffer commands till done is received, don't set mode until done is received
        if(data.data == "done"):
            print("received tello commands, ready to execute")
            if(len(commands_array) != 0):
                global all_commands_received
                all_commands_received = True
            else:
                print("Commands array is empty, setting mode back to dfs")
                pub_move_complete.publish(1)
        else:
            commands_array.append(data.data)

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
    
# def convertToCommand(command_string):
#     if(command_string == command_string.split(";")[3])
  
def main():
    # create a  command subscriber instance
    sub = command_subscriber()
      
    print('Currently in the main function...')
    # pub = rospy.Publisher("test/raw", String, queue_size=10)
    pub_img = rospy.Publisher("tello/image_raw", Image, queue_size=10)

    pub_initialize_scale = rospy.Publisher("tello/initialize_scale", Bool, queue_size=1)
      
    # initializing the subscriber node
    rospy.init_node('command_subscriber', anonymous=True)

    #Adding the code for drone rotation control
    #step one: load the json file with ideal drone parameters
    data = loadConfig()
    #step two get the drone parameter data into variables to be put into command
    # tello.speed = int(data["speed"])
    tello.set_speed(int(data["speed"]))
    rotationAngle = int(data["rotationAngle"])
    height = int(data["height"])
    sleepTime = int(data["sleep"])
    time.sleep(sleepTime)
    numTimesExecute = 1 #number of times to execute the loop

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
        tello.land()
        
        keepRecording.clear()
        recorder.join()
        print("Killing program")

    signal.signal(signal.SIGINT, exit_handler)

    print("Press ENTER to initiate rotation commands.", flush=True)
    inputChar = readchar.readchar()

    if inputChar == readchar.key.ENTER:
        tello.takeoff()
        # cv2.imwrite("picture.png", frame_read.frame)


        time.sleep(2)

        if(tello.get_height() < height - 21):
            tello.move("up", abs(int(height - tello.get_height())))
        elif(tello.get_height() > height + 21):
            tello.move("down", abs(int(height - tello.get_height())) + 5)
        droneUp = False
        #pub_initialize_scale.publish(True);
        #next set of manouvers
        angle = 0
        time.sleep(sleepTime)#from the config file
        #360 degree turn
        flightSpeed = 50
        while True:
            updownV = 0
            leftRightV = 0
            forwardBackwardV = 0
            yawV = 0
            print("Ready for command")
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
                if(tello.get_height() < height - 21):
                    tello.move("up", abs(int(height - tello.get_height())))
                elif(tello.get_height() > height + 21):
                    tello.move("down", abs(int(height - tello.get_height())) + 5)
                time.sleep(sleepTime)
                while angle < (MAX_ANGLE):
                    print("Rotating: " + str(rotationAngle))

                    tello.set_speed(20)
                    tello.rotate_counter_clockwise(rotationAngle)
                    tello.set_speed(flightSpeed)

                    time.sleep(sleepTime)
                    if droneUp:
                        tello.move("down", 25)
                    else:
                        tello.move("up", 25)
                    droneUp = not droneUp
                    time.sleep(sleepTime)
                        
                    angle += rotationAngle
                    print("Rotation angle is now " + str(angle))
                    time.sleep(sleepTime)
                if(tello.get_height() < height - 21):
                    tello.move("up", abs(int(height - tello.get_height())))
                elif(tello.get_height() > height + 21):
                    tello.move("down", abs(int(height - tello.get_height())) + 5)
            elif inputChar == 'm':
                angle = 0
                if(tello.get_height() < height - 21):
                    tello.move("up", abs(int(height - tello.get_height())))
                elif(tello.get_height() > height + 21):
                    tello.move("down", abs(int(height - tello.get_height())) + 5)
                time.sleep(sleepTime)#from the config file
                while angle < (MAX_ANGLE): #not sure if this will over rotate
                    print("Rotating: " + str(rotationAngle))

                    tello.set_speed(20)
                    tello.rotate_clockwise(rotationAngle)
                    tello.set_speed(flightSpeed)

                    time.sleep(sleepTime)
                    if droneUp:
                        tello.move("down", 25)
                    else:
                        tello.move("up", 25)
                    droneUp = not droneUp
                    time.sleep(sleepTime)

                    angle += rotationAngle
                    print("Rotation angle is now " + str(angle))
                    time.sleep(sleepTime)#from config file
                if(tello.get_height() < height - 21):
                    tello.move("up", abs(int(height - tello.get_height())))
                elif(tello.get_height() > height + 21):
                    tello.move("down", abs(int(height - tello.get_height())) + 5)
            elif inputChar == 'l':
                pub_initialize_scale.publish(True)
                tello.move_forward(100)

                print("Press l to move back 50cm, 0 to cancel")
                inputChar = readchar.readchar()
                while(inputChar != 'l' and inputChar != '0'):
                    print("Press l to move back 50cm, 0 to cancel")
                    inputChar = readchar.readchar()
                
                if(inputChar == 'l'):
                    pub_initialize_scale.publish(True)
                    time.sleep(0.05)
                    tello.move_back(50)

                print("Press l to finish scaling, 0 to cancel")
                inputChar = readchar.readchar()
                while(inputChar != 'l' and inputChar != '0'):
                    print("Press l to finish scaling, 0 to cancel")
                    inputChar = readchar.readchar()

                # if (inputChar == 'l'):
                #     pub_initialize_scale.publish(True)
                print("Providing manual control again")
            elif inputChar == 'b':
                global all_commands_received
                if all_commands_received:
                    print("This is your commands list: ")
                    print(commands_array)
                    # print("Press c to continue with list, press 0 to escape")
                    # inputChar = readchar.readchar()
                    # while(inputChar != 'c' and inputChar != '0'):
                        # print("Press c to continue with list, press 0 to escape")
                        # inputChar = readchar.readchar()
                    # if(inputChar == 'c'):
                    for i in commands_array:
                        print("Executing " + i + ", press c to execute, 0 to stop")
                        inputChar = readchar.readchar()
                        while(inputChar != 'c' and inputChar != '0'):
                            print("Executing " + i + ", press c to execute, 0 to stop")
                            inputChar = readchar.readchar()
                        if(inputChar == 'c'):
                            command = i.split(" ")
                            if(command[0] == "forward"):
                                # tello.move("forward", int(command[1]))
                                tello.move("forward", 55)
                                print("Moving forward by 55")
                            elif(command[0] == "ccw"):
                                for i in range(int(int(command[1])/rotationAngle)):
                                    print("Rotating ccw by " + str(int(rotationAngle)))
                                    tello.set_speed(20)
                                    tello.rotate_counter_clockwise(rotationAngle)
                                    tello.set_speed(flightSpeed)

                                    time.sleep(sleepTime)
                                    if droneUp:
                                        tello.move("down", 25)
                                    else:
                                        tello.move("up", 25)
                                    droneUp = not droneUp
                                    time.sleep(sleepTime)
                                print("Rotating ccw by " + str(int(int(command[1])%rotationAngle)))
                                tello.rotate_counter_clockwise(int(int(command[1])%rotationAngle))
                            elif(command[0] == "cw"):
                                for i in range(int(int(command[1])/rotationAngle)):
                                    print("Rotating cw by " + str(int(rotationAngle)))
                                    tello.set_speed(20)
                                    tello.rotate_clockwise(rotationAngle)
                                    tello.set_speed(flightSpeed)

                                    time.sleep(sleepTime)
                                    if droneUp:
                                        tello.move("down", 25)
                                    else:
                                        tello.move("up", 25)
                                    droneUp = not droneUp
                                    time.sleep(sleepTime)
                                print("Rotating cw by " + str(int(int(command[1])%rotationAngle)))
                                tello.rotate_clockwise(int(int(command[1])%rotationAngle))
                            time.sleep(1)
                        elif(inputChar == '0'):
                            print("Exiting commands list...")
                            break
                    print("Completed commands list")
                    commands_array.clear()
                    all_commands_received = False
                    print("Setting mode back to dfs")
                    pub_move_complete.publish(1)




            else:
                print("Tello Battery Level = {}%".format(tello.get_battery()))
            # PRESS SPACEBAR TO HOVER-old script execution here
            tello.send_rc_control(leftRightV, forwardBackwardV, updownV, yawV)



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
