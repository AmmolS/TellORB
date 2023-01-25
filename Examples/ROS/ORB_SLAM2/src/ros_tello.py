import rospy

from json import load
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
MAX_ANGLE = 90

tello = Tello(skipAddressCheck=True)
tello.LOGGER.setLevel(logging.INFO)
tello.connect()
print("Tello Battery Level = {}%".format(tello.get_battery()))
keepRecording = Event()
keepRecording.set()
keepHandling = Event()
keepHandling.set()
tello.streamon()
frame_read = tello.get_frame_read()
bridge = CvBridge()

command = ""
rotationAngle = 15
height = 20
sleepTime = 1
flightSpeed = 50

ws = websocket.WebSocket()
ws.connect("ws://192.168.4.1")

# load the config.json file
def loadConfig():
    with open('drone_config.json') as f:
        data = load(f)
        tello.set_speed(int(data["speed"]))
        global rotationAngle, height, sleepTime, flightSpeed
        rotationAngle = int(data["rotationAngle"])
        height = int(data["height"])
        sleepTime = int(data["sleep"])
        flightSpeed = int(data["speed"])

def telloHandler():
    while True:
        while command is Empty:
            time.sleep(0.5)
        
        if command == "forward":
            print("Moving forward")
            tello.move_forward(20)
        elif command == "backward":
            print("Moving backward")
            tello.move_back(20)
        elif command == "left":
            print('Moving left')
            tello.move_left(20)
        elif command == "right":
            print('Moving right')
            tello.move_right(20)
        elif command == "turn left":
            print("Turning left")
            angle = 0
            if(tello.get_height() < height):
                tello.move("up", int(height - tello.get_height()))
            time.sleep(sleepTime)
            while angle < (MAX_ANGLE):
                print("Rotating: " + str(rotationAngle))

                tello.set_speed(20)
                tello.rotate_counter_clockwise(rotationAngle)
                tello.set_speed(flightSpeed)

                time.sleep(sleepTime)
                tello.move("up", 50)
                time.sleep(sleepTime)
                tello.move("down", 50)
                angle += rotationAngle
                print("Rotation angle is now " + str(angle))
                time.sleep(sleepTime)
        elif command == "turn right":
            print("Turning right")
            angle = 0
            if(tello.get_height() < height):
                tello.move("up", int(height - tello.get_height()))
            time.sleep(sleepTime)
            while angle < (MAX_ANGLE):
                print("Rotating: " + str(rotationAngle))

                tello.set_speed(20)
                tello.rotate_clockwise(rotationAngle)
                tello.set_speed(flightSpeed)

                time.sleep(sleepTime)
                tello.move("up", 50)
                time.sleep(sleepTime)
                tello.move("down", 50)
                angle += rotationAngle
                print("Rotation angle is now " + str(angle))
                time.sleep(sleepTime)
        elif command == "land":
            tello.land()
        else:
            print('Command received through ROS is forbidden or does not exist.')

        command = ""
        print("Tello Battery Level = {}%".format(tello.get_battery()))
telloCommandHandler = Thread(target=telloHandler)


#Step 3 subscribe to commands published by mono_sub.cc
#code borrowed from : https://www.geeksforgeeks.org/ros-subscribers-using-python/ 

class command_subscriber:
    def __init__(self):
        self.image_sub = rospy.Subscriber("tello/command", String, self.process_command)
        print("Initializing the command subscriber!")
        telloCommandHandler.start()

    def process_command(self, inputCommand):
        rospy.loginfo(rospy.get_caller_id() + "The commands in coming are %s", inputCommand)
        global command
        command = inputCommand

def main():
    pub_img = rospy.Publisher("tello/image_raw", Image, queue_size=10)
    pub_imu = rospy.Publisher("/imu0", _Imu.Imu, queue_size=10)

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
      
    # initializing the subscriber node
    rospy.init_node('command_subscriber', anonymous=True)

    def readImu():
        seq = 0
        oldTime = 0
        while keepRecording.is_set():
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
                # print(imu_msg)

                pub_imu.publish(imu_msg)
                seq += 1
                oldTime = time.time()

    loadConfig()
    time.sleep(1)

    recorder = Thread(target=videoRecorder)
    recorder.start()

    imu = Thread(target=readImu)
    imu.start()

    def exit_handler(signum, frame):
        tello.land()
        keepRecording.clear()
        recorder.join()
        keepHandling.clear()
        telloCommandHandler.join()
        imu.join()
        print("Killing program")
        exit(1)

    signal.signal(signal.SIGINT, exit_handler)

    print("Press ENTER to initiate rotation commands.", flush=True)
    inputChar = readchar.readchar()

    if inputChar == readchar.key.ENTER:
        tello.takeoff()

    # Create a command subscriber instance
    sub = command_subscriber()

    while recorder.is_alive():
        time.sleep(0.2)

    if (keepRecording.is_set() == True):
        keepRecording.clear()

    if (keepHandling.is_set() == True):
        keepHandling.clear()

    recorder.join()
    telloCommandHandler.join()
    imu.join()

    rospy.spin()
  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
