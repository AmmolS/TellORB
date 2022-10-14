from turtle import forward
from djitellopy import Tello
from threading import Thread, Event
import time, cv2
import signal
import logging
import readchar

tello = Tello(skipAddressCheck=True)
tello.LOGGER.setLevel(logging.INFO)
tello.connect()
keepRecording = Event()
keepRecording.set()
keepAlive = Event()
keepAlive.set()

tello.streamon()
frame_read = tello.get_frame_read()

print("Tello Battery Level = {}%".format(tello.get_battery()))

def videoRecorder():
    # create a VideoWrite object, recoring to ./video.avi
    height, width, _ = frame_read.frame.shape
    
    print("Start recording")
    video = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (width, height))

    while keepRecording.is_set():
        video.write(frame_read.frame)
        time.sleep(1 / 60)

    print("Stop recording")

    video.release()
    tello.streamoff()

# we need to run the recorder in a seperate thread, otherwise blocking options
#  would prevent frames from getting added to the video

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
    msg = "Stopping drone. Drone will now hover. Please shutdown manually by pressing the button on the drone"
    print(msg, flush=True)
    keepRecording.clear()
    keepingAlive = Thread(target=exitCatcher)
    keepingAlive.start()
    # try:
    #     inputChar = readchar.readchar()
    # except KeyboardInterrupt:
    #     print("Exiting keepAlive")
    #     keepAlive.clear()
    #     keepingAlive.join()
    #     print("Killing program")
    #     exit(1)
    
    try:
        while True:
            inputChar = readchar.readchar()
            updownV = 0
            leftRightV = 0
            forwardBackwardV = 0
            yawV = 0
            if inputChar == 'w':
                forwardBackwardV = 30
            elif inputChar == 's':
                forwardBackwardV = -30
            elif inputChar == 'a':
                leftRightV = -30
            elif inputChar == 'd':
                leftRightV = 30
            elif inputChar == 'z':
                updownV = 30
            elif inputChar == 'x':
                updownV = -30
            elif inputChar == 'q':
                yawV = -30
            elif inputChar == 'e':
                yawV = 30
            elif inputChar == readchar.key.ENTER:
                tello.land()
                keepAlive.clear()
                recorder.join()
                keepAlive.join()
                exit(1)
            elif inputChar == readchar.key.ESC:
                exit(1)
            # PRESS SPACEBAR TO HOVER
            tello.send_rc_control(leftRightV, forwardBackwardV, updownV, yawV)
    except KeyboardInterrupt:
        print("Exiting keepAlive")
        keepAlive.clear()
        recorder.join()
        keepingAlive.join()
        print("Killing program")
        exit(1)

signal.signal(signal.SIGINT, exit_handler)



tello.takeoff()
cv2.imwrite("picture.png", frame_read.frame)

time.sleep(3)

cv2.imwrite("picture2.png", frame_read.frame)

tello.move("up", 80)
cv2.imwrite("picture3.png", frame_read.frame)

tello.move("forward", 40)
cv2.imwrite("picture4.png", frame_read.frame)

tello.move("left", 20)
cv2.imwrite("picture5.png", frame_read.frame)

tello.move("right", 40)
cv2.imwrite("picture6.png", frame_read.frame)

tello.move("left", 20)
cv2.imwrite("picture7.png", frame_read.frame)

tello.move("up", 20)
cv2.imwrite("picture8.png", frame_read.frame)

tello.move("back", 30)
cv2.imwrite("picture9.png", frame_read.frame)

print("Landing drone!")
tello.land()


while recorder.is_alive():
    time.sleep(0.2)

    
if (keepRecording.is_set() == True):
    keepRecording.clear()

recorder.join()



