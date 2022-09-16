from djitellopy import Tello
from threading import Thread, Event
import time, cv2
import signal
import logging

tello = Tello(skipAddressCheck=True)
tello.LOGGER.setLevel(logging.INFO)
tello.connect()
keepRecording = Event()
keepRecording.set()

tello.streamon()
frame_read = tello.get_frame_read()

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

# we need to run the recorder in a seperate thread, otherwise blocking options
#  would prevent frames from getting added to the video

time.sleep(2)

recorder = Thread(target=videoRecorder)
recorder.start()

def exit_handler(signum, frame):
    msg = "Stopping drone. Drone will now hover. Please shutdown manually by pressing the button on the drone"
    print(msg, end="", flush=True)
    keepRecording.clear()
    tello.streamoff()
    recorder.join()
    exit(1)

signal.signal(signal.SIGINT, exit_handler)

while recorder.is_alive():
    time.sleep(0.2)


# tello.takeoff()
# cv2.imwrite("picture.png", frame_read.frame)

# time.sleep(3)

# cv2.imwrite("picture2.png", frame_read.frame)

# tello.move("up", 80)
# cv2.imwrite("picture3.png", frame_read.frame)

# tello.move("forward", 50)
# cv2.imwrite("picture4.png", frame_read.frame)

# tello.move("left", 30)
# cv2.imwrite("picture5.png", frame_read.frame)

# tello.move("right", 60)
# cv2.imwrite("picture6.png", frame_read.frame)

# tello.move("left", 30)
# cv2.imwrite("picture7.png", frame_read.frame)

# tello.move("up", 20)
# cv2.imwrite("picture8.png", frame_read.frame)

# tello.move("back", 50)
# cv2.imwrite("picture9.png", frame_read.frame)


# tello.land()

if (keepRecording.is_set() == True):
    keepRecording.clear()
    tello.streamoff()

recorder.join()



