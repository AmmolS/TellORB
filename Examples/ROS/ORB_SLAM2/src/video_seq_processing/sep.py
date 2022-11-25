#The video is divided into picture program sep.py
# coding:utf-8
#Maitreyi trying to document stuff finally
#Step one: place your video and this script in same folder
#2: run sep.py from that folder: normal python command
#.before running replace by correct video name
#also u must create rgb folder in that sub folder else it wont run correctly
#once u run the script, u will get a rgb folder in that subfolder with a million images from the video
#then remove this script and video from this folder
#then copy file.py here
#run file.py
#at the end ur folder of interest should look like
#folder_of_interest
#       rgb
#           all images/pngs
#       times.txt
#when running ros_mono publisher pass this as input

import os
import cv2
 
 
def getName(num):
    if num <10:
        strRes = '0000' + str(num)
    elif num <100:
        strRes = '000' + str(num)
    elif num <1000:
        strRes = '00' + str(num)
    elif num <10000:
        strRes = '0' + str(num)
    return strRes
 
 
cap = cv2.VideoCapture('ravi_corridor2.avi')
# Get code rate and size
fps = cap.get(cv2.CAP_PROP_FPS)
size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
 
# Read frame
success, frame = cap.read()
idx = 1

while cap.isOpened:
    write_success = cv2.imwrite('rgb/' + getName(idx) +'.png', frame)
    # print(write_success);
    success, frame = cap.read() # Get the next frame
    idx = idx + 1
    cv2.imshow('frame', frame)
    cv2.waitKey(1)
