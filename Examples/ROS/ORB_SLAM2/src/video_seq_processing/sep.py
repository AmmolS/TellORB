#The video is divided into picture program sep.py
# coding:utf-8
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
 
 
cap = cv2.VideoCapture('lab_video.avi')
# Get code rate and size
fps = cap.get(cv2.CAP_PROP_FPS)
size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
 
# Read frame
success, frame = cap.read()
idx = 1

while cap.isOpened:
    write_success = cv2.imwrite('rgb/' + getName(idx) +'.png', frame)
    print(write_success);
    success, frame = cap.read() # Get the next frame
    idx = idx + 1
    cv2.imshow('frame', frame)
    cv2.waitKey(100)
