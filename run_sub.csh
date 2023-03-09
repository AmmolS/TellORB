#!/bin/csh -f

#

# this is a comment

#





echo "Running MonoSub for you"



# rosrun ORB_SLAM2 Monosub 45 1 29 -25 48 -12 0.55 0.50 1 5 0 0 0 0 350 1 1 3 > output.txt


rosrun ORB_SLAM2 MonosubModified 45 1 29 -25 48 -12 0.80 0.40 1 30 1 1 1 80 300 0 1 3 > output.txt
