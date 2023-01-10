#!/bin/csh -f

#

# this is a comment

#





echo "Running MonoPub for you"

set recording = $1 
set folder = $2
cd ~/TellORB
if($1 == 0 ) then #0 means live no recording
rosrun ORB_SLAM2 Monopub Vocabulary/ORBvoc.txt Examples/Monocular/TELLO_Ammol.yaml -1 tello/image_raw


else 
echo "running pre recorded"
rosrun ORB_SLAM2 Monopub Vocabulary/ORBvoc.txt Examples/Monocular/TELLO_Ammol.yaml "$folder"
endif

