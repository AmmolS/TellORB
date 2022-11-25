import rospy, cv2, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import sys, getopt

def main(argv):
    inputfile = ''
    try:
        opts, args = getopt.getopt(argv,"hi:",["ifile="])
    except getopt.GetoptError:
        print('test.py -i <inputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('test.py -i <inputfile>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
    print('Input file is ', inputfile)

    bridge = CvBridge()
    rospy.init_node('ros_video_player', anonymous=True)
    pub_img = rospy.Publisher("tello/image_raw", Image, queue_size=10)
    videocap = cv2.VideoCapture(inputfile)

    success, image = videocap.read()
    count = 0
    while success:
        kernel = np.array([[0, -1, 0],
                   [-1, 5,-1],
                   [0, -1, 0]])
        image_sharp = cv2.filter2D(src=image, ddepth=-1, kernel=kernel)
        img_msg = bridge.cv2_to_imgmsg(image_sharp, encoding='bgr8')
        pub_img.publish(img_msg)
        # time.sleep(1/90)
        # cv2.imwrite("picture_unsharpened.jpg", image)
        # cv2.imwrite("picture_sharpened.jpg", image_sharp)
        # if cv2.waitKey(10) == 27:                     # exit if Escape is hit
        #     break
        success, image = videocap.read()
        count += 1
    print("Outputted {} frames".format(count))

if __name__ == "__main__":
   main(sys.argv[1:])