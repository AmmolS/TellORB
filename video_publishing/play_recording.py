import rospy, cv2, time, signal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import sys, getopt

def exit_handler(signum, frame):
    exit(1)

signal.signal(signal.SIGINT, exit_handler)

def main(argv):
    inputfile = ''
    try:
        opts, args = getopt.getopt(argv,"hi:",["ifile="])
    except getopt.GetoptError:
        print('play_recording.py -i <inputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('play_recording.py -i <inputfile>')
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
        # kernel = np.array([[0, -1, 0],
        #            [-1, 5,-1],
        #            [0, -1, 0]])
        # image_cropped = image[0:550]
        # image_sharp = cv2.filter2D(src=image, ddepth=-1, kernel=kernel)
    #     # converting to LAB color space
    #     lab = cv2.cvtColor(image_sharp, cv2.COLOR_BGR2LAB)
    #     l_channel, a, b = cv2.split(lab)

    #     # Applying CLAHE to L-channel
    #     # feel free to try different values for the limit and grid size:
    #     clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    #     cl = clahe.apply(l_channel)

    #     # merge the CLAHE enhanced L-channel with the a and b channel
    #     limg = cv2.merge((cl,a,b))

        # # Converting image from LAB Color model to BGR color spcae
        # enhanced_img = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

        # print("original shape: ", image.shape)
        # print("sharpened shape: ", image_sharp.shape)
        # print("contrasted shape: ", enhanced_img.shape)
        # Stacking the original image with the enhanced image
        # edges = cv2.Canny(image_sharp, 150, 400)
        # edges2 = cv2.Canny(image_sharp, 100, 200)
        # edges3 = cv2.Canny(image, 150, 400)
        # edges4 = cv2.Canny(image, 100, 200)
        
        img_msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')
        # result = np.hstack((edges, edges2, edges3, edges4))
        # img_msg = bridge.cv2_to_imgmsg(result, encoding='mono8')

        pub_img.publish(img_msg)
        time.sleep(1/30)
        # cv2.imwrite("picture_unsharpened.jpg", image)
        # cv2.imwrite("picture_sharpened.jpg", image_sharp)
        # cv2.imwrite("picture_final.jpg", enhanced_img)
        # if cv2.waitKey(10) == 27:                     # exit if Escape is hit
        #     break
        success, image = videocap.read()
        count += 1
    print("Outputted {} frames".format(count))

if __name__ == "__main__":
   main(sys.argv[1:])