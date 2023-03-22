import rospy
from std_msgs.msg import Bool
from time import sleep

class does_nothing:
    def __init__(self):
        pass

    def process_command(self, inputCommand):
        # rospy.loginfo(rospy.get_caller_id() + "The commands in coming are %s", inputCommand)
        # global command
        # command = inputCommand
        pass

def main():
    pub_annotate = rospy.Publisher("map/annotate", Bool, queue_size=10)
    rospy.init_node('does_nothing', anonymous=True)

    while True:
        pub_annotate.publish(True)
        sleep(2)

    

    rospy.spin()
  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
