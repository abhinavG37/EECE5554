import rospy 
from std_msgs.msg import String
def talker():
    publisher  = rospy.Publisher('talkerTopic', String, queue_size=5)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        str = "Message sent at {}".format(rospy.get_time())
        rospy.loginfo(str) 
        #print also works for logging
        publisher.publish(str)
        rate.sleep()

def main():
    print("Executing program")
    talker()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
