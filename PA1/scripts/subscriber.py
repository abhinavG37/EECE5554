import rospy 
from std_msgs.msg import String

def listenerCallback(message):
    rospy.loginfo("From Caller: {} \n Listener Received {}".format(rospy.get_caller_id(),message.data))

def listener():
    
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("talkerTopic", String, callback = listenerCallback)
    rospy.spin()

def main():
    print("Executing listener")
    listener()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
