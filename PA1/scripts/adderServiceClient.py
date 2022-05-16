from __future__ import print_function

import rospy
from PA1.srv import *


def serviceClient(x,y):
    rospy.init_node("adderServiceClient")
    rospy.wait_for_service('adder_Service')
    try:
        serviceHandle = rospy.ServiceProxy('adder_Service', adderService)
        response =serviceHandle(x,y)
        return response.sum
        rospy.loginfo("Summed {} and {} to return {}".format(req.a, req.b, sum))
    except rospy.ServiceException as e:
       print("Service call failed")


def usage():
    return "%s [x y]"%sys.argv[0]
    
if __name__ == '__main__':
    if(len(sys.argv)==3):
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit()
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, serviceClient(x, y)))
   

    