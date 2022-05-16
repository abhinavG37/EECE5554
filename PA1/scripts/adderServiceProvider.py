from __future__ import print_function

import rospy
from PA1.srv import adderService, adderServiceResponse


def requestHandler(req):
    sum = (req.a+req.b)
    rospy.loginfo("Summed {} and {} to return {}".format(req.a, req.b, sum))
    return adderServiceResponse(sum)
    
def adderServer():
    rospy.init_node('adderServiceProvider')
    serv = rospy.Service('adder_Service',adderService, requestHandler)
    print("Ready to add")
    rospy.spin()

if __name__ == '__main__':
    adderServer()
    