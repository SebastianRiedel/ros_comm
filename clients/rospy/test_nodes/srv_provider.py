import rospy
from std_srvs.srv import *

def handle_request(req):
    print "Setting bool to %s"%str(req.data)
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done.'
    return res

if __name__ == "__main__":
    rospy.init_node('setbool_service', log_level=rospy.DEBUG)
    s = rospy.Service('setbool', SetBool, handle_request)
    print "Ready to set bool."
    rospy.spin()