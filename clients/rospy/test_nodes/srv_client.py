import rospy
from std_srvs.srv import *

if __name__ == "__main__":
    rospy.init_node("setboolclient")
    service = rospy.ServiceProxy('setbool', SetBool)
    req = SetBoolRequest()
    req.data = True
    resp = service(req)
    print resp
