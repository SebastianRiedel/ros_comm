import rospy
from std_srvs.srv import *

if __name__ == "__main__":
    service = rospy.ServiceProxy('setbool', SetBool)
    req = SetBoolRequest()
    req.data = True
    resp = service(req)
    print resp
