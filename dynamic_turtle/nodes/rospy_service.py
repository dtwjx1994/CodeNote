#!/usr/bin/env python  
import rospy
from std_srvs.srv import *

def setBool(req):
    
    SetBoolResponse.
    print(req)
    return 
rospy.init_node("setBool")
s=rospy.Service("kai",SetBool,callback)
rospy.spin()