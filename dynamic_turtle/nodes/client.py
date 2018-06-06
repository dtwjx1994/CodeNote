#!/usr/bin/env python   
import zbar
import rospy
import dynamic_reconfigure.client
import time
class Change_foot():
    def __init__(self):
        rospy.init_node("dynamic_client")
        rospy.loginfo("waiting service")
        # rospy.wait_for_service("dynamic_tutorials")
        rospy.loginfo("get service")
        self.client1=dynamic_reconfigure.client.Client("/move_base/global_costmap/",timeout=30)
        self.client2=dynamic_reconfigure.client.Client("/move_base/local_costmap/",timeout=30)
        rospy.loginfo("get service")
        r=rospy.Rate(1)
    def chang(self,shape):
        if shape==0:
            # yuanxing 
            self.client1.update_configuration({"footprint":[],"robot_radius":0.2})
            self.client2.update_configuration({"footprint":[],"robot_radius":0.2})
            rospy.loginfo("yuanxing")
        elif shape==1:
            # fangxing
            self.client1.update_configuration({"footprint":[[0.25,0.37],[0.25,-0.37],[-0.25,-0.37],[-0.25,0.37]],"robot_radius":0.0})
            self.client2.update_configuration({"footprint":[[0.25,0.37],[0.25,-0.37],[-0.25,-0.37],[-0.25,0.37]],"robot_radius":0.0})
            rospy.loginfo("fangxing")
    def callback(self,config):
        # rospy.loginfo("Config set to {int_param}, {double_param}, {str_param}, {bool_param}, {size}".format(**config))
        rospy.loginfo("configure DONE")
        pass
if __name__=="__main__":
    a=Change_foot()
    b=False
    for i in range(100):
        b=not b
        a.chang(b)
        time.sleep(2);

    
    # rospy.init_node("dynamic_client")
    # rospy.loginfo("waiting service")
    # # rospy.wait_for_service("dynamic_tutorials")
    
    # rospy.loginfo("get service")
    # client1=dynamic_reconfigure.client.Client("/move_base/global_costmap/",timeout=30,config_callback=callback)
    # client2=dynamic_reconfigure.client.Client("/move_base/local_costmap/",timeout=30,config_callback=callback)
    # rospy.loginfo("get service")
    # r=rospy.Rate(1)
    # x=0
    # b=False
    # while not rospy.is_shutdown():
    #     x=x+1
    #     if x>10:
    #         x=0

    #     b=not b
    #     client1.update_configuration({"footprint":[[0.25,0.37],[0.25,-0.37],[-0.25,-0.37],[-0.25-x,0.37]]})
    #     client2.update_configuration({"footprint":[[0.25,0.37],[0.25,-0.37],[-0.25,-0.37],[-0.25-x,0.37]]})
    #     r.sleep()