#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from constrained_manipulability.srv import *
from sensor_msgs.msg import JointState
import numpy as np
import random
from rospy.numpy_msg import numpy_msg





class client_class:
    def __init__(self):
        
        rospy.wait_for_service('/get_polytope_constraints')
        rospy.Subscriber("joint_states", JointState, self.jointStateCallback)
        self.joint_state=JointState()
        self.joint_vec=[]
        self.count=0
        self.req=constrained_manipulability.srv.GetPolytopeConstraintsRequest()
        self.req.polytope_type=constrained_manipulability.srv.GetPolytopeConstraintsRequest.CONSTRAINED_ALLOWABLE_MOTION_POLYTOPE


    def jointStateCallback(self,data):
        self.joint_state=data
        self.count=self.count+1
        if(self.count==20):
            print("calling server")    
            get_polytope_constraints = rospy.ServiceProxy('get_polytope_constraints', GetPolytopeConstraints)
            self.joint_vec.append(self.joint_state)
            self.req.sampled_joint_states=self.joint_vec
            resp1 = get_polytope_constraints(self.req)
            print("Request",self.req,"size ",self.req.sampled_joint_states.count)
            print("Response",resp1," size",len(resp1.polytope_hyperplanes))
            print("=======================================================")
            self.count=0
        
        rospy.sleep(0.1)

if __name__ == "__main__":
    rospy.init_node('cm_client_test')
    client_class()
    rospy.spin()
    
    
