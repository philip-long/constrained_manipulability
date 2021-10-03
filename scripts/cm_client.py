#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from constrained_manipulability.srv import *
from sensor_msgs.msg import JointState
import numpy as np
import random
from rospy.numpy_msg import numpy_msg
from threading import Thread, Lock
from copy import deepcopy




class client_class:
    def __init__(self):

        self.mutex=Lock()
        rospy.wait_for_service('/get_polytope_constraints')
        
        self.joint_state=JointState()
        
        self.req=constrained_manipulability.srv.GetPolytopeConstraintsRequest()
        self.req.polytope_type=constrained_manipulability.srv.GetPolytopeConstraintsRequest.CONSTRAINED_ALLOWABLE_MOTION_POLYTOPE
        self.req.show_polytope=True        
        self.get_polytope_constraints = rospy.ServiceProxy('get_polytope_constraints', GetPolytopeConstraints)
        rospy.Subscriber("joint_states", JointState, self.jointStateCallback)
        rospy.Timer(rospy.Duration(3), self.callPolytopeServer)
        

    def callPolytopeServer(self, event=None):
        print("getPolytopeConstraints server")    
        self.mutex.acquire()
        local_joint_state=self.joint_state
        self.mutex.release()
        #  sample joint state
        self.req.sampled_joint_states=self.sampleJointState(local_joint_state,3)
        resp1 = self.get_polytope_constraints(self.req)
        #Convert data        
        print("Request",self.req,"size ",self.req.sampled_joint_states.count)
        #Convert data & formulate optimzation       
        # print("Response",resp1," size",len(resp1.polytope_hyperplanes))
        print("=======================================================")
        
    def sampleJointState(self,joint_state,nbr_samples):
        joint_vec=[]
        for i in range(nbr_samples):            
            local_joint_state=joint_state
            local_joint_state.position=tuple(np.asarray(local_joint_state.position) + (np.random.rand(len(local_joint_state.position)) - 0.5)*0.25)
            joint_vec.append(deepcopy(local_joint_state))
        return joint_vec
        
    def jointStateCallback(self,data):
        self.mutex.acquire()
        self.joint_state=data
        self.mutex.release()
        
if __name__ == "__main__":
    rospy.init_node('cm_client_test')
    client_class()
    rospy.spin()
    
    
