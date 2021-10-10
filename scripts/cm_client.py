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
import cvxpy as cp

# https://gist.github.com/braingineer/d801735dac07ff3ac4d746e1f218ab75
def matprint(mat, fmt="g"):
    col_maxes = [max([len(("{:"+fmt+"}").format(x)) for x in col]) for col in mat.T]
    for x in mat:
        for i, y in enumerate(x):
            print(("{:"+str(col_maxes[i])+fmt+"}").format(y), end="  ")
        print("")

def multiarray_to_np(A):
    return  np.array(A.data).reshape(A.layout.dim[0].size,A.layout.dim[1].size)


class client_class:
    def __init__(self):

        self.joint_callback_mutex=Lock()
        self.constraints_mutex=Lock()
        self.jacobian_mutex=Lock()
        
        rospy.wait_for_service('/get_polytope_constraints')
        rospy.wait_for_service('/get_jacobian_matrix')
        
        self.joint_state=JointState()
        self.jacobian=None
        
        self.req=constrained_manipulability.srv.GetPolytopeConstraintsRequest()
        self.req.polytope_type=constrained_manipulability.srv.GetPolytopeConstraintsRequest.CONSTRAINED_ALLOWABLE_MOTION_POLYTOPE
        self.req.show_polytope=True

        self.jac_req=constrained_manipulability.srv.GetJacobianMatrixRequest()
        
        self.Ahrep_constraints=[]
        self.bhrep_constraints=[]
        self.get_polytope_constraints = rospy.ServiceProxy('get_polytope_constraints', GetPolytopeConstraints)
        self.get_jacobian_matrix = rospy.ServiceProxy('get_jacobian_matrix', GetJacobianMatrix)

        
        rospy.Subscriber("joint_states", JointState, self.jointStateCallback)
        rospy.Timer(rospy.Duration(3), self.callPolytopeServer)
        rospy.Timer(rospy.Duration(0.2), self.callJacobianServer)
        
    def callJacobianServer(self, event=None):
        print("get Jacobian server")    
        self.joint_callback_mutex.acquire()
        self.jac_req.joint_states=self.joint_state
        self.joint_callback_mutex.release()
        resp = self.get_jacobian_matrix(self.jac_req)

        self.jacobian_mutex.acquire()
        self.jacobian=multiarray_to_np(resp.jacobian)
        self.jacobian_mutex.release()
        
        # print("Jacobian")
        matprint(self.jacobian)
        
    def callPolytopeServer(self, event=None):
        print("getPolytopeConstraints server")    
        self.joint_callback_mutex.acquire()
        local_joint_state=self.joint_state
        self.joint_callback_mutex.release()
        #  sample joint state
        # 
        self.req.sampled_joint_states=self.sampleJointState(local_joint_state,1)
        resp1 = self.get_polytope_constraints(self.req)

        self.constraints_mutex.acquire()
        self.Ahrep_constraints=[]
        self.bhrep_constraints=[]
        
        for i in range(len(resp1.polytope_hyperplanes)):
            Anp=multiarray_to_np(resp1.polytope_hyperplanes[i].A)
            bnp=np.array(resp1.polytope_hyperplanes[i].b)
            print("Anp")
            matprint(Anp)
            print("bnp",bnp)
            print("volume",resp1.polytope_hyperplanes[i].volume)                        
            self.Ahrep_constraints.append(deepcopy(Anp))
            self.bhrep_constraints.append(deepcopy(bnp))
        print("=======================================================")
        self.constraints_mutex.release()
        self.ik_optimiztion()

    def ik_optimiztion(self):

        self.constraints_mutex.acquire()
        Alist=self.Ahrep_constraints
        blist=self.bhrep_constraints
        self.constraints_mutex.release()

        self.jacobian_mutex.acquire()
        jacobian=self.jacobian
        self.jacobian_mutex.release()
        
        n=6 # 6 joints
        dx=np.array([0.0,0.2,0.0,0.0,0.0,0.0]) # input twist
        dq=cp.Variable(n) # decision variables, 
        prob = cp.Problem(cp.Minimize( (dx - jacobian@dq).T @ (dx - jacobian@dq)),[Alist[0] @ dq <= blist[0]])
        prob.solve()
        print("\nThe optimal value is", prob.value)
        print("A solution x is")
        print(dq.value)

        
    def sampleJointState(self,joint_state,nbr_samples):
        joint_vec=[]
        local_joint_state=joint_state
        joint_vec.append(deepcopy(local_joint_state))
        # First sample is the current joint state
        for i in range(nbr_samples-1):            
            local_joint_state=joint_state
            local_joint_state.position=tuple(np.asarray(local_joint_state.position) + (np.random.rand(len(local_joint_state.position)) - 0.5)*0.25)
            joint_vec.append(deepcopy(local_joint_state))
        return joint_vec
        
    def jointStateCallback(self,data):
        self.joint_callback_mutex.acquire()
        self.joint_state=data
        self.joint_callback_mutex.release()
        
if __name__ == "__main__":
    rospy.init_node('cm_client_test')
    client_class()
    rospy.spin()
    
    
