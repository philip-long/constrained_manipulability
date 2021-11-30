#!/usr/bin/env python3

import rospy
from constrained_manipulability.srv import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np
from threading import Lock
from copy import deepcopy
import cvxpy as cp
import time

# https://gist.github.com/braingineer/d801735dac07ff3ac4d746e1f218ab75


def matprint(mat, fmt="g"):
    col_maxes = [max([len(("{:"+fmt+"}").format(x))
                     for x in col]) for col in mat.T]
    for x in mat:
        for i, y in enumerate(x):
            print(("{:"+str(col_maxes[i])+fmt+"}").format(y), end="  ")
        print("")


def multiarray_to_np(A):
    return np.array(A.data).reshape(A.layout.dim[0].size, A.layout.dim[1].size)


class client_class:
    def __init__(self):

        self.joint_callback_mutex = Lock()
        self.constraints_mutex = Lock()
        self.jacobian_mutex = Lock()
        self.twist_callback_mutex = Lock()

        self.wait_for_jacobian = False
        self.wait_for_joint_state = False
        self.wait_for_polytope = False

        rospy.wait_for_service('/get_polytope_constraints')
        rospy.wait_for_service('/get_jacobian_matrix')

        self.twist = Twist()
        self.joint_state = JointState()
        self.jacobian = None

        self.req = constrained_manipulability.srv.GetPolytopeConstraintsRequest()
        self.req.polytope_type = constrained_manipulability.srv.GetPolytopeConstraintsRequest.CONSTRAINED_ALLOWABLE_MOTION_POLYTOPE
        self.req.show_polytope = True

        self.jac_req = constrained_manipulability.srv.GetJacobianMatrixRequest()

        self.Ahrep_constraints = []
        self.bhrep_constraints = []
        self.shift_to_sampled_joint_state = []
        self.get_polytope_constraints = rospy.ServiceProxy(
            'get_polytope_constraints', GetPolytopeConstraints)
        self.get_jacobian_matrix = rospy.ServiceProxy(
            'get_jacobian_matrix', GetJacobianMatrix)

        self.pub = rospy.Publisher('joint_states', JointState, queue_size=1)

        self.pub_joint_state = JointState()
        self.pub_joint_state.name = ["shoulder_pan_joint", "shoulder_lift_joint",
                                     "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.pub_joint_state.position = [-3.0724776152108175, -2.131256456195315, -
                                         1.0379822127460678, -1.079451235773453, 1.5783361491635128, 0.0]
        self.pub_joint_state.header.seq = 0

        while(self.pub_joint_state.header.seq < 10):
            self.pubJointState()
            time.sleep(0.1)

        rospy.Subscriber("joint_states", JointState, self.jointStateCallback)
        rospy.Subscriber("cmd_vel", Twist, self.twistCallback)
        rospy.Timer(rospy.Duration(1.0), self.callPolytopeServer)
        rospy.Timer(rospy.Duration(0.2), self.callJacobianServer)
        time.sleep(3)
        rospy.Timer(rospy.Duration(0.2), self.ik_optimiztion)

    def pubJointState(self):
        self.pub_joint_state.header.seq = self.pub_joint_state.header.seq+1
        self.pub_joint_state.header.stamp = rospy.get_rostime()
        self.pub.publish(self.pub_joint_state)
        self.joint_state = self.pub_joint_state  # instead of subscriber

    def callJacobianServer(self, event=None):

        self.joint_callback_mutex.acquire()
        self.jac_req.joint_states = self.joint_state
        self.joint_callback_mutex.release()
        resp = self.get_jacobian_matrix(self.jac_req)

        self.jacobian_mutex.acquire()
        self.jacobian = multiarray_to_np(resp.jacobian)
        self.jacobian_mutex.release()

    def callPolytopeServer(self, event=None):
        nbr_smaples = 3
        self.joint_callback_mutex.acquire()
        local_joint_state = self.joint_state
        self.joint_callback_mutex.release()
        #  sample joint state
        #
        self.constraints_mutex.acquire()
        self.req.sampled_joint_states = self.sampleJointState(
            local_joint_state, nbr_smaples, 0.2)
        resp1 = self.get_polytope_constraints(self.req)

        self.Ahrep_constraints = []
        self.bhrep_constraints = []
        self.volume = []

        for i in range(len(resp1.polytope_hyperplanes)):
            Anp = multiarray_to_np(resp1.polytope_hyperplanes[i].A)
            bnp = np.array(resp1.polytope_hyperplanes[i].b)
            # print("Anp")
            # matprint(Anp)
            # print("bnp",bnp)
            # print("volume",resp1.polytope_hyperplanes[i].volume)
            self.Ahrep_constraints.append(deepcopy(Anp))
            self.bhrep_constraints.append(deepcopy(bnp))
            self.volume.append(resp1.polytope_hyperplanes[i].volume)
        self.constraints_mutex.release()

    def ik_optimiztion(self, event=None):
        print("=======================================================")
        n = 6  # 6 joints
        dq = cp.Variable(n)  # decision variables,
        dx = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # input twist
        # Get input twist
        self.twist_callback_mutex.acquire()
        dx = np.array([self.twist.linear.x, self.twist.linear.y, self.twist.linear.z,
                      self.twist.angular.x, self.twist.angular.y, self.twist.angular.z])
        self.twist_callback_mutex.release()

        # Get current joint states
        self.joint_callback_mutex.acquire()
        local_joint_state = self.joint_state
        self.joint_callback_mutex.release()

        # Get Current constraint
        self.constraints_mutex.acquire()
        Alist = self.Ahrep_constraints
        blist = self.bhrep_constraints
        vol_list = self.volume

        sampled_joint_states = self.req.sampled_joint_states
        self.constraints_mutex.release()

        # Get current Jacobian matrix
        self.jacobian_mutex.acquire()
        jacobian = self.jacobian
        self.jacobian_mutex.release()

        cost = cp.sum_squares(jacobian@dq - dx)
        # Need to find the shift from the current position to this position
        start_time = time.time()
        best_val = 1.0
        dq_sol = np.zeros(len(local_joint_state.position))

        for i in range(len(sampled_joint_states)):
            print("vol", vol_list[i])
            if(vol_list[i] < 0.00001):
                continue
            joint_shift = np.asarray(
                local_joint_state.position)-np.asarray(sampled_joint_states[i].position)
            print("joint_shift", joint_shift)
            constraints = [Alist[i] @ (dq + joint_shift) <= blist[i]]
            prob = cp.Problem(cp.Minimize(cost), constraints)
            prob.solve()
            if(prob.value < best_val):
                best_val = prob.value
                dq_sol = dq.value

        print("--- %s seconds ---" % (time.time() - start_time))
        dx_sol = np.matmul(jacobian, dq_sol)
        print("\n dx input = ", dx)
        print("\n Best dx_sol", dx_sol, "\n Error=", best_val)
        self.pub_joint_state.position = np.asarray(
            self.pub_joint_state.position)+dq_sol
        self.pubJointState()
        print("=======================================================\n\n")

    def sampleJointState(self, joint_state, nbr_samples, a):
        joint_vec = []
        local_joint_state = joint_state
        joint_vec.append(deepcopy(local_joint_state))
        # First sample is the current joint state
        for i in range(nbr_samples-1):
            local_joint_state = joint_state
            local_joint_state.position = tuple(np.asarray(
                local_joint_state.position) + (np.random.rand(len(local_joint_state.position)) - a)*a/2.)
            joint_vec.append(deepcopy(local_joint_state))
        return joint_vec

    def twistCallback(self, data):
        self.twist_callback_mutex.acquire()
        self.twist = data
        self.twist_callback_mutex.release()

    def jointStateCallback(self, data):
        self.joint_callback_mutex.acquire()
        self.joint_state = data
        self.joint_callback_mutex.release()


if __name__ == "__main__":
    rospy.init_node('cm_client_test')
    client_class()
    rospy.spin()