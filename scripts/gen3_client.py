#!/usr/bin/env python3

import time
from threading import Lock
from copy import copy, deepcopy

import numpy as np
import cvxpy as cp

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

from kortex_driver.srv import ExecuteAction, ExecuteActionRequest, ValidateWaypointList
from kortex_driver.msg import ActionEvent, ActionNotification, AngularWaypoint, Waypoint, WaypointList

from constrained_manipulability.srv import *


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

        self.active_joints = rospy.get_param(
            'constrained_manipulability/active_dof')
        self.ndof = len(self.active_joints)
        self.joint_state_topic = rospy.get_param(
            '~joint_state', "joint_states")
        self.wait_for_joint_state = False
        self.n_samples = rospy.get_param('~n_samples', 3)

        rospy.wait_for_service('/get_polytope_constraints')
        rospy.wait_for_service('/get_jacobian_matrix')

        self.twist = Twist()
        self.joint_state = JointState()
        self.jacobian = None

        # Kinova Gen3 services
        self.action_topic_sub = rospy.Subscriber(
            '/my_gen3/action_topic', ActionNotification, self.cb_action_topic)
        self.last_action_notif_type = None

        execute_action_full_name = '/my_gen3/base/execute_action'
        rospy.wait_for_service(execute_action_full_name)
        self.execute_action = rospy.ServiceProxy(
            execute_action_full_name, ExecuteAction)

        validate_waypoint_list_full_name = '/my_gen3/base/validate_waypoint_list'
        rospy.wait_for_service(validate_waypoint_list_full_name)
        self.validate_waypoint_list = rospy.ServiceProxy(
            validate_waypoint_list_full_name, ValidateWaypointList)

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
        rospy.Subscriber(self.joint_state_topic, JointState,
                         self.jointStateCallback)
        rospy.Subscriber("cmd_vel", Twist, self.twistCallback)

        while(not self.wait_for_joint_state):
            print("waiting for joint state")
            time.sleep(1)

        rospy.Timer(rospy.Duration(1.0), self.callPolytopeServer)
        rospy.Timer(rospy.Duration(0.2), self.callJacobianServer)
        time.sleep(3)
        rospy.Timer(rospy.Duration(0.2), self.ik_optimiztion)

    def sendJointAngles(self, joint_angles):
        self.last_action_notif_type = None

        req = ExecuteActionRequest()

        trajectory = WaypointList()
        waypoint = Waypoint()
        angularWaypoint = AngularWaypoint()

        # Angles to send the arm
        angularWaypoint.angles = joint_angles

        # Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded.
        # If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
        angular_duration = 0.1
        angularWaypoint.duration = angular_duration

        # Initialize Waypoint and WaypointList
        waypoint.oneof_type_of_waypoint.angular_waypoint.append(
            angularWaypoint)
        trajectory.duration = 0.1
        trajectory.use_optimal_blending = False
        trajectory.waypoints.append(waypoint)

        try:
            res = self.validate_waypoint_list(trajectory)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ValidateWaypointList")
            return False

        error_number = len(
            res.output.trajectory_error_report.trajectory_error_elements)
        MAX_ANGULAR_DURATION = 30

        while (error_number >= 1 and angular_duration != MAX_ANGULAR_DURATION):
            angular_duration += 1
            trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration

            try:
                res = self.validate_waypoint_list(trajectory)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ValidateWaypointList")
                return False

            error_number = len(
                res.output.trajectory_error_report.trajectory_error_elements)

        if (angular_duration == MAX_ANGULAR_DURATION):
            # It should be possible to reach position within 30s
            # WaypointList is invalid (other error than angularWaypoint duration)
            rospy.loginfo("WaypointList is invalid")
            return False

        req.input.oneof_action_parameters.execute_waypoint_list.append(
            trajectory)

        # Send the angles
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def callJacobianServer(self, event=None):
        self.joint_callback_mutex.acquire()

        self.jac_req.joint_states = copy(self.joint_state)
        self.joint_callback_mutex.release()
        resp = self.get_jacobian_matrix(self.jac_req)

        self.jacobian_mutex.acquire()
        self.jacobian = multiarray_to_np(resp.jacobian)
        self.jacobian_mutex.release()

    def callPolytopeServer(self, event=None):
        self.joint_callback_mutex.acquire()
        local_joint_state = copy(self.joint_state)
        self.joint_callback_mutex.release()

        #  Sample joint state
        self.constraints_mutex.acquire()
        self.req.sampled_joint_states = self.sampleJointState(
            local_joint_state, self.n_samples, 0.2)
        resp1 = self.get_polytope_constraints(self.req)

        self.Ahrep_constraints = []
        self.bhrep_constraints = []
        self.volume = []

        for i in range(len(resp1.polytope_hyperplanes)):
            Anp = multiarray_to_np(resp1.polytope_hyperplanes[i].A)
            bnp = np.array(resp1.polytope_hyperplanes[i].b)
            self.Ahrep_constraints.append(deepcopy(Anp))
            self.bhrep_constraints.append(deepcopy(bnp))
            self.volume.append(resp1.polytope_hyperplanes[i].volume)
        self.constraints_mutex.release()

    def ik_optimiztion(self, event=None):
        dq = cp.Variable(self.ndof)  # decision variables,
        dx = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # input twist

        # Get input twist
        self.twist_callback_mutex.acquire()
        dx = np.array([self.twist.linear.x, self.twist.linear.y, self.twist.linear.z,
                      self.twist.angular.x, self.twist.angular.y, self.twist.angular.z])
        # Reset twist after command received and written
        self.twist = Twist()
        self.twist_callback_mutex.release()

        # Get current joint states
        self.joint_callback_mutex.acquire()
        local_joint_state = np.array(self.joint_state.position)[:self.ndof]
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
        # start_time = time.time()
        best_val = 1.0
        dq_sol = np.zeros(len(local_joint_state))

        for i in range(len(sampled_joint_states)):
            if(vol_list[i] < 0.00001):
                continue

            sample_joint_ns = np.asarray(
                sampled_joint_states[i].position)[:self.ndof]
            joint_shift = local_joint_state-sample_joint_ns
            constraints = [Alist[i] @ (dq + joint_shift) <= blist[i]]
            prob = cp.Problem(cp.Minimize(cost), constraints)
            prob.solve()
            if(prob.value < best_val):
                best_val = prob.value
                dq_sol = dq.value

        # print("--- %s seconds ---" % (time.time() - start_time))
        dx_sol = np.matmul(jacobian, dq_sol)
        print("\n dx input = ", dx)
        print("\n Best dx_sol", dx_sol, "\n Error=", best_val)
        joint_update = self.joint_state.position[:self.ndof] + dq_sol
        self.sendJointAngles(joint_update)

    def sampleJointState(self, joint_state, nbr_samples, a):
        joint_vec = []
        local_joint_state = joint_state
        joint_vec.append(deepcopy(local_joint_state))
        # First sample is the current joint state
        for _ in range(nbr_samples-1):
            local_joint_state = joint_state
            local_joint_state.position = np.asarray(
                local_joint_state.position) + (np.random.rand(len(local_joint_state.position)) - a)*a/2.
            joint_vec.append(deepcopy(local_joint_state))
        return joint_vec

    def twistCallback(self, data):
        self.twist_callback_mutex.acquire()
        self.twist = data
        self.twist_callback_mutex.release()

    def jointStateCallback(self, data):
        self.joint_callback_mutex.acquire()
        self.joint_state = data
        self.wait_for_joint_state = True
        self.joint_callback_mutex.release()


if __name__ == "__main__":
    rospy.init_node('gen3_client')
    client_class()
    rospy.spin()
