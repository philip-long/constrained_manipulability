#!/usr/bin/env python3

import time
from threading import Lock
from copy import deepcopy

import numpy as np
import cvxpy as cp

# ROS stuff
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from constrained_manipulability_interfaces.srv import GetJacobianMatrix, GetPolytopeConstraints


def multiarray_to_np(A):
    return np.array(A.data).reshape(A.layout.dim[0].size, A.layout.dim[1].size)


class ConstrainedManipulabilityClient(Node):
    
    def __init__(self):
        super().__init__('cm_client')

        # ROS parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sim', False),                     # Whether in "simulation", i.e., RViz, or real robot
                ('n_samples', 3),                   # No. sampled joint states for IK solver 
                ('polytope_server_rate', 0.5),      # Rate of polytope service client requests
                ('jacobian_server_rate', 0.5),      # Rate of jacobian service client requests
            ]
        )
        
        self.sim = self.get_parameter('sim').value
        self.n_samples = self.get_parameter('n_samples').value
        
        self.declare_parameter('constrained_manipulability/active_dof') 
        try:
            self.active_joints = self.get_parameter('constrained_manipulability/active_dof').get_parameter_value().string_array_value
        except Exception as e:
            self.get_logger().error("Failed to get parameter: %s" % str(e))
        
        self.ndof = len(self.active_joints)            
        self.wait_for_joint_state = False
        
        # Locks and members
        self.joint_callback_mutex = Lock()
        self.constraints_mutex = Lock()
        self.jacobian_mutex = Lock()
        self.twist_callback_mutex = Lock()
        
        self.twist_msg = Twist()
        self.joint_state = JointState()
        self.jacobian = None
        self.pub_joint_state_msg = JointState()
        self.pub_joint_state_msg.name = self.active_joints
        
        self.Ahrep_constraints = []
        self.bhrep_constraints = []
        self.shift_to_sampled_joint_state = []
        
        # Get polytope constraints ROS service
        self.get_polytope_constraints = self.create_client(GetPolytopeConstraints, 'get_polytope_constraints')
        while not self.get_polytope_constraints.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("GetPolytopeConstraints service not available, waiting again...")
        self.constr_req = GetPolytopeConstraints.Request()
        self.constr_req.polytope_type = GetPolytopeConstraints.CONSTRAINED_ALLOWABLE_MOTION_POLYTOPE
        self.constr_req.show_polytope = True
        # Get Jacobian ROS service
        self.get_jacobian_matrix = self.create_client(GetJacobianMatrix, 'get_jacobian_matrix')
        while not self.get_jacobian_matrix.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("GetJacobianMatrix service not available, waiting again...")
        self.jac_req = GetJacobianMatrix.Request()
        
        # ROS publishers/subscribers
        self.joint_pub = self.create_publisher(
            JointState, 'joint_states', 10)
                
        exclusive_grp_joint_cb = MutuallyExclusiveCallbackGroup()
        self.joint_subscriber = self.create_subscription(
            JointState, 'in_joint_states', self.joint_callback, qos_profile=1, callback_group=exclusive_grp_joint_cb)

        exclusive_grp_twist_cb = MutuallyExclusiveCallbackGroup()
        self.twist_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.twist_callback, qos_profile=1, callback_group=exclusive_grp_twist_cb)

        if(self.sim):
            self.pub_joint_state_msg.position = np.random.rand(self.ndof, )
            self.pub_joint_state_msg.header.seq = 0
            while(self.pub_joint_state_msg.header.seq < 10):
                self.publish_joint_state()
                time.sleep(0.1)
        else:
            while(not self.wait_for_joint_state):
                self.get_logger().info("Waiting for joint states...")

            # For first pub_joint_state, set to initial numpy array
            self.pub_joint_state_msg.position = np.array(self.joint_state.position)

        exclusive_grp_polytope_server = MutuallyExclusiveCallbackGroup()
        polytope_server_rate = self.get_parameter('polytope_server_rate').value
        self.polytope_timer = self.create_timer(
            polytope_server_rate, self.call_polytope_server,  callback_group=exclusive_grp_polytope_server)
        
        exclusive_grp_jacobian_server = MutuallyExclusiveCallbackGroup()
        jacobian_server_rate = self.get_parameter('jacobian_server_rate').value
        self.jacobian_timer = self.create_timer(
            jacobian_server_rate, self.call_jacobian_server,  callback_group=exclusive_grp_jacobian_server)
        
        exclusive_grp_ik = MutuallyExclusiveCallbackGroup()
        ik_opt_rate = self.get_parameter('ik_opt_rate').value
        self.ik_timer = self.create_timer(
            ik_opt_rate, self.ik_optimiztion,  callback_group=exclusive_grp_ik)


    def publish_joint_state(self):
        self.pub_joint_state_msg.header.seq += 1
        self.pub_joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(self.pub_joint_state_msg)
        
        # Instead of subscriber
        if(self.sim):
            self.joint_state = self.pub_joint_state_msg  
            
    def twist_callback(self, data):
        self.twist_callback_mutex.acquire()
        self.twist_msg = data
        self.twist_callback_mutex.release()

    def joint_callback(self, data):
        self.joint_callback_mutex.acquire()
        if(not self.sim):
            self.joint_state = data
            self.wait_for_joint_state = True
        self.joint_callback_mutex.release()
        
    def call_polytope_server(self):
        self.joint_callback_mutex.acquire()
        local_joint_state = deepcopy(self.joint_state)
        self.joint_callback_mutex.release()

        #  Sample joint state
        self.constr_req.sampled_joint_states = self.sample_joint_states(
            local_joint_state, self.n_samples, offset=0.2)
        
        # Wait for a response
        self.future = self.get_polytope_constraints.call_async(self.constr_req)
        rclpy.spin_until_future_complete(self, self.future)
        constr_resp = self.future.result()
        
        # Perform IK 
        self.ik_optimization(self.constr_req.sampled_joint_states, constr_resp.polytope_hyperplanes)

    def call_jacobian_server(self):
        self.joint_callback_mutex.acquire()
        self.jac_req.joint_states = deepcopy(self.joint_state)
        self.joint_callback_mutex.release()
        
        # Wait for a response
        self.future = self.get_jacobian_matrix.call_async(self.jac_req)
        rclpy.spin_until_future_complete(self, self.future)
        jac_resp = self.future.result()
        
        self.jacobian_mutex.acquire()
        self.jacobian = multiarray_to_np(jac_resp.jacobian)
        self.jacobian_mutex.release()

    def ik_optimization(self, sampled_joint_states, polytope_hyperplanes):
        dq = cp.Variable(self.ndof)  # decision variables
        dx = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # input twist

        # Get input twist
        self.twist_callback_mutex.acquire()
        dx = np.array([self.twist_msg.linear.x, self.twist_msg.linear.y, self.twist_msg.linear.z,
                       self.twist_msg.angular.x, self.twist_msg.angular.y, self.twist_msg.angular.z])
        # Reset twist after command received and written
        self.twist_msg = Twist()
        self.twist_callback_mutex.release()

        # Get current joint states
        self.joint_callback_mutex.acquire()
        local_joint_state = np.array(self.joint_state.position)[:self.ndof]
        self.joint_callback_mutex.release()

        # Get current constraints
        Ahrep_constraints = []
        bhrep_constraints = []
        vol_list = []

        for i in range(len(polytope_hyperplanes)):
            Anp = multiarray_to_np(polytope_hyperplanes[i].half_spaces_normals)
            bnp = np.array(polytope_hyperplanes[i].shifted_distances)
            Ahrep_constraints.append(Anp)
            bhrep_constraints.append(bnp)
            vol_list.append(polytope_hyperplanes[i].volume)

        # Get current Jacobian matrix
        self.jacobian_mutex.acquire()
        jacobian = deepcopy(self.jacobian)
        self.jacobian_mutex.release()

        cost = cp.sum_squares(jacobian @ dq - dx)
        # Need to find the shift from the current position to this position
        start_time = time.time()
        best_val = 1.0
        dq_sol = np.zeros(len(local_joint_state))

        for i in range(len(sampled_joint_states)):
            if(vol_list[i] < 0.00001):
                continue

            sample_joint_ns = np.asarray(
                sampled_joint_states[i].position)[:self.ndof]
            joint_shift = local_joint_state-sample_joint_ns
            constraints = [Ahrep_constraints[i] @ (dq + joint_shift) <= bhrep_constraints[i]]
            prob = cp.Problem(cp.Minimize(cost), constraints)
            prob.solve()
            if(prob.value < best_val):
                best_val = prob.value
                dq_sol = dq.value

        self.get_logger().info("--- %s seconds ---" % (time.time() - start_time))
        dx_sol = np.matmul(jacobian, dq_sol)
        self.get_logger().info("\n dx input = {}".format(dx))
        self.get_logger().info("\n Best dx_sol = {}\n Error = {}".format(dx_sol, best_val))
        self.pub_joint_state_msg.position[:self.ndof] += dq_sol
        self.publish_joint_state()

    def sample_joint_states(self, joint_state, nbr_samples, offset):
        joint_vec = []
        joint_vec.append(joint_state)
        
        # First sample is the current joint state
        for _ in range(nbr_samples-1):
            variation = (np.random.rand(len(joint_state.position)) - offset) * (offset / 2.)
            sampled_joint_state = np.asarray(joint_state.position) + variation
            joint_vec.append(sampled_joint_state)
            
        return joint_vec


def main(args=None):
    rclpy.init(args=args)

    cm_client = ConstrainedManipulabilityClient()

    executor = MultiThreadedExecutor()
    executor.add_node(cm_client)
    executor.spin()

    # Destroy the node explicitly
    cm_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()