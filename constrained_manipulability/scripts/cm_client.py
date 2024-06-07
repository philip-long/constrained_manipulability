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

from rcl_interfaces.srv import GetParameters

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from constrained_manipulability_interfaces.srv import GetJacobianMatrix, GetPolytopes


def multiarray_to_np(A):
    return np.array(A.data).reshape(A.rows, A.columns)


class ConstrainedManipulabilityClient(Node):
    
    def __init__(self):
        super().__init__('cm_client')

        # Declare ROS parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sim', False),                                      # Whether in "simulation", i.e., RViz, or real robot
                ('n_samples', 5),                                    # No. sampled joint states for IK solver 
                ('ik_opt_rate', 0.25),                               # Rate of IK optimization loop as timer period
                ('constrained_manipulability_node/active_dof', [])   # No. of active DoFs
            ]
        )
        
        self.sim = self.get_parameter('sim').value
        self.n_samples = self.get_parameter('n_samples').value
        ik_opt_rate = self.get_parameter('ik_opt_rate').value

        # GetParameters client to grab parameter from constrained_manipulability_node
        params_client = self.create_client(GetParameters, 'constrained_manipulability_node/get_parameters')
        while not params_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for parameter service...")

        request = GetParameters.Request()
        request.names = ['active_dof']
        future = params_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.active_joints = future.result().values[0].string_array_value
        else:
            self.get_logger().error("Failed to get parameter 'constrained_manipulability_nodee/active_dof'")
            raise SystemExit
            
        # Set no. DoFs based on active robot joints
        self.ndof = len(self.active_joints)
        if (self.ndof == 0):
            self.get_logger().error("Need the constrained_manipulability_node/active_dof parameter set. Exiting node.")
            raise SystemExit
        
        # Initialize locks and members
        self.joint_callback_mutex = Lock()
        self.twist_callback_mutex = Lock()

        self.joint_state = JointState()
        self.twist_msg = Twist()
        self.wait_for_joints = False
        self.pub_joint_state_msg = JointState()
        self.pub_joint_state_msg.name = self.active_joints
        
        # ROS joint publishing
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        if self.sim:
            self.pub_joint_state_msg.position = np.random.rand(self.ndof).tolist()
            self.publish_joint_state(self.pub_joint_state_msg)
        else:
            while not self.wait_for_joints:
                self.get_logger().info("Waiting for joint states...")

            # For first pub_joint_state, set to initial numpy array
            self.pub_joint_state_msg.position = np.array(self.joint_state.position)
            
        # Get Polytopes ROS service
        self.get_polytopes = self.create_client(GetPolytopes, 'get_polytopes')
        while not self.get_polytopes.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("GetPolytopes service not available, waiting again...")
        
        # Get Jacobian ROS service
        self.get_jacobian_matrix = self.create_client(GetJacobianMatrix, 'get_jacobian_matrix')
        while not self.get_jacobian_matrix.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("GetJacobianMatrix service not available, waiting again...")
        
        # ROS subscribers
        exclusive_grp_joint_cb = MutuallyExclusiveCallbackGroup()
        self.joint_subscriber = self.create_subscription(
            JointState, 'in_joint_states', self.joint_callback, 1, callback_group=exclusive_grp_joint_cb)

        exclusive_grp_twist_cb = MutuallyExclusiveCallbackGroup()
        self.twist_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.twist_callback, 1, callback_group=exclusive_grp_twist_cb)

        # ROS timers
        exclusive_grp_ik = MutuallyExclusiveCallbackGroup()
        self.ik_timer = self.create_timer(
            ik_opt_rate, self.ik_optimization,  callback_group=exclusive_grp_ik)

    def publish_joint_state(self, joint_state):
        joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(joint_state)
        
        # Instead of subscriber
        if self.sim:
            self.joint_state = joint_state
            
    def twist_callback(self, data):
        with self.twist_callback_mutex:
            self.twist_msg = data

    def joint_callback(self, data):
        with self.joint_callback_mutex:
            if not self.sim:
                self.joint_state = data
                self.wait_for_joints = True
                
    async def call_polytope_server(self, sampled_joint_states):
        constr_req = GetPolytopes.Request()
        constr_req.polytopes_type = constr_req.CONSTRAINED_ALLOWABLE_MOTION_POLYTOPE
        constr_req.show_polytopes = True
        constr_req.joint_states = sampled_joint_states
        
        return await self.get_polytopes.call_async(constr_req)

    async def call_jacobian_server(self, joint_state):
        jac_req = GetJacobianMatrix.Request()
        jac_req.joint_state = joint_state
        
        return await self.get_jacobian_matrix.call_async(jac_req)
            
    async def ik_optimization(self):
        dq = cp.Variable(self.ndof)  # decision variables
        dx = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # input twist
        
        # Get current joint states
        with self.joint_callback_mutex:
            local_joint_state = deepcopy(self.joint_state)
            local_joint_state_pos = np.array(local_joint_state.position)[:self.ndof]

        # Get input twist
        with self.twist_callback_mutex:
            dx = np.array([self.twist_msg.linear.x, self.twist_msg.linear.y, self.twist_msg.linear.z,
                        self.twist_msg.angular.x, self.twist_msg.angular.y, self.twist_msg.angular.z])
            # Reset twist after command received and written
            self.twist_msg = Twist()
        
        # Get current Jacobian matrix
        jacobian_server_resp = await self.call_jacobian_server(local_joint_state)
        jacobian = multiarray_to_np(jacobian_server_resp.jacobian)

        sampled_joint_states = self.sample_joint_states(local_joint_state, self.n_samples, offset=0.2)

        # Get polytope constraints
        polytopes_server_resp = await self.call_polytope_server(sampled_joint_states)
        polytopes = polytopes_server_resp.polytopes

        Ahrep_constraints = []
        bhrep_constraints = []
        vol_list = []
        for i in range(len(polytopes)):
            Anp = multiarray_to_np(polytopes[i].hyperplanes)
            bnp = np.array(polytopes[i].shifted_distance)
            Ahrep_constraints.append(Anp)
            bhrep_constraints.append(bnp)
            vol_list.append(polytopes[i].volume)

        # start_time = time.time()
        cost = cp.sum_squares(jacobian @ dq - dx)
        
        # Need to find the shift from the current position to this position
        best_val = 1.0
        dq_sol = np.zeros(len(local_joint_state_pos))
        for i in range(len(sampled_joint_states)):
            if(vol_list[i] < 0.00001):
                continue

            sample_joint_ns = np.asarray(sampled_joint_states[i].position)[:self.ndof]
            joint_shift = local_joint_state_pos - sample_joint_ns
            constraints = [Ahrep_constraints[i] @ (dq + joint_shift) <= bhrep_constraints[i]]
            prob = cp.Problem(cp.Minimize(cost), constraints)
            prob.solve()
            if prob.value < best_val:
                best_val = prob.value
                dq_sol = dq.value

        dx_sol = np.matmul(jacobian, dq_sol)

        # self.get_logger().info("--- %s seconds ---" % (time.time() - start_time))
        self.get_logger().info("\n dx input = {}".format(dx))
        self.get_logger().info("\n Best dx_sol = {}\n Error = {}".format(dx_sol, best_val))
        
        # Update the existing list in place
        position_update = np.array(self.pub_joint_state_msg.position[:self.ndof]) + dq_sol
        for i in range(self.ndof):
            self.pub_joint_state_msg.position[i] = position_update[i]

        self.publish_joint_state(self.pub_joint_state_msg)

    def sample_joint_states(self, joint_state, nbr_samples, offset):
        joint_vec = []
        joint_vec.append(deepcopy(joint_state))
        
        # First sample is the current joint state
        for _ in range(nbr_samples-1):
            sampled_joint_state = joint_state
            variation = (np.random.rand(len(joint_state.position)) - offset) * (offset / 2.)
            sampled_position = np.asarray(joint_state.position) + variation
            sampled_joint_state.position = sampled_position.tolist()
            joint_vec.append(deepcopy(sampled_joint_state))
            
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