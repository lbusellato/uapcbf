import random
random.seed(420)
import csv
import numpy as np
np.random.seed(420)
import cvxpy as cp
import json
import rclpy
import time
import threading
from pynput import keyboard  

from datetime import datetime
from geometry_msgs.msg import Point, Quaternion
from jaka_interface.data_types import MoveMode
from jaka_interface.pose_conversions import rpy_to_rot_matrix, leap_to_jaka, jaka_to_se3
from jaka_interface.robots import RealRobot, SimulatedRobot
from rclpy.node import Node
from scipy.optimize import minimize_scalar
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

# TODO: move control stuff into a controller

UAPCBF = True

class SafeControl(Node):
    def __init__(self):
        super().__init__('jaka_control')
        
        self.logger = self.get_logger()

        # Init robot interface
        self.robot = RealRobot()
        self.robot.initialize(collision_recover=True)

        # Logging
        self.logger = self.get_logger()

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', qos_profile=1)
        self.joint_state_publisher_frequency = 30 #Hz
        self.joint_state_publisher_timer = self.create_timer(1 / self.joint_state_publisher_frequency, 
                                                                 self.joint_state_publisher_callback)
        
        # Hand position and forecast
        self.hand_pos_sub = self.create_subscription(String, '/leap/fusion', self.leap_fusion_callback, 1)
        self.current_hand_pos = None
        self.current_hand_radius = 0.05

        self.declare_parameter('forecasting_method', 'nn')
        self.forecasting_method = self.get_parameter('forecasting_method').value
        self.forecast_sub = self.create_subscription(String, '/applications/hand_forecasting', self.forecast_callback, 1)
        self.hand_forecast = [self.current_hand_pos]
        self.forecast_cov = [np.zeros((3,3))]

        # PCBF parameters

        self.T = 1  # Prediction horizon
        self.min_safety_distance = 0.15
        self.m = 1 
        self.perturb_delta = 1e-4  # For numerical gradient computation
        self.alpha = 125
        self.lamda = self.alpha 
        self.max_joint_vel = np.pi / 2 # maximum allowed joint velocity in rad/s
        self.n_joints = 6
        self.state_len = 6
        self.workspace_limits = {
            'x_min': -0.5, 'x_max': 0.1, # don't hit the operator/camera rig
            'y_min': -0.5, 'y_max': 0.8, # don't hit the milling
            'z_min':  0.1           # don't go below table height
        }
        self.slack_penalty = np.array([100, 100])
        self.gamma = 5

        self.home = np.array([-215, -409, 300, -np.pi, 0, 70*np.pi/180])

        self.joint_home = np.array([2.8651701670623897, 1.205664146851847, -1.2036138701180468, 1.5687460232699513, -4.712388900003348, 0.8580415406663497])
            
        self.approachS = np.array([-215, -409, 300, -np.pi, 0, 70*np.pi/180]) # position above the pickup point, in Jaka's base frame
        self.pickS = np.array([-215, -409, -20, -np.pi, 0, 70*np.pi/180]) # position to pick the object, in Jaka's base frame
        self.trajS_jaka = np.array([-500, -409, 250, -np.pi, 0, 70*np.pi/180]) # position to start the handover trajectory, in Jaka's base frame
        self.trajT = np.array([-0.5, 0.4, 0.25, -np.pi, 0, 70*np.pi/180]) # initial target position for the handover, in Jaka's base frame (but in ros units). The robot will try to follow the hand trajectory starting from this point.

        self.tcp_target = self.trajT
        self.q_target = self.robot.get_joint_position()
        self.pos_tolerance = 30e-3
        self.rot_tolerance = 1e-2
        self.KP_pos = 1  # position gain
        self.rot_gain = 1  # orientation gain
        self.converged = False

        self.t = 0
        self.dt = 0.008

        self.control_loop_timer = self.create_timer(1.0/60, self.control_loop) 
        self.homed = False

        self.u_smooth = 0.95
        self.prev_u = np.zeros(6)

    def joint_state_publisher_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        msg.position = self.robot.get_joint_position()
        self.joint_state_publisher.publish(msg)

    def reset_data(self):
        self.record_leap = []
        self.record_forecast = []
        self.h_star = []
        self.h_now = []
        self.joint_positions = []
        self.tcp_positions = []
        self.hand_positions = []
        self.future_hand_positions = []
        self.u_nominals = []
        self.u_actuals = []   
        self.deltas = []   
        self.applied_std = []  
        self.em_stops = []
        self.qp_fails = []

    def homing(self):
        self.robot.open_gripper()
        self.robot.disable_servo_mode()
        self.robot.linear_move(self.approachS, MoveMode.ABSOLUTE, 250, True)
        self.robot.linear_move(self.pickS, MoveMode.ABSOLUTE, 250, True)
        self.robot.close_gripper()
        self.robot.linear_move(self.approachS, MoveMode.ABSOLUTE, 250, True)
        self.robot.linear_move(self.trajS_jaka, MoveMode.ABSOLUTE, 250, True)
        self.robot.enable_servo_mode()
        self.homed = True
        self.q_target = self.robot.get_joint_position()

    def control_loop(self):   
        if not self.homed:self.homing()
                
        self.t += self.dt

        current_state = self.robot.get_joint_position()
        
        if not self.converged:
            # Compute the safe control command, smooth it with the previous command to avoid jerks
            u = self.calculate_u_uapcbf(self.t, current_state)                
            u = self.u_smooth * u + (1 - self.u_smooth) * self.prev_u
            self.prev_u = u
            
            # Integrate the control command to get the new target joint positions
            self.q_target += u * self.dt

            self.robot.servo_j(self.q_target, MoveMode.ABSOLUTE)

            tcp_pose = self.robot.get_tcp_position()
            tcp_pose[0] /= 1000
            tcp_pose[1] /= 1000
            tcp_pose[2] /= 1000

            self.converged = np.allclose(tcp_pose[:3], self.tcp_target[:3], atol=self.pos_tolerance)

        else:
            self.loginfo("Trial completed")
            self.control_loop_timer.cancel()
    
    def leap_fusion_callback(self, msg):
        msg = json.loads(msg.data)
        hands = msg.get('hands')
        if hands:
            for h in hands:
                if h.get('confidence') > 0.50:
                    hand = hands[0].get('hand_keypoints')
                    if h.get('hand_type')=='right':
                        self.current_hand_pos = leap_to_jaka(hand.get('palm_position'))/1000
                    elif h.get('hand_type')=='left':
                        target_pos = leap_to_jaka(hand.get('palm_position'))/1000
                        self.trajT[:3] = target_pos[:3]
                        self.trajT[2] += 0.05

        self.hand_forecast[0] = self.current_hand_pos

    def forecast_callback(self, msg):
        msg = json.loads(msg.data)
        forecast = msg.get('future_trajectory')
        forecast = [leap_to_jaka(f)/1000 for f in forecast]
        forecast = [self.current_hand_pos] + forecast
        self.hand_forecast = forecast
        R = np.array([[0,-1,0], [0,0,1], [-1,0,0]])
        stds = [R.T @ np.diag(std) @ R for std in msg.get('uncertainty')]
        self.forecast_cov = [[[0, 0, 0], [0, 0, 0], [0, 0, 0]]] + stds

    def get_updated_obstacle(self, t):
        u = (t - self.t) / self.T        
        n = len(self.hand_forecast)
        idx = int(np.floor(u * (n - 1)))
        idx = max(0, min(idx, n - 1))
        obstacle_pos = self.hand_forecast[idx]
        obstacle_cov = self.forecast_cov[idx]
        
        return obstacle_pos, obstacle_cov    

    def hand_tcp_dist(self, t, state):
        """Returns the current distance between the bounding surfaces of hand and tcp,
        as well as the direction of it.
        """
        r_cyl = 0.045  # cylinder radius [m]
        h_cyl = 0.35   # cylinder height [m]

        # TCP pose and orientation
        tcp_pose = self.robot.kine_forward(state)
        tcp_pos = tcp_pose.t
        tcp_z_axis = tcp_pose.R[:, 2]  # z-axis of EE

        # Endpoint of cylinder axis
        cyl_top = tcp_pos - h_cyl * tcp_z_axis

        # Obstacle position and radius at time t in [tau, tau + T]
        obstacle_pos, _ = self.get_updated_obstacle(t)

        # Project obstacle onto cylinder axis
        axis_vec = cyl_top - tcp_pos
        axis_dir = axis_vec / np.linalg.norm(axis_vec)
        vec_to_obs = obstacle_pos - tcp_pos
        proj_length = np.dot(vec_to_obs, axis_dir)
        proj_length_clamped = np.clip(proj_length, 0, np.linalg.norm(axis_vec))
        closest_point_on_axis = tcp_pos + proj_length_clamped * axis_dir

        # Min distance between sphere and cylinder
        min_vec = obstacle_pos - closest_point_on_axis
        min_dist = np.linalg.norm(min_vec)
        min_vec_unit = min_vec / min_dist

        # Distance from obstacle to closest surface point on the cylinder
        dist_to_cylinder_surface = min_dist - r_cyl

        return dist_to_cylinder_surface, min_vec_unit

    def h_func(self, t, state, return_cov=False, risk_field=False):
        
        # Obstacle position and radius at time t in [tau, tau + T]
        _, obstacle_cov = self.get_updated_obstacle(t)

        # Distance between hand and tcp
        dist_to_cylinder_surface, min_vec_unit = self.hand_tcp_dist(t, state)

        # Uncertainty handling: project sigma onto the direction from obstacle to tcp, use that as dynamic, uncertain margin
        sigma_dir = np.clip(self.gamma * min_vec_unit.T @ obstacle_cov @ min_vec_unit, 0, self.min_safety_distance)
    
        self.slack_penalty[0] = 100 * (1 - sigma_dir / self.min_safety_distance)

        # Safety margin
        # obstacle_radius is already contained in min_dist!!!!
        ret = (self.min_safety_distance + sigma_dir) - dist_to_cylinder_surface
        if return_cov:
            return ret, sigma_dir if UAPCBF else 0
        else:
            return ret
    
    def h_func_with_gradient(self, t, state):
        """Numerically computes the gradient of h with respect to the state q."""
        h_val = self.h_func(t, state)
        dh = np.zeros(self.state_len)
        for i in range(self.state_len):
            state_plus = state.copy()
            state_minus = state.copy()
            state_plus[i] += self.perturb_delta
            state_minus[i] -= self.perturb_delta
            h_plus = self.h_func(t, state_plus)
            h_minus = self.h_func(t, state_minus)
            dh[i] = (h_plus - h_minus) / (2 * self.perturb_delta)
        return h_val, dh

    def path_func(self, tau, t, state):
        delta_t = tau - t
        u_nom = self.mu_func(state)
        q_pred = state + u_nom * delta_t
        dp_dtau = u_nom      # ∂p/∂τ
        dp_dt    = -u_nom    # ∂p/∂t
        dx       = np.eye(self.state_len)
        return q_pred, dp_dtau, dp_dt, dx
    
    def find_max(self, t, state):
        """
        Finds the time tau within [t, t+T] that maximizes the safety margin h.
        Returns tau, the corresponding h value, and an approximate derivative dtau_dx.
        """
                
        # Solve the optimization using trust-constr with the zero Hessian.
        result = minimize_scalar(
            lambda tau: -self.h_func(tau, self.path_func(tau, t, state)[0]),
            method="bounded",
            bounds=[t, t + self.T],
            options={'xatol': 1e-6}
        )
        tau = result.x
        h_of_tau = -result.fun

        # Approximate dtau/dx via finite differences.
        dtau_dx = np.zeros(self.state_len)
        for i in range(self.state_len):
            state_perturbed = state.copy()
            state_perturbed[i] += self.perturb_delta
            result_perturbed = minimize_scalar(
                lambda tau: -self.h_func(tau, self.path_func(tau, t, state_perturbed)[0]),
                method="bounded",
                bounds=[t, t + self.T],
                options={'xatol': 1e-6}
            )
            tau_perturbed = result_perturbed.x
            dtau_dx[i] = (tau_perturbed - tau) / self.perturb_delta

        return tau, h_of_tau, dtau_dx
    
    def find_zero(self, tau, t, x):
        f = lambda z: self.h_func(z, self.path_func(z, t, x)[0])
        
        eta = self.fzero(f, t, tau)

        p, dp_dtau, _, dp_dx = self.path_func(eta, t, x)
        _, dh = self.h_func_with_gradient(tau, p)
        dx = -dh @ dp_dx / (dh @ dp_dtau)
        
        return eta, dx

    def fzero(self, func, t1, t2, tol=1e-3, max_iter=50):
        h1, h2 = func(t1), func(t2)
        # if no sign‐change, try to find one on a coarse grid
        if h1 * h2 > 0:
            xs = np.linspace(t1, t2, 20)
            hs = [func(x) for x in xs]
            for j in range(len(xs)-1):
                if hs[j]*hs[j+1] <= 0:
                    t1, t2 = xs[j], xs[j+1]
                    h1, h2 = hs[j], hs[j+1]
                    break
            else:
                # still no bracket–give up and pretend safe at M1star
                return t1

        # standard bisection
        for _ in range(max_iter):
            tm = 0.5*(t1+t2)
            hm = func(tm)
            if abs(hm) < tol:
                return tm
            if h1*hm <= 0:
                t2, h2 = tm, hm
            else:
                t1, h1 = tm, hm
        return 0.5*(t1+t2)
    
    def m_func(self, lambda_val):
        m_val = self.m * lambda_val**2
        dm_val = 2 * self.m * lambda_val
        return m_val, dm_val
    
    def hstar_func(self, t, state):
        """
        Computes the predictive CBF value h* and its derivatives for the single integrator (velocity control) case.
        equation references from the PCBF paper by bredeen et al. (https://ieeexplore.ieee.org/document/9992926/)
        """

        # Find first local maxima of h over the time horizon
        M1star, h_of_M1star, dM1star_dx = self.find_max(t, state)

        # R(tau, t, x)   eqn 9        
        if h_of_M1star <= 1e-3: # First local maxima is safe (eqn 9, case 2)
            R = M1star 
            dR_dx = dM1star_dx
        else: # First local maxima is unsafe, compute root of h before it
            try:
                R, dR_dx = self.find_zero(M1star, t, state)
            except Exception as e:
                self.loginfo(f"find_zero failed ({e}), falling back to R=M1star")
                R, dR_dx = M1star, dM1star_dx
            if np.linalg.norm(dR_dx) >= 10 * np.linalg.norm(dM1star_dx):
                # Switch to M1star when the derivative gets too large
                dR_dx = dM1star_dx

        _, cov = self.h_func(R, state, return_cov=True)

        m, dm = self.m_func(R - t)

        hstar = h_of_M1star - m

        predicted_state, _, _, dp_of_tau_dx = self.path_func(M1star, t, state)
        _, dh_of_tau_dx = self.h_func_with_gradient(M1star, predicted_state)

        g = np.eye(6) # Single integrator model
                
        switching_dt = 1e-4
        if M1star <= t + switching_dt or (M1star <= t + self.T + switching_dt and h_of_M1star <= 0):
            # Case iii - eqn 18
            q_pred_plus = self.path_func(M1star + self.dt, t, state)[0]
            h_plus_delta = self.h_func(M1star + self.dt, q_pred_plus)
            dh_dtau = (h_plus_delta - h_of_M1star) / self.dt
            dtau_dt = 1 # Because the system is of high degree
            dt = dh_dtau * dtau_dt
            du = (dh_of_tau_dx @ dp_of_tau_dx) @ g
        elif M1star < t + self.T:
            # Case i - eqn 16
            dt = dm
            du = (dh_of_tau_dx @ dp_of_tau_dx - dm * dR_dx) @ g
        elif M1star <= t + self.T + switching_dt and h_of_M1star > 0:
            # Case ii - eqn 17
            q_pred_plus = self.path_func(M1star + self.dt, t, state)[0]
            h_plus_delta = self.h_func(M1star + self.dt, q_pred_plus)
            dh_dtau = (h_plus_delta - h_of_M1star) / self.dt
            dtau_dt = 1 # For simplicity
            dt = dm + dh_dtau * dtau_dt
            du = (dh_of_tau_dx @ dp_of_tau_dx - dm * dR_dx) @ g
        else:
            self.loginfo("Warning: m1star > t+T")
            dt = 0
            du = np.zeros(self.state_len)
        
        return hstar, dt, du
    
    def mu_func(self, state, dt=None):
        """
        Nominal control law for velocity control.
        Uses a task-space proportional controller to generate a joint velocity command.
        """
        q = state
        current_pose = self.robot.kine_forward(q)
        current_pos = current_pose.t
        target_pos = self.tcp_target[:3]
        pos_error = target_pos - current_pos
        pos_error_mag = np.linalg.norm(pos_error)
        rot_current = rpy_to_rot_matrix(current_pose.rpy())
        rot_target = rpy_to_rot_matrix(self.tcp_target[3:6])
        rot_error_mat = rot_target.dot(rot_current.T)
        rot_error = 0.5 * np.array([
            rot_error_mat[2, 1] - rot_error_mat[1, 2],
            rot_error_mat[0, 2] - rot_error_mat[2, 0],
            rot_error_mat[1, 0] - rot_error_mat[0, 1]
        ])
        rot_error_norm = np.linalg.norm(rot_error)
                    
        v_cmd = np.zeros(6)
        
        if pos_error_mag > self.pos_tolerance:
            v_cmd[:3] = self.KP_pos * pos_error
        
        if rot_error_norm > self.rot_tolerance:
            v_cmd[3:] = self.rot_gain * rot_error
            
        J = self.robot.jacobian(q)
        J_pinv = np.linalg.pinv(J)
        u = J_pinv @ v_cmd
        
        return u
    
    def solve_qp_mult(self, u_nom, As, bs): 
        u = cp.Variable(self.n_joints)

        delta = cp.Variable(2, nonneg=True) 

        objective = cp.Minimize(cp.sum_squares(u) + cp.sum_squares(self.slack_penalty@delta))

        constraints = []
        for A, b, d in zip(As, bs, delta):
            constraints.append(A @ u <= b + d)
        
        problem = cp.Problem(objective, constraints)

        # Naive safety: stop the robot
        u_safe = -u_nom 
        try:
            problem.solve(solver=cp.OSQP)
            if u.value is None or np.any(np.isnan(u.value)):
                self.loginfo("QP failed")
            else:
                u_safe = u.value
        except Exception as e:
            self.loginfo(f"QP solver error: {e}")

        u = u_nom + u_safe
        
        return u

    def calculate_u_uapcbf(self, t, state):
        u_nom = self.mu_func(state)
        if self.current_hand_pos is None:
            return u_nom
        
        # PCBF
        H, dHdt, dHdu = self.hstar_func(t, state)
                
        A_pcbf = dHdu
        b_pcbf = - self.alpha * H - dHdt
        
        # CBF
        h_val, grad_h_q = self.h_func_with_gradient(t, state)
        
        A_cbf = grad_h_q
        b_cbf = -self.lamda * h_val

        u = self.solve_qp_mult(u_nom, [A_pcbf, A_cbf], [b_pcbf, b_cbf])

        return u

    def calculate_u_nominal(self, t, state):
        u_nom = self.mu_func(state)
        return u_nom
    
    def loginfo(self, msg):
        self.logger.info(str(msg))

def main():
    rclpy.init()
    node = SafeControl()
    rclpy.spin(node)


if __name__=='__main__':
    main()