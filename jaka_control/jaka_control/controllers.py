import numpy as np
import cvxpy as cp
from scipy.optimize import minimize_scalar

def sphere_cyl_dist(x_n, x_h):
    r_cyl = 0.045  # cylinder radius [m]
    h_cyl = 0.35   # cylinder height [m]

    tcp_pos = x_n.t
    tcp_z_axis = x_n.R[:, 2]  # z-axis of EE

    # Endpoint of cylinder axis
    cyl_top = tcp_pos - h_cyl * tcp_z_axis

    # Project obstacle onto cylinder axis
    axis_vec = cyl_top - tcp_pos
    axis_dir = axis_vec / np.linalg.norm(axis_vec)
    vec_to_obs = x_h - tcp_pos
    proj_length = np.dot(vec_to_obs, axis_dir)
    proj_length_clamped = np.clip(proj_length, 0, np.linalg.norm(axis_vec))
    closest_point_on_axis = tcp_pos + proj_length_clamped * axis_dir

    # Min distance between sphere and cylinder
    min_vec = x_h - closest_point_on_axis
    min_dist = np.linalg.norm(min_vec)

    # Distance from obstacle to closest surface point on the cylinder
    return min_dist - r_cyl

class Controller:
    
    def __init__(self, robot, params):
        self.robot = robot
        self.params = params

        self.x_n = None
        self.x_r = None
        self.x_h = None

    def mu(self, q):
        """
        Nominal control law: computes u_nom based on current joint state.
        """
        # position error
        pos_err = self.x_r[:3] - self.x_n[:3]
        # orientation error
        rot_current = self.robot.rpy_to_rot_matrix(self.x_n[3:])
        rot_target = self.robot.rpy_to_rot_matrix(self.x_r[3:])
        R_err = rot_target.dot(rot_current.T)
        rot_err = 0.5 * np.array([
            R_err[2,1] - R_err[1,2],
            R_err[0,2] - R_err[2,0],
            R_err[1,0] - R_err[0,1]
        ])

        v = np.zeros(6)
        if np.linalg.norm(pos_err) > self.params['pos_tol']:
            v[:3] = self.params['Kp_pos'] * pos_err
        if np.linalg.norm(rot_err) > self.params['rot_tol']:
            v[3:] = self.params['Kp_rot'] * rot_err

        J = self.robot.jacobian(q)
        return np.linalg.pinv(J) @ v
    
    def update(self, q):
        return self.mu(q)