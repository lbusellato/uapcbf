import numpy as np


class KalmanFilter:
    """
    A simple linear Kalman Filter for 3D position+velocity.
    State vector: [x, y, z, vx, vy, vz].
    """

    def __init__(self, dt, Q=None, R=None):
        # Time step
        self.dt = dt
        # State-transition matrix
        self.F = np.eye(6)
        for i in range(3):
            self.F[i, i + 3] = dt
        # Observation matrix: we observe positions only
        self.H = np.zeros((3, 6))
        self.H[:, :3] = np.eye(3)
        # Covariance matrices
        self.Q = Q if Q is not None else np.eye(6) * 1e-4
        self.R = R if R is not None else np.eye(3) * 1e-2
        # Initialize state and covariance
        self.x = np.zeros(6)
        self.P = np.eye(6)

    def initialize(self, first_pos, first_vel=None):
        """
        Set initial state with position and optional velocity.
        """
        self.x[:3] = first_pos
        if first_vel is not None:
            self.x[3:] = first_vel
        self.P = np.eye(6)

    def update(self, z):
        """
        Perform one predict-update cycle with measurement z (3-vector).
        """
        # Predict
        self.x = self.F.dot(self.x)
        self.P = self.F.dot(self.P).dot(self.F.T) + self.Q
        # Update
        y = z - self.H.dot(self.x)
        S = self.H.dot(self.P).dot(self.H.T) + self.R
        K = self.P.dot(self.H.T).dot(np.linalg.inv(S))
        self.x = self.x + K.dot(y)
        self.P = (np.eye(6) - K.dot(self.H)).dot(self.P)

    def predict(self, steps):
        """
        Forecast future positions for `steps` timesteps ahead.
        Returns lists of positions and variances.
        """
        traj = []
        covs = []
        x_pred = self.x.copy()
        P_pred = self.P.copy()
        for _ in range(steps):
            x_pred = self.F.dot(x_pred)
            P_pred = self.F.dot(P_pred).dot(self.F.T) + self.Q
            traj.append(x_pred[:3].tolist())
            covs.append(np.diag(P_pred)[:3].tolist())
        return traj, covs


def kalman_filter(trajectories, rotations, target_hz, T_out=None):
    dt = 1.0 / float(target_hz)
    # Initial velocity estimate
    if trajectories.shape[0] >= 2:
        vel0 = (trajectories[1] - trajectories[0]) / dt
    else:
        vel0 = np.zeros(3)
    # Instantiate filter
    kf = KalmanFilter(dt)
    kf.initialize(trajectories[0], vel0)
    # Feed measurements
    for z in trajectories:
        kf.update(z)
    # Forecast
    steps = T_out or len(trajectories)
    traj, covs = kf.predict(steps)
    # Constant last rotation
    last_rot = rotations[-1]
    rots = [last_rot] * steps
    return traj, covs, rots


def __test__():
    T_out = 30
    target_hz = 30
    meas = np.random.rand(30, 3)  # 30 measurements in 3D
    # Create a dummy trajectory of length 30
    # Run Kalman filter
    traj, covs, rots = kalman_filter(meas, [0, 0, 1], target_hz, T_out)
    print("Forecasted Trajectory:", traj)
    print("Covariances:", covs)

if __name__ == "__main__":
    __test__()
