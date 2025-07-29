import numpy as np


def particle_filtering(
    meas,
    T_out,
    target_hz,
    num_particles,
    init_pos_std=0.1,
    init_vel_std=0.1,
    process_pos_std=0.01,
    process_vel_std=0.01,
):
    """
    Forecast future positions and velocities using a particle filter.
    This function propagates a set of particles forward in time without measurement updates to forecast
    a trajectory based on past measurements. It initializes particles around the latest measurement with
    some added randomness and estimates the initial velocity using a backward difference method. At each
    future time step, the particle positions and velocities are updated with process noise, and the mean
    position, trace of the covariance matrix (normalized by 3), and unit vector of the mean velocity are computed.
    Parameters:
        meas (numpy.ndarray): An array of past measurements with shape (N, 3), where N >= 2. Each row represents a 3D position.
        T_out (int): The number of future time steps to forecast.
        target_hz (float): The rate (in Hz) at which measurements are provided, used to calculate the time step length.
        num_particles (int): The number of particles to simulate in the filter.
        init_pos_std (float, optional): Standard deviation for the initial position noise. Defaults to 0.1.
        init_vel_std (float, optional): Standard deviation for the initial velocity noise. Defaults to 0.1.
        process_pos_std (float, optional): Standard deviation for the process noise in position during propagation. Defaults to 0.01.
        process_vel_std (float, optional): Standard deviation for the process noise in velocity during propagation. Defaults to 0.01.
    Returns:
        tuple: A tuple containing:
            - future_trajectory (list): A list of predicted 3D positions for each future time step.
            - covs (list): A list of scalar values representing the average variance (trace of the covariance matrix divided by 3) at each time step.
            - future_rotations (list): A list of unit vectors (as lists) representing the mean velocity direction at each time step.
    Raises:
        ValueError: If the number of measurements provided is less than 2, since at least 2 measurements are necessary to estimate the velocity.
    """
    
    N = meas.shape[0]
    if N < 2:
        raise ValueError("Need at least 2 measurements to estimate velocity.")

    # 2) Forecast parameters
    M = T_out  # number of future steps
    dt = 1.0 / target_hz  # time‐step length
    # total_T = seconds_in_future         # should equal M * dt

    # 3) Estimate initial velocity by backward‐difference
    vel0 = (meas[-1] - meas[-2]) / dt  # shape [3,]

    # 4) Particle‐filter hyperparameters
    P = num_particles

    # 5) Initialize particles [x,y,z, vx,vy,vz]
    particles = np.zeros((P, 6))
    particles[:, :3] = meas[-1] + np.random.randn(P, 3) * init_pos_std
    particles[:, 3:] = vel0 + np.random.randn(P, 3) * init_vel_std

    # Buffers for output
    traj = np.zeros((M, 3))
    covs = []
    rots = []

    # 6) Propagate particles forward with no measurement updates
    for t in range(M):
        # Predict step
        particles[:, :3] += particles[:, 3:] * dt + np.random.randn(P, 3) * process_pos_std
        particles[:, 3:] += np.random.randn(P, 3) * process_vel_std

        # Estimate mean position and variance
        mean_pos = particles[:, :3].mean(axis=0)
        traj[t] = mean_pos
        cov_mat = np.cov(particles[:, :3].T)
        covs.append(np.trace(cov_mat) / 3.0)

        # Rotation = unit‐vector of mean velocity
        mean_vel = particles[:, 3:].mean(axis=0)
        norm = np.linalg.norm(mean_vel) + 1e-8
        rots.append(mean_vel / norm)

    # Convert final arrays to native types
    future_trajectory = traj.tolist()
    future_rotations = [r.tolist() for r in rots]

    return future_trajectory, covs, future_rotations


def __test__():
    T_out = 30
    target_hz = 30
    meas = np.random.rand(30, 3)  # 30 measurements in 3D
    num_particles = 1000
    future_trajectory, covs, future_rotations = particle_filtering(meas, T_out, target_hz, num_particles)
    print("Future Trajectory:", future_trajectory)
    print("Covariances:", covs)


if __name__ == "__main__":
    __test__()
