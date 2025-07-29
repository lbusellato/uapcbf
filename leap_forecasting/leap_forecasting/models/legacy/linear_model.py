import numpy as np


def linear_interpolation(p_prev, p_last, rotations, target_hz, T_out):
    """
    Predicts future positions and rotations using a simple linear model.
    Given the previous and last observed positions, a sequence of observed rotations,
    the target frequency, and the number of future steps to forecast, this function
    estimates future positions by linear extrapolation and assumes constant rotation
    using the last observed rotation.
    Args:
        p_prev (np.ndarray): Previous observed position (as a NumPy array).
        p_last (np.ndarray): Last observed position (as a NumPy array).
        rotations (list or np.ndarray): Sequence of observed rotations.
        target_hz (float): Target frequency (in Hz) for the forecast.
        T_out (int): Number of future time steps to forecast.
    Returns:
        tuple:
            forecast_positions (list): List of predicted future positions (as lists).
            forecast_rotations (list): List of predicted future rotations (constant, last observed).
    """

    dt = 1.0 / float(target_hz)
    velocity = (p_last - p_prev) / dt

    # Build future trajectory
    forecast_positions = []
    for i in range(1, T_out + 1):
        t = i * dt
        forecast_positions.append((p_last + velocity * t).tolist())

    # Constant rotation (last observed)
    rot_last = rotations[-1]
    forecast_rotations = [rot_last] * T_out

    return forecast_positions, forecast_rotations


def __test__():
    T_out = 30
    target_hz = 30
    dummy_traj = np.random.randn(30, 3)
    dummy_rotations = [np.random.rand(3) for _ in range(30)]
    p_prev = dummy_traj[-2]
    p_last = dummy_traj[-1]
    forecast_positions, forecast_rotations = linear_interpolation(p_prev, p_last, dummy_rotations, target_hz, T_out)
    print("Forecasted Positions:", forecast_positions)
    print("Forecasted Rotations:", forecast_rotations)


if __name__ == "__main__":
    __test__()
