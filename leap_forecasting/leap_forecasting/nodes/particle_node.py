#!/usr/bin/env python3
"""
Particle filter forecasting node: subclasses ForecastingNode and predicts future hand trajectory
using a particle filter over past observations.
"""
import argparse
import json
import os
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from leapmotion import LeapFrame

# Import the base template node
try:
    from ..template_forecasting_node import ForecastingNode
    from ..models.legacy.particle_model import particle_filtering
except ImportError:
    import sys

    sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/..")
    from template_forecasting_node import ForecastingNode
    from models.legacy.particle_model import particle_filtering


def parse_args():
    parser = argparse.ArgumentParser(description="Particle filter based forecasting node")
    parser.add_argument("--debug", action="store_true", default=False, help="Enable debug/mockup mode")
    parser.add_argument("--hz", type=int, default=30, help="Input data rate (Hz)")
    parser.add_argument("--horizon", type=int, default=30, help="Forecast horizon in number of steps")
    parser.add_argument("--particles", type=int, default=1000, help="Number of particles to use in filter")
    parser.add_argument("--topic_name", type=str, default="/applications/hand_forecasting", help="Publish topic name for the node.")
    return parser.parse_known_args()


# class ParticleFilter:
#     """
#     A simple particle filter for 3D position+velocity estimation.
#     State vector: [x, y, z, vx, vy, vz].
#     """
#     def __init__(self, num_particles, dt, process_noise_std=1e-2, meas_noise_std=1e-1):
#         self.N = num_particles
#         self.dt = dt
#         # Particles shape: (N, 6)
#         self.particles = np.zeros((self.N, 6))
#         # Equal initial weights
#         self.weights = np.ones(self.N) / self.N
#         # Noise parameters
#         self.process_noise_std = process_noise_std
#         self.meas_noise_cov = np.eye(3) * (meas_noise_std**2)
#         self.meas_noise_inv = np.linalg.inv(self.meas_noise_cov)

#     def initialize(self, first_pos, first_vel=None):
#         # Initialize particles around the first observation
#         self.particles[:, :3] = first_pos + np.random.randn(self.N, 3) * self.process_noise_std
#         if first_vel is not None:
#             self.particles[:, 3:] = first_vel + np.random.randn(self.N, 3) * self.process_noise_std
#         else:
#             self.particles[:, 3:] = np.zeros((self.N, 3))
#         self.weights.fill(1.0 / self.N)

#     def predict(self):
#         # Propagate each particle via simple kinematic model + noise
#         # x_t+1 = x_t + v_t * dt + noise_pos
#         # v_t+1 = v_t + noise_vel
#         noise = np.random.randn(self.N, 6) * self.process_noise_std
#         self.particles[:, :3] += self.particles[:, 3:] * self.dt + noise[:, :3]
#         self.particles[:, 3:] += noise[:, 3:]

#     def update(self, z):
#         # Compute weights by measurement likelihood
#         # Gaussian: exp(-0.5 * (z - pos).T * R^{-1} * (z - pos))
#         diffs = self.particles[:, :3] - z
#         exponents = -0.5 * np.sum(diffs @ self.meas_noise_inv * diffs, axis=1)
#         weights = np.exp(exponents)
#         self.weights = weights / (np.sum(weights) + 1e-12)
#         # Resample
#         indices = np.random.choice(self.N, size=self.N, p=self.weights)
#         self.particles = self.particles[indices]
#         self.weights.fill(1.0 / self.N)

#     def estimate(self):
#         # Return weighted mean and covariance of positions
#         mean = np.average(self.particles, axis=0, weights=self.weights)
#         diffs = self.particles - mean
#         cov = np.cov(diffs.T, aweights=self.weights)
#         return mean, cov

#     def predict_trajectory(self, steps):
#         # Copy particles and weights to simulate future
#         sim_particles = self.particles.copy()
#         traj = []
#         covs = []
#         for _ in range(steps):
#             # propagate sim particles
#             noise = np.random.randn(self.N, 6) * self.process_noise_std
#             sim_particles[:, :3] += sim_particles[:, 3:] * self.dt + noise[:, :3]
#             sim_particles[:, 3:] += noise[:, 3:]
#             # estimate
#             mean = np.mean(sim_particles, axis=0)
#             cov = np.diag(np.cov(sim_particles.T))[:3]
#             traj.append(mean[:3].tolist())
#             covs.append(cov.tolist())
#         return traj, covs


class ParticleFilter:
    """
    A simple particle filter for 3D position+velocity estimation.
    State vector: [x, y, z, vx, vy, vz].
    """

    def __init__(self, num_particles, dt, process_noise_std=1e-2, meas_noise_std=1e-1):
        self.N = num_particles
        self.dt = dt
        # Particles shape: (N, 6)
        self.particles = np.zeros((self.N, 6))
        # Equal initial weights
        self.weights = np.ones(self.N) / self.N
        # Noise parameters
        self.process_noise_std = process_noise_std
        self.meas_noise_cov = np.eye(3) * (meas_noise_std**2)
        self.meas_noise_inv = np.linalg.inv(self.meas_noise_cov)

    def initialize(self, first_pos, first_vel=None):
        # Initialize particles around the first observation
        self.particles[:, :3] = first_pos + np.random.randn(self.N, 3) * self.process_noise_std
        if first_vel is not None:
            self.particles[:, 3:] = first_vel + np.random.randn(self.N, 3) * self.process_noise_std
        else:
            self.particles[:, 3:] = np.random.randn(self.N, 3) * self.process_noise_std * 0.1
        self.weights.fill(1.0 / self.N)

    def predict(self):
        # Propagate each particle via simple kinematic model + noise
        # x_t+1 = x_t + v_t * dt + noise_pos
        # v_t+1 = v_t + noise_vel
        noise = np.random.randn(self.N, 6) * self.process_noise_std
        self.particles[:, :3] += self.particles[:, 3:] * self.dt + noise[:, :3]
        self.particles[:, 3:] += noise[:, 3:]

    def update(self, z):
        # Compute weights by measurement likelihood
        # Gaussian: exp(-0.5 * (z - pos).T * R^{-1} * (z - pos))
        diffs = self.particles[:, :3] - z

        # Correct multivariate Gaussian likelihood calculation
        exponents = -0.5 * np.sum((diffs @ self.meas_noise_inv) * diffs, axis=1)

        # Prevent numerical underflow by subtracting max exponent
        max_exponent = np.max(exponents)
        weights = np.exp(exponents - max_exponent)

        # Update and normalize weights
        self.weights = weights / (np.sum(weights) + 1e-12)

        # Compute effective sample size
        n_eff = 1.0 / np.sum(np.square(self.weights))

        # Resample if effective sample size is too small (typically < N/2)
        if n_eff < self.N / 2:
            self.systematic_resample()

    def systematic_resample(self):
        """
        Systematic resampling algorithm to reduce variance in resampling process
        """
        # Cumulative sum of weights
        cumulative_sum = np.cumsum(self.weights)

        # Generate N ordered points spaced evenly on [0, 1)
        u0 = np.random.uniform(0, 1.0 / self.N)
        u = np.zeros(self.N)
        for i in range(self.N):
            u[i] = u0 + i / self.N

        # Find the first index where the cumulative sum exceeds each point
        indices = np.zeros(self.N, dtype=np.int64)
        i, j = 0, 0
        while i < self.N and j < self.N:
            if u[i] < cumulative_sum[j]:
                indices[i] = j
                i += 1
            else:
                j += 1

        # Finish remaining indices if any
        while i < self.N:
            indices[i] = self.N - 1
            i += 1

        # Resample particles based on indices
        self.particles = self.particles[indices]

        # Reset weights to uniform
        self.weights.fill(1.0 / self.N)

    def estimate(self):
        # Return weighted mean and covariance of particles
        mean = np.average(self.particles, axis=0, weights=self.weights)

        # Calculate weighted covariance with regularization
        diffs = self.particles - mean
        cov = np.zeros((6, 6))

        # Manual weighted covariance calculation for stability
        for i in range(self.N):
            diff = diffs[i].reshape(-1, 1)
            cov += self.weights[i] * (diff @ diff.T)

        # Add small regularization to ensure positive definite covariance
        cov += np.eye(6) * 1e-8

        return mean, cov

    def predict_trajectory(self, steps):
        # Copy particles and weights to simulate future
        sim_particles = self.particles.copy()
        sim_weights = self.weights.copy()
        traj = []
        covs = []

        for _ in range(steps):
            # Propagate sim particles
            noise = np.random.randn(self.N, 6) * self.process_noise_std
            sim_particles[:, :3] += sim_particles[:, 3:] * self.dt + noise[:, :3]
            sim_particles[:, 3:] += noise[:, 3:]

            # Estimate weighted mean position and covariance
            mean = np.average(sim_particles, axis=0, weights=sim_weights)

            # Calculate position covariance (first 3 dimensions)
            diffs = sim_particles[:, :3] - mean[:3]
            cov = np.zeros((3, 3))
            for i in range(self.N):
                diff = diffs[i].reshape(-1, 1)
                cov += sim_weights[i] * (diff @ diff.T)

            traj.append(mean[:3].tolist())
            covs.append(np.diag(cov).tolist())  # Just the variances for simplicity

        return traj, covs


class ParticleFilterNode(ForecastingNode):
    """
    ForecastingNode subclass using a Particle Filter.
    """

    def __init__(
        self,
        name: str = "particle_forecasting_node",
        target_hz: int = 30,
        debug: bool = False,
        num_particles: int = 1000,
         publish_topic_name: str|None = None,
        **kwargs,
    ):
        super().__init__(name=name, target_hz=target_hz, publish_topic_name=publish_topic_name, **kwargs)
        self.debug = debug
        self.num_particles = num_particles
        # Forecast horizon
        self.T_out = 30  # can be overridden by launch/config
        # Input window size
        self.T_in = getattr(self, "window_size", target_hz)
        self.wrist_window_deque = deque(maxlen=self.T_in)
        self.rotations_window_deque = deque(maxlen=self.T_in)
        self.seconds_in_future = getattr(self, "seconds_in_future", self.T_out * (1.0 / target_hz))

    def do_forecasting(self, input_trajectory_raw):

        meas = input_trajectory_raw.cpu().numpy()  # shape [N,3]
        future_trajectory, covs, future_rotations = particle_filtering(
            meas,
            self.T_out,
            self.target_hz,
            self.num_particles,
        )
        future_position = future_trajectory[-1]  # last position in the forecast
        total_T = self.T_out / float(self.target_hz)
        last_rot = future_rotations[-1]

        return {
            "future_position":        future_position,      # list[3]
            "future_trajectory":      future_trajectory,    # list[M][3]
            "future_rotation":        last_rot,             # list[3]
            "future_rotations":       future_rotations,     # list[M][3]
            "time_seconds_in_future": self.seconds_in_future,
            "time_step_in_future":    self.T_out,
            "uncertainty": {
                "overall_variance": float(np.mean(covs)),
                "last_variance":    float(covs[-1]),
                "overall_certainty": None,
                "last_certainty":    None,
            },
        }


def main():
    args, _ = parse_args()
    rclpy.init()
    node = ParticleFilterNode(
        name="particle_forecasting_node", target_hz=args.hz, 
        debug=args.debug, 
        num_particles=args.particles,
        publish_topic_name=args.topic_name
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
