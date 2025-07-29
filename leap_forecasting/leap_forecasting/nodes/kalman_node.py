#!/usr/bin/env python3
"""
Kalman forecasting node: subclasses ForecastingNode and predicts future hand trajectory
using a linear Kalman filter over past observations.
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
    from ..models.legacy.kalman_model import kalman_filter
except ImportError:
    import sys

    sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/..")
    from template_forecasting_node import ForecastingNode
    from models.legacy.kalman_model import kalman_filter



def parse_args():
    parser = argparse.ArgumentParser(description="Kalman filter based forecasting node")
    parser.add_argument("--debug", action="store_true", default=False, help="Enable debug/mockup mode")
    parser.add_argument("--hz", type=int, default=30, help="Input data rate (Hz)")
    parser.add_argument("--horizon", type=int, default=30, help="Forecast horizon in number of steps")
    parser.add_argument("--topic_name", type=str, default="/applications/hand_forecasting", help="Publish topic name for the node.")
    return parser.parse_known_args()

class KalmanNode(ForecastingNode):
    """
    ForecastingNode subclass using a Kalman Filter.
    """

    def __init__(self, name: str = "kalman_forecasting_node", target_hz: int = 30, debug: bool = False,  publish_topic_name: str|None = None, **kwargs):
        super().__init__(name=name, target_hz=target_hz, publish_topic_name=publish_topic_name, **kwargs)
        self.debug = debug
        # Forecast horizon
        self.T_out = 30  # set by template or launch
        # Override deques based on template T_in
        self.T_in = getattr(self, "window_size", None)
        if self.T_in is None:
            # If template doesn't set window_size, default to horizon
            self.T_in = target_hz
        # Recreate sliding windows
        self.wrist_window_deque = deque(maxlen=self.T_in)
        self.rotations_window_deque = deque(maxlen=self.T_in)
        # Compute seconds_in_future
        self.seconds_in_future = getattr(self, "seconds_in_future", self.T_out * (1.0 / target_hz))

    def do_forecasting(self, input_trajectory_raw):
        # Convert to numpy array
        trajectories = input_trajectory_raw.cpu().numpy()
        steps = self.T_out or len(trajectories)
        traj, covs, rots = kalman_filter(trajectories, list(self.rotations_window_deque), self.target_hz, self.T_out)
        last_rot = rots[-1]
        return {
            "future_position": traj[-1],
            "future_trajectory": traj,
            "future_rotation": last_rot,
            "future_rotations": rots,
            "time_seconds_in_future": self.seconds_in_future,
            "time_step_in_future": steps,
            "uncertainty": {
                "overall_variance": float(np.mean(covs)),
                "last_variance": covs[-1],
                "overall_certainty": None,
                "last_certainty": None,
            },
        }


def main():
    args, _ = parse_args()
    rclpy.init()
    node = KalmanNode(name="kalman_forecasting_node", target_hz=args.hz, debug=args.debug, publish_topic_name=args.topic_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
