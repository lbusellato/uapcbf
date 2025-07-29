#!/usr/bin/env python3
import argparse
import os
import json
import time
import numpy as np
import torch
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64

from ament_index_python import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from leapmotion import LeapFrame

# Attempt relative imports for package structure
try:
    from ..models import LeapUncertainty
    from ..template_forecasting_node import ForecastingNode
except ImportError:
    import sys

    sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/..")
    from models import LeapUncertainty
    from template_forecasting_node import ForecastingNode


# Command-line parsing (if needed in future)
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug", default=False, action="store_true", help="Enable debug mode (mock topics).")
    parser.add_argument(
        "--topic_name", type=str, default="/applications/hand_forecasting", help="Publish topic name for the node."
    )
    return parser.parse_known_args()


# Functions to locate and load the model checkpoint
def get_model_path():
    try:
        pkg_dir = get_package_share_directory("collaborice_forecasting_node")
    except PackageNotFoundError:
        pkg_dir = os.path.join(os.getcwd(), "src", "collaborice_forecasting_node")
    ckpt = os.path.join(pkg_dir, "checkpoints", "LeapUncertainty-prob_model_1000-hz=30-Tin=30-Tout=30.ckpt")
    if not os.path.exists(ckpt):
        raise FileNotFoundError(f"Model checkpoint not found: {ckpt}")
    return ckpt


def load_model(device: torch.device):
    path = get_model_path()
    # Extract T_in and T_out from filename
    T_in = int(path.split("Tin=")[1].split("-")[0])
    T_out = int(path.split("Tout=")[1].split(".")[0])
    model = LeapUncertainty(T_in=T_in, T_out=T_out)
    state = torch.load(path, weights_only=True)
    model.load_state_dict(state["state_dict"])
    model.to(device)
    model.eval()
    return model, T_in, T_out


class NNNode(ForecastingNode):
    """
    Node for forecasting using a neural network model.
    """

    def __init__(
        self,
        model,
        name: str = "nn_prob_forecasting_node",
        target_hz: int = 30,
        publish_topic_name: str | None = None,
        **kwargs,
    ):
        # Initialize base template node
        super().__init__(name=name, target_hz=target_hz, publish_topic_name=publish_topic_name, **kwargs)
        # Attach the neural model
        self.model = model
        # Determine device from model parameters
        self.device = next(self.model.parameters()).device

        # Set time-windows based on model
        self.T_in = model.T_in
        self.T_out = model.T_out
        # Seconds into the future for the last step
        self.seconds_in_future = self.T_out * (1.0 / self.target_hz)

        # Sliding windows for input data
        self.wrist_window_deque = deque(maxlen=self.T_in)
        self.rotations_window_deque = deque(maxlen=self.T_in)

    def do_forecasting(self, input_trajectory_raw: torch.Tensor) -> dict:
        """
        Override the abstract forecasting logic to run the neural network.
        :param input_trajectory_raw: torch.Tensor of shape [T_in, 3]
        :returns: dict with keys matching the output schema
        """
        # Normalize input positions
        norm_traj, first = self.normalize_input(input_trajectory_raw)
        # Prepare GPU tensors
        traj_gpu = norm_traj.unsqueeze(0).to(self.device)  # shape [1, T_in, 3]
        rot_raw = torch.stack(list(self.rotations_window_deque))
        rot_gpu = rot_raw.unsqueeze(0).to(self.device)  # shape [1, T_in, 4]

        # Run model inference
        with torch.no_grad():
            pred: tuple[torch.Tensor, torch.Tensor] = self.model(traj_gpu)

        mu, log_var = pred
        trajectory = mu.detach().cpu().squeeze() + first  # [T, 3], denormalized
        trajectory = trajectory.tolist()  # shape [T_out, 3]
        traj_std = torch.exp(0.5 * log_var).detach().cpu().squeeze().tolist()  # shape [T_out, 3]

        tmp = {
            "future_position": trajectory[-1],
            "future_trajectory": trajectory,
            "future_rotation": [],
            "future_rotations": [],
            "time_seconds_in_future": self.seconds_in_future,
            "time_step_in_future": self.T_out,
            "uncertainty": traj_std,
        }
        return tmp


def main():
    # Parse args (currently unused)
    args, _ = parse_args()
    # Initialize ROS
    rclpy.init()
    # Determine device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    # Load neural forecasting model
    model, T_in, T_out = load_model(device)
    # Instantiate custom forecasting node
    node = NNNode(model=model, publish_topic_name=args.topic_name)
    # Spin until shutdown
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
