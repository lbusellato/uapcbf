import argparse
import os
import json
import time
import numpy as np
import torch
import threading
from collections import deque

from ament_index_python import PackageNotFoundError
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.node import Node
from std_msgs.msg import String, Float64
from ament_index_python.packages import get_package_share_directory
from leapmotion import LeapFrame


logger = rclpy.logging.get_logger("forecasting_node")

parser = argparse.ArgumentParser()
parser.add_argument("--debug", default=False, action="store_true", help="Enable debug mode.")
args, unknown = parser.parse_known_args()

DEBUG = args.debug
CUDA_DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


# LEAP_TOPIC_NAME = "/sensors/leap/json"
# LEAP_TOPIC_NAME = "/sensors/leapDesk/json"
LEAP_TOPIC_NAME = "/leap/fusion"

FORECASTING_TOPIC_NAME = "/applications/hand_forecasting"
if DEBUG:
    LEAP_TOPIC_NAME += "/mockup"
    FORECASTING_TOPIC_NAME += "/mockup"
    # set the logger level to debug
    logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
    logger.debug("[D] Debug mode enabled.")


class ForecastingNode(LifecycleNode):
    wrist_window_deque: deque[torch.Tensor]
    rotations_window_deque: deque[torch.Tensor]

    def __init__(
        self,
        name: str = "collaborice_forecasting_node",
        target_hz: int = 30,
        publish_topic_name: str | None = None,
        *args,
        **kwargs,
    ):
        super().__init__(name, *args, **kwargs)
        if publish_topic_name is None:
            publish_topic_name = FORECASTING_TOPIC_NAME
        self.target_hz = target_hz
        self.publisher = self.create_publisher(String, publish_topic_name, 10)
        # Subscribe to the leap sensor data topic
        self.subscription = self.create_subscription(String, LEAP_TOPIC_NAME, self.leap_callback, 10)
        self.latest_leap_data = None
        self.lock = threading.Lock()

        wait_time = 1.0 / self.target_hz  # 1/30
        self.timer = self.create_timer(wait_time, self.forecast)

        logger.info(
            f"ForecastingNode initialized with target_hz={self.target_hz}. Starting to listen to {LEAP_TOPIC_NAME} and publishing to {publish_topic_name}."
        )

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        # Handle shutdown here
        self.timer.cancel()
        if self.subscription:
            self.subscription.destroy()
            self.subscription = None
        if self.publisher:
            self.publisher.destroy()
            self.publisher = None
        return TransitionCallbackReturn.SUCCESS

    def leap_callback(self, msg: String):
        """Subscription callback that stores the latest leap data."""
        self.latest_leap_data = msg.data

    def forecast(self):
        with self.lock:
            sec, nsec = self.get_clock().now().seconds_nanoseconds()
            out_data = {
                "timestamp": self.get_clock().now().nanoseconds,  # Better ROS practice than time.time()
                "timestamp_format": "nanoseconds",
                "future_position": [],
                "future_trajectory": [],
                "future_rotation": [],
                "future_rotations": [],
                "time_seconds_in_future": self.seconds_in_future,  # Time in future
                "time_step_in_future": self.T_out,  # Time step in future
                "uncertainty": [],
            }
            """Timer callback that processes the latest leap data and performs forecasting."""
            if self.latest_leap_data is None:
                # No new data available
                self.wrist_window_deque.clear()
                self.rotations_window_deque.clear()
                self.publisher.publish(String(data=json.dumps(out_data)))
                return

            try:
                # Parse the JSON formatted string into a dictionary
                data = json.loads(self.latest_leap_data)
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Failed to parse leap data: {e}")
                # self.latest_leap_data = None
                return

            # Add the new data to the fixed-size window
            # get wrist
            wrist, rot, future_wrist_gt = self.parse_leap_data(data)  # future_wrist_gt is None if not DEBUG
            if wrist is None:
                self.wrist_window_deque.clear()
                self.rotations_window_deque.clear()
                self.publisher.publish(String(data=json.dumps(out_data)))
                return

            self.wrist_window_deque.append(wrist)
            self.rotations_window_deque.append(rot)

            # Reset the latest data (so that we process new data next time)
            # self.latest_leap_data = None

            # Once the window is full, use the model to forecast future output
            if len(self.wrist_window_deque) != self.wrist_window_deque.maxlen:
                return

            # Convert window (a deque of dictionaries/numeric values) to a NumPy array.
            # Here, we assume that the model expects a 3D array with shape
            # (1, window_size, feature_dimension). Adjust as necessary.
            input_trajectory_raw = torch.stack(list(self.wrist_window_deque))
            if DEBUG:
                # if mockup, the data comes in millimeters, so we need to convert it to meters
                input_trajectory_raw /= 1000  # the model expects meters
                future_wrist_gt /= 1000  # the model expects meters

            results_dict = self.do_forecasting(input_trajectory_raw)
            out_data.update(results_dict)

            # Publish the forecast message
            msg = String(data=json.dumps(out_data))
            self.publisher.publish(msg)

    def do_forecasting(self, input_trajectory_raw: torch.Tensor):
        raise NotImplementedError("Forecasting logic not implemented. Provide implementation on subclass.")

    def parse_leap_data(self, data: dict) -> tuple[torch.Tensor, torch.Tensor, LeapFrame | None]:
        none_output = None, None, None

        frame = LeapFrame(**data)

        hands = frame.hands
        if not hands:
            return none_output

        # hand = hands[0]
        # get right hand only
        hand = None
        for h in hands:
            if h.hand_type.lower() == "right":  # only right hand
                hand = h
                break
        if hand is None:
            return none_output

        joints = torch.tensor(hand.joints(include_palm=False, include_arm=False), dtype=torch.float32)
        # to meters; ALREADY IN METERS
        # joints = joints / 1000

        wrist = joints[0]
        rotation = torch.tensor(hand.get_hand_rotation(), dtype=torch.float32)

        if "future" in data:
            future_data = data.pop("future")
            future_data = LeapFrame(**future_data)
            future_hand = [h for h in future_data.hands if h.hand_id == hand.hand_id]
            if future_hand:
                future_data = future_hand[0]
            future_wrist = torch.tensor(future_data.joints(include_palm=False, include_arm=False), dtype=torch.float32)
            # to meters; ALREADY IN METERS
        else:
            future_wrist = None

        return wrist, rotation, future_wrist

    def normalize_input(self, window_wrists: torch.Tensor) -> torch.Tensor:
        # input: [T_in, 3]
        first_wrist = window_wrists[0, :]  # (3,)
        window_wrists = window_wrists - first_wrist  # (T_in, 3)

        return window_wrists, first_wrist

    def parse_output(
        self,
        output: torch.Tensor | tuple[torch.Tensor, torch.Tensor] | dict[str, torch.Tensor],
        first_wrist: torch.Tensor,
        max_accepted_error_meters: float = 0.3,
    ) -> torch.Tensor:
        if isinstance(output, tuple):
            raise NotImplementedError()
        elif isinstance(output, dict):
            # for instance, ProbV2
            #     return {
            #     "trajectory_samples": trajectory_samples,  # [N, B, T, 3]
            #     "rotation_samples": rotation_samples,  # [N, B, T, 4]
            #     "traj_mu": traj_mu,
            #     "traj_logvar": traj_logvar,
            #     "rot_mu": rot_mu,
            #     "rot_logvar": rot_logvar,
            # }
            trajectories = output["trajectory_samples"].squeeze().detach().cpu()  # [N, T, 3]
            rotations = output["rotation_samples"].squeeze().detach().cpu()  # [N, T, 4]

            main_trajectory = trajectories.mean(dim=0)  # [T, 3]
            main_trajectory = main_trajectory + first_wrist  # [T, 3]
            uncertainty = self.get_uncertainty(trajectories, first_wrist, max_accepted_error_meters)

            main_rotation = rotations.mean(dim=0)  # [T, 4]
            main_rotation = main_rotation.squeeze(0)  # [4]

            # compute variance of the last position

            return main_trajectory, main_rotation, uncertainty

        elif isinstance(output, torch.Tensor):
            raise NotImplementedError()
        else:
            raise RuntimeError("Unexpected output type.")

    def get_uncertainty(
        self, trajectories: torch.Tensor, first_wrist: torch.Tensor, max_accepted_error_meters: float = 0.3
    ) -> dict:
        # Assume trajectories has shape [N, T, 3],
        # where N: number of samples, T: number of time steps, and 3: x, y, z coordinates.

        # Denormalize the trajectories by adding the starting position.
        # First positions: usually trajectories[:, 0, :] may be zeros or small deviations.
        first_points = trajectories[:, 0, :] + first_wrist  # Shape: [N, 3]
        # Last positions:
        last_points = trajectories[:, -1, :] + first_wrist  # Shape: [N, 3]

        # Function to compute the covariance matrix for a set of points.
        def compute_cov(points: torch.Tensor) -> torch.Tensor:
            # Calculate the mean position for the given points.
            mean = points.mean(dim=0)  # Shape: [3]
            # Center the points by subtracting the mean.
            centered = points - mean  # Shape: [N, 3]
            # Compute the covariance matrix:
            # For N > 1, use (X^T X) / (N-1). Otherwise, return zeros.
            if points.shape[0] > 1:
                cov = centered.t().mm(centered) / (points.shape[0] - 1)
            else:
                cov = torch.zeros(centered.shape[1], centered.shape[1], device=points.device)
            return cov

        # Compute covariance matrices for the first and last positions.
        first_cov = compute_cov(first_points)  # 3x3 covariance matrix for the first position
        last_cov = compute_cov(last_points)  # 3x3 covariance matrix for the last position

        # Extract the variance (uncertainty) along each axis (diagonal elements).
        first_variances = torch.diag(first_cov)
        last_variances = torch.diag(last_cov)

        return {
            "first_cov": first_cov.tolist(),  # Covariance matrix for the first position
            "last_cov": last_cov.tolist(),
            "first_variances": first_variances.tolist(),  # Uncertainty per axis at the first position.
            "last_variances": last_variances.tolist(),  # Uncertainty per axis at the last position.
            "error": torch.sqrt(last_variances).mean().tolist(),  # Overall uncertainty at the last position.
        }


def main(args=None):
    rclpy.init(args=args)
    node = ForecastingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
