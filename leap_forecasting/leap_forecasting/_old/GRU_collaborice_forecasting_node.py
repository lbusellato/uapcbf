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
from rclpy.node import Node
from std_msgs.msg import String, Float64
from ament_index_python.packages import get_package_share_directory
from leapmotion import LeapFrame

try:
    from .models import RNNWRotation
    from .forecasting_utils import uncertainty_heuristic, uncertainty_covariance_det, uncertainty_covariance_trace
except ImportError:
    from models import RNNWRotation
    from forecasting_utils import uncertainty_heuristic, uncertainty_covariance_det, uncertainty_covariance_trace

logger = rclpy.logging.get_logger("forecasting_node")

parser = argparse.ArgumentParser()
parser.add_argument("--debug", default=False, action="store_true", help="Enable debug mode.")
args, unknown = parser.parse_known_args()

## General Params; TODO; move to a config file
CUDA_IDX = 0
CUDA_DEVICE = torch.device(f"cuda:{CUDA_IDX}" if torch.cuda.is_available() else "cpu")
if not torch.cuda.is_available():
    logger.warn("[W] CUDA not available. Running on CPU.")

MODEL_CHECKPOINT_NAME = "RNNWRotation-rnn_with_rotation_mpjpe-hz=30-Tin=30-Tout=30.ckpt"

DEBUG = args.debug

LEAP_TOPIC_NAME = "/sensors/leap/json"
FORECASTING_TOPIC_NAME = "/applications/hand_forecasting"
if DEBUG:
    LEAP_TOPIC_NAME += "/mockup"
    FORECASTING_TOPIC_NAME += "/mockup"
    # set the logger level to debug
    logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
    logger.debug("[D] Debug mode enabled.")


def get_model_path():
    try:
        package_path = get_package_share_directory("collaborice_forecasting_node")
    except PackageNotFoundError as e:
        package_path = "src/collaborice_forecasting_node"
        logger.warn("[W] Are you in debug? Trying to get the path from the current directory: " + package_path)
    model_path = os.path.join(package_path, "checkpoints", MODEL_CHECKPOINT_NAME)
    assert os.path.exists(model_path), f"Model checkpoint not found: {model_path}"
    return model_path


def get_model():
    model_path = get_model_path()
    T_in = int(model_path.split("Tin=")[1].split("-")[0])
    T_out = int(model_path.split("Tout=")[1].split(".")[0])

    logger.info("Creating model...")
    model = RNNWRotation(T_in=T_in, T_out=T_out)
    logger.info("Loading pre-trained model...")
    pretrained = torch.load(model_path, weights_only=True)
    model.load_state_dict(pretrained["state_dict"])
    model = model.to(CUDA_DEVICE)
    model.eval()
    logger.info("Model loaded successfully.")
    logger.info("Model warmup...")
    inference_time = []
    with torch.no_grad():
        for i in range(10):
            xin = torch.rand(1, T_in, 3).to(CUDA_DEVICE)  # FIXME: cuda device ID
            qin = torch.rand(1, T_in, 4).to(CUDA_DEVICE)
            start = time.time()
            model(xin, qin)
            inference_time.append(time.time() - start)

    logger.info("Model warmup successful.")

    return model, T_in, T_out, np.mean(inference_time).item()


class ForecastingNode(Node):
    def __init__(self, name: str = "collaborice_forecasting_node", target_hz: int = 30, *args, **kwargs):
        super().__init__(name, *args, **kwargs)

        self.target_hz = target_hz  # 30 hz
        model, window_size, T_out, avg_inference_time = get_model()
        self.model: RNNWRotation = model
        self.window_size: int = window_size
        self.T_out: int = T_out

        self.seconds_in_future = self.T_out * (1 / self.target_hz)  # 1 second

        self.wrist_window_deque = deque(maxlen=self.window_size)
        self.rotations_window_deque = deque(maxlen=self.window_size)

        wait_time = 1.0 / self.target_hz  # 1/30

        self.timer = self.create_timer(wait_time, self.forecast)

        self.publisher = self.create_publisher(String, FORECASTING_TOPIC_NAME, 10)
        # self.test_data_publisher_det = self.create_publisher(
        #     Float64, "/applications/hand_forecasting/test_data/score_det", 10
        # )
        # self.test_data_publisher_trace = self.create_publisher(
        #     Float64, "/applications/hand_forecasting/test_data/score_trace", 10
        # )
        # Subscribe to the leap sensor data topic
        self.subscription = self.create_subscription(String, LEAP_TOPIC_NAME, self.leap_callback, 10)
        self.latest_leap_data = None

        self.lock = threading.Lock()

    def leap_callback(self, msg: String):
        """Subscription callback that stores the latest leap data."""
        self.latest_leap_data = msg.data

    def forecast(self):
        with self.lock:
            out_data = {
                # "TEST": 0.0,
                "timestamp": time.time(),
                "future_position": [],
                "future_trajectory": [],
                "future_rotation": [],
                "future_rotations": [],
                "time_seconds_in_future": self.seconds_in_future,  # Time in future
                "time_step_in_future": self.T_out,  # Time step in future
                "uncertainty": {
                    "cov": [],  # variance of all the forecasted positions": [],  # variance of all the forecasted positions
                    "heuristic": 0.0,  # Lower is better
                    #### from probabilistic ####
                    "score": 0.0,  # Lower is better
                    # "variance_last_position": 0.0,  #  variance of the last position in the forecast
                },
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
            wrist, rot, future_wrist_gt = self.parse_leap_data(data)
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

            input_trajectory, _denorm = self.normalize_input(input_trajectory_raw)
            input_rotations = torch.stack(list(self.rotations_window_deque))

            # Perform forecasting
            with torch.no_grad():
                input_trajectory_gpu = input_trajectory.unsqueeze(0).to(CUDA_DEVICE)
                input_rotations_gpu = input_rotations.unsqueeze(0).to(CUDA_DEVICE)
                forecast_output = self.model.forward(input_trajectory_gpu, input_rotations_gpu)
            forecast_fixed, rotations_fixed = self.parse_output(forecast_output, _denorm)

            # Create the output message
            out_data["timestamp"] = time.time()
            out_data["future_position"] = forecast_fixed[-1, :].tolist()  # Only the last position
            out_data["future_trajectory"] = forecast_fixed.tolist()  # All positions
            out_data["future_rotation"] = rotations_fixed[-1, :].tolist()  # Only the last rotation
            out_data["future_rotations"] = rotations_fixed.tolist()  # All rotations

            # Compute uncertainty heuristic
            # score = uncertainty_heuristic(input_trajectory_raw, forecast_fixed)
            # score_det = uncertainty_covariance_det(cov)
            # score_trace = uncertainty_covariance_trace(cov)
            # out_data["uncertainty"]["heuristic"] = score_det.item()
            # out_data["uncertainty"]["heuristic_det"] = score_det.item()
            # out_data["uncertainty"]["heuristic_trace"] = score_trace.item()
            # out_data["uncertainty"]["cov"] = cov.squeeze().tolist()
            out_data["uncertainty"] = None

            ### TMP
            # if future_wrist_gt.numel() == 0:
            #     dist = torch.tensor(99999)
            # else:
            #     dist = torch.norm(forecast_fixed[-1, :] - future_wrist_gt[-1, :])
            # test_out = [score.item(), score2.item()]
            # out_data["TEST"] = test_out
            # self.test_data_publisher_det.publish(Float64(data=score_det.item()))
            # self.test_data_publisher_trace.publish(Float64(data=score_trace.item()))
            # ### TMP

            ## Loggers
            # logger.debug("Wrist: " + str(forecast_fixed[-1, :] / 1000))
            # logger.debug(f"Uncertainty heuristic: {score.item()}")
            # if dist > 0.10 or dist < 0.07:
            #     logger.debug(f"Uncertainty heuristic: {test_out}")

            # Publish the forecast message
            msg = String(data=json.dumps(out_data))
            self.publisher.publish(msg)

    def parse_leap_data(self, data: dict) -> tuple[torch.Tensor, torch.Tensor, LeapFrame | None]:
        frame = LeapFrame(**data)

        hands = frame.hands
        if not hands:
            return None, None, None

        hand = hands[0]
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
    ) -> torch.Tensor:
        if isinstance(output, tuple):
            output, rotations = output
        elif isinstance(output, dict):
            raise NotImplementedError("Not implemented for dict output.")
        elif isinstance(output, torch.Tensor):
            rotations = None
        else:
            raise RuntimeError("Unexpected output type.")

        output = output.squeeze(0).detach().cpu()
        if rotations is not None:
            rotations = rotations.squeeze(0).detach().cpu()

        return output + first_wrist, rotations


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
