import json
import rclpy
from rclpy.node import Node
import numpy as np
from ament_index_python.packages import get_package_share_directory
from os.path import join
from std_msgs.msg import String

class FakeLeapStreamer(Node):
    def __init__(self):
        super().__init__('leap_streamer')

        self.declare_parameter("camera_name", "desk")
        self.camera_name = self.get_parameter('camera_name').value

        self.leap_pub = self.create_publisher(String, '/leap/streamer/' + self.camera_name, 1)
        self.forecast_pub = self.create_publisher(String, '/applications/hand_forecasting/nn', 1)
        
        self.create_timer(1 / 120, self.timer_callback)

        path = join(get_package_share_directory('leap_stream'), 'data/fake_hand_pos.npy')
        self.fake_pos = np.load(path)

        self.counter = 0

    def timer_callback(self):
        hand_pos = self.fake_pos[self.counter, :].tolist()
        self.counter += 1
        if self.counter >= self.fake_pos.shape[0] - 120:
            self.counter = 0
        # TODO: instead of generating fake joint data, record actual jsons and just relay them here
        frame_data = {
            'frame_id': 0,
            'tracking_frame_id': 0,
            'timestamp': 0,
            'hands': [
                {
                    'hand_type': "right",
                    'hand_id': 0,
                    'confidence': 1.0,
                    'hand_keypoints': {
                        'palm_position': hand_pos,
                        'palm_orientation': [],
                        'arm': {
                            'prev_joint': [],
                            'next_joint': [],
                            'rotation': []
                        },
                        'fingers': {},
                        'grab_angle': 0.0 # in radians
                }
            }]
        }
        # Convert to JSON
        json_output = json.dumps(frame_data)
        
        # Publish JSON
        msg = String()
        msg.data = json_output
        self.leap_pub.publish(msg)

        forecast = self.fake_pos[self.counter:self.counter + 120].tolist()
        json_output = json.dumps({
            'future_position' : forecast[-1],
            'future_trajectory' : forecast
            })
        msg = String()
        msg.data = json_output
        self.forecast_pub.publish(msg)   
        
def main():
    rclpy.init()
    node = FakeLeapStreamer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()