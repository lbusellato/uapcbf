import json
import rclpy
from rclpy.node import Node
import numpy as np
from ament_index_python.packages import get_package_share_directory
from os.path import join
from std_msgs.msg import String

class FakeLeapStreamer(Node):
    def __init__(self):
        super().__init__('fake_leap_streamer')

        self.declare_parameter("camera_name", "desk")
        self.camera_name = self.get_parameter('camera_name').value

        self.declare_parameter("forecasting_method", "kalman")
        self.forecasting_method = self.get_parameter('forecasting_method').value

        self.leap_pub = self.create_publisher(String, '/leap/fusion', 1)
        self.forecast_pub = self.create_publisher(String, '/applications/hand_forecasting', 1)
        
        self.create_timer(1 / 120, self.timer_callback)

        path = join(get_package_share_directory('leap_stream'), 'data/forecast_' + self.forecasting_method + '.npy')
        self.fake_forecast = np.load(path, allow_pickle=True)
        
        path = join(get_package_share_directory('leap_stream'), 'data/leap_' + self.forecasting_method + '.npy')
        self.fake_pos = np.load(path, allow_pickle=True)


        # Find when the forecast starts, to sync the signals
        for f in self.fake_forecast:
            json_data = json.loads(f.data)
            ts = json_data.get('timestamp')
            if ts is not None:
                break
        
        prev_diff = None 
        self.forecast_start = np.inf
        for i, f in enumerate(self.fake_pos):
            diff = ts - json.loads(f.data).get('sync_timestamp')
            if prev_diff is not None and np.sign(diff) != np.sign(prev_diff):
                self.forecast_start = i
            prev_diff = diff

        self.counter = 0
        self.forecast_counter = 0

    def timer_callback(self):
        self.counter += 1
        if self.counter >= len(self.fake_pos):
            self.counter = 0
            
        frame_data =  self.fake_pos[self.counter]
        
        # Publish JSON
        msg = String()
        msg.data = frame_data.data
        self.leap_pub.publish(msg)
        
        if self.counter % 4 == 0: 
            self.forecast_counter += 1
            if self.forecast_counter >= len(self.fake_forecast):
                self.forecast_counter = 0
            forecast = self.fake_forecast[self.forecast_counter]
            msg = String()
            msg.data = forecast.data
            self.forecast_pub.publish(msg)   
        
def main():
    rclpy.init()
    node = FakeLeapStreamer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()