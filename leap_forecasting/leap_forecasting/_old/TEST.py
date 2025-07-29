import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from leapmotion import LeapFrame

LEAP_TOPIC_NAME = "/sensors/leap/json"

class ReaderNode(Node):
    def __init__(self, name: str = "collaborice_forecasting_node", target_hz: int = 30, *args, **kwargs):
        super().__init__(name, *args, **kwargs)

        wait_time = 1.0 / target_hz
        self.subscription = self.create_subscription(String, LEAP_TOPIC_NAME, self.leap_callback, 10)
        self.latest_leap_data = None
        self.timer = self.create_timer(wait_time, self.workon)
        self.publisher = self.create_publisher(String, "/TEST", 10)


    def leap_callback(self, msg: String):
        """Subscription callback that stores the latest leap data."""
        self.latest_leap_data = msg.data

    def workon(self):
        
        try:
            # Parse the JSON formatted string into a dictionary
            data = json.loads(self.latest_leap_data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse leap data: {e}")
            # self.latest_leap_data = None
            return

        frame = LeapFrame(**data)
        n_hands = len(frame.hands)
        msg = String(data=str(n_hands))
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
