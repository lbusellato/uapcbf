import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import String

import json

from leap_fusion.lib_fusion import separate_hands, fuse_hand, rotations_top_hand


class LeapFusion(Node):

    def __init__(self):
        super().__init__('leap_fusion')
        self.subscription1 = self.create_subscription(String, '/sensors/leapDesk/json', self.listener_callback_1, 1)
        self.subscription2 = self.create_subscription(String, '/sensors/leapScreen/json', self.listener_callback_2, 1)

        self.hand_left_leap1 = None
        self.hand_right_leap1 = None
        self.last_update_leap1 = 0

        self.hand_left_leap2 = None
        self.hand_right_leap2 = None
        self.last_update_leap2 = 0

        self.time_passed = 1000
        self.publisher_ = self.create_publisher(String, '/sensors/leapFusion/json', 1)
        # self.timer = self.create_timer(0.1, self.publish_joints)  # Timer for publishing data

        self.frame_counter = 0

    def listener_callback_1(self, msg):
        new_data = msg.data
        try:
            # Parse the JSON formatted string into a dictionary
            data = json.loads(new_data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse leap data: {e}")
            # self.latest_leap_data = None
            return

        this_time = data['timestamp']

        if this_time < self.last_update_leap1:
            return
        self.last_update_leap1 = this_time
        hands = data['hands']
        left_hand, right_hand = separate_hands(hands)
        self.hand_left_leap1 = left_hand
        self.hand_right_leap1 = right_hand
        self.fuse_data()

    def listener_callback_2(self, msg):
        new_data = msg.data
        try:
            # Parse the JSON formatted string into a dictionary
            data = json.loads(new_data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse leap data: {e}")
            # self.latest_leap_data = None
            return

        this_time = data['timestamp']
        if this_time < self.last_update_leap2:
            return
        self.last_update_leap2 = this_time
        hands = data['hands']
        left_hand, right_hand = separate_hands(hands)
        self.hand_left_leap2 = rotations_top_hand(left_hand)
        self.hand_right_leap2 = rotations_top_hand(right_hand)
        self.fuse_data()

    def publish_joints(self):
        pass  # The listener handles publishing data

    def create_frame(self, fused_left_hand, fused_right_hand, current_time):
        hands_to_send = []
        if fused_left_hand:
            hands_to_send.append(fused_left_hand)
        if fused_right_hand:
            hands_to_send.append(fused_right_hand)
        frame_data = {
            'frame_id': self.frame_counter,
            'timestamp': current_time,
            'hands': hands_to_send
        }
        print(frame_data)
        json_output = json.dumps(frame_data)

        # Publish JSON
        msg = String()
        msg.data = json_output
        return msg

    def fuse_data(self):
        """ Fuse the hand data from both Leap Motion devices """
        current_time = time.time()
        # check time
        '''if (current_time - self.last_update_leap1 >= self.time_passed and
                current_time - self.last_update_leap2 >= self.time_passed):
            return'''

        fused_left_hand = fuse_hand(self.hand_left_leap1, self.hand_left_leap2)
        fused_right_hand = fuse_hand(self.hand_right_leap1, self.hand_right_leap2)

        self.frame_counter += 1
        msg = self.create_frame(fused_left_hand, fused_right_hand, current_time)
        self.send_fused_data(msg)

    def transform_coordinates(self, joint_positions):
        """ Apply a rotation and translation to the second Leap Motion data """
        # Define the rotation matrix and translation vector (to be set according to Leap positioning)
        palm_rot = joint_positions[3:7]  # Elements from index 3 to 6 (Python slicing is exclusive on the end index)
        joint_positions = joint_positions[:3] + joint_positions[7:]  # Everything before index 3 and after index 6
        R = np.eye(3)  # Rotation matrix (to be determined)
        T = np.array([0.0, 0.0, 0.0])  # Translation vector (to be determined)

        joint_positions = np.array(joint_positions).reshape(-1, 3)  # Convert to Nx3 array
        transformed_positions = np.dot(joint_positions, R.T) + T  # Apply rotation and translation
        transformed_positions = transformed_positions.flatten()
        newlist = list(transformed_positions[:3])
        newlist.extend(palm_rot)
        newlist.extend(list(transformed_positions[7:]))

        return newlist  # Return as a 1D array

    def send_fused_data(self, msg):
        self.publisher_.publish(msg)
        print('Published fused data')


def main(args=None):
    rclpy.init(args=args)
    leap_fusion = LeapFusion()
    rclpy.spin(leap_fusion)
    leap_fusion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
