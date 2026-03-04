import os
import time
import json
import rclpy
from rclpy.node import Node
import leap
from leap import EventType, TrackingMode, HandType


"""
ROS 2 Leap Motion Data Publisher for [ScreenTop/Desktop] Mode.

This script initializes a ROS 2 node to connect to a Leap Motion device,
set the appropriate tracking mode, listen for hand tracking events,
process the data, and publish it as a JSON string to a ROS 2 topic.

Dependencies:
- rclpy (ROS 2 Python client library)
- leap-python (Leap Motion's Python SDK)
- std_msgs (for String message type)
"""


# Standard ROS2 message types
from std_msgs.msg import Header, String


class LeapMotionPublisher(Node):
    def __init__(self):
        super().__init__('leapmotion_publisher')
        self.publisher_ = self.create_publisher(String, '/sensors/leapDesk/json', 1)
        # self.timer = self.create_timer(0.1, self.publish_joints)
        self.connection = leap.Connection()
        self.listener = MyListener(self.get_logger(), self.publisher_, self)
        self.connection.add_listener(self.listener)

        with self.connection.open():
            #    print("Connected")
            self.connection.set_tracking_mode(leap.TrackingMode.Desktop)
            while True:
                time.sleep(1)


def convert_vector_to_list(vector, scale_factor=1000.0):
    """
    Convert LeapMotion vector to list
    Optional scale_factor to convert units (default converts mm to meters)
    """
    return [
        vector.x / scale_factor,
        vector.y / scale_factor,
        vector.z / scale_factor
    ]


def convert_quaternion_to_list(quaternion):
    """Convert LeapMotion quaternion to list"""
    return [
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w
    ]

def get_this_time_method():
    return time.time()
def create_data(event, frame_counter):
    # Prepare frame data following the Pydantic LeapFrame model
    frame_data = {
        'frame_id': frame_counter,
        'tracking_frame_id': event.tracking_frame_id,
        'timestamp': get_this_time_method(),
        'hands': []
    }

    # Process each hand
    for hand in event.hands:
        # Prepare hand data following the Pydantic LeapHand and HandData models
        hand_data = {
            'hand_type': hand.type.name.lower(),
            'hand_id': hand.id,
            'confidence': hand.confidence,
            'hand_keypoints': {
                'palm_position': convert_vector_to_list(hand.palm.position),
                'palm_orientation': convert_quaternion_to_list(hand.palm.orientation),
                'arm': {
                    'prev_joint': convert_vector_to_list(hand.arm.prev_joint),
                    'next_joint': convert_vector_to_list(hand.arm.next_joint),
                    'rotation': convert_quaternion_to_list(hand.arm.rotation)
                },
                'fingers': {},
                'grab_angle': hand.grab_angle  # in radians
            }
        }

        # Process fingers
        finger_types = ['thumb', 'index', 'middle', 'ring', 'pinky']

        for finger_name, finger in zip(finger_types, [hand.thumb, hand.index, hand.middle, hand.ring, hand.pinky]):
            finger_data = {
                'metacarpal': {
                    'prev_joint': convert_vector_to_list(finger.metacarpal.prev_joint),
                    'next_joint': convert_vector_to_list(finger.metacarpal.next_joint),
                    'rotation': convert_quaternion_to_list(finger.metacarpal.rotation)
                },
                'proximal': {
                    'prev_joint': convert_vector_to_list(finger.proximal.prev_joint),
                    'next_joint': convert_vector_to_list(finger.proximal.next_joint),
                    'rotation': convert_quaternion_to_list(finger.proximal.rotation)
                },
                'intermediate': {
                    'prev_joint': convert_vector_to_list(finger.intermediate.prev_joint),
                    'next_joint': convert_vector_to_list(finger.intermediate.next_joint),
                    'rotation': convert_quaternion_to_list(finger.intermediate.rotation)
                },
                'distal': {
                    'prev_joint': convert_vector_to_list(finger.distal.prev_joint),
                    'next_joint': convert_vector_to_list(finger.distal.next_joint),
                    'rotation': convert_quaternion_to_list(finger.distal.rotation)
                }
            }

            hand_data['hand_keypoints']['fingers'][finger_name] = finger_data

        frame_data['hands'].append(hand_data)

    # Convert to JSON
    json_output = json.dumps(frame_data)

    # Publish JSON
    msg = String()
    msg.data = json_output
    return msg


class MyListener(leap.Listener):
    def __init__(self, logger, publisher, node: Node):
        super().__init__()
        self.logger = logger
        self.node = node
        self.publisher = publisher
        self.frame_counter = 0

    def on_connection_event(self, event):
        print("Connected")

    def on_tracking_event(self, event):
        self.logger.debug("Tracking event: " + str(event.timestamp) + " hands: " + str(len(event.hands)))
        msg = create_data(event, self.frame_counter)  # String
        self.frame_counter += 1
        self.publisher.publish(msg)


def main():
    rclpy.init()
    leapmotion_publisher = LeapMotionPublisher()
    rclpy.spin(leapmotion_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()