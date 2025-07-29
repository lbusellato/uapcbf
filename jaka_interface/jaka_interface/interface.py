import rclpy

from jaka_interface.pose_conversions import *
from jaka_interface.data_types import *
from jaka_interface.real_robot import RealRobot
from jaka_interface.simulated_robot import SimulatedRobot
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JakaInterface(Node):
    
    def __init__(self, 
                 ip: str='10.5.5.100', 
                 gripper_control_id: int=0, 
                 gripper_power_supply_id: int=1, 
                 use_jaka_kinematics: bool=False,
                 publish_state: bool=False,
                 simulated: bool=False):

        # Init node
        super().__init__('jaka_interface_node')

        # Logging
        self.logger = self.get_logger()  

        # Create robot instance
        if not simulated:
            self.robot = RealRobot(ip, gripper_power_supply_id, gripper_control_id, use_jaka_kinematics)
        else:
            self.robot = SimulatedRobot(gripper_power_supply_id, gripper_control_id)

        if publish_state:
            self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', qos_profile=1)
            self.joint_state_publisher_frequency = 30 #Hz
            self.joint_state_publisher_timer = self.create_timer(1 / self.joint_state_publisher_frequency, 
                                                                 self.joint_state_publisher_callback)
        
    #########################################
    #                                       #
    # Robot interface                       #
    #                                       #
    #########################################

    def initialize(self):
        self.robot.login()
        # Get the initial status
        self.robot.update_status()
        self.robot.power_on()
        self.robot.enable_robot()

    def shutdown(self):
        self.robot.disable_robot()
        self.robot.power_off()
        self.robot.logout()

    def joint_state_publisher_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        msg.position = self.robot.get_joint_position()
        self.joint_state_publisher.publish(msg)
    
def main():
    rclpy.init()
    node = JakaInterface()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()