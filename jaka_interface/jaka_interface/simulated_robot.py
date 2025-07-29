import numpy as np
import os
import roboticstoolbox as rtb
import time

from ament_index_python.packages import get_package_share_directory
from jaka_interface.base_robot import BaseRobot
from jaka_interface.data_types import *
from jaka_interface.exceptions import JAKA_ERR_CODES
from jaka_interface.pose_conversions import *

SUCCESSFUL_RET = (JAKA_ERR_CODES.SUCCESS_CODE.value, )

class SimulatedRobot(BaseRobot):

    def __init__(self,
                 gripper_power_pin: int = 0,
                 gripper_control_pin: int = 1,
                 gripper_timeout: float = 0.5,
                 urdf_package: str='jaka_description',
                 urdf_name: str='jaka.urdf'):

        # Simulated digital output bus
        self.dout = np.array([False] * 16)

        # Simulated cartesian and joint positions
        self.tcp_position = np.zeros(6)
        self.joint_position = np.zeros(6)

        # Gripper DOUT pins
        self.gripper_power_pin = gripper_power_pin
        self.gripper_control_pin = gripper_control_pin
        # How long to wait for the gripper to open/close
        self.gripper_timeout = gripper_timeout

        # For servo movement, ideally
        self.servo_block_time = 0.008

        package_share_path = get_package_share_directory(urdf_package)
        urdf_path = os.path.join(package_share_path, 'urdf', urdf_name)
        self.chain = rtb.ERobot.URDF(urdf_path)

        super().__init__()
    
    #########################################
    #                                       #
    # Robot initialization                  #
    #                                       #
    #########################################

    def _enable_robot(self)->None: return SUCCESSFUL_RET

    def _disable_robot(self)->None: return SUCCESSFUL_RET

    def _power_on(self)->None: return SUCCESSFUL_RET

    def _power_off(self)->None: return SUCCESSFUL_RET

    def _login(self)->None: return SUCCESSFUL_RET

    def _logout(self)->None: return SUCCESSFUL_RET
    
    def _enable_servo_mode(self)->None: return SUCCESSFUL_RET
    
    def _disable_servo_mode(self)->None: return SUCCESSFUL_RET

    #########################################
    #                                       #
    # Robot motion                          #
    #                                       #
    #########################################

    def _open_gripper(self)->None:
        ret = self.set_digital_output(self.gripper_control_pin, False)
        time.sleep(self.gripper_timeout)
        return ret

    def _close_gripper(self)->None:
        ret = self.set_digital_output(self.gripper_control_pin, True)
        time.sleep(self.gripper_timeout)
        return ret
    
    def _power_off_gripper(self)->None: 
        return self.set_digital_output(self.gripper_control_pin, True)

    def _power_on_gripper(self)->None:
        return self.set_digital_output(self.gripper_control_pin, False)

    #########################################
    #                                       #
    # Robot status                          #
    #                                       #
    #########################################

    def update_status(self)->None:
        robot_status = self.get_robot_status()
        self.state.is_connected = robot_status[RobotStatus.IS_SOCKET_CONNECT.value]
        self.state.is_powered_on = robot_status[RobotStatus.POWERED_ON.value]
        self.state.is_enabled = robot_status[RobotStatus.ENABLED.value]
        self.state.is_in_drag_mode = robot_status[RobotStatus.DRAG_STATUS.value]
        self.state.is_em_stop_pressed = robot_status[RobotStatus.PROTECTIVE_STOP.value]
        dout = robot_status[RobotStatus.DOUT.value]        
        self.state.is_gripper_closed = dout[self.gripper_power_pin] and dout[self.gripper_control_pin]
        self._joint_position = robot_status[RobotStatus.JOINT_POSITION.value]
        self._tcp_position = robot_status[RobotStatus.CART_POSITION.value]
        robot_state = self.get_robot_state()
        self.state.is_in_servo_mode = robot_state[RobotStatus.SERVO_ENABLED.value]

    def _get_robot_state(self)->tuple:
        ret = (self.state.is_em_stop_pressed, self.state.is_powered_on, self.state.is_in_servo_mode)
        return (JAKA_ERR_CODES.SUCCESS_CODE.value, ret)

    def _get_robot_status(self)->tuple:
        ret = (
            JAKA_ERR_CODES.SUCCESS_CODE.value,
            None,                           # inpos
            self.state.is_powered_on,       # powered_on
            self.state.is_enabled,          # enabled
            None,                           # rapidrate
            None,                           # protective_stop TODO: collision checking is kinda easy to implement
            None,                           # drag_status
            None,                           # on_soft_limit
            None,                           # current_user_id
            None,                           # current_tool_id
            self.dout,                      # dout
            None,                           # din
            None,                           # aout
            None,                           # ain
            None,                           # tio_dout
            None,                           # tio_din
            None,                           # tio_ain
            None,                           # extio
            self.tcp_position,              # cart_position
            self.joint_position,            # joint_position
            None,                           # robot_monitor_data
            None,                           # torq_sensor_monitor_data TODO: will need to pull data from mujoco
            self.state.is_connected,        # is_socket_connect
            self.state.is_em_stop_pressed,  # emergency_stop
            None                            # tio_key
        )
        return (JAKA_ERR_CODES.SUCCESS_CODE.value, ret)
    
    def _get_tcp_position(self)->list: return (JAKA_ERR_CODES.SUCCESS_CODE.value, self.tcp_position)

    def _get_joint_position(self)->list: return (JAKA_ERR_CODES.SUCCESS_CODE.value, self.joint_position)
    
    #########################################
    #                                       #
    # Robot IO                              #
    #                                       #
    #########################################

    def _get_digital_output(self, index: int, iotype: IOType=IOType.CABINET)->bool: 
        return (JAKA_ERR_CODES.SUCCESS_CODE.value, self.dout[index])

    def _set_digital_output(self, index: int, value: bool, iotype: IOType=IOType.CABINET)->None:
        self.dout[index] = value
        return SUCCESSFUL_RET
    
    #########################################
    #                                       #
    # Robot kinematics                      #
    #                                       #
    #########################################

    def _rpy_to_rot_matrix(self, rpy: list)->list:
        return (JAKA_ERR_CODES.SUCCESS_CODE.value, rpy_to_rot_matrix(rpy))

    def _rot_matrix_to_rpy(self, rot_matrix: list)->list:
        return (JAKA_ERR_CODES.SUCCESS_CODE.value, rot_matrix_to_rpy(rot_matrix))

    def _quaternion_to_rot_matrix(self, quaternion: list)->list:
        return (JAKA_ERR_CODES.SUCCESS_CODE.value, quaternion_to_rot_matrix(quaternion))

    def _rot_matrix_to_quaternion(self, rot_matrix: list)->list:
        return (JAKA_ERR_CODES.SUCCESS_CODE.value, rot_matrix_to_quaternion(rot_matrix))

    def _kine_inverse(self, ref_pos: list, cartesian_pose: list)->list:
        if ref_pos is None:
            ref_pos = self.get_joint_position()
        ret = self.chain.ik_LM(jaka_to_se3(cartesian_pose), q0=ref_pos)
        return ((JAKA_ERR_CODES.ERR_KINE_INVERSE_ERR.value,) if not ret[1] else (JAKA_ERR_CODES.SUCCESS_CODE.value, ret[0]))

    def _kine_forward(self, joint_pos: list)->list:
        return (JAKA_ERR_CODES.SUCCESS_CODE.value, self.chain.fkine(joint_pos))
   
    def jacobian(self, joint_position: list=None)->np.ndarray:
        if joint_position is None: # Wanting the Jacobian at the current state might be implied
            joint_position = self.get_joint_position()
        return self.chain.jacob0(joint_position)
    
    #########################################
    #                                       #
    # Robot servo mode                      #
    #                                       #
    #########################################

    def _servo_j(self, joint_pos: list, move_mode: MoveMode, step_num: int=1)->None:
        # TODO: check joint limits, velocity limits
        if move_mode == MoveMode.ABSOLUTE:
            self.joint_position = joint_pos
        elif move_mode == MoveMode.INCREMENTAL:
            self.joint_position += joint_pos
            
        time.sleep(self.servo_block_time)

        return SUCCESSFUL_RET

    def _servo_p(self, cartesian_pose: list, move_mode: MoveMode, step_num: int=1)->None:
        # TODO: check joint limits, velocity limits
        curr_tcp = self.tcp_position
        if move_mode == MoveMode.ABSOLUTE:
            target_tcp = cartesian_pose
        elif move_mode == MoveMode.INCREMENTAL:
            target_tcp = curr_tcp + cartesian_pose
        target_joint = self.kine_inverse(self.joint_position, target_tcp)

        time.sleep(self.servo_block_time)

        return self.servo_j(target_joint, MoveMode.ABSOLUTE)