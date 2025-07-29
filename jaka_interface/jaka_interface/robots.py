import inspect

from jaka_interface.data_types import *
from jaka_interface.decorators import *
from jaka_messages.msg import RobotState
from spatialmath import SE3
from typing import Any

try:
    import jkrc
except Exception:
    from jaka_interface.exceptions import SDKNotFoundException
    raise SDKNotFoundException('Failed to import jkrc. Did you add the path to the lib folder to both PYTHONPATH and \
                               LD_LIBRARY_PATH environment variables?')
import numpy as np
import os
import roboticstoolbox as rtb
import time

from ament_index_python.packages import get_package_share_directory
from jaka_interface.pose_conversions import *

from jaka_interface.data_types import *
from jaka_interface.exceptions import JAKA_ERR_CODES, SUCCESSFUL_RET

class BaseRobot():
    
    def __init__(self):

        # Initialize the robot state
        self.state = RobotState()
        self.state.is_connected       = False
        self.state.is_powered_on      = False
        self.state.is_enabled         = False
        self.state.is_em_stop_pressed = False
        self.state.is_gripper_closed  = False
        self.state.is_in_servo_mode   = False
        self.state.is_in_drag_mode    = False

    def __str__(self):
        return self.__class__.__name__

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        # only fill in if subclass has no docstring of its own
        if cls.__doc__ is None:
            cls.__doc__ = BaseRobot.__doc__

    def initialize(self, collision_recover: bool=False):
        try:
            self.login()
            # Get the initial status
            self.update_status()
            self.power_on()
            self.enable_robot()
            self.power_on_gripper()

            if collision_recover and self.get_robot_status()[5]:
                self.collision_recover()

        except KeyboardInterrupt:
            pass

    def shutdown(self):
        try:
            pass
            #self.disable_robot()
            #self.power_off()
            #self.logout()
            #self.power_off_gripper()
        except KeyboardInterrupt:
            pass

    #########################################
    #                                       #
    # Basic operation of robots             #
    #                                       #
    #########################################
    
    @process_sdk_call(update_context='state', update_attr='is_connected', update_mode=3, update_val=True, success_log_msg='Logged in.')
    def login(self)->None:
        """Connects to the robot.
        """
        return self._login()
    def _login(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(update_context='state', update_attr='is_connected', update_mode=3, update_val=False, success_log_msg='Logged off.')
    def logout(self)->None:
        """Disconnects from the robot.
        """
        return self._logout()
    def _logout(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True, update_context='state', update_attr='is_powered_on', update_mode=3, update_val=True, success_log_msg='Powered on.')
    def power_on(self)->None:
        """Powers on the robot.
        """
        return self._power_on()
    def _power_on(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True, update_context='state', update_attr='is_powered_on', update_mode=3, update_val=False, success_log_msg='Powered off.')
    def power_off(self)->None:
        """Powers off the robot.
        """
        return self._power_off()
    def _power_off(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def servo_move_enable(self, value: bool)->None:
        """Enable or disable servo mode.

        Parameters
        ----------
        value : bool
            Whether to enable servo mode.
        """
        
        msg = 'You should not directly call servo_move_enable, because it does not update the robot state. This can\
            and will lead to undefined, unstable and potentially dangerous behaviour.'
        rclpy.logging.get_logger('BaseRobot').warning(msg)
        return self._servo_move_enable(value)
    def _servo_move_enable(self, value: bool)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")
    
    @process_sdk_call(connected=True, update_context='state', update_attr='is_in_servo_mode', update_mode=3, update_val=True, success_log_msg='Enabled servo mode.')
    def enable_servo_mode(self)->None:
       """Enables servo mode.
       """
       return self._enable_servo_mode()
    def _enable_servo_mode(self)->None:
       raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")
    
    @process_sdk_call(connected=True, update_context='state', update_attr='is_in_servo_mode', update_mode=3, update_val=False, success_log_msg='Disabled servo mode.')
    def disable_servo_mode(self)->None:
       """Disable servo mode.
       """
       return self._disable_servo_mode()
    def _disable_servo_mode(self) -> None:
       raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def shutdown_cabinet(self)->None:
        """Shuts down the robot's cabinet.
        """
        return self._shutdown_cabinet()
    def _shutdown_cabinet(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")
    
    @process_sdk_call(powered_on=True, update_context='state', update_attr='is_enabled', update_mode=3, update_val=True, success_log_msg='Enabled robot.')
    def enable_robot(self)->None:
        """Enables the robot.
        """
        return self._enable_robot()
    def _enable_robot(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(powered_on=True, update_context='state', update_attr='is_enabled', update_mode=3, update_val=False, success_log_msg='Disabled robot.')
    def disable_robot(self)->None:
        """Disables the robot.
        """
        return self._disable_robot()
    def _disable_robot(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def get_sdk_version(self) -> str:
        return self._get_sdk_version()
    def _get_sdk_version(self) -> str:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def get_controller_ip(self) -> str:
        return self._get_controller_ip()
    def _get_controller_ip(self) -> str:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(powered_on=True)
    def drag_mode_enable(self, state)->None:
        return self._drag_mode_enable(state)
    def _drag_mode_enable(self, state)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(enabled=True, update_context='state', update_attr='is_connected', update_mode=2)
    def is_in_drag_mode(self) -> bool:
        return self._is_in_drag_mode()
    def _is_in_drag_mode(self) -> bool:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def set_SDK_log_filepath(self, path: str)->None:
        return self._set_SDK_log_filepath(path)
    def _set_SDK_log_filepath(self, path: str)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    #########################################
    #                                       #
    # Robot motion                          #
    #                                       #
    #########################################

    @process_sdk_call(enabled=True, servo_disabled=True)
    def jog(self, aj_num: int, move_mode: MoveMode, coord_type: CoordType, jog_vel: float, pos_cmd: float)->None:
        """Jog the robot, either in joint or Cartesian space. Note that jog will fail if the robot is close to a singularity. Also keep in mind that jog is a non-blocking function. Further commands received during motion will be discarded.

        Parameters
        ----------
        aj_num : int
            The [0-5] index of the joint or Cartesian coordinate to jog.
        move_mode : MoveMode
            Movement mode. Either MoveMode.ABSOLUTE, MoveMode.INCREMENTAL or MoveMode.CONTINUOUS.
        coord_type : CoordType
            Coordinate frame. Either CoordType.COORD_BASE, CoordType.COORD_JOINT or CoordType.COORD_TOOL.
        jog_vel : float
            Jog velocity, expressed in rad/s for joint space motion or mm/s for Cartesian space motion.
        pos_cmd : float
            The target value for the jog, in rad for joint space motion or mm for Cartesian space motion.
        """
        return self._jog(aj_num, move_mode, coord_type, jog_vel, pos_cmd)
    def _jog(self, aj_num: int, move_mode: MoveMode, coord_type: CoordType, jog_vel: float, pos_cmd: float)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(enabled=True, servo_disabled=True)
    def jog_stop(self, aj_num: int)->None:
        """_summary_

        Parameters
        ----------
        aj_num : int
            The [0-5] index of the joint or Cartesian coordinate to jog, or -1 to stop jogging across all axes.
        """
        return self._jog_stop(aj_num)
    def _jog_stop(self, aj_num: int)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(enabled=True, servo_disabled=True)
    def joint_move(self, joint_pos: list, move_mode: MoveMode, speed: float, is_blocking: bool)->None: 
        """Move the robot linearly in joint space.

        Parameters
        ----------
        joint_pos : list
            The target joint position, in rad.
        move_mode : MoveMode
            Movement mode. Either MoveMode.ABSOLUTE, MoveMode.INCREMENTAL or MoveMode.CONTINUOUS
        speed : float
            The speed of the motion.
        is_blocking : bool
            Whether or not the SDK function call is blocking.
        """
        return self._joint_move(joint_pos, move_mode, speed, is_blocking)
    def _joint_move(self, joint_pos: list, move_mode: MoveMode, speed: float, is_blocking: bool)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(enabled=True, servo_disabled=True)
    def joint_move_extend(self, joint_pos: list, move_mode: MoveMode, speed: float, acc: float, tol: float, is_blocking: bool)->None: 
        """Move the robot linearly in joint space. Extended to allow for acceleration specification as well as target position tolerance.

        Parameters
        ----------
        joint_pos : list
            The target joint position, in rad.
        move_mode : MoveMode
            Movement mode. Either MoveMode.ABSOLUTE, MoveMode.INCREMENTAL or MoveMode.CONTINUOUS
        speed : float
            The speed of the motion, in rad/s.
        acc : float
            The acceleration of the motion, in rad/s^2.
        tol : float
            The endpoint error.
        is_blocking : bool
            Whether or not the SDK function call is blocking.
        """
        return self._joint_move_extend(joint_pos, move_mode, speed, acc, tol, is_blocking)
    def _joint_move_extend(self, joint_pos: list, move_mode: MoveMode, speed: float, acc: float, tol: float, is_blocking: bool)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(enabled=True, servo_disabled=True)
    def linear_move(self, tcp_pos: list, move_mode: MoveMode, speed: float, is_blocking: bool):
        """Move the robot linearly in Cartesian space. 

        Parameters
        ----------
        end_pos : list
            The target TCP position, in mm.
        move_mode : MoveMode
            Movement mode. Either MoveMode.ABSOLUTE, MoveMode.INCREMENTAL or MoveMode.CONTINUOUS
        speed : float
            The speed of the motion, in mm/s.
        is_blocking : bool
            Whether or not the SDK function call is blocking.
        """
        return self._linear_move(tcp_pos, move_mode, speed, is_blocking)
    def _linear_move(self, tcp_pos: list, move_mode: MoveMode, speed: float, is_blocking: bool):
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(enabled=True, servo_disabled=True)    
    def linear_move_extend(self, tcp_pos: list, move_mode: MoveMode, speed: float, acc: float, tol: float, is_blocking: bool)->None:
        """Move the robot linearly in Cartesian space. Extended to allow for acceleration specification as well as target position tolerance.

        Parameters
        ----------
        tcp_pos : list
            The target TCP position, in mm.
        move_mode : MoveMode
            Movement mode. Either MoveMode.ABSOLUTE, MoveMode.INCREMENTAL or MoveMode.CONTINUOUS
        speed : float
            The speed of the motion, in mm/s.
        acc : float
            The acceleration of the motion, in mm/s^2.
        tol : float
            The endpoint error.
        is_blocking : bool
            Whether or not the SDK function call is blocking.
        """
        return self._linear_move_extend(tcp_pos, move_mode, speed, acc, tol, is_blocking)
    def _linear_move_extend(self, tcp_pos: list, move_mode: MoveMode, speed: float, acc: float, tol: float, is_blocking: bool)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(enabled=True, servo_disabled=True)
    def circular_move(self, tcp_pos: list, midpoint: list, move_mode: MoveMode, speed: float, acc: float, tol: float, is_blocking: bool)->None:
        """Move the robot in an arc in Cartesian space.

        Parameters
        ----------
        tcp_pos : list
            The target TCP position, in mm.
        midpoint : list
            Midpoint of the TCP along the arc, in mm.
        move_mode : MoveMode
            Movement mode. Either MoveMode.ABSOLUTE, MoveMode.INCREMENTAL or MoveMode.CONTINUOUS
        speed : float
            The speed of the motion, in mm/s.
        acc : float
            The acceleration of the motion, in mm/s^2.
        tol : float
            The endpoint error.
        is_blocking : bool
            Whether or not the SDK function call is blocking.
        """
        return self._circular_move(tcp_pos, midpoint, move_mode, speed, acc, tol, is_blocking)
    def _circular_move(self, tcp_pos: list, midpoint: list, move_mode: MoveMode, speed: float, acc: float, tol: float, is_blocking: bool)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(enabled=True, servo_disabled=True)
    def circular_move_extend(self, tcp_pos: list, midpoint: list, move_mode: MoveMode, speed: float, acc: float, 
                               tol: float, is_blocking: bool, circle_cnt: int)->None:
        """Move the robot in an arc in Cartesian space. Adds a `circle_cnt` parameter of unknown meaning.

        Parameters
        ----------
        tcp_pos : list
            The target TCP position, in mm.
        midpoint : list
            Midpoint of the TCP along the arc, in mm.
        move_mode : MoveMode
            Movement mode. Either MoveMode.ABSOLUTE, MoveMode.INCREMENTAL or MoveMode.CONTINUOUS
        speed : float
            The speed of the motion, in mm/s.
        acc : float
            The acceleration of the motion, in mm/s^2.
        tol : float
            The endpoint error.
        is_blocking : bool
            Whether or not the SDK function call is blocking.
        circle_cnt : int
            Per the SDK manual: 'circle number of MoveC'.
        """
        return self._circular_move_extend(tcp_pos, midpoint, move_mode, speed, acc, tol, is_blocking, circle_cnt)
    def _circular_move_extend(self, tcp_pos: list, midpoint: list, move_mode: MoveMode, speed: float, acc: float, 
                                tol: float, is_blocking: bool, circle_cnt: int)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")      

    @process_sdk_call(enabled=True)
    def get_motion_status(self) -> list:
        """Get the robot's motion status.

        Returns
        -------
        list
            The robot motion status, containing in order:
            - int: the current motion command id
            - int: reserved
            - bool: whether the previous motion command reached target position
            - bool: whether the previous motion command was dropped by the controller
            - int: index of the motion command in the buffer
            - int: index of the motion command in the blending buffer
            - bool: whether the motion queue is full
            - bool: whether the motion command is paused
        """
        return self._get_motion_status()
    def _get_motion_status(self) -> list:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True, update_context='state', update_attr='is_gripper_closed', update_mode=3, update_val=True)
    def open_gripper(self)->None:
        """Open the gripper by turning on the corresponding digital outputs.
        """
        return self._open_gripper()
    def _open_gripper(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")
    
    @process_sdk_call(connected=True, update_context='state', update_attr='is_gripper_closed', update_mode=3, update_val=False)
    def close_gripper(self)->None:
        """Close the gripper by turning on the corresponding digital output.
        """
        return self._close_gripper()
    def _close_gripper(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")
    
    @process_sdk_call(connected=True)
    def power_on_gripper(self)->None:
        """Provide power to the gripper.
        """
        return self._power_on_gripper()
    def _power_on_gripper(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")
    
    @process_sdk_call(connected=True)
    def power_off_gripper(self)->None:
        """Remove power to the gripper.
        """
        return self._power_off_gripper()
    def _power_off_gripper(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    #########################################
    #                                       #
    # Set and interrogate robot information #
    #                                       #
    #########################################

    def update_status(self) -> None:
        "Update self.state"
        return self._update_status
    def _update_status(self) -> None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_robot_status_simple(self) -> tuple:
        """Get the simplified robot status.

        Returns
        -------
        list
            The simplified robot status: (error_code, error_message, powered_on, enabled).
        """
        return self._get_robot_status_simple()
    def _get_robot_status_simple(self) -> tuple:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_robot_status(self) -> tuple:
        """Get the full robot status.

        Returns
        -------
        tuple
            The full robot status, containing the following values:
            - error_code
            - inpos: whether the robot is in position
            - powered_on: whether the robot is powered on
            - enabled: whether the robot is enabled
            - rapidrate: robot speed rate
            - protective_stop: whether the robot detects a collision
            - drag_status: whether the robot is in drag mode
            - on_soft_limit: whether the robot is on limit
            - current_user_id: the current user coordinate frame ID used by the robot
            - current_tool_id: the current tool coordinate frame ID used by the robot
            - dout: control cabinet digital output signal
            - din: control cabinet digital input signal
            - aout: control cabinet analog output signal
            - ain: control cabinet analog input signal
            - tio_dout: end tool digital output signal
            - tio_din: end tool digital input signal
            - tio_ain: end tool analog input signal
            - extio: external extension IO module signal
            - cart_position: TCP cartesian position
            - joint_position: robot joint space position
            - robot_monitor_data: (scb major version, scb minor version, controller temperature, robot average voltage, robot average current, monitor data of 6 robot joints (transient current, voltage and temperature))
            - torq_sensor_monitor_data: (torque sensor IP addresses and port numbers, tool payload (payload mass, centroid x-axis, y-axis and z-axis coordinates), torque sensor status and exception error code, actual contact force values of 6 torque sensors and original reading values of 6 torque sensors)
            - is_socket_connect: whether the connection channel of SDK and controller is normal
            - emergency_stop: whether the robot is emergency stopped
            - tio_key: Robot end tool button [0] free; [1] point; [2] end light button
        """
        return self._get_robot_status()
    def _get_robot_status(self) -> tuple:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_dh_params(self) -> dict:
        """Get the robot's Denavit-Hartenberg parameters

        Returns
        -------
        dict
            Dictionary of parameters.
        """
        return self._get_dh_params()
    def _get_dh_params(self) -> dict:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True, update_context='', update_attr='joint_position', update_mode=2)
    def get_joint_position(self) -> list:
        """Get the current joint position of the robot.

        Returns
        -------
        list
            The current joint position of the robot.
        """
        return self._get_joint_position()
    def _get_joint_position(self) -> list:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True, update_context='', update_attr='tcp_position', update_mode=2)
    def get_tcp_position(self) -> list:
        """Get the current position of the TCP.

        Returns
        -------
        list
            The current position of the TCP.
        """
        return self._get_tcp_position()
    def _get_tcp_position(self) -> list:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def set_user_frame_data(self, id: int, user_frame, name: str)->None:
        """Set a user frame.

        Parameters
        ----------
        id : int
            The [1-10] id of the user frame. 0 is reserved for the robot frame.
        user_frame : list
            The [x, y, z, r, p, y] offset of the frame.
        name : str
            String alias for the user frame.
        """
        return self._set_user_frame_data(id, user_frame, name)
    def _set_user_frame_data(self, id: int, user_frame, name: str)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def get_user_frame_data(self, id: int) -> tuple:
        """Get data of a user frame.

        Parameters
        ----------
        id : int
            The [1-10] id of the user frame. 0 is reserved for the robot frame.

        Returns
        -------
        tuple
            The (id, [x, y, z, r, p, y]) data of the frame.
        """
        return self._get_user_frame_data(id)
    def _get_user_frame_data(self, id: int) -> tuple:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def get_user_frame_id(self) -> int:
        """Get the id of the user frame in use.

        Returns
        -------
        int
            The id of the frame.
        """
        return self._get_user_frame_id()
    def _get_user_frame_id(self) -> int:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def set_user_frame_id(self, id: int)->None:
        """Set the id of the user frame to use.

        Parameters
        ----------
        int
            The [1-10] id of the user frame to use.
        """
        return self._set_user_frame_id(id)
    def _set_user_frame_id(self, id: int)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")
    
    @process_sdk_call(connected=True)
    def get_tool_id(self) -> int:
        """Get the id of the tool in use. 0 represents the end flange.

        Returns
        -------
        int
            The id of the frame.
        """
        return self._get_tool_id()
    def _get_tool_id(self) -> int:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def set_tool_id(self, id: int)->None:
        """Set the id of the tool in use. 0 represents the end flange.

        Parameters
        -------
        int
            The id of the tool.
        """
        return self._set_tool_id(id)
    def _set_tool_id(self, id: int)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def set_tool_data(self, id: int, tool: list, name: str)->None:
        """Set a tool data.

        Parameters
        ----------
        id : int
            The [1-10] id of the tool. 0 is reserved for the end flange.
        tool : list
            The [x, y, z, r, p, y] offset of the tool.
        name : str
            String alias for the tool.
        """
        return self._set_tool_data(id, tool, name)
    def _set_tool_data(self, id: int, tool: list, name: str)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_tool_data(self, id: int) -> tuple:
        """Get data of a tool.

        Parameters
        ----------
        id : int
            The [1-10] id of the tool. 0 is reserved for the end flange.

        Returns
        -------
        tuple
            The (frame_id, [x, y, z, r, p, y]) data of the tool.
        """
        return self._get_tool_data(id)
    def _get_tool_data(self, id: int) -> tuple:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def set_digital_output(self, index: int, value: bool, iotype: IOType = IOType.CABINET)->None:
        """Set the state of the index-th (0 based) digital output.

        Parameters
        ----------
        index : int
            The 0-based index of the digital output port.  
        value : bool
            The value to set the output to.  
        iotype : IOType, optional
            The IO type. Either IOType.CABINET, IOType.TOOL or IOType.EXTEND, by default IOType.CABINET
        """
        return self._set_digital_output(index, value, iotype)
    def _set_digital_output(self, index: int, value: bool, iotype: IOType = IOType.CABINET)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def set_analog_output(self, index: int, value: float, iotype: IOType = IOType.CABINET)->None:
        """Set the state of the index-th (0 based) analog output.

        Parameters
        ----------
        index : int
            The 0-based index of the analog output port.  
        value : float
            The value to set the output to.  
        iotype : IOType, optional
            The IO type. Either IOType.CABINET, IOType.TOOL or IOType.EXTEND, by default IOType.CABINET
        """
        return self._set_analog_output(index, value, iotype)
    def _set_analog_output(self, index: int, value: float, iotype: IOType = IOType.CABINET)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_digital_input(self, index: int, iotype: IOType = IOType.CABINET) -> bool:
        """Read the state of the index-th (0 based) digital input.

        Parameters
        ----------
        index : int
            The 0-based index of the digital input port.  
        iotype : IOType, optional
            The IO type. Either IOType.CABINET, IOType.TOOL or IOType.EXTEND, by default IOType.CABINET

        Returns
        -------
        bool
            The state of the given digital input.
        """
        return self._get_digital_input(index, iotype)
    def _get_digital_input(self, index: int, iotype: IOType = IOType.CABINET) -> bool:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_digital_output(self, index: int, iotype: IOType = IOType.CABINET) -> bool:
        """Read the state of the index-th (0 based) digital output.

        Parameters
        ----------
        index : int
            The 0-based index of the digital output port.  
        iotype : IOType, optional
            The IO type. Either IOType.CABINET, IOType.TOOL or IOType.EXTEND, by default IOType.CABINET

        Returns
        -------
        bool
            The state of the given digital output.
        """
        return self._get_digital_output(index, iotype)
    def _get_digital_output(self, index: int, iotype: IOType = IOType.CABINET) -> bool:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def get_analog_input(self, index: int, iotype: IOType = IOType.CABINET) -> float:
        """Read the state of the index-th (0 based) analog input.

        Parameters
        ----------
        index : int
            The 0-based index of the analog input port.  
        iotype : IOType, optional
            The IO type. Either IOType.CABINET, IOType.TOOL or IOType.EXTEND, by default IOType.CABINET

        Returns
        -------
        float
            The state of the given analog input.
        """
        return self._get_analog_input(index, iotype)
    def _get_analog_input(self, index: int, iotype: IOType = IOType.CABINET) -> float:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_analog_output(self, index: int, iotype: IOType = IOType.CABINET) -> float:
        """Read the state of the index-th (0 based) analog output.

        Parameters
        ----------
        index : int
            The 0-based index of the analog output port.  
        iotype : IOType, optional
            The IO type. Either IOType.CABINET, IOType.TOOL or IOType.EXTEND, by default IOType.CABINET

        Returns
        -------
        float
            The state of the given analog output.
        """
        return self._get_analog_output(index, iotype)
    def _get_analog_output(self, index: int, iotype: IOType = IOType.CABINET) -> float:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def is_extio_running(self) -> bool:
        """Check if the IO extension is running.

        Returns
        -------
        bool
            Whether the IO extension is running.
        """
        return self._is_extio_running()
    def _is_extio_running(self) -> bool:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def set_payload(self, payload: tuple)->None:
        """Set the payload at the TCP.

        Parameters
        ----------
        payload : tuple
            The payload (m, [x,y,z]) at the TCP.
        """
        return self._set_payload(payload)
    def _set_payload(self, payload: tuple)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)        
    def get_payload(self) -> tuple:
        """Get the payload at the TCP.

        Returns
        -------
        tuple
            The payload (m, [x,y,z]) at the TCP.
        """
        return self._get_payload()
    def _get_payload(self) -> tuple:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def set_tio_vout_param(self, vout_enable: bool, vout_vol: VOut)->None:
        """Set the TIO voltage parameters.

        Parameters
        ----------
        vout_enable : bool
            Enable/disable TIO voltage output.
        vout_vol : VOut
            Either VOut.V24 or VOut.V12.
        """
        return self._set_tio_vout_param(vout_enable, vout_vol)
    def _set_tio_vout_param(self, vout_enable: bool, vout_vol: VOut)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_tio_vout_param(self) -> tuple:
        """Get the TIO voltage parameters.

        Returns
        ----------
        tuple
            TIO voltage parameters (vout_enable, vout_vol)
        """
        return self._get_tio_vout_param()
    def _get_tio_vout_param(self) -> tuple:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def get_robot_state(self) -> tuple:
        """Get the robot state.

        Returns
        -------
        tuple
            The robot state: (em_stop, power_on, servo_enabled).
        """
        return self._get_robot_state()
    def _get_robot_state(self) -> tuple:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def add_tio_rs_signal(self, sign_info: dict)->None:
        """Add or modify a TIO signal.

        Parameters
        ----------
        sign_info : dict
            Signal dictionary  
                - 'sig_name'  //signal name 
                - 'chn_id'    //RS485 chanel ID
                - 'sig_type'  //semaphore type
                - 'sig_addr'  //register address
                - 'value'     //value, invalid when setting 
                - 'frequency' //The refresh frequency of semaphore in the controller, maximum 10
        """
        return self._add_tio_rs_signal(sign_info)
    def _add_tio_rs_signal(self, sign_info: dict)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def del_tio_rs_signal(self, sign_name: str)->None:
        """Delete a TIO signal.

        Parameters
        ----------
        sign_name : str
            The signal's name.
        """
        return self._del_tio_rs_signal(sign_name)
    def _del_tio_rs_signal(self, sign_name: str)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def send_tio_rs_command(self, chn_id: int, cmd: str)->None:
        """Send a RS485 command.

        Parameters
        ----------
        chn_id : int
            The id of the TIO channel.
        cmd : str
            The command.
        """
        return self._send_tio_rs_command(chn_id, cmd)
    def _send_tio_rs_command(self, chn_id: int, cmd: str)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def get_rs485_signal_info(self) -> list:
        """Get information on TIO signals.

        Returns
        -------
        list
            TIO signals information [{'value': 0, 'chn_id': 0, 'sig_addr': 0, 'sig_name': '', 'sig_type': 0, 'frequency': 0}, ...] 
        """
        return self._get_rs485_signal_info()
    def _get_rs485_signal_info(self) -> list:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def set_tio_pin_mode(self, pin_type: PinType, pin_mode: int)->None:
        """Set TIO mode.

        Parameters
        ----------
        pint_type : PinType
            Either PinType.DIGITAL_INPUT, PinType.DIGITAL_OUTPUT or PinType.ANALOG_INPUT.
        pin_mode : int
            - DI Pins: 0:0x00 DI2 is NPN, DI1 is NPN, 1:0x01 DI2 is NPN, DI1 is PNP, 2:0x10 DI2 is PNP, DI1 is NPN, 3:0x11 DI2 is PNP, DI1 is PNP
            - DO Pins: The lower 8 bits of data and the upper 4 bits are configured as DO2, The lower four bits are DO1 configuration, 0x0 DO is NPN output, 0x1 DO is PNP output, 0x2 DO is push-pull output, 0xF RS485H interface
            - AI Pins: Analog input function is enabled, RS485L is disabled, 1:RS485L interface is enabled and analog input function is disabled.
        """
        return self._set_tio_pin_mode(pin_type, pin_mode)
    def _set_tio_pin_mode(self, pin_type: PinType, pin_mode: int)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def get_tio_pin_mode(self, pin_type: PinType)->None:
        """Get TIO mode.

        Parameters
        ----------
        pint_type : PinType
            Either PinType.DIGITAL_INPUT, PinType.DIGITAL_OUTPUT or PinType.ANALOG_INPUT.
        """
        return self._get_tio_pin_mode(pin_type)
    def _get_tio_pin_mode(self, pin_type: PinType)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def set_rs485_chn_comm(self, chn_id: int, slave_id: int, baudrate: BaudRate, 
                           databit: RS485DataBit, stopbit: RS485StopBit, parity: RS485Parity)->None:
        """Set TIO RS485 parameters.

        Parameters
        ----------
        chn_id : int
            Channel ID.
        slave_id : int
            Modbus slave node ID.
        baudrate : BaudRate
            Either BaudRate.B4800, BaudRate.B9600, BaudRate.B14400, BaudRate.B19200, BaudRate.B38400, BaudRate.B57600, BaudRate.B115200 or BaudRate.B230400.
        databit : RS485DataBit
            Either RS485DataBit.SEVEN or RS485DataBit.EIGHT.
        stopbit : RS485StopBit
            Either RS485StopBit.ONE or RS485StopBit.TWO.
        parity : RS485Parity
            Either RS485Parity.NOPARITY, RS485Parity.ODDPARITY or RS485Parity.EVENPARITY. 
        """
        return self._set_rs485_chn_comm(chn_id, slave_id, baudrate, databit, stopbit, parity)
    def _set_rs485_chn_comm(self, chn_id: int, slave_id: int, baudrate: BaudRate, 
                            databit: RS485DataBit, stopbit: RS485StopBit, parity: RS485Parity)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def get_rs485_chn_comm(self) -> tuple:
        """Get TIO RS485 parameters

        Returns
        -------
        tuple
            The TIO RS485 parameters (chn_id, slave_id,baudrate, databit, stopbit, parity).
        """
        return self._get_rs485_chn_comm()
    def _get_rs485_chn_comm(self) -> tuple:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def set_rs485_chn_mode(self, chn_id: RS485Channel, chn_mode: CommType)->None:
        """Set the RS485 communication mode.

        Parameters
        ----------
        chn_id : RS485Channel
            Either RS485Channel.RS485H or RS485Channel.RS485L.
        chn_mode : CommType
            Either CommType.MODBUS_RTU, CommType.RAW_RS485 or CommType.TORQUE_SENSOR.
        """
        return self._set_rs485_chn_mode(chn_id, chn_mode)
    def _set_rs485_chn_mode(self, chn_id: RS485Channel, chn_mode: CommType)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_rs485_chn_mode(self, chn_id: RS485Channel) -> int:
        """Get the RS485 communication mode.

        Parameters
        ----------
        chn_id : RS485Channel
            Either RS485Channel.RS485H or RS485Channel.RS485L.

        Returns
        -------
        CommType
            Either CommType.MODBUS_RTU, CommType.RAW_RS485 or CommType.TORQUE_SENSOR.
        """
        return self._get_rs485_chn_mode(chn_id)
    def _get_rs485_chn_mode(self, chn_id: RS485Channel) -> int:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def set_installation_angle(self, angle_x: float, angle_y: float)->None:
        """Set the robot mounting angle.

        Parameters
        ----------
        angle_x : float
            The mounting angle in the x-axis, [0-PI] rad.
        angle_y : float
            The mounting angle in the z-axis, [0-2PI] rad.
        """
        return self._set_installation_angle(angle_x, angle_y)
    def _set_installation_angle(self, angle_x: float, angle_y: float)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def get_installation_angle(self) -> tuple:
        """Get the robot mounting angle.

        Returns
        -------
        tuple
            The robot's mounting angle (qs, qx, qy, qz, rx, ry, rz).
        """
        return self._get_installation_angle()
    def _get_installation_angle(self) -> tuple:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def set_user_var(self, id: int, value: Any, name: str)->None:
        """Set an user variable.

        Parameters
        ----------
        id : int
            Id of the user variable.
        value : Any
            Value of the user variable.
        name : str
            String alias for the user variable.
        """
        return self._set_user_var(id, value, name)
    def _set_user_var(self, id: int, value: Any, name: str)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def get_user_var(self, id: int) -> Any:
        """Get an user variable.

        Parameters
        ----------
        id : int
            Id of the user variable.

        Returns
        -------
        Any
            The value of the user variable.
        """
        return self._get_user_var(id)
    def _get_user_var(self, id: int) -> Any:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    #########################################
    #                                       #
    # Robot safety status settings          #
    #                                       #
    #########################################

    @process_sdk_call(connected=True)
    def is_on_limit(self) -> bool:
        """Check if the robot reached joint limits.

        Returns
        -------
        bool
            Whether the robot reached joint limits.
        """
        return self._is_on_limit()
    def _is_on_limit(self) -> bool:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def is_in_collision(self) -> bool:
        """Check if the robot is in collision.

        Returns
        -------
        bool
            Whether a collision was detected.
        """
        return self._is_in_collision()
    def _is_in_collision(self) -> bool:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)        
    def collision_recover(self)->None:
        """Recover from a collision state.
        """
        return self._collision_recover()
    def _collision_recover(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def set_collision_level(self, level: CollisionLevel)->None:
        """Set the threshold for collision detection.

        Parameters
        ----------
        level : CollisionLevel
            The force threshold for collision detection.
        """
        return self._set_collision_level(level)
    def _set_collision_level(self, level: CollisionLevel)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_collision_level(self) -> CollisionLevel:
        """Get the threshold for collision detection.

        Returns
        -------
        CollisionLevel
            The force threshold for collision detection.
        """
        return self._get_collision_level()
    def _get_collision_level(self) -> CollisionLevel:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def set_network_exception_handle(self, delay: int, network_exception_handle: NetworkExceptionHandle)->None:
        """Set the allowed network communication delay, and the action to perform in case motion is happening after such a delay.

        Parameters
        ----------
        delay : int
            The delay, in milliseconds.
        network_exception_handle : NetworkExceptionHandle
            The action to perform if motion is happening, after the delay.
        """
        return self._set_network_exception_handle(delay, network_exception_handle)
    def _set_network_exception_handle(self, delay: int, network_exception_handle: NetworkExceptionHandle)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    #########################################
    #                                       #
    # App script program                    #
    #                                       #
    #########################################

    @process_sdk_call(connected=True)    
    def program_load(self, file_name: str)->None:
        """Load a program-

        Parameters
        ----------
        file_name : str
            The program's name.
        """
        return self._program_load(file_name)
    def _program_load(self, file_name: str)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def get_loaded_program(self) -> str:
        """Get the currently loaded program.

        Returns
        -------
        str
            The name of the currently loaded program.
        """ 
        return self._get_loaded_program()
    def _get_loaded_program(self) -> str:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def get_current_line(self) -> int:
        """Get the current program line.

        Returns
        -------
        int
            The number of the line.
        """
        return self._get_current_line()
    def _get_current_line(self) -> int:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def program_run(self)->None:
        """Run the loaded program"""
        return self._program_run()
    def _program_run(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")   
    
    @process_sdk_call(connected=True)    
    def program_pause(self)->None:
        """Pause the loaded program"""
        return self._program_pause()
    def _program_pause(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")        
    
    @process_sdk_call(connected=True)    
    def program_resume(self)->None:
        """Resume the loaded program"""
        return self._program_resume()
    def _program_resume(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")     
    
    @process_sdk_call(connected=True)    
    def program_abort(self)->None:
        """Abort the loaded program"""
        return self._program_abort()
    def _program_abort(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")        

    @process_sdk_call(connected=True)    
    def get_program_state(self) -> ProgramState:
        """Get the program state.

        Returns
        -------
        ProgramState
            The state of the program.
        """
        return self._get_program_state()
    def _get_program_state(self) -> ProgramState:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)    
    def set_rapidrate(self, rapid_rate: float)->None:
        """Set rapid rate.

        Parameters
        ----------
        rapid_rate : float
            Rapid rate.
        """
        return self._set_rapidrate(rapid_rate)
    def _set_rapidrate(self, rapid_rate: float)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_rapidrate(self) -> float:
        """Get rapid rate.

        Returns
        -------
        float
            Rapid rate.
        """
        return self._get_rapidrate()
    def _get_rapidrate(self) -> float:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    #########################################
    #                                       #
    # Trajectory reproduction               #
    #                                       #
    #########################################

    @process_sdk_call(connected=True)
    def set_traj_config(self, xyz_interval: list, rpy_interval: list, vel: float, acc: float)->None:
        """Set the trajectory track configuration parameters.

        Parameters
        ----------
        xyz_interval : list
            Space position acquisition speed.
        rpy_interval : list
            Orientation capture accuracy.
        vel : float
            Script execution running speed.
        acc : float
            Script execition running acceleration.
        """
        return self._set_traj_config(xyz_interval, rpy_interval, vel, acc)
    def _set_traj_config(self, xyz_interval: list, rpy_interval: list, vel: float, acc: float)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_traj_config(self) -> tuple:
        """Get the trajectory track configuration parameters.

        Returns
        -------
        tuple
            The parameters (xyz_interval, rpy_interval, vel, acc)
        """
        return self._get_traj_config()
    def _get_traj_config(self) -> tuple:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def set_traj_sample_mode(self, mode: bool, filename: str)->None:
        """Set the trajectory sample mode.

        Parameters
        ----------
        mode : bool
            True for starting data collection, False to stop data collection.
        filename : str
            Filename for data storage.
        """
        return self._set_traj_sample_mode(mode, filename)
    def _set_traj_sample_mode(self, mode: bool, filename: str)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_traj_sample_status(self) -> bool:
        """Get the status of trajectory sampling.

        Returns
        -------
        bool
            True when collecting data, False otherwise.
        """
        return self._get_traj_sample_status()
    def _get_traj_sample_status(self) -> bool:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")
  
    #########################################
    #                                       #
    # Robot kinematics                      #
    #                                       #
    #########################################

    @process_sdk_call(connected=True)
    def rpy_to_rot_matrix(self, rpy: list) -> list:
        """Convert RPY angles to a rotation matrix.

        Parameters
        ----------
        rpy : list
            The xyz roll, pitch and yaw.

        Returns
        -------
        list
            The corresponding ZYX rotation matrix
        """
        return self._rpy_to_rot_matrix(rpy)
    def _rpy_to_rot_matrix(self, rpy: list) -> list:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def rot_matrix_to_rpy(self, rot_matrix: list) -> list:
        """Convert a rotation matrix to RPY angles.

        Parameters
        ----------
        rot_matrix : list
            The ZYX rotation matrix.

        Returns
        -------
        list
            The corresponding xyz roll, pitch and yaw.
        """
        return self._rot_matrix_to_rpy(rot_matrix)
    def _rot_matrix_to_rpy(self, rot_matrix: list) -> list:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def quaternion_to_rot_matrix(self, quaternion: list) -> list:
        """Convert a quaternion to a rotation matrix.

        Parameters
        ----------
        quaternion : list
            The [w, x, y, z] quaternion.        

        Returns
        -------
        list
            The corresponding ZYX rotation matrix.
        """
        return self._quaternion_to_rot_matrix(quaternion)
    def _quaternion_to_rot_matrix(self, quaternion: list) -> list:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def kine_inverse(self, ref_pos: list, cartesian_pose: list) -> list:
        """Compute inverse kinematics.

        Parameters
        ----------
        ref_pos : list
            Reference joint position, in radians.
        cartesian_pose : list
            Target TCP pose, in millimeters and radians.

        Returns
        -------
        list
            The corresponding joint pose, if a solution exists.
        """
        return self._kine_inverse(ref_pos, cartesian_pose)
    def _kine_inverse(self, ref_pos: list, cartesian_pose: list) -> list:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def kine_forward(self, joint_pos: list) -> SE3:
        """Compute forward kinematics.

        Parameters
        ----------
        joint_pos : list
            Target joint position.

        Returns
        -------
        list
            The corresponding TCP pose.
        """
        return self._kine_forward(joint_pos)
    def _kine_forward(self, joint_pos: list) -> SE3:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def rot_matrix_to_quaternion(self, rot_matrix: list) -> list:
        """Convert a rotation matrix to a quaternion.

        Parameters
        ----------
        rot_matrix : list
            The ZYX rotation matrix.

        Returns
        -------
        list
            The corresponding [w, x, y, z] quaternion
        """
        return self._rot_matrix_to_quaternion(rot_matrix)
    def _rot_matrix_to_quaternion(self, rot_matrix: list) -> list:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    #########################################
    #                                       #
    # Robot servo mode                      #
    #                                       #
    #########################################

    @process_sdk_call(enabled=True, servo_enabled=True)
    def servo_j(self, joint_pos: list, move_mode: MoveMode, step_num: int=1)->None:
        """Servo move in joint space.

        Parameters
        ----------
        joint_pos : list
            Target joint position.
        move_mode : MoveMode
            Movement mode.
        step_num : int
            Step number.
        """
        return self._servo_j(joint_pos, move_mode, step_num)
    def _servo_j(self, joint_pos: list, move_mode: MoveMode, step_num: int=1)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(enabled=True, servo_enabled=True)
    def servo_p(self, cartesian_pose: list, move_mode: MoveMode, step_num: int=1)->None:
        """Servo move in Cartesian space.

        Parameters
        ----------
        cartesian_pose : list
            Target Cartesian position.
        move_mode : MoveMode
            Movement mode.
        step_num : int
            Step number.
        """
        return self._servo_p(cartesian_pose, move_mode, step_num)
    def _servo_p(self, cartesian_pose: list, move_mode: MoveMode, step_num: int=1)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True, servo_disabled=True)
    def servo_move_use_none_filter(self)->None:
        """Disable all filters.
        """
        return self._servo_move_use_none_filter()
    def _servo_move_use_none_filter(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True, servo_disabled=True)
    def servo_move_use_joint_LPF(self, cutoffFreq: float)->None:
        """Apply joint space first-order low-pass filter.

        Parameters
        ----------
        cutoffFreq : float
            Cutoff frequency for the filter.
        """
        return self._servo_move_use_joint_LPF(cutoffFreq)
    def _servo_move_use_joint_LPF(self, cutoffFreq: float)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True, servo_disabled=True)
    def servo_move_use_joint_NLF(self, max_vr: float, max_ar: float, max_jr: float)->None:
        """Apply joint space non-linear filter.

        Parameters
        ----------
        max_vr : float
            Maximum absolute value for angular velocity, in rad/s.
        max_ar : float
            Maximum absolute value for angular acceleration, in rad/s^2
        max_jr : float
            Maximum absolute value for angular jerk, in rad/s^3
        """
        return self._servo_move_use_joint_NLF(max_vr, max_ar, max_jr)
    def _servo_move_use_joint_NLF(self, max_vr: float, max_ar: float, max_jr: float)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True, servo_disabled=True)
    def servo_move_use_carte_NLF(self, max_vp: float, max_ap: float, max_jp: float, 
                                       max_vr: float, max_ar: float, max_jr: float)->None:
        """Apply Cartesian space non-linear filter.

        Parameters
        ----------
        max_vp : float
            Maximum absolute value for linear velocity, in mm/s.
        max_ap : float
            Maximum absolute value for linear acceleration, in mm/s^2
        max_jp : float
            Maximum absolute value for linear jerk, in mm/s^3
        max_vp : float
            Maximum absolute value for angular velocity, in rad/s.
        max_ap : float
            Maximum absolute value for angular acceleration, in rad/s^2
        max_jp : float
            Maximum absolute value for angular jerk, in rad/s^3
        """
        return self._servo_move_use_carte_NLF(max_vp, max_ap, max_jp, max_vr, max_ar, max_jr)
    def _servo_move_use_carte_NLF(self, max_vp: float, max_ap: float, max_jp: float, 
                                        max_vr: float, max_ar: float, max_jr: float)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")
    
    @process_sdk_call(connected=True, servo_disabled=True)
    def _servo_move_use_joint_MMF(self, max_buf: int, kp: float, kv: float, ka: float)->None:
        """Apply joint space multi-order mean filter.

        Parameters
        ----------
        max_buf : int
            Size of the buffer for the mean filter.
        kp : float
            Acceleration filter factor.
        kv : float
            Velocity filter factor.
        ka : float
            Position filter factor.
        """
        return self._servo_move_use_joint_MMF(max_buf, kp, kv, ka)
    def _servo_move_use_joint_MMF(self, max_buf: int, kp: float, kv: float, ka: float)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True, servo_disabled=True)
    def servo_speed_foresight(self, max_buf: int, kp: float)->None:
        """Set speed foresight parameters.

        Parameters
        ----------
        max_buf : int
            Size of the buffer for the mean filter.
        kp : float
            Acceleration filter factor.
        """
        return self._servo_speed_foresight(max_buf, kp)
    def _servo_speed_foresight(self, max_buf: int, kp: float)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    #########################################
    #                                       #
    # Force control functions               #
    #                                       #
    #########################################

    @process_sdk_call(connected=True)
    def set_torsensor_brand(self, sensor_brand: int)->None:
        """Set the torque sensor brand.

        Parameters
        ----------
        sensor_brand : int
            ID of the sensor [1-6]. 10 is reserved for the robot's internal sensor.
        """
        return self._set_torsensor_brand(sensor_brand)
    def _set_torsensor_brand(self, sensor_brand: int)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_sensor_brand(self)->int:
        """Get the current sensor id.

        Returns
        -------
        int
            Sensor brand [1-6]. 10 is reserved for the robot's internal sensor.
        """
        return self._get_sensor_brand()
    def _get_sensor_brand(self)->int:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(enabled=True)
    def start_torq_sensor_payload_identify(self, joint_pos: list)->None:
        """Start the payload identification procedure.

        Parameters
        ----------
        joint_pos : list
            Target joint position for the procedure. Refer to the User Manual for Force Control Products.
        """
        return self._start_torq_sensor_payload_identify(joint_pos)
    def _start_torq_sensor_payload_identify(self, joint_pos: list)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")
    
    @process_sdk_call(connected=True)
    def get_torq_sensor_identify_status(self)->IdentificationStatus:
        """Get the status of the payload identification process.

        Returns
        -------
        IdentificationResult
            The status of the procedure.
        """
        return self._get_torq_sensor_identify_status()
    def _get_torq_sensor_identify_status(self)->IdentificationStatus:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_torq_sensor_payload_identify_result(self)->tuple:
        """Get the result of the payload identification procedure.

        Returns
        -------
        tuple
            The payload (m, [x,y,z]).
        """
        return self._get_torq_sensor_payload_identify_result()
    def _get_torq_sensor_payload_identify_result(self)->tuple:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def set_torq_sensor_tool_payload(self, mass: float, centroid: list)->None:
        """Set the payload information.

        Parameters
        ----------
        mass : float
            Mass of the payload, in kilograms.
        centroid : list
            Center of mass of the payload, in millimeters.
        """
        return self._set_torq_sensor_tool_payload(mass, centroid)
    def _set_torq_sensor_tool_payload(self, mass: float, centroid: list)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} imass, centroids not implemented")

    @process_sdk_call(connected=True)
    def get_torq_sensor_tool_payload(self)->tuple:
        """Get the payload at the TCP.

        Returns
        -------
        tuple
            The payload (m, [x,y,z]).
        """
        return self._get_torq_sensor_tool_payload()
    def _get_torq_sensor_tool_payload(self)->tuple:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    @deprecated
    def set_compliant_type(self, sensor_compensation: SensorCompensation, compliance_type: ComplianceType)->None:
        """Set the force control settings.

        Parameters
        ----------
        sensor_compensation : SensorCompensation
            Compensation mode for the sensor.
        compliance_type : ComplianceType
            Force compliance type.
        """
        return self._set_compliant_type(sensor_compensation, compliance_type)
    def _set_compliant_type(self, sensor_compensation: SensorCompensation, compliance_type: ComplianceType)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    @deprecated
    def get_compliant_type(self)->tuple:
        """Get the force control settings.

        Returns
        -------
        tuple
            The force control settings (sensor_compensation, compliance_type).
        """
        return self._get_compliant_type()
    def _get_compliant_type(self)->tuple:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    @deprecated
    def set_ft_ctrl_frame(self, ftFrame: FTFrame)->None:
        """Set the force control frame.

        Parameters
        ----------
        ftFrame : FTFrame
            The target frame.
        """
        return self._set_ft_ctrl_frame(ftFrame)
    def _set_ft_ctrl_frame(self, ftFrame: FTFrame)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    @deprecated
    def get_ft_ctrl_frame(self)->FTFrame:
        """Get the force control frame.

        Returns
        -------
        FTFrame
            The target frame.
        """
        return self._get_ft_ctrl_frame()
    def _get_ft_ctrl_frame(self)->FTFrame:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    @deprecated
    def set_admit_ctrl_config(self, axis: int, opt: bool, ftUser: float, 
                              ftReboundFK: float, ftConstant: float, ftNormalTrack: float)->None:
        """Set the compliance control parameters.

        Parameters
        ----------
        axis : int
            Cartesian space [0-5] axis.
        opt : bool
            Whether to turn on compliance control.
        ftUser : float
            The force the user needs to apply to move the robot in the given direction at the maximum speed.
        ftReboundFK : float
            The rebound force that is applied once the user stops applying force.
        ftConstant : float
            The constant force reference.
        ftNormalTrack : float
            The normal tracking force.
        """
        return self._set_admit_ctrl_config(axis, opt, ftUser, ftReboundFK, ftConstant, ftNormalTrack)
    def _set_admit_ctrl_config(self, axis: int, opt: bool, ftUser: float, 
                               ftReboundFK: float, ftConstant: float, ftNormalTrack: float)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    @deprecated
    def get_admit_ctrl_config(self)->tuple:
        """Get the compliance control parameters.

        Returns
        -------
        tuple
            The control parameters for each axis (opt, ftUser, ftReboundFK, ftConstant, ftNormalTrack).
        """
        return self._get_admit_ctrl_config()
    def _get_admit_ctrl_config(self)->tuple:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def set_torque_sensor_comm(self, type: int, ip_addr: str, port: int)->None:
        """Set the torque sensor's communication parameter.

        Parameters
        ----------
        type : int
            Sensor type [1-6].
        ip_addr : str
            Sensor IP address.
        port : int
            Sensor port.
        """
        return self._set_torque_sensor_comm(type, ip_addr, port)
    def _set_torque_sensor_comm(self, type: int, ip_addr: str, port: int)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_torque_sensor_comm(self)->tuple:
        """Get the torque sensor's communication parameters.

        Returns
        -------
        tuple
            The sensor's communication parameters (type, ip_addr, port).
        """
        return self._get_torque_sensor_comm()
    def _get_torque_sensor_comm(self)->tuple:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    @deprecated
    def disable_force_control(self)->None:
        """Turn off force control.
        """
        return self._disable_force_control()
    def _disable_force_control(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    @deprecated
    def set_vel_compliant_ctrl(self, level: int, rate1: float, rate2: float, rate3: float, rate4: float)->None:
        """Set the compliance control speed parameters.

        Parameters
        ----------
        level : int
            The compliance control level.
        rate1 : float
            Rate 1.
        rate2 : float
            Rate 2.
        rate3 : float
            Rate 3.
        rate4 : float
            Rate 4.
        """
        return self._set_vel_compliant_ctrl(level, rate1, rate2, rate3, rate4)
    def _set_vel_compliant_ctrl(self, level: int, rate1: float, rate2: float, rate3: float, rate4: float)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    @deprecated
    def set_compliance_condition(self, fx: float, fy: float, fz: float,
                                       tx: float, ty: float, tz: float)->None:
        """Set the compliance control torque conditions.

        Parameters
        ----------
        fx : float
            Force along x axis, in N.
        fy : float
            Force along y axis, in N.
        fz : float
            Force along z axis, in N.
        tx : float
            Torque around x axis, in Nm.
        ty : float
            Torque around y axis, in Nm.
        tz : float
            Torque around z axis, in Nm.
        """
        return self._set_compliance_condition(fx, fy, fz, tx, ty, tz)
    def _set_compliance_condition(self, fx: float, fy: float, fz: float,
                                       tx: float, ty: float, tz: float)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def set_torque_sensor_filter(self, torque_sensor_filter: float)->None:
        """Set the torque sensor low-pass filter cutoff frequency.

        Parameters
        ----------
        torque_sensor_filter : float
            Filter cutoff frequency, in Hz.
        """
        return self._set_torque_sensor_filter(torque_sensor_filter)
    def _set_torque_sensor_filter(self, torque_sensor_filter: float)->None:

        raise NotImplementedError(f"{inspect.currentframe().f_code.co_nametorque_sensor_filter} is not implemented")

    @process_sdk_call(connected=True)
    @deprecated
    def set_torque_sensor_soft_limit(self, fx: float, fy: float, fz: float,
                                       tx: float, ty: float, tz: float)->None:
        """Set the soft limit of the sensor.

        Parameters
        ----------
        fx : float
            Force along x axis, in N.
        fy : float
            Force along y axis, in N.
        fz : float
            Force along z axis, in N.
        tx : float
            Torque around x axis, in Nm.
        ty : float
            Torque around y axis, in Nm.
        tz : float
            Torque around z axis, in Nm.
        """
        return self._set_torque_sensor_soft_limit(fx, fy, fz, tx, ty, tz)
    def _set_torque_sensor_soft_limit(self, fx: float, fy: float, fz: float,
                                       tx: float, ty: float, tz: float)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")    

    @process_sdk_call(connected=True)
    @deprecated
    def get_torque_sensor_soft_limit(self)->tuple:
        """Get the torque sensor's soft limit.

        Returns
        -------
        tuple
            The sensor's soft limit (fx, fy, fz, tx, ty, tz).
        """
        return self._get_torque_sensor_soft_limit()
    def _get_torque_sensor_soft_limit(self)->tuple:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    #########################################
    #                                       #
    # FTP services                          #
    #                                       #
    #########################################

    @process_sdk_call(connected=True)
    def init_ftp_client(self)->None:
        """Connect to the cabinet via FTP.
        """
        return self._init_ftp_client()
    def _init_ftp_client(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def close_ftp_client(self)->None:
        """Close the connection to the cabinet.
        """
        return self._close_ftp_client()
    def _close_ftp_client(self)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def get_ftp_dir(self, remotedir: str, type: FTPType)->str:
        """Get the contents of an FTP directory.

        Parameters
        ----------
        remotedir : str
            The internal folder name.   
        type : FTPType
            Data type.
        """
        return self._get_ftp_dir(remotedir, type)
    def _get_ftp_dir(self, remotedir: str, type: FTPType)->str:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def download_file(self, local: str, remote: str, opt: FTPOption)->None:
        """Download a file or directory from a remote FTP directory.

        Parameters
        ----------
        local : str
            Absolute path to download the file to.
        remote : str
            Absolute path of the remote file or directory to download.
        opt : FTPOption
            Data type.
        """
        return self._download_file(local, remote, opt)
    def _download_file(self, local: str, remote: str, opt: FTPOption)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")

    @process_sdk_call(connected=True)
    def upload_file(self, local: str, remote: str, opt: FTPOption)->None:
        """Upload a file or directory to a remote FTP directory.

        Parameters
        ----------
        local : str
            Absolute path of the file or directory to upload.
        remote : str
            Absolute path to upload the file to.
        opt : FTPOption
            Data type.
        """
        return self._upload_file(local, remote, opt)
    def _upload_file(self, local: str, remote: str, opt: FTPOption)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")
    
    @process_sdk_call(connected=True)
    def rename_ftp_file(self, remote: str, des: str, opt: FTPOption)->None:
        """Rename a remote file or directory FTP directory.

        Parameters
        ----------
        remote : str
            Absolute path of the file or directory to rename.
        des : str
            The name to rename the file or directory to.
        opt : FTPOption
            Data type.
        """
        return self._rename_ftp_file(remote, des, opt)
    def _rename_ftp_file(self, remote: str, des: str, opt: FTPOption)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")
    
    @process_sdk_call(connected=True)
    def del_ftp_file(self, remote: str, opt: FTPOption)->None:
        """Delete a remote file or directory FTP directory.

        Parameters
        ----------
        remote : str
            Absolute path of the file or directory to delete.
        opt : FTPOption
            Data type.
        """
        return self._del_ftp_file(remote, opt)
    def _del_ftp_file(self, remote: str, opt: FTPOption)->None:
        raise NotImplementedError(f"{inspect.currentframe().f_code.co_name} is not implemented")
    
class RealRobot(BaseRobot):
    
    def __init__(self, 
                 ip: str='10.5.5.100',
                 gripper_power_pin: int = 1,
                 gripper_control_pin: int = 0,
                 gripper_timeout: float = 0.5,
                 use_jaka_kinematics: bool=False,
                 urdf_package: str='jaka_description',
                 urdf_name: str='jaka.urdf'):
        # Initialize the robot
        self.ip = ip
        self.robot = jkrc.RC(self.ip)

        # Gripper DOUT pins
        self.gripper_power_pin = gripper_power_pin
        self.gripper_control_pin = gripper_control_pin
        # How long to wait for the gripper to open/close
        self.gripper_timeout = gripper_timeout

        self.use_jaka_kinematics = use_jaka_kinematics
        if not self.use_jaka_kinematics:
            # We won't use JAKA's kinematics functions
            package_share_path = get_package_share_directory(urdf_package)
            urdf_path = os.path.join(package_share_path, 'urdf', urdf_name)
            self.chain = rtb.ERobot.URDF(urdf_path)

        super().__init__()

    #########################################
    #                                       #
    # Robot initialization                  #
    #                                       #
    #########################################

    def _enable_robot(self)->None:
        """Enables the robot.
        """
        return self.robot.enable_robot()

    def _disable_robot(self)->None:
        """Disables the robot.
        """
        return self.robot.disable_robot()

    def _power_on(self)->None:
        """Powers on the robot.
        """
        return self.robot.power_on()

    def _power_off(self)->None:
        """Powers off the robot.
        """
        return self.robot.power_off()

    def _login(self)->None:
        """Connects to the robot.
        """
        # Try to ping the robot to see if it is alive
        cmd = f"ping -c 1 {self.ip} > /dev/null 2>&1"
        if os.system(cmd) != 0:
            return (JAKA_ERR_CODES.ERR_COMMUNICATION_ERR.value,)
        return self.robot.login()

    def _logout(self)->None:
        """Disconnects from the robot.
        """
        return self.robot.logout()
    
    def _enable_servo_mode(self)->None:
        """Enable servo mode.
        """
        return self.robot.servo_move_enable(True)
    
    def _disable_servo_mode(self)->None:
        """Disable servo mode.
        """
        return self.robot.servo_move_enable(False)

    def _servo_move_enable(self, value: bool)->None:
        """For consistency with JAKA's SDK. This function just wraps enable_servo_mode and disable_servo_mode.

        Parameters
        ----------
        value : bool
            Whether to enable servo mode.
        """
        return self.enable_servo_mode() if value else self.disable_servo_mode()

    def _shutdown_cabinet(self)->None:
        """Shuts down the robot's cabinet.
        """
        return self.robot.shut_down()
    
    #########################################
    #                                       #
    # Robot motion                          #
    #                                       #
    #########################################

    def _jog(self, aj_num: int, move_mode: MoveMode, coord_type: CoordType, jog_vel: float, pos_cmd: float)->None:
        """Jog the robot, either in joint or Cartesian space. Note that jog will fail if the robot is close to a 
        singularity. Also keep in mind that jog is a non-blocking function. Further commands received during motion will 
        be discarded.

        Parameters
        ----------
        aj_num : int
            The [0-5] index of the joint or Cartesian coordinate to jog.
        move_mode : MoveMode
            Movement mode. Either MoveMode.ABSOLUTE, MoveMode.INCREMENTAL or MoveMode.CONTINUOUS.
        coord_type : CoordType
            Coordinate frame. Either CoordType.BASE, CoordType.JOINT or CoordType.TOOL.
        jog_vel : float
            Jog velocity, expressed in rad/s for joint space motion or mm/s for Cartesian space motion.
        pos_cmd : float
            The target value for the jog, in rad for joint space motion or mm for Cartesian space motion.
        """
        return self.robot.jog(aj_num, move_mode.value, coord_type.value, jog_vel, pos_cmd)

    def _jog_stop(self, aj_num: int)->None:
        """_summary_

        Parameters
        ----------
        aj_num : int
            The [0-5] index of the joint or Cartesian coordinate to jog, or -1 to stop jogging across all axes.
        """
        return self.robot.jog_stop(aj_num)

    def _joint_move(self, joint_pos: list, move_mode: MoveMode, speed: float, is_blocking: bool)->None: 
        """Move the robot linearly in joint space.

        Parameters
        ----------
        joint_pos : list
            The target joint position, in rad.
        move_mode : MoveMode
            Movement mode. Either MoveMode.ABSOLUTE, MoveMode.INCREMENTAL or MoveMode.CONTINUOUS
        speed : float
            The speed of the motion.
        is_blocking : bool
            Whether or not the SDK function call is blocking.
        """
        return self.robot.joint_move(joint_pos, move_mode.value, is_blocking, speed)

    def _joint_move_extend(self, joint_pos: list, move_mode: MoveMode, speed: float, acc: float, tol: float, is_blocking: bool)->None: 
        """Move the robot linearly in joint space. Extended to allow for acceleration specification as well as target position tolerance.

        Parameters
        ----------
        joint_pos : list
            The target joint position, in rad.
        move_mode : MoveMode
            Movement mode. Either MoveMode.ABSOLUTE, MoveMode.INCREMENTAL or MoveMode.CONTINUOUS
        speed : float
            The speed of the motion, in rad/s.
        acc : float
            The acceleration of the motion, in rad/s^2.
        tol : float
            The endpoint error.
        is_blocking : bool
            Whether or not the SDK function call is blocking.
        """
        return self.robot.joint_move_extend(joint_pos, move_mode.value, is_blocking, speed, acc, tol)

    def _linear_move(self, tcp_pos: list, move_mode: MoveMode, speed: float, is_blocking: bool):
        """Move the robot linearly in Cartesian space. 

        Parameters
        ----------
        end_pos : list
            The target TCP position, in mm.
        move_mode : MoveMode
            Movement mode. Either MoveMode.ABSOLUTE, MoveMode.INCREMENTAL or MoveMode.CONTINUOUS
        speed : float
            The speed of the motion, in mm/s.
        is_blocking : bool
            Whether or not the SDK function call is blocking.
        """
        return self.robot.linear_move(tcp_pos, move_mode.value, is_blocking, speed)
    
    def _linear_move_extend(self, tcp_pos: list, move_mode: MoveMode, speed: float, acc: float, tol: float, is_blocking: bool)->None:
        """Move the robot linearly in Cartesian space. Extended to allow for acceleration specification as well as target position tolerance.

        Parameters
        ----------
        tcp_pos : list
            The target TCP position, in mm.
        move_mode : MoveMode
            Movement mode. Either MoveMode.ABSOLUTE, MoveMode.INCREMENTAL or MoveMode.CONTINUOUS
        speed : float
            The speed of the motion, in mm/s.
        acc : float
            The acceleration of the motion, in mm/s^2.
        tol : float
            The endpoint error.
        is_blocking : bool
            Whether or not the SDK function call is blocking.
        """
        return self.robot.linear_move_extend(tcp_pos, move_mode.value, is_blocking, speed, acc, tol)

    @untested
    def _circular_move(self, tcp_pos: list, midpoint: list, move_mode: MoveMode, speed: float, acc: float, tol: float, is_blocking: bool)->None:
        """Move the robot in an arc in Cartesian space.

        Parameters
        ----------
        tcp_pos : list
            The target TCP position, in mm.
        midpoint : list
            Midpoint of the TCP along the arc, in mm.
        move_mode : MoveMode
            Movement mode. Either MoveMode.ABSOLUTE, MoveMode.INCREMENTAL or MoveMode.CONTINUOUS
        speed : float
            The speed of the motion, in mm/s.
        acc : float
            The acceleration of the motion, in mm/s^2.
        tol : float
            The endpoint error.
        is_blocking : bool
            Whether or not the SDK function call is blocking.
        """
        return self.robot.circular_move(tcp_pos, midpoint, move_mode.value, is_blocking, speed, acc, tol)

    @untested
    def _circular_move_extend(self, tcp_pos: list, midpoint: list, move_mode: MoveMode, speed: float, acc: float, 
                  tol: float, is_blocking: bool, circle_cnt: int)->None:
        """Move the robot in an arc in Cartesian space. Adds a `circle_cnt` parameter of unknown meaning.

        Parameters
        ----------
        tcp_pos : list
            The target TCP position, in mm.
        midpoint : list
            Midpoint of the TCP along the arc, in mm.
        move_mode : MoveMode
            Movement mode. Either MoveMode.ABSOLUTE, MoveMode.INCREMENTAL or MoveMode.CONTINUOUS
        speed : float
            The speed of the motion, in mm/s.
        acc : float
            The acceleration of the motion, in mm/s^2.
        tol : float
            The endpoint error.
        is_blocking : bool
            Whether or not the SDK function call is blocking.
        circle_cnt : int
            Per the SDK manual: 'circle number of MoveC'.
        """
        # Last argument is a placeholder 'opt_cond', which the SDK suggests to set to None
        return self.robot.circular_move_extend(tcp_pos, midpoint, move_mode.value, is_blocking, 
                                               speed, acc, tol, circle_cnt, None)        

    @untested
    def _get_motion_status(self)->list:
        """Get the robot's motion status.

        Returns
        -------
        list
            The robot motion status, containing in order:
            - int: the current motion command id
            - int: reserved
            - bool: whether the previous motion command reached target position
            - bool: whether the previous motion command was dropped by the controller
            - int: index of the motion command in the buffer
            - int: index of the motion command in the blending buffer
            - bool: whether the motion queue is full
            - bool: whether the motion command is paused
        """
        return self.robot.get_motion_status()

    def _open_gripper(self)->None:
        """Open the gripper by turning off the corresponding digital outputs. This function is blocking for self.gripper_timeout seconds.
        """
        ret = self.robot.set_digital_output(IOType.CABINET.value, self.gripper_control_pin, 0)
        time.sleep(self.gripper_timeout)
        return ret

    def _close_gripper(self)->None:
        """Close the gripper by turning on the control digital output. This function is blocking for self.gripper_timeout seconds.
        """
        ret = self.robot.set_digital_output(IOType.CABINET.value, self.gripper_control_pin, 1)
        time.sleep(self.gripper_timeout)
        return ret
    
    def _power_off_gripper(self)->None:
        """Remove power to the gripper.
        """
        return self.robot.set_digital_output(IOType.CABINET.value, self.gripper_power_pin, 0)

    def _power_on_gripper(self)->None:
        """Provide power to the gripper.
        """
        return self.robot.set_digital_output(IOType.CABINET.value, self.gripper_power_pin, 1)

    #########################################
    #                                       #
    # Robot status                          #
    #                                       #
    #########################################

    def _update_status(self)->None:
        """Update the state variables of the robot.
        """
        robot_status = self.get_robot_status()
        self.state.is_connected = robot_status[RobotStatus.IS_SOCKET_CONNECT.value]
        self.state.is_powered_on = robot_status[RobotStatus.POWERED_ON.value]
        self.state.is_enabled = robot_status[RobotStatus.ENABLED.value]
        self.state.is_in_drag_mode = robot_status[RobotStatus.DRAG_STATUS.value]
        self.state.is_em_stop_pressed = robot_status[RobotStatus.PROTECTIVE_STOP.value]
        dout = robot_status[RobotStatus.DOUT.value]        
        self.state.is_gripper_closed = dout[self.gripper_power_pin] and dout[self.gripper_control_pin]
        self._joint_position = list(robot_status[RobotStatus.JOINT_POSITION.value])
        self._tcp_position = list(robot_status[RobotStatus.CART_POSITION.value])
        robot_state = self.get_robot_state()
        self.state.is_in_servo_mode = robot_state[RobotStatus.SERVO_ENABLED.value]

    @untested
    def _get_tool_data(self, id: int)->tuple:
        """Get data of a tool.

        Parameters
        ----------
        id : int
            The [1-10] id of the tool. 0 is reserved for the end flange.

        Returns
        -------
        tuple
            The (frame_id, [x, y, z, r, p, y]) data of the tool.
        """
        return self.robot.get_tool_data(id)
    
    @untested
    def _set_tool_data(self, id: int, tool: list, name: str)->None:
        """Set a tool data.

        Parameters
        ----------
        id : int
            The [1-10] id of the tool. 0 is reserved for the end flange.
        tool : list
            The [x, y, z, r, p, y] offset of the tool.
        name : str
            String alias for the tool.
        """
        return self.robot.set_tool_data(id, tool, name)
    
    @untested
    def _set_tool_id(self, id: int)->None:
        """Set the id of the tool in use. 0 represents the end flange.

        Parameters
        -------
        int
            The id of the tool.
        """
        return self.robot.set_tool_id(id)

    @untested
    def _get_tool_id(self)->int:
        """Get the id of the tool in use. 0 represents the end flange.

        Returns
        -------
        int
            The id of the frame.
        """
        return self.robot.get_tool_id()

    @untested
    def _set_user_frame_id(self, id: int)->None:
        """Set the id of the user frame to use.

        Parameters
        ----------
        int
            The [1-10] id of the user frame to use.
        """
        return self.robot.set_user_frame_id(id)
    
    @untested
    def _get_user_frame_id(self)->int:
        """Get the id of the user frame in use.

        Returns
        -------
        int
            The id of the frame.
        """
        return self.robot.get_user_frame_id()
    
    @untested
    def _get_user_frame_data(self, id: int)->tuple:
        """Get data of a user frame.

        Parameters
        ----------
        id : int
            The [1-10] id of the user frame. 0 is reserved for the robot frame.

        Returns
        -------
        tuple
            The (id, [x, y, z, r, p, y]) data of the frame.
        """
        return self.robot.get_user_frame_data(id)
    
    @untested
    def _set_user_frame_data(self, id: int, user_frame: list, name: str)->None:
        """Set a user frame.

        Parameters
        ----------
        id : int
            The [1-10] id of the user frame. 0 is reserved for the robot frame.
        user_frame : list
            The [x, y, z, r, p, y] offset of the frame.
        name : str
            String alias for the user frame.
        """
        return self.robot.set_user_frame_data(id, user_frame, name)

    def _get_tcp_position(self)->list:
        """Get the current position of the TCP.

        Returns
        -------
        list
            The current position of the TCP.
        """
        return self.robot.get_tcp_position()
    
    def _get_dh_params(self):
        """Get the robot's Denavit-Hartenberg params.

        Returns
        -------
        dict
            The dict of params.
        """
        return self.robot.get_dh_params()

    def _get_joint_position(self)->list:
        """Get the current joint position of the robot.

        Returns
        -------
        list
            The current joint position of the robot.
        """
        return self.robot.get_joint_position()
    
    def _get_robot_state(self)->tuple:
        """Get the robot state.

        Returns
        -------
        tuple
            The robot state: (em_stop, power_on, servo_enabled).
        """
        return self.robot.get_robot_state()

    def _get_robot_status(self)->tuple:
        """Get the full robot status.

        Returns
        -------
        tuple
            The full robot status, containing the following values:
            - error_code
            - inpos: whether the robot is in position
            - powered_on: whether the robot is powered on
            - enabled: whether the robot is enabled
            - rapidrate: robot speed rate
            - protective_stop: whether the robot detects a collision
            - drag_status: whether the robot is in drag mode
            - on_soft_limit: whether the robot is on limit
            - current_user_id: the current user coordinate frame ID used by the robot
            - current_tool_id: the current tool coordinate frame ID used by the robot
            - dout: control cabinet digital output signal
            - din: control cabinet digital input signal
            - aout: control cabinet analog output signal
            - ain: control cabinet analog input signal
            - tio_dout: end tool digital output signal
            - tio_din: end tool digital input signal
            - tio_ain: end tool analog input signal
            - extio: external extension IO module signal
            - cart_position: TCP cartesian position
            - joint_position: robot joint space position
            - robot_monitor_data: (scb major version, scb minor version, controller temperature, robot average voltage, robot average current, monitor data of 6 robot joints (transient current, voltage and temperature))
            - torq_sensor_monitor_data: (torque sensor IP addresses and port numbers, tool payload (payload mass, centroid x-axis, y-axis and z-axis coordinates), torque sensor status and exception error code, actual contact force values of 6 torque sensors and original reading values of 6 torque sensors)
            - is_socket_connect: whether the connection channel of SDK and controller is normal
            - emergency_stop: whether the robot is emergency stopped
            - tio_key: Robot end tool button [0] free; [1] point; [2] end light button
        """
        return self.robot.get_robot_status()

    def _get_robot_status_simple(self)->tuple:
        """Get the simplified robot status.

        Returns
        -------
        list
            The simplified robot status: (error_code, error_message, powered_on, enabled).
        """
        return self.robot.get_robot_status_simple()

    @untested
    def _set_SDK_log_filepath(self, path: str)->None:
        """Sets the path for the SDK's log.

        Parameters
        ----------
        path
            The path to save the log in.
        """
        return self.robot.set_SDK_filepath(path)

    def _is_in_drag_mode(self)->bool:
        """Checks whether the robot is currently in drag mode.

        Returns
        -------
        bool
            True if the robot is in drag mode.
        """
        return self.robot.is_in_drag_mode()

    def _enable_drag_mode(self)->None:
        """Enable drag mode.
        """
        return self.robot.drag_mode_enable(True)
    
    def _disable_drag_mode(self)->None:
        """Disable drag mode.
        """
        return self.robot.drag_mode_enable(False)
    
    @untested
    def _get_controller_ip(self)->str:
        """Get the IP address of the controller.

        Returns
        -------
        str
            The IP address of the controller.
        """
        return self.robot.get_controller_ip()
    
    @untested
    def _get_sdk_version(self)->str:
        """Get the SDK version.

        Returns
        -------
        str
            The SDK version.
        """
        return self.robot.get_sdk_version()
    
    @untested
    def _is_extio_running(self)->bool:
        """Check if the IO extension is running.

        Returns
        -------
        bool
            Whether the IO extension is running.
        """
        return self.robot.is_extio_running()
        
    @untested
    def _get_payload(self)->tuple:
        """Get the payload at the TCP.

        Returns
        -------
        tuple
            The payload (m, [x,y,z]) at the TCP.
        """
        return self.robot.get_payload()
    
    @untested
    def _set_payload(self, payload: tuple)->None:
        """Set the payload at the TCP.

        Parameters
        ----------
        payload : tuple
            The payload (m, [x,y,z]) at the TCP.
        """
        return self.robot.set_payload(payload[0], payload[1:])
    
    @untested
    def _set_installation_angle(self, angle_x: float, angle_y: float)->None:
        """Set the robot mounting angle.

        Parameters
        ----------
        angle_x : float
            The mounting angle in the x-axis, [0-PI] rad.
        angle_y : float
            The mounting angle in the z-axis, [0-2PI] rad.
        """
        return self.robot.set_installation_angle(angle_x, angle_y)
    
    @untested
    def _get_installation_angle(self)->tuple:
        """Get the robot mounting angle.

        Returns
        -------
        tuple
            The robot's mounting angle (qs, qx, qy, qz, rx, ry, rz).
        """
        return self.robot.get_installation_angle()
    
    @untested
    def _set_user_var(self, id: int, value: Any, name: str)->None:
        """Set an user variable.

        Parameters
        ----------
        id : int
            Id of the user variable.
        value : Any
            Value of the user variable.
        name : str
            String alias for the user variable.
        """
        return self.robot.set_user_var(id, value, name)
    
    @untested
    def _get_user_var(self, id: int)->Any:
        """Get an user variable.

        Parameters
        ----------
        id : int
            Id of the user variable.

        Returns
        -------
        Any
            The value of the user variable.
        """
        return self.robot.get_user_var(id)

    #########################################
    #                                       #
    # Robot IO                              #
    #                                       #
    #########################################

    @untested
    def _get_digital_input(self, index: int, iotype: IOType=IOType.CABINET)->bool:
        """Read the state of the index-th (0 based) digital input.

        Parameters
        ----------
        index : int
            The 0-based index of the digital input port.  
        iotype : IOType, optional
            The IO type. Either IOType.CABINET, IOType.TOOL or IOType.EXTEND, by default IOType.CABINET

        Returns
        -------
        bool
            The state of the given digital input.
        """
        return self.robot.get_digital_input(iotype.value, index)

    def _get_digital_output(self, index: int, iotype: IOType=IOType.CABINET)->bool:
        """Read the state of the index-th (0 based) digital output.

        Parameters
        ----------
        index : int
            The 0-based index of the digital output port.  
        iotype : IOType, optional
            The IO type. Either IOType.CABINET, IOType.TOOL or IOType.EXTEND, by default IOType.CABINET

        Returns
        -------
        bool
            The state of the given digital output.
        """
        return self.robot.get_digital_output(iotype.value, index)
    
    @untested
    def _get_analog_input(self, index: int, iotype: IOType=IOType.CABINET)->float:
        """Read the state of the index-th (0 based) analog input.

        Parameters
        ----------
        index : int
            The 0-based index of the analog input port.  
        iotype : IOType, optional
            The IO type. Either IOType.CABINET, IOType.TOOL or IOType.EXTEND, by default IOType.CABINET

        Returns
        -------
        float
            The state of the given analog input.
        """
        return self.robot.get_analog_input(iotype.value, index)

    @untested
    def _get_analog_output(self, index: int, iotype: IOType=IOType.CABINET)->float:
        """Read the state of the index-th (0 based) analog output.

        Parameters
        ----------
        index : int
            The 0-based index of the analog output port.  
        iotype : IOType, optional
            The IO type. Either IOType.CABINET, IOType.TOOL or IOType.EXTEND, by default IOType.CABINET

        Returns
        -------
        float
            The state of the given analog output.
        """
        return self.robot.get_analog_output(iotype.value, index)

    def _set_digital_output(self, index: int, value: bool, iotype: IOType=IOType.CABINET)->None:
        """Set the state of the index-th (0 based) digital output.

        Parameters
        ----------
        index : int
            The 0-based index of the digital output port.  
        value : bool
            The value to set the output to.  
        iotype : IOType, optional
            The IO type. Either IOType.CABINET, IOType.TOOL or IOType.EXTEND, by default IOType.CABINET
        """
        return self.robot.set_digital_output(iotype.value, index, value)
    
    @untested
    def _set_analog_output(self, index: int, value: float, iotype: IOType=IOType.CABINET)->None:
        """Set the state of the index-th (0 based) analog output.

        Parameters
        ----------
        index : int
            The 0-based index of the analog output port.  
        value : float
            The value to set the output to.  
        iotype : IOType, optional
            The IO type. Either IOType.CABINET, IOType.TOOL or IOType.EXTEND, by default IOType.CABINET
        """
        return self.robot.set_analog_output(iotype.value, index, value)
    
    #########################################
    #                                       #
    # Tool IO                               #
    #                                       #
    #########################################

    @untested
    def _set_tio_vout_param(self, vout_enable: bool, vout_vol: VOut)->None:
        """Set the TIO voltage parameters.

        Parameters
        ----------
        vout_enable : bool
            Enable/disable TIO voltage output.
        vout_vol : VOut
            Either VOut.V24 or VOut.V12.
        """
        return self.robot.set_tio_vout_param(vout_enable, vout_vol)

    @untested
    def _get_tio_vout_param(self)->tuple:
        """Get the TIO voltage parameters.

        Returns
        ----------
        tuple
            TIO voltage parameters (vout_enable, vout_vol)
        """
        return self.robot.get_tio_vout_param()
    
    @untested
    def _add_tio_rs_signal(self, sign_info: dict)->None:
        """Add or modify a TIO signal.

        Parameters
        ----------
        sign_info : dict
            Signal dictionary  
                - 'sig_name'  //signal name 
                - 'chn_id'    //RS485 chanel ID
                - 'sig_type'  //semaphore type
                - 'sig_addr'  //register address
                - 'value'     //value, invalid when setting 
                - 'frequency' //The refresh frequency of semaphore in the controller, maximum 10
        """
        return self.robot.add_tio_rs_signal(sign_info)
    
    @untested
    def _del_tio_rs_signal(self, sign_name: str)->None:
        """Delete a TIO signal.

        Parameters
        ----------
        sign_name : str
            The signal's name.
        """
        return self.robot.del_tio_rs_signal(sign_name)
    
    @untested
    def _send_tio_rs_command(self, chn_id: int, cmd: str)->None:
        """Send a RS485 command.

        Parameters
        ----------
        chn_id : int
            The id of the TIO channel.
        cmd : str
            The command.
        """
        return self.robot.send_tio_rs_command(chn_id, cmd)
    
    @untested
    def _get_rs485_signal_info(self)->list:
        """Get information on TIO signals.

        Returns
        -------
        list
            TIO signals information [{'value': 0, 'chn_id': 0, 'sig_addr': 0, 'sig_name': '', 'sig_type': 0, 'frequency': 0}, ...] 
        """
        return self.robot.get_rs485_signal_info()

    @untested
    def _set_tio_pin_mode(self, pin_type: PinType, pin_mode: int)->None:
        """Set TIO mode.

        Parameters
        ----------
        pint_type : PinType
            Either PinType.DIGITAL_INPUT, PinType.DIGITAL_OUTPUT or PinType.ANALOG_INPUT.
        pin_mode : int
            - DI Pins: 0:0x00 DI2 is NPN, DI1 is NPN, 1:0x01 DI2 is NPN, DI1 is PNP, 2:0x10 DI2 is PNP, DI1 is NPN, 3:0x11 DI2 is PNP, DI1 is PNP
            - DO Pins: The lower 8 bits of data and the upper 4 bits are configured as DO2, The lower four bits are DO1 configuration, 0x0 DO is NPN output, 0x1 DO is PNP output, 0x2 DO is push-pull output, 0xF RS485H interface
            - AI Pins: Analog input function is enabled, RS485L is disabled, 1:RS485L interface is enabled and analog input function is disabled.
        """
        return self.robot.set_tio_pin_mode(pin_type.value, pin_mode)
    
    @untested
    def _get_tio_pin_mode(self, pin_type: PinType)->None:
        """Get TIO mode.

        Parameters
        ----------
        pint_type : PinType
            Either PinType.DIGITAL_INPUT, PinType.DIGITAL_OUTPUT or PinType.ANALOG_INPUT.
        """
        return self.robot.get_tio_pin_mode(pin_type)
    
    @untested
    def _set_rs485_chn_comm(self, chn_id: int, slave_id: int, baudrate: BaudRate, 
                           databit: RS485DataBit, stopbit: RS485StopBit, parity: RS485Parity)->None:
        """Set TIO RS485 parameters.

        Parameters
        ----------
        chn_id : int
            Channel ID.
        slave_id : int
            Modbus slave node ID.
        baudrate : BaudRate
            Either BaudRate.B4800, BaudRate.B9600, BaudRate.B14400, BaudRate.B19200, BaudRate.B38400, BaudRate.B57600, BaudRate.B115200 or BaudRate.B230400.
        databit : RS485DataBit
            Either RS485DataBit.SEVEN or RS485DataBit.EIGHT.
        stopbit : RS485StopBit
            Either RS485StopBit.ONE or RS485StopBit.TWO.
        parity : RS485Parity
            Either RS485Parity.NOPARITY, RS485Parity.ODDPARITY or RS485Parity.EVENPARITY. 
        """
        return self.robot.set_rs485_chn_comm(chn_id, slave_id, baudrate.value, databit.value, stopbit.value, parity.value)
    
    @untested
    def _get_rs485_chn_comm(self)->tuple:
        """Get TIO RS485 parameters

        Returns
        -------
        tuple
            The TIO RS485 parameters (chn_id, slave_id,baudrate, databit, stopbit, parity).
        """
        return self.robot.get_rs485_chn_comm()

    @untested
    def _set_rs485_chn_mode(self, chn_id: RS485Channel, chn_mode: CommType)->None:
        """Set the RS485 communication mode.

        Parameters
        ----------
        chn_id : RS485Channel
            Either RS485Channel.RS485H or RS485Channel.RS485L.
        chn_mode : CommType
            Either CommType.MODBUS_RTU, CommType.RAW_RS485 or CommType.TORQUE_SENSOR.
        """
        return self.robot.set_rs485_chn_mode(chn_id.value, chn_mode.value)

    @untested
    def _get_rs485_chn_mode(self, chn_id: RS485Channel)->int:
        """Get the RS485 communication mode.

        Parameters
        ----------
        chn_id : RS485Channel
            Either RS485Channel.RS485H or RS485Channel.RS485L.

        Returns
        -------
        CommType
            Either CommType.MODBUS_RTU, CommType.RAW_RS485 or CommType.TORQUE_SENSOR.
        """
        return self.robot.get_rs485_chn_mode(chn_id.value)

    #########################################
    #                                       #
    # Robot safety status                   #
    #                                       #
    #########################################

    @untested
    def _is_on_limit(self)->bool:
        """Check if the robot reached joint limits.

        Returns
        -------
        bool
            Whether the robot reached joint limits.
        """
        return self.robot.is_on_limit()

    @untested
    def _is_in_collision(self)->bool:
        """Check if the robot is in collision.

        Returns
        -------
        bool
            Whether a collision was detected.
        """
        return self.robot.is_in_collision()
        
    def _collision_recover(self)->None:
        """Recover from a collision state.
        """
        return self.robot.collision_recover()
    
    @untested
    def _set_collision_level(self, level: CollisionLevel)->None:
        """Set the threshold for collision detection.

        Parameters
        ----------
        level : CollisionLevel
            The force threshold for collision detection.
        """
        return self.robot.set_collision_level(level.value)

    @untested
    def _get_collision_level(self)->CollisionLevel:
        """Get the threshold for collision detection.

        Returns
        -------
        CollisionLevel
            The force threshold for collision detection.
        """
        return self.robot.get_collision_level()

    @untested
    def _set_network_exception_handle(self, delay: int, network_exception_handle: NetworkExceptionHandle)->None:
        """Set the allowed network communication delay, and the action to perform in case motion is happening after such a delay.

        Parameters
        ----------
        delay : int
            The delay, in milliseconds.
        network_exception_handle : NetworkExceptionHandle
            The action to perform if motion is happening, after the delay.
        """
        return self.robot.set_network_exception_handle(delay, network_exception_handle.value)

    #########################################
    #                                       #
    # App script program                    #
    #                                       #
    #########################################
 
    @untested
    def _program_load(self, file_name: str)->None:
        """Load a program-

        Parameters
        ----------
        file_name : str
            The program's name.
        """
        return self.robot.program_load(file_name)
 
    @untested
    def _get_loaded_program(self)->str:
        """Get the currently loaded program.

        Returns
        -------
        str
            The name of the currently loaded program.
        """ 
        return self.robot.get_loaded_program()

    @untested
    def _get_current_line(self)->int:
        """Get the current program line.

        Returns
        -------
        int
            The number of the line.
        """
        return self.robot.get_current_line()
  
    @untested
    def _program_run(self)->None:
        """Run the loaded program"""
        return self.robot.program_run()   
     
    @untested
    def _program_pause(self)->None:
        """Pause the loaded program"""
        return self.robot.program_pause()        
     
    @untested
    def _program_resume(self)->None:
        """Resume the loaded program"""
        return self.robot.program_resume()     
      
    @untested
    def _program_abort(self)->None:
        """Abort the loaded program"""
        return self.robot.program_abort()        
  
    @untested
    def _get_program_state(self)->int:
        """Get the program state.

        Returns
        -------
        int
            The state of the program.
        """
        return  self.robot.get_program_state()
   
    @untested
    def _set_rapidrate(self, rapid_rate: float)->None:
        """Set rapid rate.

        Parameters
        ----------
        rapid_rate : float
            Rapid rate.
        """
        return self.robot.set_rapidrate(rapid_rate)

    @untested
    def _get_rapidrate(self)->float:
        """Get rapid rate.

        Returns
        -------
        float
            Rapid rate.
        """
        return self.robot.get_rapidrate()
    
    #########################################
    #                                       #
    # Trajectory reproduction               #
    #                                       #
    #########################################

    @untested
    def _set_traj_config(self, xyz_interval: list, rpy_interval: list, vel: float, acc: float)->None:
        """Set the trajectory track configuration parameters.

        Parameters
        ----------
        xyz_interval : list
            Space position acquisition speed.
        rpy_interval : list
            Orientation capture accuracy.
        vel : float
            Script execution running speed.
        acc : float
            Script execition running acceleration.
        """
        return self.robot.set_traj_config(xyz_interval, rpy_interval, vel, acc)        

    @untested
    def _get_traj_config(self)->tuple:
        """Get the trajectory track configuration parameters.

        Returns
        -------
        tuple
            The parameters (xyz_interval, rpy_interval, vel, acc)
        """
        return self.robot.get_traj_config()

    @untested
    def _set_traj_sample_mode(self, mode: bool, filename: str)->None:
        """Set the trajectory sample mode.

        Parameters
        ----------
        mode : bool
            True for starting data collection, False to stop data collection.
        filename : str
            Filename for data storage.
        """
        return self.robot.set_traj_sample_mode(mode, filename)

    @untested
    def _get_traj_sample_status(self)->bool:
        """Get the status of trajectory sampling.

        Returns
        -------
        bool
            True when collecting data, False otherwise.
        """
        return self.robot.get_traj_sample_status()
    
    #########################################
    #                                       #
    # Robot kinematics                      #
    #                                       #
    #########################################

    @untested
    def _rpy_to_rot_matrix(self, rpy: list)->list:
        """Convert RPY angles to a rotation matrix.

        Parameters
        ----------
        rpy : list
            The xyz roll, pitch and yaw.

        Returns
        -------
        list
            The corresponding ZYX rotation matrix
        """
        if self.use_jaka_kinematics:
            return self.robot.rpy_to_rot_matrix(rpy)
        else:
            return (JAKA_ERR_CODES.SUCCESS_CODE.value, rpy_to_rot_matrix(rpy))

    @untested
    def _rot_matrix_to_rpy(self, rot_matrix: list)->list:
        """Convert a rotation matrix to RPY angles.

        Parameters
        ----------
        rot_matrix : list
            The ZYX rotation matrix.

        Returns
        -------
        list
            The corresponding xyz roll, pitch and yaw.
        """
        if self.use_jaka_kinematics:
            return self.robot.rot_matrix_to_rpy(rot_matrix)
        else:
            return (JAKA_ERR_CODES.SUCCESS_CODE.value, rot_matrix_to_rpy(rot_matrix))

    @untested
    def _quaternion_to_rot_matrix(self, quaternion: list)->list:
        """Convert a quaternion to a rotation matrix.

        Parameters
        ----------
        quaternion : list
            The [w, x, y, z] quaternion.        

        Returns
        -------
        list
            The corresponding ZYX rotation matrix.
        """
        if self.use_jaka_kinematics:
            return self.robot.quaternion_to_rot_matrix(quaternion)
        else:
            return (JAKA_ERR_CODES.SUCCESS_CODE.value, quaternion_to_rot_matrix(quaternion))

    @untested
    def _rot_matrix_to_quaternion(self, rot_matrix: list)->list:
        """Convert a rotation matrix to a quaternion.

        Parameters
        ----------
        rot_matrix : list
            The ZYX rotation matrix.

        Returns
        -------
        list
            The corresponding [w, x, y, z] quaternion
        """
        if self.use_jaka_kinematics:
            return self.robot.rot_matrix_to_quaternion(rot_matrix)
        else:
            return (JAKA_ERR_CODES.SUCCESS_CODE.value, rot_matrix_to_quaternion(rot_matrix))

    def _kine_inverse(self, ref_pos: list, cartesian_pose: list)->list:
        """Compute inverse kinematics.

        Parameters
        ----------
        ref_pos : list
            Reference joint position, in radians.
        cartesian_pose : list
            Target TCP pose, in millimeters and radians.

        Returns
        ------- 
        list
            The corresponding joint pose, if a solution exists.
        """
        if self.use_jaka_kinematics:
            if ref_pos is None:
                ref_pos = self.get_joint_position()
            return self.robot.kine_inverse(ref_pos, cartesian_pose)
        else:
            if ref_pos is None:
                ref_pos = self.get_joint_position()
            ret = self.chain.ik_LM(jaka_to_se3(cartesian_pose), q0=ref_pos)
            return ((JAKA_ERR_CODES.ERR_KINE_INVERSE_ERR.value,) if not ret[1] else (JAKA_ERR_CODES.SUCCESS_CODE.value, ret[0]))

    def _kine_forward(self, joint_pos: list)->SE3:
        """Compute forward kinematics.

        Parameters
        ----------
        joint_pos : list
            Target joint position.

        Returns
        -------
        list
            The corresponding TCP pose.
        """
        if self.use_jaka_kinematics:
            return self.robot.kine_forward(joint_pos)
        else:
            return (JAKA_ERR_CODES.SUCCESS_CODE.value, self.chain.fkine(joint_pos))
   
    def jacobian(self, joint_position: list=None)->np.ndarray:
        """Compute the robot's Jacobian matrix at the given (or current if None) joint position.

        Parameters
        ----------
        joint_position : list, optional
            Target joint position, by default None

        Returns
        -------
        np.ndarray
            The full Jacobian matrix of the robot.
        """
        if self.use_jaka_kinematics:
            raise NotImplementedError('JAKA\'s SDK does not expose the Jacobian.')
        else:
            # TODO: this is another big bottleneck, we lose about 15Hz with each call. A nice project would be to compute the robot's kinematics analytically (like UR did), and store the expression here.
            if joint_position is None: # Wanting the Jacobian at the current state might be implied
                joint_position = self.get_joint_position()
            return self.chain.jacob0(joint_position)
        
    #########################################
    #                                       #
    # Robot servo mode                      #
    #                                       #
    #########################################

    def _servo_j(self, joint_pos: list, move_mode: MoveMode, step_num: int=1)->None:
        """Servo move in joint space.

        Parameters
        ----------
        joint_pos : list
            Target joint position.
        move_mode : MoveMode
            Movement mode.
        """
        return self.robot.servo_j(joint_pos, move_mode.value, step_num)

    def _servo_p(self, cartesian_pose: list, move_mode: MoveMode, step_num: int=1)->None:
        """Servo move in Cartesian space.

        Parameters
        ----------
        cartesian_pose : list
            Target Cartesian position.
        move_mode : MoveMode
            Movement mode.
        """
        return self.robot.servo_p(cartesian_pose, move_mode.value, step_num)

    @untested
    def _servo_move_use_none_filter(self)->None:
        """Disable all filters.
        """
        return self.robot.servo_move_use_none_filter()

    @untested
    def _servo_move_use_joint_LPF(self, cutoffFreq: float)->None:
        """Apply joint space first-order low-pass filter.

        Parameters
        ----------
        cutoffFreq : float
            Cutoff frequency for the filter.
        """
        return self.robot.servo_move_use_joint_LPF(cutoffFreq)

    @untested
    def _servo_move_use_joint_NLF(self, max_vr: float, max_ar: float, max_jr: float)->None:
        """Apply joint space non-linear filter.

        Parameters
        ----------
        max_vr : float
            Maximum absolute value for angular velocity, in rad/s.
        max_ar : float
            Maximum absolute value for angular acceleration, in rad/s^2
        max_jr : float
            Maximum absolute value for angular jerk, in rad/s^3
        """
        return self.robot.servo_move_use_joint_NLF(max_vr, max_ar, max_jr)

    @untested
    def _servo_move_use_carte_NLF(self, max_vp: float, max_ap: float, max_jp: float, 
                                        max_vr: float, max_ar: float, max_jr: float)->None:
        """Apply Cartesian space non-linear filter.

        Parameters
        ----------
        max_vp : float
            Maximum absolute value for linear velocity, in mm/s.
        max_ap : float
            Maximum absolute value for linear acceleration, in mm/s^2
        max_jp : float
            Maximum absolute value for linear jerk, in mm/s^3
        max_vp : float
            Maximum absolute value for angular velocity, in rad/s.
        max_ap : float
            Maximum absolute value for angular acceleration, in rad/s^2
        max_jp : float
            Maximum absolute value for angular jerk, in rad/s^3
        """
        return self.robot.servo_move_use_carte_NLF(max_vp, max_ap, max_jp, max_vr, max_ar, max_jr)
    
    @untested
    def _servo_move_use_joint_MMF(self, max_buf: int, kp: float, kv: float, ka: float)->None:
        """Apply joint space multi-order mean filter.

        Parameters
        ----------
        max_buf : int
            Size of the buffer for the mean filter.
        kp : float
            Acceleration filter factor.
        kv : float
            Velocity filter factor.
        ka : float
            Position filter factor.
        """
        return self.robot.servo_move_use_carte_NLF(max_buf, kp, kv, ka)

    @untested
    def _servo_speed_foresight(self, max_buf: int, kp: float)->None:
        """Set speed foresight parameters.

        Parameters
        ----------
        max_buf : int
            Size of the buffer for the mean filter.
        kp : float
            Acceleration filter factor.
        """
        return self.robot.servo_speed_foresight(max_buf, kp)

    #########################################
    #                                       #
    # Force control functions               #
    #                                       #
    #########################################

    @untested
    def _set_torsensor_brand(self, sensor_brand: int)->None:
        """Set the torque sensor id.

        Parameters
        ----------
        sensor_brand : int
            ID of the sensor [1-6]. 10 is reserved for the robot's internal sensor.
        """
        return self.robot.set_torsensor_brand(sensor_brand)

    @untested
    def _get_sensor_brand(self)->int:
        """Get the current sensor id.

        Returns
        -------
        int
            Sensor brand [1-6]. 10 is reserved for the robot's internal sensor.
        """
        return self.robot.get_sensor_brand()
    
    @untested
    def _start_torq_sensor_payload_identify(self, joint_pos: list)->None:
        """Start the payload identification procedure.

        Parameters
        ----------
        joint_pos : list
            Target joint position for the procedure. Refer to the User Manual for Force Control Products.
        """
        return self.robot.start_torq_sensor_payload_identify(joint_pos)
    
    @untested
    def _get_torq_sensor_identify_status(self)->int:
        """Get the status of the payload identification process.

        Returns
        -------
        int
            The status of the procedure.
        """
        return self.robot.get_torq_sensor_identify_status()
    
    @untested
    def _get_torq_sensor_payload_identify_result(self)->tuple:
        """Get the result of the payload identification procedure.

        Returns
        -------
        tuple
            The payload (m, [x,y,z]).
        """
        return self.robot.get_torq_sensor_payload_identify_result()
    
    @untested
    def _set_torq_sensor_tool_payload(self, mass: float, centroid: list)->None:
        """Set the payload information.

        Parameters
        ----------
        mass : float
            Mass of the payload, in kilograms.
        centroid : list
            Center of mass of the payload, in millimeters.
        """
        return self.robot.set_torq_sensor_tool_payload(mass, centroid)

    @untested
    def _get_torq_sensor_tool_payload(self)->tuple:
        """Get the payload at the TCP.

        Returns
        -------
        tuple
            The payload (m, [x,y,z]).
        """
        return self.robot.get_torq_sensor_tool_payload()
    
    @untested
    def _set_compliant_type(self, sensor_compensation: SensorCompensation, compliance_type: ComplianceType)->None:
        """Set the force control settings.

        Parameters
        ----------
        sensor_compensation : SensorCompensation
            Compensation mode for the sensor.
        compliance_type : ComplianceType
            Force compliance type.
        """
        return self.robot.set_compliant_type(sensor_compensation.value, compliance_type.value)
    
    @untested
    def _get_compliant_type(self)->tuple:
        """Get the force control settings.

        Returns
        -------
        tuple
            The force control settings (sensor_compensation, compliance_type).
        """
        return self.robot.get_compliant_type()

    @untested
    def _set_ft_ctrl_frame(self, ftFrame: FTFrame)->None:
        """Set the force control frame.

        Parameters
        ----------
        ftFrame : FTFrame
            The target frame.
        """
        return self.robot.set_ft_ctrl_frame(ftFrame.value)

    @untested
    def _get_ft_ctrl_frame(self)->FTFrame:
        """Get the force control frame.

        Returns
        -------
        FTFrame
            The target frame.
        """
        return self.robot.get_ft_ctrl_frame()

    @untested
    def _set_admit_ctrl_config(self, axis: int, opt: bool, ftUser: float, 
                              ftReboundFK: float, ftConstant: float, ftNormalTrack: float)->None:
        """Set the compliance control parameters.

        Parameters
        ----------
        axis : int
            Cartesian space [0-5] axis.
        opt : bool
            Whether to turn on compliance control.
        ftUser : float
            The force the user needs to apply to move the robot in the given direction at the maximum speed.
        ftReboundFK : float
            The rebound force that is applied once the user stops applying force.
        ftConstant : float
            The constant force reference.
        ftNormalTrack : float
            The normal tracking force.
        """
        return self.robot.set_admit_ctrl_config(axis, opt, ftUser, ftReboundFK, ftConstant, ftNormalTrack)
    
    @untested
    def _get_admit_ctrl_config(self)->tuple:
        """Get the compliance control parameters.

        Returns
        -------
        tuple
            The control parameters for each axis (opt, ftUser, ftReboundFK, ftConstant, ftNormalTrack).
        """
        return self.robot.get_admit_ctrl_config()
    
    @untested
    def _set_torque_sensor_comm(self, type: int, ip_addr: str, port: int)->None:
        """Set the torque sensor's communication parameter.

        Parameters
        ----------
        type : int
            Sensor type [1-6].
        ip_addr : str
            Sensor IP address.
        port : int
            Sensor port.
        """
        return self.robot.set_torque_sensor_comm(type, ip_addr, port)

    @untested
    def _get_torque_sensor_comm(self)->tuple:
        """Get the torque sensor's communication parameters.

        Returns
        -------
        tuple
            The sensor's communication parameters (type, ip_addr, port).
        """
        return self.robot.get_torque_sensor_comm()

    @untested
    def _disable_force_control(self)->None:
        """Turn off force control.
        """
        return self.robot.disable_force_control()
    
    @untested
    def _set_vel_compliant_ctrl(self, level: int, rate1: float, rate2: float, rate3: float, rate4: float)->None:
        """Set the compliance control speed parameters.

        Parameters
        ----------
        level : int
            The compliance control level.
        rate1 : float
            Rate 1.
        rate2 : float
            Rate 2.
        rate3 : float
            Rate 3.
        rate4 : float
            Rate 4.
        """
        return self.robot.set_vel_compliant_ctrl(level, rate1, rate2, rate3, rate4)

    @untested
    def _set_compliance_condition(self, fx: float, fy: float, fz: float,
                                       tx: float, ty: float, tz: float)->None:
        """Set the compliance control torque conditions.

        Parameters
        ----------
        fx : float
            Force along x axis, in N.
        fy : float
            Force along y axis, in N.
        fz : float
            Force along z axis, in N.
        tx : float
            Torque around x axis, in Nm.
        ty : float
            Torque around y axis, in Nm.
        tz : float
            Torque around z axis, in Nm.
        """
        return self.robot.set_compliance_condition(fx, fy, fz, tx, ty, tz)

    @untested
    def _set_torque_sensor_filter(self, torque_sensor_filter: float)->None:
        """Set the torque sensor low-pass filter cutoff frequency.

        Parameters
        ----------
        torque_sensor_filter : float
            Filter cutoff frequency, in Hz.
        """
        return self.robot.set_torque_sensor_filter(torque_sensor_filter)

    @untested
    def _set_torque_sensor_soft_limit(self, fx: float, fy: float, fz: float,
                                       tx: float, ty: float, tz: float)->None:
        """Set the soft limit of the sensor.

        Parameters
        ----------
        fx : float
            Force along x axis, in N.
        fy : float
            Force along y axis, in N.
        fz : float
            Force along z axis, in N.
        tx : float
            Torque around x axis, in Nm.
        ty : float
            Torque around y axis, in Nm.
        tz : float
            Torque around z axis, in Nm.
        """
        return self.robot.set_torque_sensor_soft_limit(fx, fy, fz, tx, ty, tz)
    
    @untested
    def _get_torque_sensor_soft_limit(self)->tuple:
        """Get the torque sensor's soft limit.

        Returns
        -------
        tuple
            The sensor's soft limit (fx, fy, fz, tx, ty, tz).
        """
        return self.robot.get_torque_sensor_soft_limit()
    
    #########################################
    #                                       #
    # FTP services                          #
    #                                       #
    #########################################

    @untested
    def _init_ftp_client(self)->None:
        """Connect to the cabinet via FTP.
        """
        return self.robot.init_ftp_client()

    @untested
    def _close_ftp_client(self)->None:
        """Close the connection to the cabinet.
        """
        return self.robot.close_ftp_client()

    @untested
    def _get_ftp_dir(self, remotedir: str, type: FTPType)->str:
        """Get the contents of an FTP directory.

        Parameters
        ----------
        remotedir : str
            The internal folder name.   
        type : FTPType
            The folder type.
        """
        return self.robot.get_ftp_dir(remotedir, type.value)

    @untested
    def _download_file(self, local: str, remote: str, opt: FTPOption)->None:
        """Download a file or directory from a remote FTP directory.

        Parameters
        ----------
        local : str
            Absolute path to download the file to.
        remote : str
            Absolute path of the remote file or directory to download.
        opt : FTPOption
            Data type.
        """
        return self.robot.download_file(local, remote, opt.value)

    @untested
    def _upload_file(self, local: str, remote: str, opt: FTPOption)->None:
        """Upload a file or directory to a remote FTP directory.

        Parameters
        ----------
        local : str
            Absolute path of the file or directory to upload.
        remote : str
            Absolute path to upload the file to.
        opt : FTPOption
            Data type.
        """
        return self.robot.upload_file(local, remote, opt.value)
    
    @untested
    def _rename_ftp_file(self, remote: str, des: str, opt: FTPOption)->None:
        """Rename a remote file or directory FTP directory.

        Parameters
        ----------
        remote : str
            Absolute path of the file or directory to rename.
        des : str
            The name to rename the file or directory to.
        opt : FTPOption
            Data type.
        """
        return self.robot.rename_ftp_file(remote, des, opt.value)
    
    @untested
    def del_ftp_file(self, remote: str, opt: FTPOption)->None:
        """Delete a remote file or directory FTP directory.

        Parameters
        ----------
        remote : str
            Absolute path of the file or directory to delete.
        opt : FTPOption
            Data type.
        """
        return self.robot.del_ftp_file(remote, opt.value)

class SimulatedRobot(BaseRobot):

    def __init__(self,
                 gripper_power_pin: int = 1,
                 gripper_control_pin: int = 0,
                 gripper_timeout: float = 0.5,
                 urdf_package: str='jaka_description',
                 urdf_name: str='jaka.urdf'):

        super().__init__()

        # Simulated digital output bus
        self.dout = np.array([False] * 16)

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

        # Simulated cartesian and joint positions
        self.tcp_position = np.array([-0.48195396,  0.11419133,  0.27716227, -np.pi, 0, -20*np.pi/180])
        self.joint_position = np.array([-np.pi/4, np.pi/2, np.pi/2, np.pi/2, -np.pi/2, 0])

        self.dh_params = {'alpha': (-0.0005976007248150393, 1.571012681834878, 0.0026186919768399993, 0.0029585076850969266, 1.570418930079973, -1.572518299931471), 
                          'a': (0.0, 0.0, 430.33050537109375, 368.6116943359375, 0.0, 0.0), 
                          'd': (119.59429931640625, 0.0, 0.0, -113.51529693603516, 113.65059661865234, 106.31169891357422), 
                          'joint_homeoff': (-0.005741609284850061, -0.01212340607442922, -0.006475695062646038, 0.0004614650491747612, -0.006085788520060811, 0.017154317024852567)}
    
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

    def _linear_move(self, tcp_pos: list, move_mode: MoveMode, speed: float, is_blocking: bool):
        # Compute target joint position from inverse kinematics
        target_joints = self.kine_inverse(self.joint_position, tcp_pos)
        
        # Define number of steps based on desired speed and servo_block_time
        max_joint_diff = np.max(np.abs(target_joints - self.joint_position))
        steps = max(int(max_joint_diff / (speed * self.servo_block_time)), 1)

        # Interpolate joint positions
        for i in range(1, steps + 1):
            intermediate_joints = self.joint_position + (target_joints - self.joint_position) * (i / steps)
            self._servo_j(intermediate_joints, MoveMode.ABSOLUTE)

        return SUCCESSFUL_RET
    
    def _open_gripper(self)->None:
        ret = self._set_digital_output(self.gripper_control_pin, 0)
        time.sleep(self.gripper_timeout)
        return ret

    def _close_gripper(self)->None:
        ret = self._set_digital_output(self.gripper_control_pin, 1)
        time.sleep(self.gripper_timeout)
        return ret
    
    def _power_off_gripper(self)->None: 
        return self._set_digital_output(self.gripper_control_pin, 1)

    def _power_on_gripper(self)->None:
        return self._set_digital_output(self.gripper_control_pin, 0)

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

    def _get_dh_params(self)->dict: return (JAKA_ERR_CODES.SUCCESS_CODE.value, self.dh_params)

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
            ref_pos = self._get_joint_position()
        ret = self.chain.ik_LM(jaka_to_se3(cartesian_pose), q0=ref_pos)
        return ((JAKA_ERR_CODES.ERR_KINE_INVERSE_ERR.value,) if not ret[1] else (JAKA_ERR_CODES.SUCCESS_CODE.value, ret[0]))

    def _kine_forward(self, joint_pos: list)->SE3:
        return (JAKA_ERR_CODES.SUCCESS_CODE.value, self.chain.fkine(joint_pos))
   
    def jacobian(self, joint_position: list=None)->np.ndarray:
        if joint_position is None: # Wanting the Jacobian at the current state might be implied
            joint_position = self._get_joint_position()
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
            
        self.tcp_position = se3_to_jaka(self.kine_forward(self.joint_position))
            
        time.sleep(self.servo_block_time)

        return SUCCESSFUL_RET

    def _servo_p(self, cartesian_pose: list, move_mode: MoveMode, step_num: int=1)->None:
        # TODO: check joint limits, velocity limits
        curr_tcp = self.tcp_position
        if move_mode == MoveMode.ABSOLUTE:
            target_tcp = cartesian_pose
        elif move_mode == MoveMode.INCREMENTAL:
            target_tcp = curr_tcp + cartesian_pose
        target_joint = self._kine_inverse(self.joint_position, target_tcp)

        return self._servo_j(target_joint, MoveMode.ABSOLUTE)