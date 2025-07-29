from collections import defaultdict
from enum import Enum

class JakaInterfaceException(Exception):
    """
    Base exception for all Jaka SDK errors.
    
    Parameters
    ----------
        message : str
            A descriptive message for the exception.
    """
    def __init__(self, message: str = ""):
        self.message = message or "An error occurred in the robot SDK."
        super().__init__(f"{self.message}")

JAKA_ERR_MSGS = {
        0  : '',
        2  : 'ERR_FUNCTION_CALL_ERROR: Abnormal call, abnormal call interface, the controller does not support',
       -1  : 'ERR_INVALID_HANDLER: An invalid handle',
       -2  : 'ERR_INVALID_PARAMETER: An invalid argument',
       -3  : 'ERR_COMMUNICATION_ERR: There was a communication error',
       -4  : 'ERR_KINE_INVERSE_ERR: The reverse solution failed',
       -5  : 'ERR_EMERGENCY_PRESSED: The emergency stop key has not been released',
       -6  : 'ERR_NOT_POWERED: The robot is not powered on',
       -7  : 'ERR_NOT_ENABLED: The robot is not enabled',
       -8  : 'ERR_DISABLE_SERVOMODE: SERVOMODE mode is not entered',
       -9  : 'ERR_NOT_OFF_ENABLE: The power is not lowered before power is turned off',
       -10 : 'ERR_PROGRAM_IS_RUNNING: The program is running',
       -11 : 'ERR_CANNOT_OPEN_FILE: Opening the file failed',
       -12 : 'ERR_MOTION_ABNORMAL: Abnormalities during the running process',
       -14 : 'ERR_FTP_PREFROM: Abnormal FTP',
       -15 : 'ERR_VALUE_OVERSIZE: Insufficient reserved memory',
       -16 : 'ERR_KINE_FORWARD: Kine_forward error',
       -17 : 'ERR_EMPTY_FOLDER: Not support empty folder',
       -20 : 'ERR_PROTECTIVE_STOP: Collision detected',
       -21 : 'ERR_EMERGENCY_STOP: Protective stop',
       -22 : 'ERR_SOFT_LIMIT: On soft limit',
       -30 : 'ERR_CMD_ENCODE: Fail to encode cmd string',
       -31 : 'ERR_CMD_DECODE: Fail to decode cmd string',
       -32 : 'ERR_UNCOMPRESS: Fail to uncompress port 10004 string',
       -40 : 'ERR_MOVEL: Move linear error',
       -41 : 'ERR_MOVEJ: Move joint error',
       -42 : 'ERR_MOVEC: Move circular error',
       -50 : 'ERR_MOTION_TIMEOUT: Block_wait timeout',
       -51 : 'ERR_POWERON_TIMEOUT: Power on timeout',
       -52 : 'ERR_POWEROFF_TIMEOUT: Power off timeout',
       -53 : 'ERR_ENABLE_TIMEOUT: Enable timeoff',
       -54 : 'ERR_DISABLE_TIMEOUT: Disable timeout',
       -55 : 'ERR_USERFRAME_SET_TIMEOUT: Set userframe timeout',
       -56 : 'ERR_TOOL_SET_TIMEOUT: Set tool timeout',
       -60 : 'ERR_IO_SET_TIMEOUT: Set io timeout',
}
JAKA_ERR_MSGS = defaultdict(lambda: 'ERR_UNKNOW: Unknown error code', JAKA_ERR_MSGS)

class JAKA_ERR_CODES(Enum):
    SUCCESS_CODE              =  0
    ERR_FUNCTION_CALL_ERROR   =  2
    ERR_INVALID_HANDLER       = -1
    ERR_INVALID_PARAMETER     = -2
    ERR_COMMUNICATION_ERR     = -3
    ERR_KINE_INVERSE_ERR      = -4
    ERR_EMERGENCY_PRESSED     = -5
    ERR_NOT_POWERED           = -6
    ERR_NOT_ENABLED           = -7
    ERR_DISABLE_SERVOMODE     = -8
    ERR_NOT_OFF_ENABLE        = -9
    ERR_PROGRAM_IS_RUNNING    = -10
    ERR_CANNOT_OPEN_FILE      = -11
    ERR_MOTION_ABNORMAL       = -12
    ERR_FTP_PREFROM           = -14
    ERR_VALUE_OVERSIZE        = -15
    ERR_KINE_FORWARD          = -16
    ERR_EMPTY_FOLDER          = -17
    ERR_PROTECTIVE_STOP       = -20
    ERR_EMERGENCY_STOP        = -21
    ERR_SOFT_LIMIT            = -22
    ERR_CMD_ENCODE            = -30
    ERR_CMD_DECODE            = -31
    ERR_UNCOMPRESS            = -32
    ERR_MOVEL                 = -40
    ERR_MOVEJ                 = -41
    ERR_MOVEC                 = -42
    ERR_MOTION_TIMEOUT        = -50
    ERR_POWERON_TIMEOUT       = -51
    ERR_POWEROFF_TIMEOUT      = -52
    ERR_ENABLE_TIMEOUT        = -53
    ERR_DISABLE_TIMEOUT       = -54
    ERR_USERFRAME_SET_TIMEOUT = -55
    ERR_TOOL_SET_TIMEOUT      = -56
    ERR_IO_SET_TIMEOUT        = -60
    
SUCCESSFUL_RET = (JAKA_ERR_CODES.SUCCESS_CODE.value, )

class FunctionCallException(JakaInterfaceException):
    """Raised when there is an abnormal function call or unsupported interface (Error code: 2)."""
    pass


class InvalidHandlerException(JakaInterfaceException):
    """Raised when an invalid handle is used (Error code: -1)."""
    pass


class InvalidParameterException(JakaInterfaceException):
    """Raised when an invalid parameter is provided (Error code: -2)."""
    pass


class CommunicationException(JakaInterfaceException):
    """Raised when a communication error occurs (Error code: -3)."""
    pass


class KineInverseException(JakaInterfaceException):
    """Raised when the inverse kinematics solution fails (Error code: -4)."""
    pass


class EmergencyPressedException(JakaInterfaceException):
    """Raised when the emergency stop key has not been released (Error code: -5)."""
    pass


class NotPoweredException(JakaInterfaceException):
    """Raised when the robot is not powered on (Error code: -6)."""
    pass


class NotEnabledException(JakaInterfaceException):
    """Raised when the robot is not enabled (Error code: -7)."""
    pass


class ServoModeException(JakaInterfaceException):
    """Raised when the required servo mode is not entered (Error code: -8)."""
    pass


class NotOffEnableException(JakaInterfaceException):
    """Raised when the power is not lowered before power is turned off (Error code: -9)."""
    pass


class ProgramRunningException(JakaInterfaceException):
    """Raised when a program is running and prevents the requested action (Error code: -10)."""
    pass


class FileOpenException(JakaInterfaceException):
    """Raised when opening a file fails (Error code: -11)."""
    pass


class MotionAbnormalException(JakaInterfaceException):
    """Raised when abnormalities occur during motion (Error code: -12)."""
    pass


class FTPException(JakaInterfaceException):
    """Raised when there is an abnormal FTP operation (Error code: -14)."""
    pass


class ValueOversizeException(JakaInterfaceException):
    """Raised when there is insufficient reserved memory (Error code: -15)."""
    pass


class KineForwardException(JakaInterfaceException):
    """Raised when the forward kinematics calculation fails (Error code: -16)."""
    pass


class EmptyFolderException(JakaInterfaceException):
    """Raised when an operation is not supported on an empty folder (Error code: -17)."""
    pass


class ProtectiveStopException(JakaInterfaceException):
    """Raised when a protective stop is triggered (Error code: -20)."""
    pass


class EmergencyStopException(JakaInterfaceException):
    """Raised when an emergency stop condition is detected (Error code: -21)."""
    pass


class SoftLimitException(JakaInterfaceException):
    """Raised when the robot is on a soft limit (Error code: -22)."""
    pass


class CommandEncodeException(JakaInterfaceException):
    """Raised when encoding the command string fails (Error code: -30)."""
    pass


class CommandDecodeException(JakaInterfaceException):
    """Raised when decoding the command string fails (Error code: -31)."""
    pass


class UncompressException(JakaInterfaceException):
    """Raised when uncompressing the port 10004 string fails (Error code: -32)."""
    pass


class MoveLinearException(JakaInterfaceException):
    """Raised when a linear move error occurs (Error code: -40)."""
    pass


class MoveJointException(JakaInterfaceException):
    """Raised when a joint move error occurs (Error code: -41)."""
    pass


class MoveCircularException(JakaInterfaceException):
    """Raised when a circular move error occurs (Error code: -42)."""
    pass


class MotionTimeoutException(JakaInterfaceException):
    """Raised when a motion operation times out (Error code: -50)."""
    pass


class PowerOnTimeoutException(JakaInterfaceException):
    """Raised when powering on the robot times out (Error code: -51)."""
    pass


class PowerOffTimeoutException(JakaInterfaceException):
    """Raised when powering off the robot times out (Error code: -52)."""
    pass


class EnableTimeoutException(JakaInterfaceException):
    """Raised when enabling the robot times out (Error code: -53)."""
    pass


class DisableTimeoutException(JakaInterfaceException):
    """Raised when disabling the robot times out (Error code: -54)."""
    pass


class UserFrameSetTimeoutException(JakaInterfaceException):
    """Raised when setting the user frame times out (Error code: -55)."""
    pass


class ToolSetTimeoutException(JakaInterfaceException):
    """Raised when setting the tool times out (Error code: -56)."""
    pass


class IOSetTimeoutException(JakaInterfaceException):
    """Raised when setting IO times out (Error code: -60)."""
    pass

class SDKNotFoundException(JakaInterfaceException):
    """Raised when the SDK was not found."""
    pass

ERROR_EXCEPTION_MAP = {
    2: FunctionCallException,
    -1: InvalidHandlerException,
    -2: InvalidParameterException,
    -3: CommunicationException,
    -4: KineInverseException,
    -5: EmergencyPressedException,
    -6: NotPoweredException,
    -7: NotEnabledException,
    -8: ServoModeException,
    -9: NotOffEnableException,
    -10: ProgramRunningException,
    -11: FileOpenException,
    -12: MotionAbnormalException,
    -14: FTPException,
    -15: ValueOversizeException,
    -16: KineForwardException,
    -17: EmptyFolderException,
    -20: ProtectiveStopException,
    -21: EmergencyStopException,
    -22: SoftLimitException,
    -30: CommandEncodeException,
    -31: CommandDecodeException,
    -32: UncompressException,
    -40: MoveLinearException,
    -41: MoveJointException,
    -42: MoveCircularException,
    -50: MotionTimeoutException,
    -51: PowerOnTimeoutException,
    -52: PowerOffTimeoutException,
    -53: EnableTimeoutException,
    -54: DisableTimeoutException,
    -55: UserFrameSetTimeoutException,
    -56: ToolSetTimeoutException,
    -60: IOSetTimeoutException,
}
ERROR_EXCEPTION_MAP = defaultdict(lambda: JakaInterfaceException, ERROR_EXCEPTION_MAP)
