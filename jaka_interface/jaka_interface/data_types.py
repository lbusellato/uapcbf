from enum import Enum

class IOType(Enum):
    CABINET = 0
    TOOL = 1
    EXTEND = 2

class CoordType(Enum):
    BASE   =  0 # ID of the robot's base frame (or the user defined one if applicable)
    JOINT  =  1 # ID of the joint space 'frame'
    TOOL   =  2 # ID of the tool's frame

class PinType(Enum):
    DIGITAL_INPUT  = 0
    DIGITAL_OUTPUT = 1
    ANALOG_INPUT   = 2

class MoveMode(Enum):
    ABSOLUTE    = 0 # ID for absolute motion
    INCREMENTAL = 1 # ID for incremental motion
    CONTINUOUS  = 2 # ID for continuous motion

class RS485Channel(Enum):
    RS485H = 1
    RS485L = 2

class RS485DataBit(Enum):
    SEVEN = 7
    EIGHT = 8

class RS485StopBit(Enum):
    ONE = 1
    TWO = 2

class RS485Parity(Enum):
    NOPARITY   = 78
    ODDPARITY  = 79
    EVENPARITY = 69

class CommType(Enum):
    MODBUS_RTU       =  0
    RAW_RS485        =  1
    TORQUE_SENSOR    =  2

class VOut(Enum):
    V24 =  0
    V12 =  1

class BaudRate(Enum):
    B4800   =   4800 
    B9600   =   9600 
    B14400  =  14400 
    B19200  =  19200 
    B38400  =  38400 
    B57600  =  57600 
    B115200 = 115200 
    B230400 = 230400

class CollisionLevel(Enum):
    N0   = 0
    N25  = 1
    N50  = 2
    N75  = 3
    N100 = 4
    N125 = 5

class NetworkExceptionHandle(Enum):
    KEEP_MOTION  = 0
    PAUSE_MOTION = 1
    STOP_MOTION  = 2

class ProgramState(Enum):
    STOPPED = 0
    RUNNING = 1
    PAUSED  = 2

class FTPType(Enum):
    FILE_AND_FOLDER = 0
    FILE            = 1
    FOLDER          = 2
    
class FTPOption(Enum):
    FILE            = 1
    FOLDER          = 2

class IdentificationStatus(Enum):
    COMPLETED  = 0
    IN_PROCESS = 1
    FAILED     = 2

class SensorCompensation(Enum):
    NO_CALIBRATION   = 0
    ZERO_CALIBRATION = 1

class ComplianceType(Enum):
    NO_COMPLIANCE       = 0
    CONSTANT_COMPLIANCE = 1
    SPEED_COMPLIANCE    = 2

class FTFrame(Enum):
    TOOL  = 0
    WORLD = 1

class RobotStatus(Enum):
    # Definitions for get_robot_status
    ERROR_CODE               =  0
    INPOS                    =  1
    POWERED_ON               =  2
    ENABLED                  =  3
    RAPIDRATE                =  4
    PROTECTIVE_STOP          =  5
    DRAG_STATUS              =  6
    ON_SOFT_LIMIT            =  7
    CURRENT_USER_ID          =  8
    CURRENT_FRAME_ID         =  9
    DOUT                     = 10
    DIN                      = 11
    AOUT                     = 12
    AIN                      = 13
    TIO_DOUT                 = 14
    TIO_DIN                  = 15
    TIO_AIN                  = 16
    EXTIO                    = 17
    CART_POSITION            = 18
    JOINT_POSITION           = 19
    ROBOT_MONITOR_DATA       = 20
    TORQ_SENSOR_MONITOR_DATA = 21
    IS_SOCKET_CONNECT        = 22
    TIO_KEY                  = 24
    # Definitions for get_robot_state
    EM_STOP                  = 0
    POWER_ON                 = 1
    SERVO_ENABLED            = 2