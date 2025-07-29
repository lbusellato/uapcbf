import math
import numpy as np

from geometry_msgs.msg import Pose
from spatialmath import SE3
from tf_transformations import euler_from_quaternion, quaternion_from_euler

#########################################
#                                       #
# Conversion utility functions          #
#                                       #
#########################################    

def jaka_to_se3(jaka_pose: list)->SE3:
    """Convert a pose from jaka format to an SE3 object.

    Parameters
    ----------
    jaka_pose : list
        The target pose.

    Returns
    -------
    SE3
        The converted pose.
    """
    return SE3(*(np.array(jaka_pose[:3])/1000)) * SE3.RPY(jaka_pose[3:])

def se3_to_jaka(se3_pose: SE3)->list:
    """Convert a pose from SE3 format to jaka format.

    Parameters
    ----------
    se3_pose : SE3
        The target pose.

    Returns
    -------
    list
        The converted pose.
    """
    return [*se3_pose.t] + [*se3_pose.eul()]

def rpy_to_rot_matrix(rpy: list) -> np.ndarray:
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
    roll, pitch, yaw = rpy
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    # Compose the rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R = np.array([
         [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
         [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
         [-sp,   cp*sr,            cp*cr]
        ])
    return R

def rot_matrix_to_rpy(R: list) -> list:
    """Convert a rotation matrix to RPY angles.

    Parameters
    ----------
    R : list
        The ZYX rotation matrix.

    Returns
    -------
    list
        The corresponding xyz roll, pitch and yaw.
    """
    # Check for singularity: R[2][0] = -sin(pitch)
    pitch = math.asin(-R[2][0])
    cp = math.cos(pitch)
    # When cp is near zero (singularity) we fix roll=0 and compute yaw differently
    if abs(cp) > 1e-6:
        roll = math.atan2(R[2][1], R[2][2])
        yaw = math.atan2(R[1][0], R[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-R[0][1], R[1][1])
    return [roll, pitch, yaw]

def quaternion_to_rot_matrix(quaternion: list) -> list:
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
    w, x, y, z = quaternion
    R = [
         [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
         [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
         [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
        ]
    return R

def rot_matrix_to_quaternion(R: list) -> list:
    """Convert a rotation matrix to a quaternion.

    Parameters
    ----------
    R : list
        The ZYX rotation matrix.

    Returns
    -------
    list
        The corresponding [w, x, y, z] quaternion
    """
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2  # S = 4 * w
        qw = 0.25 * S
        qx = (R[2][1] - R[1][2]) / S
        qy = (R[0][2] - R[2][0]) / S
        qz = (R[1][0] - R[0][1]) / S
    elif (R[0][0] > R[1][1]) and (R[0][0] > R[2][2]):
        S = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2  # S = 4 * qx
        qw = (R[2][1] - R[1][2]) / S
        qx = 0.25 * S
        qy = (R[0][1] + R[1][0]) / S
        qz = (R[0][2] + R[2][0]) / S
    elif R[1][1] > R[2][2]:
        S = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2  # S = 4 * qy
        qw = (R[0][2] - R[2][0]) / S
        qx = (R[0][1] + R[1][0]) / S
        qy = 0.25 * S
        qz = (R[1][2] + R[2][1]) / S
    else:
        S = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2  # S = 4 * qz
        qw = (R[1][0] - R[0][1]) / S
        qx = (R[0][2] + R[2][0]) / S
        qy = (R[1][2] + R[2][1]) / S
        qz = 0.25 * S
    return [qw, qx, qy, qz]

def pose_ros_to_se3(ros_pose: Pose)->SE3:
    """Convert a pose from ROS format to an SE3 object.

    Parameters
    ----------
    ros_pose : Pose
        Target pose.        

    Returns
    -------
    SE3
        Converted pose.
    """
    pose = ros_to_list(ros_pose)
    return SE3(pose[:3]) * SE3.RPY(euler_from_quaternion(pose[3:]))

def se3_to_ros(se3_pose: SE3)->Pose:
    """Convert a pose from SE3 format to ROS format.

    Parameters
    ----------
    se3_pose : SE3
        Target pose.        

    Returns
    -------
    Pose
        Converted pose.
    """
    return Pose(position=se3_pose.t.tolist(), orientation=list(quaternion_from_euler(*se3_pose.rpy())))

def jaka_to_ros(jaka_pose: list)->Pose:
    """Convert a pose from JAKA format to ROS format.

    Parameters
    ----------
    jaka_pose : list
        Target pose.        

    Returns
    -------
    Pose
        Converted pose.
    """
    return Pose(position = jaka_pose[:3], orientation = list(quaternion_from_euler(*jaka_pose[3:])))

def ros_to_jaka(ros_pose: Pose)->list:
    """Convert a pose from ROS format to JAKA format.

    Parameters
    ----------
    ros_pose : Pose
        Target pose.        

    Returns
    -------
    list
        Converted pose.
    """
    position = [ros_pose.position.x, ros_pose.position.y, ros_pose.position.z]
    orientation = list(euler_from_quaternion([ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z, ros_pose.orientation.w]))
    return position + orientation

def ros_to_list(ros_pose: Pose)->list:
    """Convert a pose from ROS format to a list.

    Parameters
    ----------
    ros_pose : Pose
        Target pose.        

    Returns
    -------
    list
        Converted pose.
    """
    return [ros_pose.position.x,
            ros_pose.position.y,
            ros_pose.position.z,
            ros_pose.orientation.x,
            ros_pose.orientation.y,
            ros_pose.orientation.z,
            ros_pose.orientation.w]

# t: np.ndarray= np.array([0.50, -0.375, 0.01]) for table center
def leap_to_jaka(leap_pose: list,
                 R: np.ndarray= np.array([[0,-1,0], 
                                          [0,0,1], 
                                          [-1,0,0]]),
                 t: np.ndarray= np.array([0.50, 0, 0.01]))->np.ndarray:
    """Converts a pose in LEAP format to JAKA format. Set the proper rotation matrix and translation vector to properly 
    align LEAP's frame to the robot's.

    Parameters
    ----------
    leap_pose : list
        Target pose.
    R : np.ndarray, optional
        The rotation matrix from the LEAP frame to the robot frame, by default np.array([[0,-1,0], [0,0,1], [-1,0,0]])
    t : np.ndarray, optional
        The translation vector from LEAP frame's origin to the robot frame's origin, by default np.array([0.400, 0, 0.025])

    Returns
    -------
    np.ndarray
        Converted pose.
    """
    return (((np.array(leap_pose[:3])) @ R) - t) * 1000