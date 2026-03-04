import time

import numpy as np
from scipy.spatial.transform import Rotation as Rscipy


# Identity rotation matrix (3x3)
R = np.eye(3)

# Zero translation vector (3,)
t = np.zeros(3)
def transform_quaternion(q):

    rot = Rscipy.from_quat(q)
    transformed_rot = Rscipy.from_matrix(R) * rot
    return (transformed_rot.as_quat()).tolist()

def avg_quat(q1, q2):
    avg = [(a + b) / 2 for a, b in zip(q1, q2)]
    norm = np.linalg.norm(avg)
    return [x / norm for x in avg] if norm != 0 else avg

def transform_point(p):
    return ((R @ np.array(p)) + t).tolist()


def weighted_average(v1, v2, c1, c2):#da verificare
    total = c1 + c2
    if total == 0:
        return [(a + b) / 2 for a, b in zip(v1, v2)]
    return [(a * c1 + b * c2) / total for a, b in zip(v1, v2)]

def avg_vec(v1, v2):
    return [(a + b) / 2 for a, b in zip(v1, v2)]


def rotations_top_hand(h1):
    if not h1:
        return []
    c1 = h1['confidence']
    avg_hand = {
        'hand_type': h1['hand_type'],  # assume same
        'hand_id': -1,  # new ID
        'confidence': c1,
        'hand_keypoints': {
            'palm_position': transform_point(h1['hand_keypoints']['palm_position']),
            'palm_orientation': transform_quaternion(h1['hand_keypoints']['palm_orientation']),
            'arm': {
                'prev_joint': transform_point(h1['hand_keypoints']['arm']['prev_joint']),
                'next_joint': transform_point(h1['hand_keypoints']['arm']['next_joint']),
                'rotation': transform_quaternion(h1['hand_keypoints']['arm']['rotation']),
            },
            'fingers': {},
            'grab_angle': h1['hand_keypoints']['grab_angle']
        }
    }

    for finger_name in h1['hand_keypoints']['fingers']:
        avg_hand['hand_keypoints']['fingers'][finger_name] = {}
        for bone in ['metacarpal', 'proximal', 'intermediate', 'distal']:
            b1 = h1['hand_keypoints']['fingers'][finger_name][bone]
            #b2 = h2['hand_keypoints']['fingers'][finger_name][bone]
            avg_hand['hand_keypoints']['fingers'][finger_name][bone] = {
                'prev_joint': transform_point(b1['prev_joint'] ),
                'next_joint': transform_point(b1['next_joint'] ),
                'rotation': transform_quaternion(b1['rotation'] ),
            }
    return avg_hand

def average_hand_data(h1, h2):
    c1 = h1['confidence']
    c2 = h2['confidence']#NOT USED YET

    avg_hand = {
        'hand_type': h1['hand_type'],  # assume same
        'hand_id': -1,  # new ID
        'confidence': min(c1, c2),
        'hand_keypoints': {
            'palm_position': avg_vec(h1['hand_keypoints']['palm_position'], h2['hand_keypoints']['palm_position']),
            'palm_orientation': avg_quat(h1['hand_keypoints']['palm_orientation'],
                                         h2['hand_keypoints']['palm_orientation']),
            'arm': {
                'prev_joint': avg_vec(h1['hand_keypoints']['arm']['prev_joint'],
                                      h2['hand_keypoints']['arm']['prev_joint']),
                'next_joint': avg_vec(h1['hand_keypoints']['arm']['next_joint'],
                                      h2['hand_keypoints']['arm']['next_joint']),
                'rotation': avg_quat(h1['hand_keypoints']['arm']['rotation'], h2['hand_keypoints']['arm']['rotation']),
            },
            'fingers': {},
            'grab_angle': (h1['hand_keypoints']['grab_angle'] + h2['hand_keypoints']['grab_angle']) / 2
        }
    }

    for finger_name in h1['hand_keypoints']['fingers']:
        avg_hand['hand_keypoints']['fingers'][finger_name] = {}
        for bone in ['metacarpal', 'proximal', 'intermediate', 'distal']:
            b1 = h1['hand_keypoints']['fingers'][finger_name][bone]
            b2 = h2['hand_keypoints']['fingers'][finger_name][bone]
            avg_hand['hand_keypoints']['fingers'][finger_name][bone] = {
                'prev_joint': avg_vec(b1['prev_joint'], b2['prev_joint']),
                'next_joint': avg_vec(b1['next_joint'], b2['next_joint']),
                'rotation': avg_quat(b1['rotation'], b2['rotation']),
            }
    return avg_hand


def separate_hands(hands):
    total_joints = len(hands)
    if total_joints == 0:
        return [], []  # No data received

    first_hand = hands[0]["hand_type"]
    if total_joints > 1:  # 2 hands
        if first_hand == "left":
            return hands[0], hands[1]
        else:
            return hands[1], hands[2]

    if first_hand == "left":  #1 hand
        return hands[0], []
    else:
        return [], hands[0]


def fuse_hand(hand1, hand2):
    if hand1 is not None and hand2 is not None:  # both hand visible
        hand_fused = fuse_both(hand1, hand2)

        return hand_fused
    # just one hand
    if hand1:
        return hand1
    if hand2:
        return hand2
    # no hands
    return None

def flatten_hand_data(hand_data):
    flat_data = []

    # Add metadata
    hand_type=hand_data['hand_type'] == 'left'  # or encode as 0/1
    hand_id=hand_data['hand_id']
    confidence= hand_data['confidence']
    grab_angle= hand_data['hand_keypoints']['grab_angle']

    # Palm info
    flat_data.extend(hand_data['hand_keypoints']['palm_position'])
    flat_data.extend(hand_data['hand_keypoints']['palm_orientation'])

    # Arm info
    arm = hand_data['hand_keypoints']['arm']
    flat_data.extend(arm['prev_joint'])
    flat_data.extend(arm['next_joint'])
    flat_data.extend(arm['rotation'])

    # Fingers
    for finger_name in ['thumb', 'index', 'middle', 'ring', 'pinky']:
        finger = hand_data['hand_keypoints']['fingers'][finger_name]
        for bone_name in ['metacarpal', 'proximal', 'intermediate', 'distal']:
            bone = finger[bone_name]
            flat_data.extend(bone['prev_joint'])
            flat_data.extend(bone['next_joint'])
            flat_data.extend(bone['rotation'])

    return flat_data

def fuse_both(hand1, hand2):
    print('fusion')
    fused_hand=average_hand_data(hand1,hand2)
    #fused_hand = hand2
    return fused_hand


def get_this_time_method():
    return time.time()

