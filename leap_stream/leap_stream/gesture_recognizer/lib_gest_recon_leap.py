import time
import json
import os
import numpy as np
from scipy.spatial.distance import cdist
import pickle
from sklearn.naive_bayes import GaussianNB
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn import metrics, svm

joint_n=21# 41 entrambi, 21 solo un joint per osso
joint_d=3

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
        if hand.type.name.lower() != 'right':
            #print('mano sinistra')
            continue
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
    return json_output


def load_leap_data_from_file(filepath):
    frames = []
    with open(filepath, 'r') as f:
        for line in f:
            try:
                frame = json.loads(line.strip())
                frames.append(frame)
            except json.JSONDecodeError as e:
                print(f"Errore nel file {filepath}: {e}")
    return frames




def extract_joint_positions_from_frame(frame):
    joints = []

    for hand in frame.get('hands', []):
        keypoints = hand['hand_keypoints']

        # Palmo
        joints.extend(keypoints['palm_position'])

        # Braccio
        #joints.extend(keypoints['arm']['prev_joint'])
        #joints.extend(keypoints['arm']['next_joint'])

        # Dita
        fingers = keypoints['fingers']
        for finger_name in ['thumb', 'index', 'middle', 'ring', 'pinky']:
            for bone_name in ['metacarpal', 'proximal', 'intermediate', 'distal']:
                bone = fingers[finger_name][bone_name]
                #joints.extend(bone['prev_joint'])
                joints.extend(bone['next_joint'])

    return joints  # flat list of all joints for the frame




def load_all_frames_and_labels(root_dir):
    all_data = []
    all_labels = []

    for user_folder in sorted(os.listdir(root_dir)):
        user_path = os.path.join(root_dir, user_folder)
        if not os.path.isdir(user_path):
            continue

        for file_name in sorted(os.listdir(user_path)):
            if not file_name.endswith('.txt'):
                continue
            label = file_name.replace('.txt', '')
            file_path = os.path.join(user_path, file_name)

            with open(file_path, 'r') as f:
                for line in f:
                    try:
                        frame = json.loads(line.strip())

                        frame_extracted=extract_joint_positions_from_frame(frame)#estraggo da json a lista
                        if frame_extracted:
                            frame_jcded = JCD(frame_extracted)#da lista a jcd

                            all_data.append(frame_jcded)
                            all_labels.append(label)
                    except json.JSONDecodeError as e:
                        print(f"Errore nel file {file_path}: {e}")

    return all_data, all_labels


def data_loader_frames(base_path):
    train_dir = os.path.join(base_path, 'train')
    test_dir = os.path.join(base_path, 'test')

    train_data, train_labels = load_all_frames_and_labels(train_dir)
    test_data, test_labels = load_all_frames_and_labels(test_dir)

    print(f"  Frame caricati:")
    print(f"  Train: {len(train_data)} frame")
    print(f"  Test:  {len(test_data)} frame")
    #print(f"  Esempio train frame: {train_data[0]} - label: {train_labels[0]}")

    return train_data, train_labels, test_data, test_labels


def JCD(p):  # immagine della distanza euclidea dei joint
    # upper triangle index with offset 1, which means upper triangle without diagonal
    #C = Configuration_dataset()
    p = np.copy(np.array(p).reshape([joint_n, joint_d]))
    # prendi solo certe distanza e fai fisher map
    iu = np.triu_indices(joint_n, 1, joint_n)
    d_m = cdist(p, p, 'euclidean')
    d_m = d_m[iu]
    return d_m

def multipleTrain(directory_train_results,X_train, X_test, y_train, y_test):
    ldaname = 'poselda.sav'
    svmname = 'posesvm.sav'
    bayname = 'posebay.sav'
    model = GaussianNB()
    model.fit(X_train, y_train)
    yprob = model.predict(X_test)  # .predict(X_test)
    print("Accuracy gaussian:", metrics.accuracy_score(y_test, yprob))
    pickle.dump(model, open(directory_train_results + bayname, 'wb'))

    clf = LinearDiscriminantAnalysis()
    clf.fit(X_train, y_train)
    yprob = clf.predict(X_test)
    print("Accuracy lda:", metrics.accuracy_score(y_test, yprob))
    pickle.dump(clf, open(directory_train_results + ldaname, 'wb'))

    # Create a svm Classifier
    clf = svm.SVC(kernel='poly')  # linear, (kernel='sigmoid'),poly

    # Train the model using the training sets
    clf.fit(X_train, y_train)

    # Predict the response for test dataset
    y_pred = clf.predict(X_test)
    # yprob=clf.predict_proba(X_test)
    # print(yprob)
    # Model Accuracy: how often is the classifier correct?
    print("Accuracy SVM:", metrics.accuracy_score(y_test, y_pred))

    # print(confusion_matrix(y_test, y_pred))

    pickle.dump(clf, open(directory_train_results + svmname, 'wb'))  # svm save

    print("END TRAIN PHASE")

def preparing_frame(frame):
    frame_extracted = extract_joint_positions_from_frame(frame)  # estraggo da json a lista

    frame_jcded = JCD(frame_extracted)  # da lista a jcd


    return frame_jcded

def recognition_gesture_frame(static_model,frame):
    pred = static_model.predict([frame])
    return pred


def net_loader(path,name):
    net_name = path+ name
    with open(net_name, 'rb') as fd:
        static_model = pickle.load(fd)

    return static_model