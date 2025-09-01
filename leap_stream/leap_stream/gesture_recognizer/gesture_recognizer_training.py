

from lib_gest_recon_leap import data_loader_frames, multipleTrain

def models_training(base_path):
    print('net training')
    train_data, train_labels, test_data, test_labels =data_loader_frames(base_path+'data/')#load data

    multipleTrain(base_path+'nets/',train_data, test_data, train_labels, test_labels)#train and save net


def main():
    base_path = 'gesture_recognizer/'
    models_training(base_path)#training
    #training_execution(base_path)







if __name__ == "__main__":
    main()
