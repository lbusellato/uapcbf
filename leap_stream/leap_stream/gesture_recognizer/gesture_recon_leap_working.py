import time

import leap
from leap import TrackingMode
import json

from lib_gest_recon_leap import create_data, preparing_frame,recognition_gesture_frame,net_loader

NNname = "poselda.sav"
net_path = '../gesture_recognizer/nets/'
static_model = net_loader(net_path, NNname)

# === Main Recorder ===
class LeapRecorder(leap.Listener):
    def __init__(self):
        super().__init__()
        self.frame_counter = 0
        self.recording = False
        self.frames = []

    def on_connection_event(self, event):
        print("Leap Motion connesso")


    def on_tracking_event(self, event):

        json_data = create_data(event, self.frame_counter)
        parsed_data = json.loads(json_data)

        # Ora puoi accedere alla chiave "hands"
        if not parsed_data['hands']:  # Controlla se c'è almeno una manor
            return  # Skippa il frame se non c'è mano destra


        jcd_frame=preparing_frame(parsed_data)
        this_recon=recognition_gesture_frame(static_model,jcd_frame)
        print(this_recon)



# === Avvio ===
def main():
    listener = LeapRecorder()
    connection = leap.Connection()
    connection.add_listener(listener)


    with connection.open():
        connection.set_tracking_mode(TrackingMode.Desktop)
        try:
            while True:
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\nUscita manuale.")

if __name__ == "__main__":
    main()