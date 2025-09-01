import time
import keyboard
from datetime import datetime
import leap
from leap import TrackingMode
import json

from lib_gest_recon_leap import create_data

# === Main Recorder ===
class LeapRecorder(leap.Listener):
    def __init__(self):
        super().__init__()
        self.frame_counter = 0
        self.recording = False
        self.frames = []

    def on_connection_event(self, event):
        print("Leap Motion connesso")
        print("Premi 'r' per iniziare/interrompere la registrazione, 's' per salvare i dati")

    def on_tracking_event(self, event):
        # Toggle registrazione
        if keyboard.is_pressed('r'):
            self.recording = not self.recording
            print("Inizio registrazione" if self.recording else "Pausa registrazione")
            time.sleep(0.5)

        # Salvataggio
        if keyboard.is_pressed('s'):
            if self.frames:
                self.save_data()
                self.frames.clear()
            else:
                print("Nessun frame da salvare.")
            time.sleep(0.5)

        if self.recording:

            json_data = create_data(event, self.frame_counter)
            parsed_data = json.loads(json_data)

            # Ora puoi accedere alla chiave "hands"
            if not parsed_data['hands']:  # Controlla se c'è almeno una manor
                return  # Skippa il frame se non c'è mano destra
            self.frames.append(json_data)
            self.frame_counter += 1

    def save_data(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"gesture_recognizer/data/leap_data_{timestamp}.txt"
        with open(filename, 'w') as f:
            for frame in self.frames:
                f.write(frame + '\n')
        print(f"Salvato: {filename}")

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
