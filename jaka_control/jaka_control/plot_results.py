import numpy as np
import csv
import matplotlib.pyplot as plt
import matplotlib
font = {'size'   : 22}
matplotlib.rc('font', **font)

csv_filename = f"/home/jaka/ros2_ws/recordings/run_2025-07-10 14:04_kalman_data_m0.1_a150_l150"

with open(csv_filename, 'r') as f:
    reader = csv.reader(f)
    data = list(reader)

data = np.array(data[1:], dtype='float')

time = data[:, 0]
h_star = data[:, 1]
h_now = data[:, 2]
tcp_x = data[:, 9]/1000
tcp_y = data[:, 10]/1000
tcp_z = data[:, 11]/1000
hand_x = data[:, 15]
hand_y = data[:, 16]
hand_z = data[:, 17]
future_hand_x = data[:, 18]
future_hand_y = data[:, 19]
future_hand_z = data[:, 20]


#plt.subplot(311)
#plt.plot(time, tcp_x, label="TCP X")
#plt.plot(time, hand_x, label="Current Hand X")
#plt.plot(time, future_hand_x, label="Future Hand X")
#plt.grid()
#plt.legend()
#plt.ylabel("X position [m]")
#
#plt.subplot(312)
#plt.plot(time, tcp_y, label="TCP Y")
#plt.plot(time, hand_y, label="Current Hand Y")
#plt.plot(time, future_hand_y, label="Future Hand Y")
#plt.grid()
#plt.legend()
#plt.ylabel("Y position [m]")
#
#plt.subplot(313)
#plt.plot(time, tcp_z, label="TCP Z")
#plt.plot(time, hand_z, label="Current Hand Z")
#plt.plot(time, future_hand_z, label="Future Hand Z")
#plt.grid()
#plt.legend()
#plt.ylabel("Z position [m]")
#plt.xlabel('Time [s]')
#
plt.figure()
plt.plot(time, h_star, label='H*')
plt.plot(time, h_now, label='h')
plt.plot(time, [0]*len(time))
plt.grid()
plt.legend()

plt.show()

