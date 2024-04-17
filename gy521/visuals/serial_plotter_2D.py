import serial
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time

port = '/dev/ttyACM0'

ser = serial.Serial(port, baudrate=115200)

# Number of data points to display
data_points = 500

# Initialize deque for each variable
roll_accel_data = deque(maxlen=data_points)
pitch_accel_data = deque(maxlen=data_points)
roll_kalman_data = deque(maxlen=data_points)
pitch_kalman_data = deque(maxlen=data_points)

# Initialize plots
fig1, ax1 = plt.subplots()
fig2, ax2 = plt.subplots()

major_ticks = [-90, -45, 0, 45, 90]
minor_ticks = [-60, -30, 30, 60]

line_roll_accel, = ax1.plot([], [], '-r', label='Roll Trig')
line_pitch_accel, = ax2.plot([], [], '-b', label='Pitch Trig')
line_roll_kalman, = ax1.plot([], [], '-g', label='Roll Kalman')
line_pitch_kalman, = ax2.plot([], [], '-y', label='Pitch Kalman')

ax1.set_xlim(0, data_points)
ax1.set_ylim(-100, 100)
ax1.set_yticks(major_ticks)
ax1.set_yticks(minor_ticks, minor=True)
ax1.grid()
ax1.grid(which='minor', alpha=0.2)
ax1.grid(which='major', alpha=0.5)
ax1.legend()

ax2.set_xlim(0, data_points)
ax2.set_ylim(-100, 100)
ax2.set_yticks(major_ticks)
ax2.set_yticks(minor_ticks, minor=True)
ax2.grid()
ax2.grid(which='minor', alpha=0.2)
ax2.grid(which='major', alpha=0.5)
ax2.legend()

# Function to read data from serial port
def read_serial():
    while True:
        try:
            line = ser.readline().decode().strip()
            data = line.split(',')
            if len(data) == 4:  # Ensure all data is present
                a, b, c, d = map(float, data)
                roll_accel_data.append(a)
                pitch_accel_data.append(b)
                roll_kalman_data.append(c)
                pitch_kalman_data.append(d)
        except Exception as e:
            print("Error reading from serial:", e)

# Function to update plot for roll data
def update_roll(frame):
    line_roll_accel.set_data(range(len(roll_accel_data)), roll_accel_data)
    line_roll_kalman.set_data(range(len(roll_kalman_data)), roll_kalman_data)
    line_roll_accel.set_zorder(1)  # Set accelerometer line to higher zorder
    line_roll_kalman.set_zorder(2)   # Set kalmanscope line to lower zorder
    ax1.relim()
    ax1.autoscale_view()
    return line_roll_accel, line_roll_kalman

# Function to update plot for pitch data
def update_pitch(frame):
    line_pitch_accel.set_data(range(len(pitch_accel_data)), pitch_accel_data)
    line_pitch_kalman.set_data(range(len(pitch_kalman_data)), pitch_kalman_data)
    line_pitch_accel.set_zorder(1)  # Set accelerometer line to higher zorder
    line_pitch_kalman.set_zorder(2)   # Set kalmanscope line to lower zorder
    ax2.relim()
    ax2.autoscale_view()
    return line_pitch_accel, line_pitch_kalman

# Start serial reading thread
serial_thread = threading.Thread(target=read_serial)
serial_thread.daemon = True
serial_thread.start()

# Start animation for roll data
ani_roll = FuncAnimation(fig1, update_roll, frames=None, blit=True, interval=50)

# Start animation for pitch data
ani_pitch = FuncAnimation(fig2, update_pitch, frames=None, blit=True, interval=50)

plt.show()
