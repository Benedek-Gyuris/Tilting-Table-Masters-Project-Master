import time
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from math import isnan  # Ensure this is imported


# Setup serial communication with Arduino
arduino = serial.Serial(port='COM12', baudrate=115200, timeout=.1)
time.sleep(2)  # Wait for the Arduino to initialize

# Initialize lists for storing timestamps and velocity data
timestamps = []
velocity_data = []

# Initialize the plot
plt.style.use("seaborn-v0_8")
fig, ax = plt.subplots()
line, = ax.plot([], [], label="Angular Velocity", marker='o')

# Marker for the highest magnitude velocity
max_velocity_marker, = ax.plot([], [], 'ro', label="Max Velocity")  # Red dot for max velocity

# Configure plot appearance
ax.set_xlim(0, 10)  # Initial x-axis range (time in seconds)
ax.set_ylim(-10, 10)  # Initial y-axis range (velocity)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Velocity (rad/s)")
ax.legend()
ax.grid(True)

def send_start_signal():
    """
    Sends the 'R' character to the Arduino to initiate motor movement.
    """
    try:
        arduino.write(b'R')
        arduino.flush()  # Ensure the data is sent immediately
        print("Sent 'R' to Arduino to start motor movement.")
    except Exception as e:
        print(f"Error sending start signal: {e}")


def update(frame):
    global timestamps, velocity_data

    try:
        # Read and decode a line of data
        raw_line = arduino.readline()  # Read raw data as bytes
        line_data = raw_line.decode('utf-8').strip()  # Decode and strip newline characters

        print(f"Raw line: {raw_line}, Decoded line: '{line_data}'")  # Debugging

        if line_data:
            # Split the data and clean it
            parsed_data = [item.strip() for item in line_data.split(',')]
            if len(parsed_data) == 2:
                try:
                    # Parse timestamp and velocity
                    timestamp = float(parsed_data[0])
                    velocity = float(parsed_data[1])

                    # Validate parsed values
                    if not isnan(timestamp) and not isnan(velocity):
                        timestamps.append(timestamp)
                        velocity_data.append(velocity)

                        # Update plot limits
                        ax.set_xlim(0, max(timestamps) + 1)
                        ax.set_ylim(min(velocity_data) - 5, max(velocity_data) + 5)

                        # Update plot data
                        line.set_data(timestamps, velocity_data)

                        # Find the point of maximum absolute velocity
                        max_velocity = max(velocity_data, key=abs)
                        max_index = velocity_data.index(max_velocity)
                        max_time = timestamps[max_index]

                        # Update the marker for the maximum velocity
                        max_velocity_marker.set_data([max_time], [max_velocity])

                        print(f"Timestamp: {timestamp:.2f}, Velocity: {velocity:.2f}")
                        print(f"Max Velocity: {max_velocity:.2f} at Time: {max_time:.2f}")
                    else:
                        print(f"Invalid values: {parsed_data}")
                except ValueError as e:
                    print(f"Error parsing values: {parsed_data} -> {e}")
            else:
                print(f"Malformed data: {line_data}")
        else:
            print("Empty line received")
    except Exception as e:
        print(f"Unexpected error: {e}")

    return line, max_velocity_marker

# Send the 'R' signal before starting the animation
send_start_signal()

# Start the animation
ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)

# Display the plot
plt.show()
