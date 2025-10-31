from collections import deque
import numpy as np
import cv2
import imutils
import serial
import time
import matplotlib.pyplot as plt

# Limit number of points for plotting
MAX_POINTS = 100

# Initialize variables for plotting
positions_x = []  # Up/Down (X-axis)
positions_z = []  # Left/Right (Z-axis)
velocities_x_raw = []  # Raw velocity in X-axis
velocities_z_raw = []  # Raw velocity in Z-axis
velocities_x_filtered = []  # Filtered velocity in X-axis
velocities_z_filtered = []  # Filtered velocity in Z-axis
angles_x = []  # Angular data for X-axis
angles_z = []  # Angular data for Z-axis
timestamps = []

# Initialize observer variables
velocity_est_x = 0  # Estimated velocity in X
velocity_est_z = 0  # Estimated velocity in Z
alpha = 0.3  # Blending factor for complementary filter

# Setup live plotting
plt.ion()
fig, axs = plt.subplots(2, 2, figsize=(12, 8))

# X-axis velocity and angle
axs[0, 0].set_title("X-Axis Velocity")
axs[0, 0].set_xlabel("Time (s)")
axs[0, 0].set_ylabel("Velocity (pixels/s)")
axs[0, 0].grid()
vel_x_raw_line, = axs[0, 0].plot([], [], 'b--', label="Raw Velocity X")
vel_x_filtered_line, = axs[0, 0].plot([], [], 'b-', label="Filtered Velocity X")
axs[0, 0].legend()

axs[0, 1].set_title("X-Axis Angle")
axs[0, 1].set_xlabel("Time (s)")
axs[0, 1].set_ylabel("Angle (degrees)")
axs[0, 1].grid()
angle_x_line, = axs[0, 1].plot([], [], 'r-', label="Angle X")
axs[0, 1].legend()

# Z-axis velocity and angle
axs[1, 0].set_title("Z-Axis Velocity")
axs[1, 0].set_xlabel("Time (s)")
axs[1, 0].set_ylabel("Velocity (pixels/s)")
axs[1, 0].grid()
vel_z_raw_line, = axs[1, 0].plot([], [], 'g--', label="Raw Velocity Z")
vel_z_filtered_line, = axs[1, 0].plot([], [], 'g-', label="Filtered Velocity Z")
axs[1, 0].legend()

axs[1, 1].set_title("Z-Axis Angle")
axs[1, 1].set_xlabel("Time (s)")
axs[1, 1].set_ylabel("Angle (degrees)")
axs[1, 1].grid()
angle_z_line, = axs[1, 1].plot([], [], 'm-', label="Angle Z")
axs[1, 1].legend()

# Setup serial communication with Arduino
ser = serial.Serial(port='COM12', baudrate=115200, timeout=.1)
time.sleep(2)  # Wait for the Arduino to initialize
start_time = time.time()

# OpenCV setup
vs = cv2.VideoCapture(1)  # Use the second camera (index 1)
if not vs.isOpened():
    print("No camera found or cannot access camera.")
    exit()

time.sleep(2.0)  # Warm up camera

# Define HSV range for green and red
greenLower = (80, 50, 50)
greenUpper = (100, 255, 255)
redLower1 = (0, 120, 70)
redUpper1 = (10, 255, 255)
redLower2 = (170, 120, 70)
redUpper2 = (180, 255, 255)

# Initialize variables
prev_center = None
prev_time = None
platform_center = None
boundary_contour = None
boundary_initialized = False

while True:
    # Read frame
    ret, frame = vs.read()
    if not ret:
        print("Failed to grab frame.")
        break

    # Resize and process frame
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Create a mask for detecting red boundary
    mask1 = cv2.inRange(hsv, redLower1, redUpper1)
    mask2 = cv2.inRange(hsv, redLower2, redUpper2)
    boundary_mask = cv2.bitwise_or(mask1, mask2)
    boundary_mask = cv2.erode(boundary_mask, None, iterations=2)
    boundary_mask = cv2.dilate(boundary_mask, None, iterations=2)

    # Find contours for the boundary
    if not boundary_initialized:
        boundary_cnts = cv2.findContours(boundary_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boundary_cnts = imutils.grab_contours(boundary_cnts)

        if len(boundary_cnts) > 0:
            boundary_contour = max(boundary_cnts, key=cv2.contourArea)
            M_boundary = cv2.moments(boundary_contour)
            if M_boundary["m00"] != 0:
                platform_center = (int(M_boundary["m10"] / M_boundary["m00"]),
                                   int(M_boundary["m01"] / M_boundary["m00"]))
                boundary_initialized = True
                print(f"Platform center initialized at: {platform_center}")

    # Draw the boundary and platform center
    if platform_center is not None:
        cv2.circle(frame, platform_center, 5, (0, 255, 0), -1)
        if boundary_contour is not None:
            cv2.drawContours(frame, [boundary_contour], -1, (0, 255, 0), 2)

    # Detect green marker
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=4)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    center = None
    if len(cnts) > 0 and platform_center is not None:
        c = max(cnts, key=cv2.contourArea)
        M = cv2.moments(c)

        if M["m00"] > 0:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(frame, center, 5, (0, 255, 0), -1)

            # Calculate relative position to platform center
            rel_x = platform_center[1] - center[1]  # Up/Down
            rel_z = center[0] - platform_center[0]  # Left/Right

            # Append position for plotting
            positions_x.append(rel_x)
            positions_z.append(rel_z)

            # Scale velocity with exponential curve near edges
            if cv2.pointPolygonTest(boundary_contour, center, True) > 0:  # Inside boundary
                dist_to_edge = cv2.pointPolygonTest(boundary_contour, center, True)
                scaling_factor = np.exp(-0.01 * abs(dist_to_edge))  # Exponential decay
            else:
                scaling_factor = 0.1  # Near the edge

            # Calculate velocity
            cur_time = time.time()
            if prev_center is not None and prev_time is not None:
                dt = cur_time - prev_time
                if dt > 0:
                    dx = rel_x - (platform_center[1] - prev_center[1])
                    dz = rel_z - (prev_center[0] - platform_center[0])
                    velocity_meas_x = dx / dt
                    velocity_meas_z = dz / dt

                    velocity_meas_x *= scaling_factor
                    velocity_meas_z *= scaling_factor

                    # Apply complementary filter
                    velocity_est_x = alpha * velocity_meas_x + (1 - alpha) * velocity_est_x
                    velocity_est_z = alpha * velocity_meas_z + (1 - alpha) * velocity_est_z

                    # Append velocities
                    velocities_x_raw.append(velocity_meas_x)
                    velocities_z_raw.append(velocity_meas_z)
                    velocities_x_filtered.append(velocity_est_x)
                    velocities_z_filtered.append(velocity_est_z)

                    # Calculate angles
                    angle_x = max(-3.5, min(3.5, -velocity_est_x * 3.5 / 200))
                    angle_z = max(-2.5, min(2.5, velocity_est_z * 2.5 / 200))
                    angles_x.append(angle_x)
                    angles_z.append(angle_z)

                    # Send to Arduino
                    output_data = f"X:{angle_x:.2f}Z{angle_z:.2f}\n"
                    ser.write(output_data.encode('utf-8'))
                    ser.flush()

                    timestamps.append(cur_time - start_time)
            else:
                velocities_x_raw.append(0)
                velocities_z_raw.append(0)
                velocities_x_filtered.append(0)
                velocities_z_filtered.append(0)
                angles_x.append(0)
                angles_z.append(0)
                timestamps.append(0)

            prev_center = center
            prev_time = cur_time

            # Limit number of points for plotting
            velocities_x_raw = velocities_x_raw[-MAX_POINTS:]
            velocities_z_raw = velocities_z_raw[-MAX_POINTS:]
            velocities_x_filtered = velocities_x_filtered[-MAX_POINTS:]
            velocities_z_filtered = velocities_z_filtered[-MAX_POINTS:]
            angles_x = angles_x[-MAX_POINTS:]
            angles_z = angles_z[-MAX_POINTS:]
            timestamps = timestamps[-MAX_POINTS:]

            # Update plots
            vel_x_raw_line.set_data(timestamps, velocities_x_raw)
            vel_x_filtered_line.set_data(timestamps, velocities_x_filtered)
            angle_x_line.set_data(timestamps, angles_x)
            vel_z_raw_line.set_data(timestamps, velocities_z_raw)
            vel_z_filtered_line.set_data(timestamps, velocities_z_filtered)
            angle_z_line.set_data(timestamps, angles_z)

            for ax_row in axs:
                for ax in ax_row:
                    ax.relim()
                    ax.autoscale_view()

            plt.pause(0.01)

    # Show frame
    cv2.imshow("Frame", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

# Release resources
vs.release()
ser.close()
cv2.destroyAllWindows()
plt.ioff()
plt.show()
