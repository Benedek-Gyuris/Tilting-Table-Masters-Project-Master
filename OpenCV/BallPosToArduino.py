from collections import deque
import numpy as np
import argparse
import cv2
import imutils
import serial
import time

# Setup serial communication with Arduino
ser2 = serial.Serial(port='COM12', baudrate=115200, timeout=.1)
time.sleep(2)  # Wait for the Arduino to initialize
data_send_count = 0
start_time = time.time()

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
                help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
                help="max buffer size")
args = vars(ap.parse_args())

# Define the lower and upper boundaries for white color in HSV
whiteLower = (0, 0, 200)
whiteUpper = (180, 80, 255)
pts = deque(maxlen=args["buffer"])

# Define the lower and upper boundaries for the red color in HSV
# Red is split into two ranges in HSV space
RedLower1 = (0, 120, 70)
RedUpper1 = (10, 255, 255)
RedLower2 = (170, 120, 70)
RedUpper2 = (180, 255, 255)


# Initialize variables to store the boundary center and flag for setup
platform_center = None
boundary_initialized = False
boundary_contour = None

# if a video path was not supplied, grab the reference
# to the USB webcam using CAP_DSHOW
if not args.get("video", False):
    vs = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Use index 1 for the USB cam

    # Check if the camera opened successfully
    if not vs.isOpened():
        print("Cannot open USB camera")
        exit()
else:
    vs = cv2.VideoCapture(args["video"])

# allow the camera or video file to warm up
time.sleep(2.0)

# Define mapping function for relative position to angle
def map_to_angle(value, in_min, in_max, out_min, out_max):
    """Maps a value from one range to another."""
    return out_min + (float(value - in_min) * (out_max - out_min) / (in_max - in_min))

# keep looping
while True:
    # grab the current frame
    ret, frame = vs.read()

    # If frame was not captured successfully, break the loop
    if not ret:
        print("Failed to grab frame.")
        break

    # resize the frame, blur it, and convert it to the HSV color space
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Create a mask for the boundary color
    # Create a mask for the red boundary
    mask1 = cv2.inRange(hsv, RedLower1, RedUpper1)
    mask2 = cv2.inRange(hsv, RedLower2, RedUpper2)
    boundary_mask = cv2.bitwise_or(mask1, mask2)
    boundary_mask = cv2.erode(boundary_mask, None, iterations=2)
    boundary_mask = cv2.dilate(boundary_mask, None, iterations=2)

    # Find contours for the boundary only if it hasn't been initialized
    if not boundary_initialized:
        boundary_cnts = cv2.findContours(boundary_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boundary_cnts = imutils.grab_contours(boundary_cnts)

        # Calculate the boundary center if boundary is detected
        if len(boundary_cnts) > 0:
            boundary_contour = max(boundary_cnts, key=cv2.contourArea)
            M_boundary = cv2.moments(boundary_contour)
            if M_boundary["m00"] != 0:
                platform_center = (int(M_boundary["m10"] / M_boundary["m00"]),
                                   int(M_boundary["m01"] / M_boundary["m00"]))
                boundary_initialized = True
                print(f"Platform center initialized at: {platform_center}")

    # Draw the saved platform center on the frame (even if partially covered)
    if platform_center is not None:
        cv2.circle(frame, platform_center, 5, (0, 255, 0), -1)  # Draw platform center in green
        cv2.drawContours(frame, [boundary_contour], -1, (0, 255, 0), 2)  # Draw boundary in green

    # Detect white dot
    mask = cv2.inRange(hsv, whiteLower, whiteUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=4)

    # Find contours for the white dot
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    if len(cnts) > 0 and boundary_contour is not None:
        # Find the largest contour for the white dot
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)

        if radius > 10:  # Minimum size for the dot
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # Check if the center of the white dot is within the red boundary
            if cv2.pointPolygonTest(boundary_contour, center, False) >= 0:
                # Draw the circle and centroid
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                # Calculate the relative position
                if platform_center is not None:
                    relative_position = (center[0] - platform_center[0], center[1] - platform_center[1])

                    # Map the relative position to angles in range [-30, 30]
                    angle_x = map_to_angle(relative_position[0], -200, 200, -3.5, 3.5)  # Adjust input range (-300, 300) as needed
                    angle_z = map_to_angle(relative_position[1], -200, 200, -2.5, 2.5)  # Adjust input range (-300, 300) as needed

                    # Send data to Arduino
                    output_data = f"X:{angle_x:.2f}Z{angle_z:.2f}\n"
                    ser2.write(output_data.encode('utf-8'))
                    ser2.flush()  # Ensure data is sent immediately
                    time.sleep(0.01)  # Short delay for Arduino to process
                     # Increment data send count
                    data_send_count += 1
                    
                    # print(output_data)  # Replace with ser2.write(output_data.encode('utf-8')) to send to Arduino

    # update the points queue
    pts.appendleft(center)

    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    # Measure and print rate every second
    if time.time() - start_time >= 1.0:
        print(f"Data transmission rate: {data_send_count} messages/second")
        data_send_count = 0
        start_time = time.time()

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# release the video stream or file
vs.release()
cv2.destroyAllWindows()
