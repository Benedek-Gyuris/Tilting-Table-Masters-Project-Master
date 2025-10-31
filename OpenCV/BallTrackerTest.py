# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages
from collections import deque
import numpy as np
import argparse
import cv2

import imutils
import time

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

# Define the lower and upper boundaries for the boundary color in HSV (example for red)
RedLower = (0, 120, 70)  # Adjust as needed for your chosen color
RedUpper = (10, 255, 255)

# Initialize variables to store the boundary center and flag for setup
platform_center = None
boundary_initialized = False  # Flag to indicate if the boundary has been initialized

# if a video path was not supplied, grab the reference
# to the USB webcam using CAP_DSHOW
if not args.get("video", False):
    vs = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Use index 1 for the USB cam

    # Check if the camera opened successfully
    if not vs.isOpened():
        print("Cannot open USB camera")
        exit()

# otherwise, grab a reference to the video file
else:
    vs = cv2.VideoCapture(args["video"])

# allow the camera or video file to warm up
time.sleep(2.0)

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

    # construct a mask for the color "white", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, whiteLower, whiteUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=4)
    cv2.imshow("Mask", mask)

    # Create a mask for the boundary color
    boundary_mask = cv2.inRange(hsv, RedLower, RedUpper)
    boundary_mask = cv2.erode(boundary_mask, None, iterations=2)
    boundary_mask = cv2.dilate(boundary_mask, None, iterations=2)

    # Find contours for the boundary only if it hasn't been initialized
    if not boundary_initialized:
        boundary_cnts = cv2.findContours(boundary_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boundary_cnts = imutils.grab_contours(boundary_cnts)

        # Calculate the boundary center if boundary is detected
        if len(boundary_cnts) > 0:
            boundary = max(boundary_cnts, key=cv2.contourArea)
            M_boundary = cv2.moments(boundary)
            if M_boundary["m00"] != 0:
                platform_center = (int(M_boundary["m10"] / M_boundary["m00"]), int(M_boundary["m01"] / M_boundary["m00"]))
                boundary_initialized = True  # Set the flag to True after initial setup
                print(f"Platform center initialized at: {platform_center}")

    # Draw the saved platform center on the frame (even if partially covered)
    if platform_center is not None:
        cv2.circle(frame, platform_center, 5, (0, 255, 0), -1)  # Draw platform center in green

    # Ball detection
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # only proceed if at least one contour for the ball was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
        c = max(cnts, key=cv2.contourArea)
        c = cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True)  # Smooth contour

        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame, then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # Calculate the relative position if both the ball center and platform center are detected
            if platform_center is not None:
                relative_position = (center[0] - platform_center[0], center[1] - platform_center[1])
                distance = np.sqrt(relative_position[0]**2 + relative_position[1]**2)
                angle = np.degrees(np.arctan2(relative_position[1], relative_position[0]))

                # Display the distance and angle on the frame
                cv2.putText(frame, f"Distance: {distance:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, f"Angle: {angle:.2f} deg", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # Draw a line from the platform center to the ball
                cv2.line(frame, platform_center, center, (255, 0, 0), 2)

    # update the points queue
    pts.appendleft(center)

    # loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore them
        if pts[i - 1] is None or pts[i] is None:
            continue

        # otherwise, compute the thickness of the line and draw the connecting lines
        thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# release the video stream or file
vs.release()

# close all windows
cv2.destroyAllWindows()
