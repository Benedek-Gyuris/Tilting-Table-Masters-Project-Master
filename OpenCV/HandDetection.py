import cv2
import serial
import numpy as np
from cvzone.HandTrackingModule import HandDetector # type: ignore

# Initialize serial port (replace "COM3" with your serial port)
ser2 = serial.Serial("COM4", 9600)

# Initialize webcam
cap = cv2.VideoCapture(0)

# Initialize hand detector
detector = HandDetector(maxHands=1, detectionCon=0.8)

# Check if the camera is opened
if not cap.isOpened():
    print("Error opening the camera")
    exit()

while cap.isOpened():
    # Get hand landmarks
    success, img = cap.read()
    if not success:
        break
    
    hands, img = detector.findHands(img)  # Get hand data
    if hands:
        hand = hands[0]
        handType = hand["type"]  # Get hand type (Left or Right)
        
        # Calculate distance between thumb and pointer finger
        thumb_tip = hand["lmList"][4]  # Thumb tip
        index_tip = hand["lmList"][8]  # Index tip
        distance = int(np.linalg.norm(np.array(thumb_tip) - np.array(index_tip)))

        # Normalize the distance for motor speed (e.g., 0 to 400)
        speed = np.clip(int(distance * 2), 0, 400)  # Scale distance to speed
        
        # Display hand type and speed on the image
        cv2.putText(img, f'{handType} Hand Detected', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(img, f'Speed: {speed}', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # Send hand type and speed to Arduino via serial port
        if handType == "Right":
            ser2.write(f'R{speed}\n'.encode())  # Send 'R' and speed for right hand
        elif handType == "Left":
            ser2.write(f'L{speed}\n'.encode())  # Send 'L' and speed for left hand
    else:
        ser2.write(b'S')  # Send 'S' to stop the motor when no hand is detected

    cv2.imshow('Hand Detection', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

ser2.write(b'S')  # Send 'S' to stop the motor when program ends

cap.release()
cv2.destroyAllWindows()
ser2.close()
