#!/usr/bin/python3
"""THIS SCRIPT WROTE USING THE HELP OF AI SO DON`T BLAME SOMEONE"""

import cv2
import mediapipe as mp
import math

# Initialize MediaPipe Hands module.
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False,
                       max_num_hands=1,
                       min_detection_confidence=0.5,
                       min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

def classify_gesture(hand_landmarks):
    # Check for invalid landmark coordinates.
    for lm in hand_landmarks.landmark:
        if not (0 <= lm.x < 1 and 0 <= lm.y < 1):
            return "INVALID"

    # Get required landmarks' positions.
    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    index_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    index_finger_pip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP]
    middle_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    middle_finger_pip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
    ring_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
    ring_finger_pip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP]
    pinky_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
    pinky_pip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP]

    # Closed hand detection threshold.
    closed_threshold = 0.04

    # Calculate distances from the fingertips to their PIP joints.
    thumb_dist = math.hypot(thumb_tip.x - wrist.x, thumb_tip.y - wrist.y)
    index_dist = math.hypot(index_fingertip.x - index_finger_pip.x, index_fingertip.y - index_finger_pip.y)
    middle_dist = math.hypot(middle_fingertip.x - middle_finger_pip.x, middle_fingertip.y - middle_finger_pip.y)
    ring_dist = math.hypot(ring_fingertip.x - ring_finger_pip.x, ring_fingertip.y - ring_finger_pip.y)
    pinky_dist = math.hypot(pinky_fingertip.x - pinky_pip.x, pinky_fingertip.y - pinky_pip.y)

    # Check if all fingertips are close to their respective PIP joints.
    if all(dist < closed_threshold for dist in [thumb_dist, index_dist, middle_dist, ring_dist, pinky_dist]):
        return "CLOSED"

    # Define threshold to classify gestures.
    angle_threshold_deg = 45  # Threshold in degrees for left/right classification.
    vertical_threshold = 0.1  # Threshold for vertical movement to determine top or bottom gesture.

    # Calculate vectors from wrist to fingertips for angle calculations.
    index_finger_vector = [index_fingertip.x - wrist.x, index_fingertip.y - wrist.y]
    middle_finger_vector = [middle_fingertip.x - wrist.x, middle_fingertip.y - wrist.y]

    # Calculate the angle between the wrist-middle fingertip vector and horizontal axis.
    angle_rad = math.atan2(middle_finger_vector[1], middle_finger_vector[0])
    angle_deg = math.degrees(angle_rad)

    # Normalize the angle to the range [0, 360)
    if angle_deg < 0:
        angle_deg += 360

    # Classify the left or right gesture based on the angle.
    if (0 <= angle_deg < angle_threshold_deg) or (360 - angle_threshold_deg <= angle_deg < 360):
        return "RIGHT"
    elif (180 - angle_threshold_deg <= angle_deg < 180 + angle_threshold_deg):
        return "LEFT"

    # Calculate normalized vertical component.
    vertical = index_fingertip.y - wrist.y

    # Classify the vertical gesture.
    if vertical < -vertical_threshold:
        return "FORWARD"
    elif vertical > vertical_threshold:
        return "BACK"
    
    return "STOP"

# Start capturing video from the default camera.
cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, image = cap.read()

    if not success:
        print("Ignoring empty camera frame.")
        continue

    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(image_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            gesture = classify_gesture(hand_landmarks)
            # Calculate the width and height of the text box
            text_size = cv2.getTextSize(gesture, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
            text_x = 10
            text_y = 50
            # Draw the black background rectangle
            cv2.rectangle(image, (text_x, text_y - text_size[1] - 10), (text_x + text_size[0], text_y + 10), (0, 0, 0), cv2.FILLED)
            # Draw the white text on top of the rectangle
            cv2.putText(image, gesture, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            print(gesture)

    cv2.imshow('Hand Gesture Recognition', image)

    if cv2.waitKey(5) & 0xFF == 27:  # Press 'ESC' to exit.
        break

cap.release()
cv2.destroyAllWindows()

