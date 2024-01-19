#!/usr/bin/python3

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



    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
    index_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    index_finger_dip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP]

    middle_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    middle_finger_dip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP]

    ring_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
    ring_finger_dip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_DIP]

    pinky_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
    pinky_dip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_DIP]

    closed_threshold = 0.02  # Adjust this as needed

    # Calculate distances from the fingertips to their DIP joints.
    index_dist = (index_fingertip.y - index_finger_dip.y) > 0 and (index_fingertip.y - wrist.y) < 0
    middle_dist = (middle_fingertip.y - middle_finger_dip.y) > 0 and (middle_fingertip.y - wrist.y) < 0
    ring_dist = (ring_fingertip.y - ring_finger_dip.y) > 0 and (ring_fingertip.y - wrist.y) < 0
    pinky_dist = (pinky_fingertip.y - pinky_dip.y) > 0 and (pinky_fingertip.y - wrist.y) < 0

    # Check if all fingertips are close to their respective DIP joints.
    if all(dist for dist in [index_dist, middle_dist, ring_dist, pinky_dist]):
        return "STOP"



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
        return "LEFT"
    elif (180 - angle_threshold_deg <= angle_deg < 180 + angle_threshold_deg):
        return "RIGHT"

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

