#!/usr/bin/python3
"""THIS SCRIPT WROTE USING THE HELP OF AI SO DON`T BLAME SOMEONE"""

import cv2
import mediapipe as mp

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
    index_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    middle_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]

    # Calculate the vertical and horizontal gesture orientation.
    vertical = middle_fingertip.y - wrist.y
    horizontal = index_fingertip.x - wrist.x

    # Define thresholds to classify gestures.
    vertical_threshold = 0.1
    horizontal_threshold = 0.1

    # Classify the vertical gesture.
    if vertical < -vertical_threshold:
        return "TOP"
    elif vertical > vertical_threshold:
        return "BOTTOM"

    # Classify the horizontal gesture.
    if horizontal > horizontal_threshold:
        return "RIGHT"
    elif horizontal < -horizontal_threshold:
        return "LEFT"

    # If all fingers are close together, classify as a closed hand.
    all_fingers = [mp_hands.HandLandmark.INDEX_FINGER_TIP,
                   mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
                   mp_hands.HandLandmark.RING_FINGER_TIP,
                   mp_hands.HandLandmark.PINKY_TIP]

    all_close_to_wrist = all(abs(wrist.y - hand_landmarks.landmark[finger].y) < vertical_threshold for finger in all_fingers)

    if all_close_to_wrist:
        return "CLOSED"

    return "UNDETERMINED"

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
