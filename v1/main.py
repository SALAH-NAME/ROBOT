#!/usr/bin/python3


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

    # Convert the image color from BGR to RGB for MediaPipe processing.
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Process the image and find hand landmarks.
    results = hands.process(image_rgb)

    # Draw the hand annotations on the image.
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            gesture = classify_gesture(hand_landmarks)
            cv2.putText(image, gesture, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            print(gesture)

    # Display the resulting frame.
    cv2.imshow('Hand Gesture Recognition', image)

    if cv2.waitKey(5) & 0xFF == 27:  # Press 'ESC' to exit.
        break

# Release the capture and close OpenCV windows.
cap.release()
cv2.destroyAllWindows()

