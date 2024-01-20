#!/usr/bin/python3

import sys
import cv2
import mediapipe as mp
import math
import socket
import time

serverMACAddress = '98:DA:60:00:57:23'
port = 1


s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

retry_attempts = 5 # number of retry attempts

def connect():
    global s
    for i in range(retry_attempts):
        try:
            s.connect((serverMACAddress, port))
            print("Connected successfully.")
            return True
        except socket.error:
            print(f"Attempt {i+1} of {retry_attempts} failed. Retrying...")
            time.sleep(2)  # wait for 2 seconds before retrying
            s.close()
            s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    else:
        print("All attempts to connect have failed.")
        return False

def send_safe(message):
    try:
        # Send a small data to check if connection is still active
        s.send(message.encode())
        return True
    except socket.error:
        print("Connection lost. Reconnecting...")
        return connect()

# Connect to HC-06
if not connect():
    print("Unable to establish connection.")
    sys.exit()

# Initialize MediaPipe Hands module.
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode = False,
                       max_num_hands = 1,
                       min_detection_confidence = 0.5,
                       min_tracking_confidence = 0.5)
mp_drawing = mp.solutions.drawing_utils


def check_invalid_landmarks(hand_landmarks):
    for lm in hand_landmarks.landmark:
        if not (0 <= lm.x < 1 and 0 <= lm.y < 1):
            return True
    return False

def calculate_distances(hand_landmarks):
    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
    index_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    index_finger_dip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP]
    middle_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    middle_finger_dip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP]
    ring_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
    ring_finger_dip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_DIP]
    pinky_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
    pinky_dip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_DIP]
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]

    index_dist = (index_fingertip.y - index_finger_dip.y) > 0 and (index_fingertip.y - wrist.y) < 0
    middle_dist = (middle_fingertip.y - middle_finger_dip.y) > 0 and (middle_fingertip.y - wrist.y) < 0
    ring_dist = (ring_fingertip.y - ring_finger_dip.y) > 0 and (ring_fingertip.y - wrist.y) < 0
    pinky_dist = (pinky_fingertip.y - pinky_dip.y) > 0 and (pinky_fingertip.y - wrist.y) < 0
    thumb_dist = (pinky_fingertip.x - thumb_tip.x)

    return index_dist, middle_dist, ring_dist, pinky_dist, thumb_dist

def calculate_angle(hand_landmarks):
    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
    index_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    middle_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]

    index_finger_vector = [index_fingertip.x - wrist.x, index_fingertip.y - wrist.y]
    middle_finger_vector = [middle_fingertip.x - wrist.x, middle_fingertip.y - wrist.y]

    angle_rad = math.atan2(middle_finger_vector[1], middle_finger_vector[0])
    angle_deg = math.degrees(angle_rad)

    if angle_deg < 0:
        angle_deg += 360

    return angle_deg

def classify_gesture(hand_landmarks):
    if check_invalid_landmarks(hand_landmarks):
        return "x"

    index_dist, middle_dist, ring_dist, pinky_dist, thumb_dist = calculate_distances(hand_landmarks)

    if all(dist for dist in [index_dist, middle_dist, ring_dist, pinky_dist]):
        return "x"

    angle_deg = calculate_angle(hand_landmarks)

    angle_threshold_deg = 45
    if (0 <= angle_deg < angle_threshold_deg) or (360 - angle_threshold_deg <= angle_deg < 360):
        return "a"
    elif (180 - angle_threshold_deg <= angle_deg < 180 + angle_threshold_deg):
        return "d"

    vertical_threshold = 0.1
    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
    index_fingertip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    vertical = index_fingertip.y - wrist.y

    if vertical < -vertical_threshold:
        if (thumb_dist > 0):
            return "f"
        else:
            return "w"
    elif vertical > vertical_threshold:
        return "s"

    return "x"


def process_image(image):
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(image_rgb)
    return results

def draw_landmarks_and_get_gesture(image, hand_landmarks):
    mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
    gesture = classify_gesture(hand_landmarks)
    return gesture

def draw_text_box(image, gesture):
    text_size = cv2.getTextSize(gesture, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
    text_x = 10
    text_y = 50
    cv2.rectangle(image, (text_x, text_y - text_size[1] - 10), (text_x + text_size[0], text_y + 10), (0, 0, 0), cv2.FILLED)
    cv2.putText(image, gesture, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

def main():
    cap = cv2.VideoCapture(0)

    last_action = 'x'
    while cap.isOpened():
        success, image = cap.read()

        if not success:
            print("Ignoring empty camera frame.")
            continue

        results = process_image(image)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                gesture = draw_landmarks_and_get_gesture(image, hand_landmarks)
                draw_text_box(image, gesture)
                print(gesture)
                if (last_action != gesture):
                    print(f"SEND => {gesture}")
                    last_action = gesture
                    send_safe(gesture)
        else :
            draw_text_box(image, 'x')
            #print("x")
            if (last_action != 'x'):
                print("SEND => x")
                last_action = 'x'
                send_safe('x')

        cv2.imshow('Hand Gesture Recognition', image)

        if cv2.waitKey(5) & 0xFF == 27:  # Press 'ESC' to exit.
            break


if __name__ == "__main__":
    main()
