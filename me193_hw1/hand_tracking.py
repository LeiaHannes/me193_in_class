import math
import time

import cv2
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python import vision
import legoeducation as le

MODEL_PATH = "hand_landmarker.task"

# connect to the Double Motor
card_color = le.LEGO_COLOR_AZURE
card_serial = '0997'

doublemotor = le.DoubleMotor()
doublemotor.connect(card_color=card_color, card_serial=card_serial)

# Check connection
if not doublemotor.connected:
    print('Error connecting to Double Motor.')
    exit(1)

# Reset yaw to 0 at startup so readings are relative to this orientation
doublemotor.imu_reset_yaw_axis(0)

# control vars
kp = 0.04
kd = 0
ki = 0
current_error = 0
cumulative_error = 0

# mediapipe's old mp.solutions API was removed in this version of the package;
# hand tracking now goes through the Tasks API (HandLandmarker) instead.
base_options = BaseOptions(model_asset_path=MODEL_PATH)
options = vision.HandLandmarkerOptions(
    base_options=base_options,
    running_mode=vision.RunningMode.VIDEO,
    num_hands=2,
    min_hand_detection_confidence=0.7,
    min_tracking_confidence=0.7,
)
hand_landmarker = vision.HandLandmarker.create_from_options(options)

cap = cv2.VideoCapture(0)

def map_range(value, in_min, in_max, out_min, out_max):
    """Linearly map a value from one range to another."""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def clamp(value, low=0, high=100):
    return max(low, min(high, value))

def landmark_distance(lm1, lm2):
    return math.hypot(lm1.x - lm2.x, lm1.y - lm2.y)

def get_hand_openness(hand_landmarks):
    """
    Estimates how open a hand is, on a 0-100 scale (0 = closed fist, 100 = fully open).
    Uses distance between thumb tip and pinky tip, normalized by a wrist-to-MCP
    reference distance so it's independent of hand size/distance from camera.
    """
    wrist = hand_landmarks[0]
    middle_mcp = hand_landmarks[9]  # base of the middle finger, roughly constant regardless of curl

    hand_size = landmark_distance(wrist, middle_mcp)
    if hand_size == 0:
        return 0

    thumb_tip = hand_landmarks[4]
    pinky_tip = hand_landmarks[20]
    spread_distance = landmark_distance(thumb_tip, pinky_tip)

    openness_ratio = spread_distance / hand_size

    # Empirical bounds: tune these if speed feels off for your hand/camera setup
    OPENNESS_MIN = 0.5  # fully closed fist (thumb and pinky close together)
    OPENNESS_MAX = 1.5  # fully open/spread hand

    speed = map_range(openness_ratio, OPENNESS_MIN, OPENNESS_MAX, 0, 100)
    return int(clamp(speed))


def get_hand_positions(result, frame_width, frame_height):
    """
    Returns a list of dicts, one per detected hand:
    { 'label': 'Left'/'Right', 'x': int, 'y': int, 'openness': int }
    x, y are pixel coordinates of the wrist landmark. openness is 0-100,
    only meaningful/used for the Right hand (speed control).
    """
    hand_positions = []

    for hand_landmarks, handedness in zip(result.hand_landmarks, result.handedness):
        label = handedness[0].category_name  # 'Left' or 'Right'
        wrist = hand_landmarks[0]  # landmark 0 = WRIST

        x = int(wrist.x * frame_width)
        y = int(wrist.y * frame_height)

        # clamp y to be within the frame height
        y = max(0, min(500, y))

        # Map y from [0, 500] to [-1600, 1600]
        y = int(map_range(y, 0, 500, -1600, 1600))

        openness = get_hand_openness(hand_landmarks)

        hand_positions.append({'label': label, 'x': x, 'y': y, 'openness': openness})

    return hand_positions


def draw_landmarks(frame, result):
    h, w, _ = frame.shape
    for hand_landmarks in result.hand_landmarks:
        points = [(int(lm.x * w), int(lm.y * h)) for lm in hand_landmarks]
        for connection in vision.HandLandmarksConnections.HAND_CONNECTIONS:
            start, end = points[connection.start], points[connection.end]
            cv2.line(frame, start, end, (0, 255, 0), 2)
        for point in points:
            cv2.circle(frame, point, 3, (0, 0, 255), -1)


start_time = time.time()

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    # Flip for a mirror-like preview, convert to RGB for mediapipe
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

    timestamp_ms = int((time.time() - start_time) * 1000)
    result = hand_landmarker.detect_for_video(mp_image, timestamp_ms)

    h, w, _ = frame.shape
    positions = get_hand_positions(result, w, h)

    # Print positions to console
    for hand in positions:
        print(f"{hand['label']} hand: x={hand['x']}, y={hand['y']}, openness={hand['openness']}, time={timestamp_ms}")

    # Draw landmarks + text on the frame for visual feedback
    draw_landmarks(frame, result)

    for hand in positions:
        cv2.putText(
            frame,
            f"{hand['label']}: ({hand['x']}, {hand['y']}) open={hand['openness']}",
            (hand['x'] - 40, hand['y'] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

    cv2.imshow("Hand Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        doublemotor.motor_run(direction=le.MOTOR_MOVE_DIRECTION_COUNTERCLOCKWISE, motor=le.MOTOR_LEFT, speed=0)
        doublemotor.motor_run(direction=le.MOTOR_MOVE_DIRECTION_CLOCKWISE, motor=le.MOTOR_RIGHT, speed=0)
        break

    # defaults if no hands detected
    target_yaw = 0
    speed = 0

    for hand in positions:
        if hand['label'] == 'Right':
            target_yaw = hand['y']
            speed = hand['openness']  # right hand openness now drives speed

    for i in range(5):
        yaw = doublemotor.imu_device.yaw
        error = target_yaw - yaw
        error = kp * error + kd * (error - current_error) + ki * cumulative_error
        current_error = error
        cumulative_error += error
        print(f"Target Yaw: {target_yaw}, Current Yaw: {yaw}, Error: {error}, Speed: {speed}")

        if speed == 0:
            speed_left = 0
            speed_right = 0
        else:
            speed_left = clamp(speed - error)
            speed_right = clamp(speed + error)

        doublemotor.motor_run(direction=le.MOTOR_MOVE_DIRECTION_COUNTERCLOCKWISE, motor=le.MOTOR_LEFT, speed=speed_left)
        doublemotor.motor_run(direction=le.MOTOR_MOVE_DIRECTION_CLOCKWISE, motor=le.MOTOR_RIGHT, speed=speed_right)
        time.sleep(0.001)


cap.release()
cv2.destroyAllWindows()
doublemotor.stop()
doublemotor.disconnect()