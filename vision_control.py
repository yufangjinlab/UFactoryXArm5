import cv2
import numpy as np
import math
import time
import traceback
from xarm import version
import vision_utils

def pprint(*args):
    print("[Vision]", *args)

def measure_lego_angle(contour):
   #Returns: angle in degrees (positive = counter-clockwise from horizontal)

    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    box = box.astype(int)

    # Identify longest side
    max_length = 0
    best_angle = 0
    for i in range(4):
        pt1 = box[i]
        pt2 = box[(i + 1) % 4]
        dx = pt2[0] - pt1[0]
        dy = pt2[1] - pt1[1]
        length = math.hypot(dx, dy)
        angle = math.degrees(math.atan2(dy, dx))  # angle from x-axis
        if length > max_length:
            max_length = length
            best_angle = angle

    # Normalize to [-90, 90] range for interpretation
    if best_angle > 90:
        best_angle -= 180
    elif best_angle < -90:
        best_angle += 180

    pprint(f"LEGO long edge angle relative to horizontal: {best_angle:.1f}Â°")

    time.sleep(1)

    return best_angle
def detect_blue_objects(cap):
    ret, frame = cap.read()
    if not ret:
        return None, [], None

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define blue range
    lower_blue = np.array([100, 125, 100])
    upper_blue = np.array([133, 255, 255])

    # Apply mask and cleanup
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    # Find contours
    contours_blue, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return frame, contours_blue, blue_mask

# This function returns the frame center array
def get_frame_center(frame):
    height, width = frame.shape[:2]
    frame_center = (width // 2, height // 2)
    return frame_center

# This function unpacks rect values so that they can be used for movement code
def unpack_rect(contour, frame):
    # Uses a rect to find box size and location
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect).astype(int)
    cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

    # Unpacks rect information for use in centering
    (center_x, center_y), (w, h), angle = rect
    box_center = (int(center_x), int(center_y))
    frame_center = get_frame_center(frame)
    dx = box_center[0] - frame_center[0]
    dy = box_center[1] - frame_center[1]
    return box_center, dx, dy, w, h, angle