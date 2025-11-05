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