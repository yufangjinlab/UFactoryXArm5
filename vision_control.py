import cv2
import numpy as np
import math
import time
import traceback
import vision_utils

def center_x_y_general(dx, dy, ratio, angle, adder_1, adder_2, threshold=5):
    if abs(dx) > threshold:
        x_move = dx * ratio
    else:
        x_move = 0

    if abs(dy) > threshold:
        y_move = -dy * ratio
    else:
        y_move = 0

    if abs(angle) > threshold:
        yaw_move = -angle
    else:
        yaw_move = 0



def center_x_y_precise(dx, dy, ratio, angle, threshold=2):
    if abs(dx) > threshold:
        x_move = dx * ratio
    else:
        x_move = 0

    if abs(dy) > threshold:
        y_move = -dy * ratio
    else:
        y_move = 0

    if abs(angle) > threshold:
        yaw_move = -angle
    else:
        yaw_move = 0

def center_x_y_old(dx, dy, threshold = 1):
        if abs(dx) > threshold:
            x_move = dx * .1
        else:
            x_move = 0

        if abs(dy) > threshold:
            y_move = -dy * .1
        else:
            y_move = 0

def ratio_of_lego_pixel_to_mm(self, pixel_length, real_length_mm=31.8):
    ratio = real_length_mm / pixel_length
    self.pprint(f" Lego pixel length is {pixel_length:.2f} pixels and ratio is {ratio:.4f}")
    return ratio


def ratio_of_landing_pad_pixel_to_mm(self, pixel_width, real_width_mm=47.7):  # 47.7mm is width, 127.2mm is length
    ratio = real_width_mm / pixel_width
    self.pprint(f" Landing pad pixel width is {pixel_width:.2f} pixels and ratio is {ratio:.4f}")
    return ratio

def pprint(*args):
    print("[Vision]", *args)


def get_all_color_contours(hsv_image):

   color_contours = {}
   for color_name, ranges in vision_utils.COLOR_RANGES.items():
    contours = vision_utils.get_mask_and_contours(hsv_image, ranges)
    color_contours[color_name] = contours


    return color_contours


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

def find_lego(color_name, contour, frame, frame_center):
    # Use minAreaRect for rotated boxes
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect).astype(int)
    cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

    cv2.circle(frame, frame_center, 5, (255, 255, 255), -1)
    cv2.imshow("Camera Frame 1", frame)
    cv2.waitKey(1)


    (center_x, center_y), (w, h), angle = rect
    box_center = (int(center_x), int(center_y))
    dx = box_center[0] - frame_center[0]
    dy = box_center[1] - frame_center[1]


    pixel_length = max(w, h)
    pixel_width = min(w, h)

    pprint(f"color_name is {color_name}")

    pprint(f"Lego Center 1: {box_center} | dx: {dx}px, dy: {dy}px | length: {pixel_length:.1f}px, width: {pixel_width:.1f}px")

    center_x_y_general(dx, dy, ratio_of_lego_pixel_to_mm(pixel_length), 0, 0, 0)
    time.sleep(1)
    cv2.destroyWindow("Camera Frame 1")
    return True
