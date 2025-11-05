import cv2
import numpy as np
import math
import time
import traceback
from xarm import version
import vision_utils

class Vision_control:
    def __init__(self): # <--- Remove 'image_path'
        # Initialization is now simple, no image needed here
        pass
    def pprint(self, *args):
        print("[Vision]", *args)

    def measure_lego_angle(self, contour):
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

        self.pprint(f"LEGO long edge angle relative to horizontal: {best_angle:.1f}Â°")

        time.sleep(1)

        return best_angle
