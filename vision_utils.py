import cv2
import numpy as np
from calibration import CameraConfig

_config = CameraConfig()
COLOR_RANGES = _config.get_color_ranges()

def get_mask_and_contours(hsv_image, color_info):
    """
    Returns contours for the given color definition from COLOR_RANGES.
    """
    kernel = np.ones((5, 5), np.uint8)

    if color_info["dual_range"]:
        mask1 = cv2.inRange(hsv_image, color_info["lower"][0], color_info["upper"][0])
        mask2 = cv2.inRange(hsv_image, color_info["lower"][1], color_info["upper"][1])
        mask = cv2.bitwise_or(mask1, mask2)
    else:
        mask = cv2.inRange(hsv_image, color_info["lower"], color_info["upper"])

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def detect_blue_board(hsv_image, min_area=5000):
    lo = COLOR_RANGES["blue"]["lower"]
    hi = COLOR_RANGES["blue"]["upper"]
    blue_mask = cv2.inRange(hsv_image, lo, hi)
    kernel = np.ones((5, 5), np.uint8)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, blue_mask
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return None, blue_mask
    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect).astype(int)
    return box, blue_mask


def board_roi_mask(frame_shape, board_box):
    mask = np.zeros(frame_shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [board_box], 255)
    return mask


def filter_contours_by_sv(contours, hsv, min_area=500, min_s=120, min_v=90):
    out = []
    for c in contours:
        if cv2.contourArea(c) < min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        roi = hsv[y:y+h, x:x+w]
        if roi.size == 0:
            continue
        s_mean = roi[..., 1].mean()
        v_mean = roi[..., 2].mean()
        if s_mean >= min_s and v_mean >= min_v:
            out.append(c)
    return out

# HSV color ranges and comments
COLOR_RANGES = {
    "red": {
        "lower": [np.array([0, 100, 100]), np.array([160, 100, 100])],
        "upper": [np.array([10, 255, 255]), np.array([179, 255, 255])],
        "dual_range": True,
        "comment": "Red â€” needs dual range due to hue wrapping"
    },
    "lime": {
        "lower": np.array([27, 99, 78]),
        "upper": np.array([71, 255, 255]),
        "dual_range": False,
        "comment": "Lime green"
    },
    "dark_green": {
        "lower": np.array([40, 40, 40]),
        "upper": np.array([104, 255, 220]),
        "dual_range": False,
        "comment": "Dark green"
    },
    "yellow": {
        "lower": np.array([19, 120, 104]),
        "upper": np.array([29, 255, 255]),
        "dual_range": False,
        "comment": "Yellow"
    },
    "blue": {
        "lower": np.array([100, 125, 100]),
        "upper": np.array([133, 255, 255]),
        "dual_range": False,
        "comment": "Blue (also used as landing pad)"
    }
    # "brown": {
    #     "lower": np.array([169, 26, 0]),
    #     "upper": np.array([179, 142, 120]),
    #     "dual_range": False,
    #     "comment": "Brown â€” skipped due to complexity"
    #  },
    # "black": {
    #     "lower": np.array([0, 0, 0]),
    #     "upper": np.array([180, 255, 40]),
    #     "dual_range": False,
    #     "comment": "Black â€” replaced with blue pad"
    # },
    # "grey": {
    #     "lower": np.array([0, 0, 40]),
    #     "upper": np.array([179, 50, 200]),
    #     "dual_range": False,
    #     "comment": "Grey â€” skipped due to difficulty"
    # }
}