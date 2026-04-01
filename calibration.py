import cv2
import numpy as np
import json
import os

class CameraConfig:
    """
    Manages camera settings and color calibration
    """

    def __init__(self, config_file="camera_config.json"):
        self.config_file = config_file
        self.settings = self.load_or_default()

    def load_or_default(self):
        """
        Load saved config or return defaults
        """
        if os.path.exists(self.config_file):
            with open(self.config_file, 'r') as f:
                return json.load(f)
        return self.get_defaults()

    def get_defaults(self):
        """
        Default camera and color settings
        """
        return {
            "camera": {
                "index": 0,
                "width": 640,
                "height": 480,
                "fps": 30,
                "autofocus": False,
                "auto_exposure": False,
                "exposure": -6,
                "focus": 0,
                "brightness": 128,
                "contrast": 128,
                "saturation": 128,
                "sharpness": 128,
                "auto_white_balance": False
            },
            "colors": {
                "blue": {
                    "lower": [100, 125, 100],
                    "upper": [133, 255, 255],
                    "dual_range": False
                },
                "lime": {
                    "lower": [42, 150, 120],
                    "upper": [68, 255, 255],
                    "dual_range": False
                },
                "dark_green": {
                    "lower": [72, 140, 80],
                    "upper": [88, 255, 210],
                    "dual_range": False
                },
                "yellow": {
                    "lower": [19, 120, 104],
                    "upper": [29, 255, 255],
                    "dual_range": False
                },
                "red": {
                    "lower": [[0, 100, 100], [160, 100, 100]],
                    "upper": [[10, 255, 255], [179, 255, 255]],
                    "dual_range": True
                }
            },
            "filtering": {
                "min_area": 500,
                "max_area": 10000,
                "min_saturation": 120,
                "min_value": 90,
                "morph_kernel_size": 5
            }
        }

    def save(self):
        """
        Save current settings to file
        """
        with open(self.config_file, 'w') as f:
            json.dump(self.settings, f, indent=2)
        print(f"Settings saved to {self.config_file}")

    def configure_camera(self, cap):
        """
        Apply all camera settings to an open capture object
        """
        cam = self.settings["camera"]

        # Resolution
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam["width"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam["height"])
        cap.set(cv2.CAP_PROP_FPS, cam["fps"])

        # Focus settings - CRITICAL FOR SHARPNESS
        if not cam["autofocus"]:
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            cap.set(cv2.CAP_PROP_FOCUS, cam["focus"])
        else:
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)

        # Exposure settings - CRITICAL FOR CONSISTENT LIGHTING
        if not cam["auto_exposure"]:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            cap.set(cv2.CAP_PROP_EXPOSURE, cam["exposure"])
        else:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)

        # White balance - CRITICAL FOR COLOR ACCURACY
        if not cam["auto_white_balance"]:
            cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        else:
            cap.set(cv2.CAP_PROP_AUTO_WB, 1)

        # Image quality settings
        cap.set(cv2.CAP_PROP_BRIGHTNESS, cam["brightness"])
        cap.set(cv2.CAP_PROP_CONTRAST, cam["contrast"])
        cap.set(cv2.CAP_PROP_SATURATION, cam["saturation"])
        cap.set(cv2.CAP_PROP_SHARPNESS, cam["sharpness"])

        print("Camera configured successfully")
        return cap

    def open_camera(self):
        """
        Open camera with all settings applied
        """
        cap = cv2.VideoCapture(self.settings["camera"]["index"])
        if not cap.isOpened():
            raise RuntimeError("Cannot open camera")
        return self.configure_camera(cap)

    def get_color_ranges(self):
        """
        Return COLOR_RANGES dict compatible with vision_utils.py
        """
        ranges = {}
        for color_name, color_data in self.settings["colors"].items():
            ranges[color_name] = {
                "lower": np.array(color_data["lower"]) if not color_data["dual_range"] else [np.array(l) for l in
                                                                                             color_data["lower"]],
                "upper": np.array(color_data["upper"]) if not color_data["dual_range"] else [np.array(u) for u in
                                                                                             color_data["upper"]],
                "dual_range": color_data["dual_range"]
            }
        return ranges


def interactive_calibration():
    """
    Interactive tool to tune camera settings and color ranges
    """

    config = CameraConfig()
    cap = config.open_camera()

    print("\n=== CAMERA CALIBRATION TOOL ===")
    print("This tool helps you fix:")
    print("1. Blurry camera image")
    print("2. Color detection issues (green confusion)")
    print("3. Blue board centering accuracy")
    print("\nControls:")
    print("  Press 'f' - Adjust FOCUS (fix blur)")
    print("  Press 'e' - Adjust EXPOSURE (fix lighting)")
    print("  Press 'c' - Calibrate COLORS")
    print("  Press 's' - SAVE settings")
    print("  Press 'q' - QUIT")
    print("\nStart with 'f' to fix blur, then 'c' to fix colors!\n")

    current_mode = None

    # Focus adjustment window
    cv2.namedWindow("Focus Adjustment")
    cv2.createTrackbar("Focus", "Focus Adjustment",
                       config.settings["camera"]["focus"], 255, lambda x: None)
    cv2.createTrackbar("Sharpness", "Focus Adjustment",
                       config.settings["camera"]["sharpness"], 255, lambda x: None)

    # Exposure adjustment window
    cv2.namedWindow("Exposure Adjustment")
    cv2.createTrackbar("Exposure", "Exposure Adjustment",
                       abs(config.settings["camera"]["exposure"]), 13, lambda x: None)
    cv2.createTrackbar("Brightness", "Exposure Adjustment",
                       config.settings["camera"]["brightness"], 255, lambda x: None)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Apply current focus settings in real-time
        focus_val = cv2.getTrackbarPos("Focus", "Focus Adjustment")
        sharpness_val = cv2.getTrackbarPos("Sharpness", "Focus Adjustment")
        cap.set(cv2.CAP_PROP_FOCUS, focus_val)
        cap.set(cv2.CAP_PROP_SHARPNESS, sharpness_val)

        # Apply current exposure settings in real-time
        exposure_val = -cv2.getTrackbarPos("Exposure", "Exposure Adjustment")
        brightness_val = cv2.getTrackbarPos("Brightness", "Exposure Adjustment")
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure_val)
        cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness_val)

        # Show main camera view
        display = frame.copy()

        # Draw center crosshair
        h, w = frame.shape[:2]
        cv2.line(display, (w // 2 - 30, h // 2), (w // 2 + 30, h // 2), (0, 255, 255), 2)
        cv2.line(display, (w // 2, h // 2 - 30), (w // 2, h // 2 + 30), (0, 255, 255), 2)

        # Add status text
        cv2.putText(display, f"Focus: {focus_val} | Sharpness: {sharpness_val}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(display, f"Exposure: {exposure_val} | Brightness: {brightness_val}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(display, "Press 'f' focus | 'e' exposure | 'c' colors | 's' save | 'q' quit",
                    (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Camera Calibration", display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord('s'):
            # Save current values
            config.settings["camera"]["focus"] = focus_val
            config.settings["camera"]["sharpness"] = sharpness_val
            config.settings["camera"]["exposure"] = exposure_val
            config.settings["camera"]["brightness"] = brightness_val
            config.save()
            print("✓ Settings saved!")

        elif key == ord('c'):
            print("\nStarting color calibration...")
            cap.release()
            cv2.destroyAllWindows()
            calibrate_colors(config)
            cap = config.open_camera()

        elif key == ord('f'):
            print("Adjust focus using the 'Focus Adjustment' window trackbars")

        elif key == ord('e'):
            print("Adjust exposure using the 'Exposure Adjustment' window trackbars")

    cap.release()
    cv2.destroyAllWindows()


def calibrate_colors(config):
    """
    Calibrate individual color ranges
    """

    cap = config.open_camera()

    colors_to_tune = ["lime", "dark_green", "blue"]
    current_color_idx = 0

    print("\n=== COLOR CALIBRATION ===")
    print("Fix the green confusion and blue board detection here")
    print("Press SPACE to cycle through colors")
    print("Press 's' to save, 'q' to quit\n")

    windows = {}
    for color in colors_to_tune:
        win = f"Tune {color}"
        cv2.namedWindow(win)

        color_data = config.settings["colors"][color]
        lower = color_data["lower"]
        upper = color_data["upper"]

        cv2.createTrackbar("H_low", win, lower[0], 179, lambda x: None)
        cv2.createTrackbar("S_low", win, lower[1], 255, lambda x: None)
        cv2.createTrackbar("V_low", win, lower[2], 255, lambda x: None)
        cv2.createTrackbar("H_high", win, upper[0], 179, lambda x: None)
        cv2.createTrackbar("S_high", win, upper[1], 255, lambda x: None)
        cv2.createTrackbar("V_high", win, upper[2], 255, lambda x: None)

        windows[color] = win

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        current_color = colors_to_tune[current_color_idx]
        win = windows[current_color]

        # Get trackbar values
        h_low = cv2.getTrackbarPos("H_low", win)
        s_low = cv2.getTrackbarPos("S_low", win)
        v_low = cv2.getTrackbarPos("V_low", win)
        h_high = cv2.getTrackbarPos("H_high", win)
        s_high = cv2.getTrackbarPos("S_high", win)
        v_high = cv2.getTrackbarPos("V_high", win)

        lower = np.array([h_low, s_low, v_low])
        upper = np.array([h_high, s_high, v_high])

        # Create mask
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Show mask
        cv2.imshow(win, mask)

        # Show original with overlay
        result = cv2.bitwise_and(frame, frame, mask=mask)
        display = cv2.addWeighted(frame, 0.5, result, 0.5, 0)

        cv2.putText(display, f"Tuning: {current_color.upper()}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(display, "SPACE=next color | 's'=save | 'q'=quit",
                    (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("Color Calibration", display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord(' '):
            # Save current color and move to next
            config.settings["colors"][current_color]["lower"] = [h_low, s_low, v_low]
            config.settings["colors"][current_color]["upper"] = [h_high, s_high, v_high]
            current_color_idx = (current_color_idx + 1) % len(colors_to_tune)
            print(f"Switched to tuning: {colors_to_tune[current_color_idx]}")

        elif key == ord('s'):
            # Save all colors
            for color in colors_to_tune:
                win = windows[color]
                config.settings["colors"][color]["lower"] = [
                    cv2.getTrackbarPos("H_low", win),
                    cv2.getTrackbarPos("S_low", win),
                    cv2.getTrackbarPos("V_low", win)
                ]
                config.settings["colors"][color]["upper"] = [
                    cv2.getTrackbarPos("H_high", win),
                    cv2.getTrackbarPos("S_high", win),
                    cv2.getTrackbarPos("V_high", win)
                ]
            config.save()
            print("✓ All color settings saved!")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    interactive_calibration()