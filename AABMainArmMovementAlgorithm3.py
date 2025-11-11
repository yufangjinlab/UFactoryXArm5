from doctest import debug

import cv2
import numpy as np
import math
import time
import traceback
from xarm import version
from xarm.wrapper import XArmAPI
from movement import Movement
import vision_control
from vision_utils import COLOR_RANGES, get_mask_and_contours

# first connect, i see that this was done at the bottom of the code as well, will review necessity.
arm = XArmAPI('192.168.1.207', baud_checkset=False)
# initialize movement
movement = Movement(arm)

debugs = 0
class RobotMain(object):
    def __init__(self, arm):
        self._arm = arm
        self.movement = Movement(arm)
        # REMOVED: self.vision = Vision_control() # <--- NOW CORRECTLY REMOVED
        self._tcp_speed = 50
        self.is_alive = True


    ## considered removing, believe that x_move=0 may be native to the xarm lib, will do if it is
    def center_x_y_general(self, dx, dy, ratio, angle, adder_1, adder_2, threshold=5):
        if abs(dx) > threshold:
            x_move = dx*ratio
        else:
            x_move = 0

        if abs(dy) > threshold:
            y_move = -dy*ratio
        else:
            y_move = 0

        if abs(angle) > threshold:
            yaw_move = -angle
        else:
            yaw_move = 0

        # FIX: Call on movement instance
        self.movement.move_wherever(x_move, y_move + adder_1, -250 + adder_2,0,0, yaw_move)
        time.sleep(0.5)

    def center_x_y_precise(self, dx, dy, ratio, angle, threshold=2):
        if abs(dx) > threshold:
            x_move = dx*ratio
        else:
            x_move = 0

        if abs(dy) > threshold:
            y_move = -dy*ratio
        else:
            y_move = 0

        if abs(angle) > threshold:
            yaw_move = -angle
        else:
            yaw_move = 0

        # FIX: Call on movement instance
        self.movement.move_wherever(x_move, y_move+50, 0,0,0, yaw_move)
        time.sleep(0.5)

    def center_x_y_old(self, dx, dy, threshold = 1):
        if abs(dx) > threshold:
            x_move = dx * .1
        else:
            x_move = 0

        if abs(dy) > threshold:
            y_move = -dy * .1
        else:
            y_move = 0

        # FIX: Call on movement instance
        self.movement.move_wherever(x_move, y_move, 0,0,0, 0)

    #picks up lego and moves it right for now
    ## tim here, should DEFINITELY leave this one!
    def pick_and_place(self, angle_passed, pad_center, pad_angle, pad_pixel_width, frame_center, lego_number):
        Movement.pprint("Starting pick and place sequence...") # FIX: Call Movement.pprint

        # Step 3: Move above the landing pad using visual dx/dy
        dx_pad = pad_center[0] - frame_center[0]
        dy_pad = pad_center[1] - frame_center[1]
        #ratio not self adjusting yet
        # FIX: Call ratio function on movement instance
        ratio = self.movement.ratio_of_landing_pad_pixel_to_mm(pad_pixel_width)
        Movement.pprint(f"Landing pad ratio: {ratio:.2f}") # FIX: Call Movement.pprint

        Movement.pprint(f"Landing pad dx: {dx_pad:.2f}px, dy: {dy_pad:.2f}px, angle: {pad_angle:.2f}Â°") # FIX: Call Movement.pprint

        height_added_to_z=21 + (lego_number - 1) * 10 # was 18 and 9.6mm

        if abs(dx_pad) > 5 or abs(dy_pad) > 5 or abs(pad_angle) > 5:
            if pad_angle < 45: # This occurs when the pad is pointing away from the robot, to ensure the lego is placed horizontally
                self.center_x_y_general(dx_pad, dy_pad, ratio, pad_angle, 50,100)
            else: # This occurs when the pad is pointing toward the robot. Otherwise, the centering function does so vertically
                self.center_x_y_general(dx_pad, dy_pad, ratio, pad_angle-90, 50, 100)
            Movement.pprint("moved to landing pad ayyyy") # FIX: Call Movement.pprint

        # Step 4: Drop onto the pad
        self._arm.set_position(x = 206.8, z=253.9 + height_added_to_z, speed=self._tcp_speed, wait=True)
        self._tcp_speed = 20

        self._arm.set_position(x = 206.8, z=(203.9 + height_added_to_z), speed=self._tcp_speed, wait=True)
        # FIX: Call gripper/movement methods on the movement instance
        self.movement.open_gripper()
        self.movement.move_upz(20)
        self.movement.close_gripper_full()
        self.movement.move_downz(11)
        self.movement.shimmy()
        self.movement.move_downz(4)
        self.movement.move_upz(50)
        self.movement.open_gripper()

        self._tcp_speed = 65

        time.sleep(0.5)

        Movement.pprint("Pick and place done.") # FIX: Call Movement.pprint

    # REMOVED: def detect_blue_objects(self, cap): ... (because it was redundant)

    def get_all_color_contours(self, hsv_image):
        # Imports are now at the top of the file
        color_contours = {}
        for color_name, ranges in COLOR_RANGES.items():
            contours = get_mask_and_contours(hsv_image, ranges)
            color_contours[color_name] = contours


        return color_contours

    def run_precise(self, cap, target_color):
        if not cap.isOpened():
            Movement.pprint("Error: Could not open camera.") # FIX: Call Movement.pprint
            return

        dx = dy = float('inf')  # initialize to large values
        Movement.pprint("Camera started. Press 'q' to quit.") # FIX: Call Movement.pprint
        Movement.pprint("I'm losing my mind after not seeing a lego") # FIX: Call Movement.pprint

        while self.is_alive:
            ret, frame = cap.read()
            if not ret:
                break

            height, width = frame.shape[:2]
            frame_center = (width // 2, height // 2)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            color_contours = self.get_all_color_contours(hsv)
            found_valid_contour = False  # track if we processed anything

            for target_color, contours in color_contours.items():
                if contours:
                    for contour in contours:
                        if cv2.contourArea(contour) < 500:
                            continue

                        rect = cv2.minAreaRect(contour)
                        box = cv2.boxPoints(rect).astype(int)
                        cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

                        (center_x, center_y), (w, h), angle = rect
                        box_center = (int(center_x), int(center_y))
                        dx = box_center[0] - frame_center[0]
                        dy = box_center[1] - frame_center[1]

                        pixel_length = max(w, h)
                        pixel_width = min(w, h)

                        Movement.pprint( # FIX: Call Movement.pprint
                            f"(Precision) Lego Center: {box_center} | dx: {dx}px, dy: {dy}px | length: {pixel_length:.1f}px, width: {pixel_width:.1f}px"
                        )

                        # Draw visuals
                        cv2.circle(frame, box_center, 5, (0, 0, 255), -1)
                        cv2.putText(frame, f"dx: {dx}px, dy: {dy}px", (box_center[0] - 60, box_center[1] + 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        cv2.circle(frame, frame_center, 5, (255, 255, 255), -1)
                        cv2.imshow("Camera Frame Precision", frame)
                        cv2.waitKey(1)

                        found_valid_contour = True

                        if abs(dx) > 1 or abs(dy) > 1:
                            self.center_x_y_old(dx, dy)
                            Movement.pprint("ran") # FIX: Call Movement.pprint
                        else:
                            Movement.pprint("dx and dy small enough â€” exiting.") # FIX: Call Movement.pprint
                            cv2.destroyWindow("Camera Frame Precision")
                            return

                        break  # only process one contour at a time
                    if found_valid_contour:
                        break

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        cv2.destroyWindow("Camera Frame Precision")

    def find_lego(self, color_name, contour, frame, frame_center):
        # Use minAreaRect for rotated boxes
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect).astype(int)
        cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

        cv2.circle(frame, frame_center, 5, (255, 255, 255), -1)
        cv2.imshow("Camera Frame 1", frame)
        cv2.waitKey(1)

        # Unpack values
        (center_x, center_y), (w, h), angle = rect
        box_center = (int(center_x), int(center_y))
        dx = box_center[0] - frame_center[0]
        dy = box_center[1] - frame_center[1]

        # Sort dimensions
        pixel_length = max(w, h)
        pixel_width = min(w, h)

        vision_control.pprint(f"color_name is {color_name}")

        vision_control.pprint(
            f"Lego Center 1: {box_center} | dx: {dx}px, dy: {dy}px | length: {pixel_length:.1f}px, width: {pixel_width:.1f}px")

        self.center_x_y_general(dx, dy, movement.ratio_of_lego_pixel_to_mm(pixel_length), 0, 0, 0)
        time.sleep(1)
        cv2.destroyWindow("Camera Frame 1")
        return True


    # This function is the centering function for the pad placement, returns placement information for pick_and_place, 'None' if the function fails
    # This function will be made more sophisticated such that it is less dependent on vision angle
    # Other functions will need to be improved before this can take effect, however
    def center_on_pad(self, cap, lego_number, angle_passed, frame_center):
        self._arm.set_position(x=206, y=135.8, z=503.9, roll=180, pitch=0, yaw=-88.1, speed=self._tcp_speed,
                               wait=True)

        time.sleep(1)
        frame, contours_blue, _ = vision_control.detect_blue_objects(cap)
        # Detect Blue Landing Pad (used as destination)
        pad_center = None
        pad_angle = 0
        pad_pixel_width = 0

        for pad_contour in contours_blue:
            area = cv2.contourArea(pad_contour)
            if area > 1500:
                rect_pad = cv2.minAreaRect(pad_contour)
                (pad_x, pad_y), (w_pad, h_pad), pad_angle = rect_pad
                pad_center = (pad_x, pad_y)
                pad_pixel_width = min(w_pad, h_pad)
                # self.center_x_y_precise(pad_x, pad_y, movement.ratio_of_lego_pixel_to_mm(pad_pixel_width),
                #                       pad_angle)
                vision_control.pprint(
                    f"Landing pad center: ({pad_center[0]:.2f}, {pad_center[1]:.2f}), angle: {pad_angle:.2f}, pixel length: {pad_pixel_width:.2f}")
                box_pad = cv2.boxPoints(rect_pad)
                box_pad = box_pad.astype(int)
                cv2.drawContours(frame, [box_pad], 0, (255, 0, 0), 2)
                cv2.circle(frame, (int(pad_x), int(pad_y)), 5, (255, 0, 255), -1)

                cv2.imshow("Landing Pad", frame)
                cv2.waitKey(1)
                time.sleep(1)
                # self.center_x_y_general()
                break  # only use first large one found

        lego_number += 1
        vision_control.pprint(f"Lego number: {lego_number}")

        if pad_center:
            return pad_center, pad_angle, pad_pixel_width
        else:
            vision_control.pprint("No landing pad found â€” skipping placement.")
            return None

    def search_colors(self, color_contours, cap, frame, frame_center, lego_number):
        for color_name, contours in color_contours.items():
            for contour in contours:
                if cv2.contourArea(contour) < 500 or cv2.contourArea(contour) > 1000:
                    continue

                # Sets the arm to picture taking position.  Does not take picture here.
                self._arm.set_position(x=-72.9, y=259.5, z=603.9, roll=180, pitch=0, yaw=-88.1,
                                       speed=self._tcp_speed, wait=True)

                # Finds rough location of current lego
                found_brick = self.find_lego(color_name, contour, frame, frame_center)

                self.run_precise(cap, color_name)
                time.sleep(1)
                vision_control.pprint("Ran self.run_precise()")

                # Flush buffer and get a clean new frame
                for _ in range(5):
                    cap.read()
                    time.sleep(0.05)

                ret, frame = cap.read()
                if not ret:
                    vision_control.pprint("Failed to grab frame after run_precise().")
                    break

                height, width = frame.shape[:2]
                frame_center = (width // 2, height // 2)

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                color_contours = self.get_all_color_contours(hsv)
                vision_control.pprint(color_name)
                vision_control.pprint("gets here 1")
                for new_contour in color_contours[color_name]:
                    if cv2.contourArea(new_contour) < 500 or cv2.contourArea(contour) > 1000:
                        continue
                    vision_control.pprint("gets here 2")
                    rect = cv2.minAreaRect(new_contour)
                    box = cv2.boxPoints(rect).astype(int)
                    cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

                    # Unpack values
                    (center_x, center_y), (w, h), angle = rect
                    box_center = (int(center_x), int(center_y))
                    dx = box_center[0] - frame_center[0]
                    dy = box_center[1] - frame_center[1]

                    # Sort dimensions
                    pixel_length = max(w, h)
                    pixel_width = min(w, h)

                    vision_control.pprint(
                        f"Lego Center 2: {box_center} | dx: {dx}px, dy: {dy}px | length: {pixel_length:.1f}px, width: {pixel_width:.1f}px")

                    angle_passed = vision_control.measure_lego_angle(new_contour)

                    cv2.circle(frame, box_center, 5, (0, 0, 255), -1)
                    cv2.putText(frame, f"dx: {dx}px, dy: {dy}px, angle: {angle_passed:.1f} deg",
                                (box_center[0] - 120, box_center[1] + 45),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    cv2.circle(frame, frame_center, 5, (255, 255, 255), -1)
                    cv2.imshow("Camera Frame 2", frame)
                    cv2.waitKey(1)

                    vision_control.pprint("is it getting here")

                    self.center_x_y_precise(dx, dy, movement.ratio_of_lego_pixel_to_mm(pixel_length), angle_passed)

                    movement.move_downz(150)
                    movement.close_gripper()

                    # saves pick_and_place location information based on center on pad
                    pad_center, pad_angle, pad_pixel_width = self.center_on_pad(cap, lego_number, angle_passed, frame_center)

                    # If center_on_pad found a location, pick_and_place is called to actually place down the lego
                    if (pad_center, pad_angle, pad_pixel_width):
                        self.pick_and_place(angle_passed, pad_center, pad_angle, pad_pixel_width, frame_center,
                                        lego_number)
                    break

                break

    def run(self):
        # z absolute distance to picking up range is 203.9mm
        self._arm.set_position(x=-72.9, y=259.5, z=603.9, roll=180, pitch=0, yaw=-88.1, speed=self._tcp_speed,
                               wait=True)
        movement.open_gripper()
        # camera is set to 640x480 pixels by default
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            vision_control.pprint("Error: Could not open camera.")
            return

        vision_control.pprint("Camera started. Press 'q' to quit.")

        lego_number = 0

        while self.is_alive:
            ret, frame = cap.read()
            if not ret:
                break

            height, width = frame.shape[:2]
            frame_center = (width // 2, height // 2)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            color_contours = self.get_all_color_contours(hsv)
            found_brick = False

            vision_control.pprint("searching all of the contours")

            # New function that searches through the color contours and builds the tower
            self.search_colors(color_contours, cap, frame, frame_center, lego_number)

            ##if cv2.waitKey(1) & 0xFF == ord('q'):
              ##  break
            break

        cap.release()
        cv2.destroyAllWindows()
        self._arm.release_error_warn_changed_callback(movement._error_warn_changed_callback)
        self._arm.release_state_changed_callback(movement._state_changed_callback)
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(movement._count_changed_callback)

            # The following callbacks were removed as they were not defined in RobotMain:
            # self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
            # self._arm.release_state_changed_callback(self._state_changed_callback)
            # if hasattr(self._arm, 'release_count_changed_callback'):
            #         self._arm.release_count_changed_callback(self._count_changed_callback)
if __name__ == '__main__':
    Movement.pprint('xArm-Python-SDK Version:{}'.format(version.__version__)) # FIX: Call Movement.pprint
    robot_main = RobotMain(arm)
    robot_main.run()