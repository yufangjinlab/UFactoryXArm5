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

    # Moves the camera a decided amount based on pixel to mm ratio and lowers the grabber
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

    # Moves the camera a decided amount based on pixel to mm ratio, and moves the grabber to previous camera location
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
        self.movement.move_wherever(x_move+6, y_move+53, 0,0,0, yaw_move)
        time.sleep(0.5)

    # Moves the camera a small amount in the given direction based on pixel length only
    def center_x_y_simple(self, dx, dy, threshold = 1):
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

    # This function centers precisely on the contour, bringing down disparity between the centers of the contour and camera as low as possible
    def dynamic_center_increment(self, contour, cap):
        # Uses the frame from the current position to identify the location of the lego
        if not cap.isOpened():
            Movement.pprint("Error: Could not open camera.")
            return
        _, frame = cap.read()

        # Unpacks the rect values for later use
        box_center, dx, dy, w, h, _ = vision_control.unpack_rect(contour, frame)

        # Sorts the pixel differences so they can be displayed
        pixel_length = max(w, h)
        pixel_width = min(w, h)

        # Displays lego position information in relation to the camera
        Movement.pprint(
            f"(Precision) Lego Center: {box_center} | dx: {dx}px, dy: {dy}px | length: {pixel_length:.1f}px, width: {pixel_width:.1f}px"
        )

        # Draws visuals on the screen
        frame_center = vision_control.get_frame_center(frame)
        cv2.circle(frame, box_center, 5, (0, 0, 255), -1)
        cv2.putText(frame, f"dx: {dx}px, dy: {dy}px", (box_center[0] - 60, box_center[1] + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.circle(frame, frame_center, 5, (255, 255, 255), -1)
        cv2.imshow("Camera Frame Precision", frame)
        cv2.waitKey(1)

        # When the dx and dy values are too large, robot recenters and returns False so outer function knows it is not ready yet
        if abs(dx) > 1 or abs(dy) > 1:
            self.center_x_y_simple(dx, dy)
            return False
        # When the dx and dy values are small enough, it exits the function and returns True so the outer program can move on
        else:
            Movement.pprint("dx and dy small enough â€” exiting.")  # FIX: Call Movement.pprint
            cv2.destroyWindow("Camera Frame Precision")
            return True

    # Places the lego on the landing pad in the center of the pad.
    def place_lego(self, lego_number):
        # Identifies the additional height needed to place the lego in the correct spot
        height_added_to_z = 21 + (lego_number - 1) * 10  # was 18 and 9.6mm

        # Lowers the grabber over the pad and slows down the robot
        self._arm.set_position(z=253.9 + height_added_to_z, speed=self._tcp_speed, wait=True)
        self._tcp_speed = 20

        # Lowers the grabber to the correct spot and releases the lego
        self._arm.set_position(z=(203.9 + height_added_to_z), speed=self._tcp_speed, wait=True)
        self.movement.open_gripper()

        # Puts the grabber in position to shimmy
        self.movement.move_upz(20)
        self.movement.close_gripper_full()
        self.movement.move_downz(11)

        # Runs shimmy to move the lego into place
        self.movement.shimmy()

        # Pushes the lego down and raises the grabber up to reset
        self.movement.move_downz(4)
        self.movement.move_upz(50)
        self.movement.open_gripper()

        self._tcp_speed = 65

        time.sleep(0.5)

        Movement.pprint("Lego placement done.")

    # Returns a list of all the contours in the image
    def get_all_color_contours(self, hsv_image):
        # Imports are now at the top of the file
        color_contours = {}
        for color_name, ranges in COLOR_RANGES.items():
            contours = get_mask_and_contours(hsv_image, ranges)
            color_contours[color_name] = contours


        return color_contours

    # Moves the camera so that it is directly over the lego it is looking to place on the pad by color
    def run_precise(self, cap, target_color):
        # Checks to ensure that the camera is functional
        if not cap.isOpened():
            Movement.pprint("Error: Could not open camera.")
            return

        dx = dy = float("inf")
        Movement.pprint("Camera started. Press 'q' to quit.") # FIX: Call Movement.pprint

        # This loop will run the centering increment over the lego in question until it is found
        while self.is_alive:
            # Opens the current frame and ensures it is saved in the correct format
            ret, frame = cap.read()
            if not ret:
                break
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Retrieves a list of all contours in the current frame
            color_contours = self.get_all_color_contours(hsv)
            found_valid_contour = False  # track if we processed anything

            # This loop looks through each contour for all colors so that they can be centered on
            for color, contours in color_contours.items():
                # Only checks the target color given by the outer program
                if color != target_color:
                    continue
                # This loop checks each contour of the chosen color and centers on it
                for contour in contours:
                    # This check prevents the robot from focusing on contours that are not legos
                    if cv2.contourArea(contour) < 500:
                        continue
                    # This check runs the incremented centering function and uses the returned value to decide to continue looping or to return
                    movement.pprint("Calling dynamic center")
                    if self.dynamic_center_increment(contour, cap):
                        return

                    # This loops breaks so that only 1 lego is centered on, and since a lego was found, we hold on to that value
                    found_valid_contour = True
                    break
                    # If a contour that can be centered on was found, we exit out of the contour loop
                    # The code would then return to the while loop so that it can find the contour and frame again
                if found_valid_contour:
                    break

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        cv2.destroyWindow("Camera Frame Precision")

    def center_grabber(self, cap, color_name, orientation, lego_length):
        # Opens the camera and gets frame information
        ret, frame = cap.read()
        if not ret:
            vision_control.pprint("Failed to grab frame after run_precise().")
            return
        frame_center = vision_control.get_frame_center(frame)

        # Gets contour information from the frame
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color_contours = self.get_all_color_contours(hsv)
        vision_control.pprint(color_name)

        # Searches the contours in the list of contours of the given color
        for new_contour in color_contours[color_name]:
            # Disregards contours that are too small
            if cv2.contourArea(new_contour) < 500:
                continue

            # Unpacks rect information
            box_center, dx, dy, w, h, _ = vision_control.unpack_rect(new_contour, frame)
            # Sort dimensions
            pixel_length = max(w, h)
            pixel_width = min(w, h)

            # Prints out lego location information
            vision_control.pprint(
                f"Lego Center 2: {box_center} | dx: {dx}px, dy: {dy}px | length: {pixel_length:.1f}px, width: {pixel_width:.1f}px")

            # Detects the angle of the contour so it can be grabbed
            angle_passed = vision_control.measure_lego_angle(new_contour)

            # If the orientation of the grabber is short ways, it will modify the angle such that is places the lego in the correct orientation
            if orientation == "short-ways":
                if angle_passed < 0:
                    angle_passed += 90
                else:
                    angle_passed -= 90

            # Displays movement information on the screen so it can be identified
            cv2.circle(frame, box_center, 5, (0, 0, 255), -1)
            cv2.putText(frame, f"dx: {dx}px, dy: {dy}px, angle: {angle_passed:.1f} deg",
                        (box_center[0] - 120, box_center[1] + 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            cv2.circle(frame, frame_center, 5, (255, 255, 255), -1)
            cv2.imshow("Camera Frame 2", frame)
            cv2.waitKey(1)

            # Relocates the grabber in position based on lego location information found earlier, then returns so only one lego is centered on
            self.center_x_y_precise(dx, dy, movement.ratio_of_lego_pixel_to_mm(pixel_length, lego_length), angle_passed)
            return

    # This function places the camera above the lego that it is going to place next
    def find_lego(self, contour, cap):
        # Uses the camera to identify the lego position
        if not cap.isOpened():
            Movement.pprint("Error: Could not open camera.")
            return
        _, frame = cap.read()

        # Adds a circle to the frame at the center of the picture
        frame_center = vision_control.get_frame_center(frame)
        cv2.circle(frame, frame_center, 5, (255, 255, 255), -1)
        cv2.imshow("Camera Frame 1", frame)
        cv2.waitKey(1)

        # Unpack values from rect to identify centers and movement requirements
        box_center, dx, dy, w, h, _ = vision_control.unpack_rect(contour, frame)

        # Sort dimensions
        pixel_length = max(w, h)
        pixel_width = min(w, h)

        # Print lego placement before moving into position
        vision_control.pprint(
            f"Lego Center 1: {box_center} | dx: {dx}px, dy: {dy}px | length: {pixel_length:.1f}px, width: {pixel_width:.1f}px")

        # Roughly centers the camera over the lego
        self.center_x_y_general(dx, dy, movement.ratio_of_lego_pixel_to_mm(pixel_length), 0, 0, 0)
        time.sleep(1)
        cv2.destroyWindow("Camera Frame 1")
        return True

    # This function is the centering function for the pad placement, returns whether it was successful
    def center_on_pad(self, cap):
        # Sets the robot to the initial position for taking the picture
        self._arm.set_position(x=206, y=135.8, z=403.9, roll=180, pitch=0, yaw=-88.1, speed=self._tcp_speed,
                               wait=True)

        # Detect Blue Landing Pad (used as destination)
        time.sleep(1)
        frame, contours_blue, _ = vision_control.detect_blue_objects(cap)

        # Save position information so it can be displayed at the end
        pad_pixel_width = 0
        pad_contour = None
        dx_pad = 0
        dy_pad = 0
        pad_angle = 0

        # This loops looks through the blue contours to find the pad. Only the pad contour is used
        for contour in contours_blue:
            area = cv2.contourArea(contour)
            if area > 1500:
                # Takes position information from the contour rect and draws boxes
                pad_center, dx_pad, dy_pad, _, _, pad_angle = vision_control.unpack_rect(contour, frame)
                pad_contour = contour

                # Prints pad location information
                vision_control.pprint(
                    f"Landing pad center: ({pad_center[0]:.2f}, {pad_center[1]:.2f}), angle: {pad_angle:.2f}, pixel length: {pad_pixel_width:.2f}")

                cv2.imshow("Landing Pad", frame)
                cv2.waitKey(1)
                time.sleep(1)

                # Only uses the first large contour found
                break

        # Prints landing pad location information
        Movement.pprint(
            f"Landing pad dx: {dx_pad:.2f}px, dy: {dy_pad:.2f}px, angle: {pad_angle:.2f}Â°")

        # Centers the camera over the landing pad and then places the grabber in position
        self.run_precise(cap, "blue")
        self.center_grabber(cap, "blue", "short-ways", 47.7)

        # If the pad could be found, return True, otherwise, return False
        if pad_contour.any():
            return True
        else:
            vision_control.pprint("No landing pad found â€” skipping placement.")
            return False

    def place_all_legos(self, cap):
        # Reads the frame and formats it to find contours
        ret, frame = cap.read()
        if not ret:
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Finds a list of all the contours in the frame
        color_contours = self.get_all_color_contours(hsv)

        # Initializes the legos at zero so that heights can be adjusted
        lego_number = 0

        # Loops through each color and uses the contours to place the legos
        for color_name, contours in color_contours.items():
            for contour in contours:
                # Ignores contours that are too large or too small to be legos
                if cv2.contourArea(contour) < 500 or cv2.contourArea(contour) > 1000:
                    continue

                # Sets the arm to picture taking position.  Does not take picture at this point.
                self._arm.set_position(x=-72.9, y=259.5, z=603.9, roll=180, pitch=0, yaw=-88.1,
                                       speed=self._tcp_speed, wait=True)

                # Finds rough location of current lego and then centers on it
                found_lego = self.find_lego(contour, cap)
                self.run_precise(cap, color_name)
                time.sleep(1)
                vision_control.pprint("Ran self.run_precise()")

                # Flush buffer and get a clean new frame
                for _ in range(5):
                    cap.read()
                    time.sleep(0.05)

                # Centers the grabber over the lego where the camera was, oriented long-ways, then grabs the lego
                self.center_grabber(cap, color_name, "long-ways", 31.8)
                movement.move_downz(150)
                movement.close_gripper()

                # Saves whether the grabber was successfully centered on the pad
                pad_centered = self.center_on_pad(cap)

                # If the pad was successfully centered on, place_lego is called to actually place down the lego
                if pad_centered:
                    lego_number += 1
                    vision_control.pprint(f"Lego number: {lego_number}")
                    self.place_lego(lego_number)

    def run(self):
        # z absolute distance to picking up range is 203.9mm
        self._arm.set_position(x=-72.9, y=259.5, z=603.9, roll=180, pitch=0, yaw=-88.1, speed=self._tcp_speed,
                               wait=True)
        movement.open_gripper()
        # Initializes camera is set to 640x480 pixels by default
        cap = cv2.VideoCapture(0)

        # Closes program if camera is not functional
        if not cap.isOpened():
            vision_control.pprint("Error: Could not open camera.")
            return
        vision_control.pprint("Camera started. Press 'q' to quit.")

        # Goes through all the legos and places them in the tower
        self.place_all_legos(cap)

        # Cleanup that removes the camera, destroys windows, and prepares the robot
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