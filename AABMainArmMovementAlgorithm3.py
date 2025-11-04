import cv2
import numpy as np
import math
import time
import traceback
from xarm import version
from xarm.wrapper import XArmAPI
from vision_utils import get_mask_and_contours, COLOR_RANGES


class RobotMain(object):
    def __init__(self, robot):
        self.alive = True
        self._arm = robot
        self._tcp_speed = 80
        self._arm.set_gripper_speed(2000)
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500
        self._variables = {}
        self._robot_init()
        self._last_move_time = 0
        self._move_interval = 0.1  # min time between commands in seconds

    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'register_count_changed_callback'):
            self._arm.register_count_changed_callback(self._count_changed_callback)

    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    def _state_changed_callback(self, data):
        if data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    def _count_changed_callback(self, data):
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    #simple movement commands, amounts are in mm

    # for example self._arm.set_position(x=amount, y=yamount, z=zamount, yaw, pitch, roll...)
    # try new function
    def move_wherever(self, x_amount, y_amount, z_amount, roll_amount, pitch_amount, yaw_amount):
        self._arm.set_position(x=x_amount, y=y_amount, z=z_amount, r=roll_amount, p=pitch_amount, yaw=yaw_amount, relative=True, speed=self._tcp_speed, wait=True)

    def move_left(self, amount):
        self._arm.set_position(x=-amount, relative=True, speed=self._tcp_speed, wait=True)

    def move_right(self, amount):
        self._arm.set_position(x=amount, relative=True, speed=self._tcp_speed, wait=True)

    def move_up(self, amount):
        self._arm.set_position(y=amount, relative=True, speed=self._tcp_speed, wait=True)

    def move_down(self, amount):
        self._arm.set_position(y=-amount, relative=True, speed=self._tcp_speed, wait=True)

    def move_downz(self, amount):
        self._arm.set_position(z=-amount, relative=True, speed=self._tcp_speed, wait=True)

    def move_upz(self, amount):
        self._arm.set_position(z=amount, relative=True, speed=self._tcp_speed, wait=True)

    def rotate_joint_5(self, amount):
        self._arm.set_position(yaw=amount, relative=True, speed= self._tcp_speed)

    #simple open and close gripper commands
    def open_gripper(self):
        self._arm.set_gripper_mode(0)
        self._arm.set_gripper_enable(True)
        self._arm.set_gripper_position(600, wait=True)

    def close_gripper(self):
        self._arm.set_gripper_mode(0)
        self._arm.set_gripper_enable(True)
        self._arm.set_gripper_position(200, wait=True)

    def close_gripper_full(self):
        self._arm.set_gripper_mode(0)
        self._arm.set_gripper_enable(True)
        self._arm.set_gripper_position(0, wait=True)

    #feeds ratio to center_robotxy for single-movement
    def ratio_of_lego_pixel_to_mm(self, pixel_length, real_length_mm=31.8):
        ratio = real_length_mm/pixel_length
        self.pprint(f" Lego pixel length is {pixel_length:.2f} pixels and ratio is {ratio:.4f}")
        return ratio

    def ratio_of_landing_pad_pixel_to_mm(self, pixel_width, real_width_mm=47.7):#47.7mm is width, 127.2mm is length
        ratio = real_width_mm/pixel_width
        self.pprint(f" Landing pad pixel width is {pixel_width:.2f} pixels and ratio is {ratio:.4f}")
        return ratio

    def shimmy(self):
        self.move_wherever(4,4,2,0,0,2)
        self.move_wherever(-6,-6,-3,0,0,-2)
        self.move_wherever(2,-2,-1,0,0,-1)
        self.move_wherever(-2,2,1,0,0,1)
        self.move_wherever(3,4, 1,0,0,0)

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

        self.move_wherever(x_move, y_move + adder_1, -250 + adder_2,0,0, yaw_move)
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

        self.move_wherever(x_move, y_move+50, 0,0,0, yaw_move)
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

        self.move_wherever(x_move, y_move, 0,0,0, 0)


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

    #picks up lego and moves it right for now
    def pick_and_place(self, angle_passed, pad_center, pad_angle, pad_pixel_width, frame_center, lego_number):
        self.pprint("Starting pick and place sequence...")




            # Step 3: Move above the landing pad using visual dx/dy
        dx_pad = pad_center[0] - frame_center[0]
        dy_pad = pad_center[1] - frame_center[1]
        #ratio not self adjusting yet
        ratio = self.ratio_of_landing_pad_pixel_to_mm(pad_pixel_width)
        self.pprint(f"Landing pad ratio: {ratio:.2f}")

        self.pprint(f"Landing pad dx: {dx_pad:.2f}px, dy: {dy_pad:.2f}px, angle: {pad_angle:.2f}Â°")

        height_added_to_z=21 + (lego_number - 1) * 10 # was 18 and 9.6mm

        if abs(dx_pad) > 5 or abs(dy_pad) > 5 or abs(pad_angle) > 5:
            if pad_angle < 45: # This occurs when the pad is pointing away from the robot, to ensure the lego is placed horizontally
                self.center_x_y_general(dx_pad, dy_pad, ratio, pad_angle, 50,100)
            else: # This occurs when the pad is pointing toward the robot. Otherwise, the centering function does so vertically
                self.center_x_y_general(dx_pad, dy_pad, ratio, pad_angle-90, 50, 100)
            self.pprint("moved to landing pad ayyyy")

        # Step 4: Drop onto the pad
        self._arm.set_position(x = 206.8, z=253.9 + height_added_to_z, speed=self._tcp_speed, wait=True)
        self._tcp_speed = 20

        self._arm.set_position(x = 206.8, z=(203.9 + height_added_to_z), speed=self._tcp_speed, wait=True)
        self.open_gripper()
        self.move_upz(20)
        self.close_gripper_full()
        self.move_downz(11)
        self.shimmy()
        self.move_downz(4)
        self.move_upz(50)
        self.open_gripper()

        self._tcp_speed = 65

        time.sleep(0.5)

        self.pprint("Pick and place done.")

    def detect_blue_objects(self, cap):
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

    def get_all_color_contours(self, hsv_image):
        from vision_utils import COLOR_RANGES, get_mask_and_contours

        color_contours = {}
        for color_name, ranges in COLOR_RANGES.items():
            contours = get_mask_and_contours(hsv_image, ranges)
            color_contours[color_name] = contours


        return color_contours

    def run_precise(self, cap, target_color):
        if not cap.isOpened():
            self.pprint("Error: Could not open camera.")
            return

        dx = dy = float('inf')  # initialize to large values
        self.pprint("Camera started. Press 'q' to quit.")
        self.pprint("I'm losing my mind after not seeing a lego")

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

                        self.pprint(
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
                            self.pprint("ran")
                        else:
                            self.pprint("dx and dy small enough â€” exiting.")
                            cv2.destroyWindow("Camera Frame Precision")
                            return

                        break  # only process one contour at a time
                    if found_valid_contour:
                        break

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        cv2.destroyWindow("Camera Frame Precision")





    def run(self):
        # z absolute distance to picking up range is 203.9mm
        self._arm.set_position( x=-72.9, y=259.5, z=603.9, roll=180, pitch=0, yaw=-88.1, speed=self._tcp_speed, wait=True)
        self.open_gripper()
        #camera is set to 640x480 pixels by default
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.pprint("Error: Could not open camera.")
            return

        self.pprint("Camera started. Press 'q' to quit.")

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

            self.pprint("searching all of the contours")
            for color_name, contours in color_contours.items():
                for contour in contours:
                    if cv2.contourArea(contour) < 500:
                        continue
                    self._arm.set_position(x=-72.9, y=259.5, z=603.9, roll=180, pitch=0, yaw=-88.1,
                                           speed=self._tcp_speed, wait=True)

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

                    self.pprint(f"color_name is {color_name}")

                    self.pprint(
                        f"Lego Center 1: {box_center} | dx: {dx}px, dy: {dy}px | length: {pixel_length:.1f}px, width: {pixel_width:.1f}px")


                    self.center_x_y_general(dx, dy, self.ratio_of_lego_pixel_to_mm(pixel_length), 0, 0,0)
                    time.sleep(1)
                    cv2.destroyWindow("Camera Frame 1")

                    found_brick = True
                    self.run_precise(cap, color_name)
                    time.sleep(1)
                    self.pprint("Ran self.run_precise()")

                    # Flush buffer and get a clean new frame
                    for _ in range(5):
                        cap.read()
                        time.sleep(0.05)

                    ret, frame = cap.read()
                    if not ret:
                        self.pprint("Failed to grab frame after run_precise().")
                        break

                    height, width = frame.shape[:2]
                    frame_center = (width // 2, height // 2)


                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    color_contours = self.get_all_color_contours(hsv)
                    self.pprint(color_name)
                    self.pprint("gets here 1")
                    for new_contour in color_contours[color_name]:
                        if cv2.contourArea(new_contour) < 500:
                            continue
                        self.pprint("gets here 2")
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

                        self.pprint(
                            f"Lego Center 2: {box_center} | dx: {dx}px, dy: {dy}px | length: {pixel_length:.1f}px, width: {pixel_width:.1f}px")

                        angle_passed = self.measure_lego_angle(new_contour)

                        cv2.circle(frame, box_center, 5, (0, 0, 255), -1)
                        cv2.putText(frame, f"dx: {dx}px, dy: {dy}px, angle: {angle_passed:.1f} deg", (box_center[0] - 120, box_center[1] + 45),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                        cv2.circle(frame, frame_center, 5, (255, 255, 255), -1)
                        cv2.imshow("Camera Frame 2", frame)
                        cv2.waitKey(1)

                        self.pprint("is it getting here")

                        self.center_x_y_precise(dx,dy,self.ratio_of_lego_pixel_to_mm(pixel_length),angle_passed)

                        self.move_downz(150)
                        self.close_gripper()


                        self._arm.set_position(x=206, y=135.8, z=503.9, roll=180, pitch=0, yaw=-88.1, speed=self._tcp_speed,
                                               wait=True)

                        time.sleep(1)
                        frame, contours_blue, _ = self.detect_blue_objects(cap)
                        # Detect Blue Landing Pad (used as destination)
                        pad_center = None
                        pad_angle = 0
                        pad_pixel_width = 0

                        for pad_contour in contours_blue:
                            area=cv2.contourArea(pad_contour)
                            if area > 1500:
                                rect_pad = cv2.minAreaRect(pad_contour)
                                (pad_x, pad_y), (w_pad, h_pad), pad_angle = rect_pad
                                pad_center = (pad_x, pad_y)
                                pad_pixel_width = min(w_pad, h_pad)
                                self.pprint(
                                    f"Landing pad center: ({pad_center[0]:.2f}, {pad_center[1]:.2f}), angle: {pad_angle:.2f}, pixel length: {pad_pixel_width:.2f}")
                                box_pad = cv2.boxPoints(rect_pad)
                                box_pad = box_pad.astype(int)
                                cv2.drawContours(frame, [box_pad], 0, (255, 0, 0), 2)
                                cv2.circle(frame, (int(pad_x), int(pad_y)), 5, (255, 0, 255), -1)

                                cv2.imshow("Landing Pad", frame)
                                cv2.waitKey(1)
                                time.sleep(1)
                                #self.center_x_y_general()
                                break  # only use first large one found

                        lego_number += 1
                        self.pprint(f"Lego number: {lego_number}")

                        if pad_center:
                            self.pick_and_place(angle_passed, pad_center, pad_angle, pad_pixel_width, frame_center, lego_number)
                        else:
                            self.pprint("No landing pad found â€” skipping placement.")

                        break
                    break




            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            break

        cap.release()
        cv2.destroyAllWindows()
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)


if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.207', baud_checkset=False)
    robot_main = RobotMain(arm)
    robot_main.run()