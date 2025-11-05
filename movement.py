import cv2
import numpy as np
import math
import time
import traceback
from xarm import version
from xarm.wrapper import XArmAPI

class Movement:
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
            self.pprint(
                '{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code, self._arm.connected,
                                                                                         self._arm.state,
                                                                                         self._arm.error_code, ret1, ret2))
        return self.is_alive


    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1],
                                       ' '.join(map(str, args))))
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


    # simple movement commands, amounts are in mm

    # for example self._arm.set_position(x=amount, y=yamount, z=zamount, yaw, pitch, roll...)
    # try new function
    def move_wherever(self, x_amount, y_amount, z_amount, roll_amount, pitch_amount, yaw_amount):
        self._arm.set_position(x=x_amount, y=y_amount, z=z_amount, r=roll_amount, p=pitch_amount, yaw=yaw_amount,
                               relative=True, speed=self._tcp_speed, wait=True)


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
        self._arm.set_position(yaw=amount, relative=True, speed=self._tcp_speed)


    # simple open and close gripper commands
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


    # feeds ratio to center_robotxy for single-movement
    def ratio_of_lego_pixel_to_mm(self, pixel_length, real_length_mm=31.8):
        ratio = real_length_mm / pixel_length
        self.pprint(f" Lego pixel length is {pixel_length:.2f} pixels and ratio is {ratio:.4f}")
        return ratio


    def ratio_of_landing_pad_pixel_to_mm(self, pixel_width, real_width_mm=47.7):  # 47.7mm is width, 127.2mm is length
        ratio = real_width_mm / pixel_width
        self.pprint(f" Landing pad pixel width is {pixel_width:.2f} pixels and ratio is {ratio:.4f}")
        return ratio


    def shimmy(self):
        self.move_wherever(4, 4, 2, 0, 0, 2)
        self.move_wherever(-6, -6, -3, 0, 0, -2)
        self.move_wherever(2, -2, -1, 0, 0, -1)
        self.move_wherever(-2, 2, 1, 0, 0, 1)
        self.move_wherever(3, 4, 1, 0, 0, 0)