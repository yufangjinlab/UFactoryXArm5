import math
import numpy as np
import time
from movement import Movement

class PadManager:

    # There are about 5mm between the centers of lego studs. This will be taken into account for the pad manager
    # pad_manager allows for the abstraction of where the lego is placed on the pad, so that the only required input is the stud differential from the center
    # Before placement, arm is assumed to be centered over the pad and in line with the angle, which is a quantity passed to the pad_manager object
    # The pad_manager will keep track of the height of legos at each location of the pad so that they do not collide
    # The pad_manager will not account for 180 degree rotations of the pad once placement of legos has begun

    def __init__(self, width, length, angle, movement):
        self.angle = None
        self.width = None
        self.length = None
        self.center = None
        self.current_lego_width = None
        self.current_lego_length = None
        self.current_lego_orientation = None
        self.set_width_and_length(width, length)
        self.set_angle(angle)
        self.movement = movement
        self.lego_heights = np.zeros([width, length], dtype=int)
        self.set_current_lego(0, 0, "short-ways")

    # Sets the angle that the pad is at currently
    def set_angle(self, angle):
        self.angle = angle

    # Width is considered to be the short side of the standard pad (6 studs)
    def set_width_and_length(self, width, length):
        self.width = width
        self.length = length
        self.center = [width // 2, length // 2]

    # Sets the dimensions of the current lego based on input
    # Width is considered to be the direction from which the grabber grabs the legos
    def set_current_lego(self, width, length, orientation):
        self.current_lego_width = width
        self.current_lego_length = length
        self.current_lego_orientation = orientation

    # Angle logic of move functions will need to be modified based on the angle identification function of the CV library
    # In a given scenario of the pad tilted towards the robot from the landing strip, this should work
    # This moves the grabber the given number of studs in the y direction
    def move_pad_y(self, amount):
        x_movement = amount * math.sin(self.angle) * 8
        y_movement = amount * math.cos(self.angle) * 8
        self.movement.move_wherever(x_movement, y_movement, 0, 0, 0, 0)  # places the arm based on the calculated values

    # Moves the grabber the given number of studs in the x direction
    def move_pad_x(self, amount):
        x_movement = amount * math.sin(90 - self.angle) * 8
        y_movement = -amount * math.cos(90 - self.angle) * 8
        self.movement.move_wherever(x_movement, y_movement, 0, 0, 0, 0)

    # Coordinates are based in the center of the pad
    def set_coordinates(self, x_coord, y_coord):
        self.move_pad_y(y_coord)
        self.move_pad_x(x_coord)

        # This checks that the location given would place the lego in the proper coordinates, and if so, places the lego
        pad_height = self.get_potential_location_height(x_coord + self.center[0], y_coord + self.center[1])
        if pad_height >= 0:
            self.place_lego(pad_height)
            self.set_height_values(x_coord + self.center[0], y_coord + self.center[1], pad_height)
            return True
        else:
            return False

    # Modifies the array that keeps track of the heights of the legos above every stud on the pad
    def set_height_values(self, x_coord, y_coord, pad_height):
        # Keeps track of the distance from the center in terms of studs the edges of the legos are
        lego_width_offset = self.current_lego_width // 2
        lego_length_offset = self.current_lego_length // 2

        # Loops through the studs on the pad that would be affected by the placement of the lego and changes the heights
        # An exception is caught if the coordinates go out of bounds
        for i in range(self.current_lego_width):
            for j in range(self.current_lego_length):
                try:
                    if self.current_lego_orientation == "short-ways":
                        self.lego_heights[x_coord - lego_width_offset + i, y_coord - lego_length_offset + j] = pad_height + 1
                    else:
                        self.lego_heights[x_coord - lego_length_offset + j, y_coord - lego_width_offset + i] = pad_height + 1
                except Exception as err:
                    self.movement.pprint("Coordinates are invalid")
                    return False
        return True

    # Checks the given coordinate location to see if it is a valid placement for the lego
    def get_potential_location_height(self, x_coord, y_coord):
        # Test height, height area, and offsets are recorded to ensure that the location affected contains a stable landing surface
        test_height = self.lego_heights[x_coord, y_coord]
        test_height_area = 0
        lego_width_offset = self.current_lego_width // 2
        lego_length_offset = self.current_lego_length // 2

        # Loops through the affected area, checking the height against the test height, which is always set to the largest magnitude height detected
        # If the coordinates are invalid, an exception is thrown
        for i in range(self.current_lego_width):
            for j in range(self.current_lego_length):
                try:
                    if self.current_lego_orientation == "short-ways":
                        if self.lego_heights[x_coord - lego_width_offset + i, y_coord - lego_length_offset + j] > test_height:
                            test_height = self.lego_heights[x_coord - 2 + i, y_coord - 1 + j]
                            test_height_area = 0
                        elif self.lego_heights[x_coord - lego_width_offset + i, y_coord - lego_length_offset + j] == test_height:
                            test_height_area = test_height_area + 1
                    else:
                        if self.lego_heights[
                            x_coord - lego_length_offset + j, y_coord - lego_width_offset + i] > test_height:
                            test_height = self.lego_heights[x_coord - 1 + j, y_coord - 2 + i]
                            test_height_area = 0
                        elif self.lego_heights[
                            x_coord - lego_length_offset + j, y_coord - lego_width_offset + i] == test_height:
                            test_height_area = test_height_area + 1
                except:
                    self.movement.pprint("The coordinates provided are invalid")
                    return -1

        # If the total area of the test height, which is the largest height value present, is greater than or equal to half the area of the lego, then placement can proceed
        # Otherwise, the landing area is too unstable
        if test_height_area >= (self.current_lego_width * self.current_lego_length) / 2:
            return test_height
        else:
            return -1

    def place_lego(self, pad_height):
        # Identifies the additional height needed to place the lego in the correct spot
        self.movement.pprint(pad_height)
        height_added_to_z = 21 + (pad_height) * 9.6  # was 18 and 9.6mm

        # Lowers the grabber over the pad and slows down the robot
        self.movement.move_downz(150 - height_added_to_z)
        # self._arm.set_position(z=253.9 + height_added_to_z, speed=self._tcp_speed, wait=True)
        self.movement.set_tcp_speed(20)

        # Lowers the grabber to the correct spot and releases the lego
        self.movement.move_downz(50)
        # self._arm.set_position(z=(203.9 + height_added_to_z), speed=self._tcp_speed, wait=True)
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

        self.movement.set_tcp_speed(65)

        time.sleep(0.5)

        Movement.pprint("Lego placement done.")