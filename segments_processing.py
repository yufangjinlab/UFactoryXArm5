from ultralytics import YOLO
import cv2
import numpy as np
import math
import time


class yolo_model():
    def __init__(self, cap, model='All Colors Far.pt'):

        try:
            self.model = YOLO(model)
            print('Loaded Model')
        except:
            print('Could not load model from file')

        try:
            self.cap = cap
            print('Finished Camera Setup')
        except:
            print('Could not create VideoCapture object')

        self.classes = {
            'Blue' : 0,
            'Brown' : 1,
            'Gray' : 2,
            'Green' : 3,
            'Lime' : 4,
            'Pad' : 5,
            'Red' : 6,
            'Yellow' : 7}

    def get_frame(self):

        self.get_results()

        for result in self.results:
            self.a_frame = result.plot(font_size=.5, probs=False)

        cv2.imshow('Labeled Frame', self.a_frame)

        if (cv2.waitKey(0) == ord('q')) or (cv2.getWindowProperty('Labeled Frame', cv2.WND_PROP_VISIBLE) == 0):
            self.cap.release()

            return self.frame, self.a_frame

    def get_results(self):
        _, self.frame = self.cap.read()
        self.results = self.model(self.frame)

        return self.results

    def get_color_results(self, color):
        self.get_results()

        color_results = []

        for result in self.results:
            class_ids = result.boxes.cls.tolist()

            masks_data = result.masks.data.cpu().numpy()

            for i, mask in enumerate(masks_data):
                class_id = int(class_ids[i])
                if class_id == self.classes[color]:
                    color_results.append(result.masks.xy[i].astype(np.int32).reshape(-1, 1, 2))

        return color_results

    def get_mask_center(self, mask):
        if(len(mask) > 0):
            center_x = int(np.mean(mask[:, 0, 0]))
            center_y = int(np.mean(mask[:, 0, 1]))
            return center_x, center_y
        else:
            return None

def measure_lego_angle(mask):
   #Returns: angle in degrees (positive = counter-clockwise from horizontal)

    rect = cv2.minAreaRect(mask)
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

    print(f"LEGO long edge angle relative to horizontal: {best_angle:.1f}Â°")

    time.sleep(1)

    return best_angle

# This function returns the frame center array
def get_frame_center(frame):
    height, width = frame.shape[:2]
    frame_center = (width // 2, height // 2)
    return frame_center

def unpack_rect(mask, frame):
    # Uses a rect to find box size and location
    rect = cv2.minAreaRect(mask)
    box = cv2.boxPoints(rect).astype(int)
    cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

    # Unpacks rect information for use in centering
    (center_x, center_y), (w, h), angle = rect
    box_center = (int(center_x), int(center_y))
    frame_center = get_frame_center(frame)
    dx = float(box_center[0] - frame_center[0])
    dy = float(box_center[1] - frame_center[1])
    return box_center, dx, dy, w, h, angle
'''
a = yolo_model('segments(500e Full Augmentations).pt')
masks = a.get_color_results('Red')

cap = cv2.VideoCapture(0)
_, frame = cap.read()
print(unpack_rect(masks[0], frame)) '''