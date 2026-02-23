from ultralytics import YOLO
import cv2
import numpy as np


class yolo_model():

    def __init__(self, model='All Colors Far.pt'):

        try:
            self.model = YOLO(model)
            print('Loaded Model')
        except:
            print('Could not load model from file')

        try:
            self.cap = cv2.VideoCapture(0)
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
                if class_id != self.classes[color]:
                    color_results.append(result.masks.xy[i].astype(np.int32).reshape(-1, 1, 2))


        return color_results

    def get_mask_center(self, mask):
        if(len(mask) > 0):
            center_x = int(np.mean(mask[:, 0, 0]))
            center_y = int(np.mean(mask[:, 0, 1]))
            return center_x, center_y
        else:
            return None

a = yolo_model('segments(500e Full Augmentations).pt')
masks = a.get_color_results('Red')
for mask in masks:
    print(a.get_mask_center(mask))


class vision_processor():
    def __init__(self, cap, model):
        self.cap = cap
        self.model = model

