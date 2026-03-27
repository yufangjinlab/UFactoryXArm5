from ultralytics import YOLO
import cv2
import numpy as np






class yolo_model():
     
    def __init__(self,model='All Colors Far.pt'):

        try:
            
            self.model=YOLO(model)
            print('Loaded Model')
        except:
             print('Could not load model from file')
        
        try:
            self.cap=cv2.VideoCapture(0)
            print('Finished Camera Setup')
        except:
            print('Could not create VideoCapture object') 


    def frame_cap(self):
        _, self.frame=self.cap.read()
        self.results=self.model(self.frame)
        
        for result in self.results:
            self.a_frame=result.plot(font_size=.5,probs=False)
        
        cv2.imshow('Labeled Frame',self.a_frame)

        if (cv2.waitKey(0) == ord('q')) or (cv2.getWindowProperty('Labeled Frame', cv2.WND_PROP_VISIBLE) ==0):
            self.cap.release()


        
            return self.frame, self.a_frame

a=yolo_model('segments(500e Full Augmentations).pt')

while True:

    _,a.frame=a.cap.read()
    results=a.model(a.frame)

    for result in results:
        a_frame=result.plot()
        if result.masks is not None:
            for mask in result.masks:
                # Get the polygon coordinates (xy format)
                # mask.xy returns a list of arrays, each array is a set of polygon points for one object
                polygon = mask.xy[0].astype(np.int32)

                # Reshape to OpenCV contour format (required for drawContours and moments)
                contour = polygon.reshape(-1, 1, 2)

                # Create a binary mask for the current object (optional but useful for complex analysis)
                # b_mask = np.zeros(result.orig_img.shape[:2], np.uint8)
                # cv2.drawContours(b_mask, [contour], -1, (255), cv2.FILLED)

                # Calculate moments of the contour
                M = cv2.moments(contour)

                # Calculate the centroid (cx, cy) using moments
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    print(f"Centroid: ({cx}, {cy})")

                    # Optional: Draw the centroid on the image
                    cv2.circle(a_frame, (cx, cy), 5, (0, 0, 255), -1)

        
    cv2.imshow('Labeled Frame',a_frame)

    if (cv2.waitKey(1) == ord('q')) or (cv2.getWindowProperty('Labeled Frame', cv2.WND_PROP_VISIBLE) == 0):
        break

a.cap.release()
cv2.destroyAllWindows()