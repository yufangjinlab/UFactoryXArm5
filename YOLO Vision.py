from ultralytics import YOLO
import cv2

model=YOLO('All Colors All Distances(300e).pt')

cap=cv2.VideoCapture(0)



while True:
    _, frame=cap.read()

    results=model(frame)

    for result in results:
        a_frame=result.plot()
        for box in result.obb:
            #print(box.xywhr[0,0],box.xywhr[0,1])
            cv2.circle(a_frame,(int(box.xywhr[0,0]),int(box.xywhr[0,1])), 2, (0, 0, 255), -1)


    cv2.imshow('Regular Video',a_frame)

    if cv2.waitKey(1) == ord('q'):
            break

cap.release()