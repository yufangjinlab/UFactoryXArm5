import cv2

while True:
    cap = cv2.VideoCapture(0)
    _, frame = cap.read()
    # Load image in grayscale
    #img = cv2.imread(frame, cv2.IMREAD_GRAYSCALE)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Apply Gaussian Blur to reduce noise
    #blur = cv2.GaussianBlur(frame, (5, 5), 1.4)

    # Apply Canny Edge Detector
    #edges = cv2.Canny(blur, threshold1=100, threshold2=200)

    # Display result
    #cv2.imshow("Canny Edge Detection", edges)

    # Apply Laplacian operator
    #laplacian = cv2.Laplacian(frame, cv2.CV_64F)

    # Convert to uint8
    #laplacian_abs = cv2.convertScaleAbs(laplacian)

    # Display result
    #cv2.imshow("Laplacian Edge Detection", laplacian_abs)

    # Apply Sobel operator
    frame = cv2.bilateralFilter(frame,1, 25, 25)

    sobelx = cv2.Sobel(frame, cv2.CV_64F, 1, 0, ksize=3)  # Horizontal edges
    sobely = cv2.Sobel(frame, cv2.CV_64F, 0, 1, ksize=3)  # Vertical edges

    # Compute gradient magnitude
    gradient_magnitude = cv2.magnitude(sobelx, sobely)

    # Convert to uint8
    gradient_magnitude = cv2.convertScaleAbs(gradient_magnitude)

    # Display result
    cv2.imshow("Sobel Edge Detection", gradient_magnitude)

    cv2.waitKey(0)
    cv2.destroyAllWindows()