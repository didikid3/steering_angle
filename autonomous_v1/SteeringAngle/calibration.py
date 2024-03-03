import cv2
import numpy as np

frameHeight = 240
frameWidth = 480

cap = cv2.VideoCapture(1)
cap.set(3, frameWidth)
cap.set(4, frameHeight)

def empty(a):
    pass

cv2.namedWindow("HSV")
cv2.resizeWindow("HSV", frameWidth, frameHeight)

cv2.createTrackbar("Hue Min", "HSV", 0, 179, empty)
cv2.createTrackbar("Hue Max", "HSV", 179, 179, empty)
cv2.createTrackbar("Sat Min", "HSV", 0, 255, empty)
cv2.createTrackbar("Sat Max", "HSV", 255, 255, empty)
cv2.createTrackbar("Value Min", "HSV", 0, 255, empty)
cv2.createTrackbar("Value Max", "HSV", 255, 255, empty)

cap = cv2.VideoCapture('example1.mp4')
frameCounter = 0
while True:
    frameCounter += 1

    #Loop Video
    print(frameCounter, cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if cap.get(cv2.CAP_PROP_FRAME_COUNT) == frameCounter+10:
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        frameCounter = 0

    _, img = cap.read()
    img = cv2.resize(img, (480, 240))
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("Hue Min", "HSV")
    h_max = cv2.getTrackbarPos("Hue Max", "HSV")
    s_min = cv2.getTrackbarPos("Sat Min", "HSV")
    s_max = cv2.getTrackbarPos("Sat Max", "HSV")
    v_min = cv2.getTrackbarPos("Value Min", "HSV")
    v_max = cv2.getTrackbarPos("Value Max", "HSV")

    lowerThresh = np.array([h_min, s_min, v_min])
    upperThresh = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(imgHsv, lowerThresh, upperThresh)

    result = cv2.bitwise_and(img, img, mask=mask)

    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    hstack = np.hstack([img, mask, result])

    cv2.imshow('Horizontal', hstack)
    cv2.waitKey(1)
    if cv2.waitKey(10) and 0xFF == ord('q'):
        cv2.waitKey(0)


# import cv2
# import numpy as np
# from pal.products.qcar import QCar, QCarRealSense, Camera2D

# frameHeight = 240
# frameWidth = 480

# # camera_realsense_rgb = QCarRealSense(mode='RGB')
# imageWidth = 640
# imageHeight = 480
# myCam4 = Camera2D(cameraId="3", frameWidth=imageWidth, frameHeight=imageHeight)

# def empty(a):
#     pass

# cv2.namedWindow("HSV")
# cv2.resizeWindow("HSV", frameWidth, frameHeight)

# cv2.createTrackbar("Hue Min", "HSV", 0, 179, empty)
# cv2.createTrackbar("Hue Max", "HSV", 179, 179, empty)
# cv2.createTrackbar("Sat Min", "HSV", 0, 255, empty)
# cv2.createTrackbar("Sat Max", "HSV", 255, 255, empty)
# cv2.createTrackbar("Value Min", "HSV", 0, 255, empty)
# cv2.createTrackbar("Value Max", "HSV", 255, 255, empty)

# while True:
#     # camera_realsense_rgb.read_RGB()
#     # img = camera_realsense_rgb.imageBufferRGB
#     myCam4.read()
#     img = myCam4.imageData

#     img = cv2.resize(img, (480, 240))
#     imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#     h_min = cv2.getTrackbarPos("Hue Min", "HSV")
#     h_max = cv2.getTrackbarPos("Hue Max", "HSV")
#     s_min = cv2.getTrackbarPos("Sat Min", "HSV")
#     s_max = cv2.getTrackbarPos("Sat Max", "HSV")
#     v_min = cv2.getTrackbarPos("Value Min", "HSV")
#     v_max = cv2.getTrackbarPos("Value Max", "HSV")

#     lowerThresh = np.array([h_min, s_min, v_min])
#     upperThresh = np.array([h_max, s_max, v_max])
#     mask = cv2.inRange(imgHsv, lowerThresh, upperThresh)

#     result = cv2.bitwise_and(img, img, mask=mask)

#     mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
#     hstack = np.hstack([img, mask, result])

#     cv2.imshow('Horizontal', hstack)
#     if cv2.waitKey(1) and 0xFF == ord('q'):
#         cv2.waitKey(0)
