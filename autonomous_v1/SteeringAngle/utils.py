import cv2
import numpy as np

def empty(a):
    pass

def thresholding(img, lowerThresh, upperThresh):
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    '''
    calibration video values
    78, 15, 0
    120, 255, 129
    '''
    mask = cv2.inRange(imgHsv, lowerThresh, upperThresh)
    return mask

def warping(img, points, w, h, inverse=False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0,0], [w,0], [0,h], [w,h]])

    if inverse:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1, pts2)

    imgWarp = cv2.warpPerspective(img, matrix, (w,h))
    return imgWarp

def histogram(img, minPer=0.1, display = True, region = 1):
    if region == 1:
        histValues = np.sum(img, axis=0)
    else:
        histValues = np.sum(img[img.shape[0]//region:,:], axis=0)
    maxValue =  np.max(histValues)
    minValue = minPer*maxValue

    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))

    if display:
        imgHst = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        for x, intensity in enumerate(histValues):
            intensity = int(intensity)
            cv2.line(imgHst, (x, img.shape[0]), (x, img.shape[0] - intensity//255//region), (255, 0, 255), 1)
            cv2.circle(imgHst, (basePoint, img.shape[0]), 20, (0, 255, 255), cv2.FILLED)

        return basePoint, imgHst
    return basePoint


def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

def initializeTrackbars(initialTrackbarVars, w=480, h=240):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", w, h)
    cv2.createTrackbar("Width Top", "Trackbars", initialTrackbarVars[0], w//2, empty)
    cv2.createTrackbar("Height Top", "Trackbars", initialTrackbarVars[1], h, empty)
    cv2.createTrackbar("Width Bottom", "Trackbars", initialTrackbarVars[2], w//2, empty)
    cv2.createTrackbar("Height Bottom", "Trackbars", initialTrackbarVars[3], h, empty)

    cv2.namedWindow("HSV")
    cv2.resizeWindow("HSV", w, h)
    cv2.createTrackbar("Hue Min", "HSV", 0, 179, empty)
    cv2.createTrackbar("Hue Max", "HSV", 179, 179, empty)
    cv2.createTrackbar("Sat Min", "HSV", 0, 255, empty)
    cv2.createTrackbar("Sat Max", "HSV", 255, 255, empty)
    cv2.createTrackbar("Value Min", "HSV", 0, 255, empty)
    cv2.createTrackbar("Value Max", "HSV", 255, 255, empty)

def valTrackbars(w=480, h=240):
    widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
    heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
    widthBot = cv2.getTrackbarPos("Width Bottom", "Trackbars")
    heightBot = cv2.getTrackbarPos("Height Bottom", "Trackbars")

    points = np.float32([(widthTop, heightTop), (w-widthTop, heightTop),
                         (widthBot, heightBot), (w-widthBot, heightBot)
                         ])
    
    
    h_min = cv2.getTrackbarPos("Hue Min", "HSV")
    h_max = cv2.getTrackbarPos("Hue Max", "HSV")
    s_min = cv2.getTrackbarPos("Sat Min", "HSV")
    s_max = cv2.getTrackbarPos("Sat Max", "HSV")
    v_min = cv2.getTrackbarPos("Value Min", "HSV")
    v_max = cv2.getTrackbarPos("Value Max", "HSV")
    lowerThresh = np.array([h_min, s_min, v_min])
    upperThresh = np.array([h_max, s_max, v_max])
    return points, lowerThresh, upperThresh
    # return lowerThresh, upperThresh

def drawPoints(img, points):
    for x in range(4):
        cv2.circle(img, (int(points[x][0]), int(points[x][1])), 15, (0,0,255), cv2.FILLED)
    return img