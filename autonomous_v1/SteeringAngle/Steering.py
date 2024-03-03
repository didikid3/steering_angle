import cv2
import numpy as np
# import utils as utils
import SteeringAngle.utils as utils

from pal.products.qcar import QCar, QCarRealSense

curveList = []
avgVal = 10
def getLaneCurve(img, display=2):
    height, width, channels = img.shape
    # points, lower, upper = utils.valTrackbars()
    w = width
    widthTop, heightTop, widthBot, heightBot = 145, 135, 85, 240
    points = np.float32([(widthTop, heightTop), (w-widthTop, heightTop),
                         (widthBot, heightBot), (w-widthBot, heightBot)
                         ])
    lower = np.array([0, 0, 26]) #0 0 26
    upper = np.array([179, 255, 255]) #129 255 240,,,,, 179 255 255

    imgThresh = utils.thresholding(img, lower, upper)
    imageWarpCopy = img.copy()
    imgResult = img.copy()
    
    imgWarp = utils.warping(imgThresh, points, width, height)
    imgWarpPoints = utils.drawPoints(imageWarpCopy, points)

    basePoint, imgHst_base = utils.histogram(imgWarp, minPer=0.8, display=True)
    midPoint = utils.histogram(imgWarp, minPer=0.5, display=False, region=4)
    curveRaw = basePoint - midPoint

    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)

    curve = int(sum(curveList)/len(curveList))

    if display != 0:
       imgInvWarp = utils.warping(imgWarp, points, width, height, inverse=True)
       imgInvWarp = cv2.cvtColor(imgInvWarp,cv2.COLOR_GRAY2BGR)
       imgInvWarp[0:height//3,0:width] = 0,0,0
       imgLaneColor = np.zeros_like(img)
       imgLaneColor[:] = 0, 255, 0
       imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
       imgResult = cv2.addWeighted(imgResult,1,imgLaneColor,1,0)
       midY = 450
       cv2.putText(imgResult,str(curve),(width//2-80,85),cv2.FONT_HERSHEY_COMPLEX,2,(255,0,255),3)
       cv2.line(imgResult,(width//2,midY),(width//2+(curve*3),midY),(255,0,255),5)
       cv2.line(imgResult, ((width // 2 + (curve * 3)), midY-25), (width // 2 + (curve * 3), midY+25), (0, 255, 0), 5)
       for x in range(-30, 30):
           w = width // 20
           cv2.line(imgResult, (w * x + int(curve//50 ), midY-10),
                    (w * x + int(curve//50 ), midY+10), (0, 0, 255), 2)
    if display == 2:
       imgStacked = utils.stackImages(0.7,([img,imgWarpPoints,imgWarp],
                                         [imgHst_base,imgLaneColor,imgResult]))
       cv2.imshow('ImageStack',imgStacked)
    elif display == 1:
       cv2.imshow('Resutlt',imgResult)
    
    curve = (-1) * (curve/width*2) * 1.2
    return curve


if __name__ == '__main__':
    # cap = cv2.VideoCapture('data2.mp4')
    camera_realsense_rgb = QCarRealSense(mode='RGB')

    initials = [149, 179, 30, 240]
    utils.initializeTrackbars(initials)
    frameCounter = 0

    while True:
        # frameCounter += 1
        # if cap.get(cv2.CAP_PROP_FRAME_COUNT) == frameCounter:
        #     cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        #     frameCounter = 0

        # success, img = cap.read()
        # img = cv2.resize(img, (480, 240))
        camera_realsense_rgb.read_RGB()
        img = cv2.resize(camera_realsense_rgb.imageBufferRGB, (int(480), int(240)))
        getLaneCurve(img)

        # cv2.imshow('Video Feed', img)
        cv2.waitKey(1)