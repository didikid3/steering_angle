import cv2
import numpy as np
import utils

curveList = []
avgVal = 10
def getLaneCurve(img, display=2):
    imgThresh = utils.thresholding(img)
    imageWarpCopy = img.copy()
    imgResult = img.copy()
    

    height, width, channels = img.shape
    points = utils.valTrackbars()
    imgWarp = utils.warping(imgThresh, points, width, height)
    imgWarpPoints = utils.drawPoints(imageWarpCopy, points)

    basePoint, imgHst_base = utils.histogram(imgWarp, minPer=0.9, display=True)
    midPoint = utils.histogram(imgWarp, minPer=0.5, display=False, region=4)
    curveRaw = basePoint - midPoint

    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)

    curve = int(sum(curveList)/len(curveList))
    print(curve)

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
    #    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
    #    cv2.putText(imgResult, 'FPS '+str(int(fps)), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (230,50,50), 3);
    if display == 2:
       imgStacked = utils.stackImages(0.7,([img,imgWarpPoints,imgWarp],
                                         [imgHst_base,imgLaneColor,imgResult]))
       cv2.imshow('ImageStack',imgStacked)
    elif display == 1:
       cv2.imshow('Resutlt',imgResult)


    cv2.imshow('Thresholding', imgThresh)
    cv2.imshow('Warp', imgWarp)
    cv2.imshow('Points', imgWarpPoints)
    cv2.imshow('Histogram', imgHst_base)
    return curve


if __name__ == '__main__':
    cap = cv2.VideoCapture('v3.mp4')
    initials = [149, 179, 30, 240]
    utils.initializeTrackbars(initials)
    frameCounter = 0

    while True:
        frameCounter += 1
        #Loop Video
        if cap.get(cv2.CAP_PROP_FRAME_COUNT) == frameCounter:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            frameCounter = 0

        success, img = cap.read()

        img = cv2.resize(img, (480, 240))
        getLaneCurve(img)

        cv2.imshow('Video Feed', img)
        cv2.waitKey(200)