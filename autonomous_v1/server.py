import argparse

import time
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'  # or any {'0', '1', '2'}

import cv2
from pal.products.qcar import QCar, QCarRealSense, Camera2D

import DetectLane as DetectLane

parser = argparse.ArgumentParser(
                                    prog='Q Car Contorl Handler',
                                    description='Handles Image Processing and QCar Control'
                                )
parser.add_argument(
                    "-v", "--video",
                    action='store_true',
                    help="Enable or Disable Video Capture."
                    )

args = parser.parse_args()
videoRecording = False
if args.video:
    videoRecording = True

if videoRecording:
    # Define the codec and create a VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # You can change the codec as needed (e.g., 'XVID')
    out = cv2.VideoWriter('video.mp4', fourcc, fps=30.0, frameSize=(480, 270))  # 'output.mp4' is the output file name

    # Add this line before the main loop to start recording
    out.open('video.mp4', fourcc, fps=30.0, frameSize=(480, 270))

stopthread = False

myCar = QCar(readMode=0)
max_throttle = 0.2
min_throttle = -0.2
max_steering = 0.5
min_steering = -0.5

steering = 0
throttle = 0.05
reverse = False

sampleRate = 45.0
sampleTime = 1/sampleRate

    
def main():
    def exiting():
        camera_realsense_rgb.terminate()
        if videoRecording:
            out.release()
        cv2.destroyWindow("RealSense Camera")
        global stopthread
        stopthread=True
        # Close all windows
        cv2.destroyAllWindows()
        quit()

    global throttle, steering, reverse
    #Setup Camera and Predition Model for Lane Detection
    camera_realsense_rgb = QCarRealSense(mode='RGB')
    # imageWidth = 640
    # imageHeight = 480
    # myCam4 = Camera2D(cameraId="3", frameWidth=imageWidth, frameHeight=imageHeight)
    model = DetectLane.DetectLane()

    # cap = cv2.VideoCapture('example2.mp4')
    # frameno = 0


    while True:

        #Record Wait Time Needed to Get Image Data
        start = time.time()
        camera_realsense_rgb.read_RGB()
        # myCam4.read()
        # frameno += 1
        # Loop Video
        # if cap.get(cv2.CAP_PROP_FRAME_COUNT) == frameno-10:
        #     cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        #     frameno = 0
        end = time.time()
        computation = end - start

        # success, img = cap.read()
        # img = cv2.resize(img, (480, 240))

        #Send Image for Predition
        if camera_realsense_rgb is not None:
        # if myCam4 is not None:
        # if img is not None:
            # frame = img
            frame = cv2.resize(camera_realsense_rgb.imageBufferRGB, (int(480), int(240)))
            # frame = cv2.resize(myCam4.imageData, (int(480), int(240)))
            # cv2.imshow("RealSense Camera", frame)
            if videoRecording:
                out.write(frame)
            result, steer = model.detectLanes(frame)
            cv2.imshow("RealSense Camera", result)
            print(steer)
            key = cv2.waitKey(1)

            if key == 27:
                myCar.write(throttle=0, steering=0)
                exiting()

            if (steering == min_steering and steer < min_steering
                or
                steering == max_steering and steer > max_steering):
                continue

            if steer < 0:
                steering = max(steer, min_steering)
            else:
                steering = min(steer, max_steering)
            myCar.write(throttle=throttle, steering=steering)

            
            

        #Sleep
        # sleepTime = sampleTime - (computation % sampleTime)
        # msSleep = int(1000 * sleepTime) if int(1000 * sleepTime) > 0 else 1

        #If ESC is pressed begin termination sequence
        # if key == 27:
        #     exiting()
                

    # except Exception as e:
    #     print("Encountered Error:")
    #     print(e)
    # finally:
    #     print("Exiting Main")
    #     exiting()


    

if __name__ == '__main__':
    main()