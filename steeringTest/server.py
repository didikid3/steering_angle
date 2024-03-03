import argparse

import threading
import sys
import socket
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'  # or any {'0', '1', '2'}

import cv2
from pal.products.qcar import QCar, QCarRealSense

import payload
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
WIDTH = 480
HEIGHT = 240
if args.video:
    videoRecording = True

if videoRecording:
    # Define the codec and create a VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # You can change the codec as needed (e.g., 'XVID')
    out = cv2.VideoWriter('video.mp4', fourcc, fps=30.0, frameSize=(WIDTH, HEIGHT))  # 'output.mp4' is the output file name

    # Add this line before the main loop to start recording
    out.open('video.mp4', fourcc, fps=30.0, frameSize=(WIDTH, HEIGHT))

stopthread = False

myCar = QCar(readMode=0)
max_throttle = 0.2
min_throttle = -0.2
max_steering = 0.5
min_steering = -0.5

steering = 0
throttle = 0
reverse = False

PORT = 38821  # Port to listen on (non-privileged ports are > 1023)


def drive():
    print("Driving Starting...")
    global throttle, steering, reverse
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('', PORT))
        global stopthread

        while not stopthread:
            data = s.recvfrom(100)[0].decode('utf-8')
            if not data:
                pass

            packet = payload.payload_handler(data)
            buffer = []
            try:
                '''
                /*##################################################################################*/
                /*|    Controller ID    |    Event    |    Event Dimension     |    Event Value    |*/
                /*|        4 Bit        |    4 Bit    |         8 Bit          |       4 Bit       |*/
                /*##################################################################################*/
                '''
                
                '''
                Read Payload Data
                '''
                if packet.read(buffer, 4) == -1 or \
                   packet.read(buffer, 4) == -1 or \
                   packet.read(buffer, 8) == -1 or \
                   packet.read(buffer, 8) == -1:
                    print("Warning: Packet Length Too short")
                    continue
                
                event = int(buffer[1])


                if event == 1536:
                    #IF Axis is Steering Wheel
                    if float(buffer[2]) == 0:
                        steer = -1* float(buffer[3]) * 2
                        if abs(steering - steer) < 0.05:
                            continue

                        if (steering == min_steering and steer < min_steering
                            or
                            steering == max_steering and steer > max_steering):
                            continue

                        if steer < 0:
                            steering = max(steer, min_steering)
                        else:
                            steering = min(steer, max_steering)
                        
                    #IF Axis is Throttle
                    elif float(buffer[2]) == 1:
                        
                        th = 0.6 * ((abs(float(buffer[3]) -1 ) /2 ) * 0.2)
                        if th < 0:
                            throttle = max(th, min_throttle)
                        else:
                            throttle = min(th, max_throttle)

                if event == 1539:
                    if float(buffer[2]) == 5:
                        print("Reverse Triggered")
                        reverse = True
                    elif float(buffer[2]) == 4:
                        print("Forward Triggered")
                        reverse = False

                if reverse:
                    if throttle > 0:
                        throttle *= -1
                else:
                    throttle = abs(throttle)
                myCar.write(throttle=throttle, steering=steering)
                    

            except Exception as e:
                print("Invalid Packet Size")
                print(e)

    print("Terminated Driving")
    
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
    #Setup Camera and Predition Model for Lane Detection
    camera_realsense_rgb = QCarRealSense(mode='RGB')
    model = DetectLane.DetectLane()

    
    # Start Thread For Steering
    # Accepts Steering Inputs Over UDP
    # Can Be Easily Modified to Accept Inputs from any Source
    t2 = threading.Thread(target=drive)

    try:
        t2.start()
        while t2.is_alive():

            camera_realsense_rgb.read_RGB()

            #Send Image for Predition
            if camera_realsense_rgb is not None:
                frame = cv2.resize(camera_realsense_rgb.imageBufferRGB, (int(WIDTH), int(HEIGHT)))
                result = model.detectLanes(frame)
                if videoRecording:
                    out.write(frame)
                cv2.imshow("RealSense Camera", result)


            key = cv2.waitKey(1)

            #If ESC is pressed begin termination sequence
            if key == 27:
                exiting()
                

    except Exception as e:
        print("Encountered Error:")
        print(e)
    finally:
        print("Exiting Main")
        exiting()


    

if __name__ == '__main__':
    main()