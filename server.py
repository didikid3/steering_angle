import socket
import payload


from pal.products.qcar import QCar
from pal.utilities.math import *

myCar = QCar(readMode=0)

max_throttle = 0.2
min_throttle = -0.2
max_steering = 0.5
min_steering = -0.5

steering = 0
throttle = 0


PORT = 38822  # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind(('', PORT))
    s.listen()
    print("Ready")
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")

        while True:
            data = conn.recv(100).decode('utf-8')
            if not data:
                break

            packet = payload.payload_handler(data)
            buffer = []
            try:
                '''
                /*##################################################################################*/
                /*|    Controller ID    |    Event    |    Event Dimension     |    Event Value    |*/
                /*|        4 Bit        |    4 Bit    |         8 Bit          |        Bit       |*/
                /*##################################################################################*/
                '''

                if packet.read(buffer, 4) == -1:
                    continue
                if packet.read(buffer, 4) == -1:
                    continue
                if packet.read(buffer, 8) == -1:
                    continue
                if packet.read(buffer, 4) == -1:
                    continue

                if float(buffer[2]) == 0:
                    steer = -1* float(buffer[3])
                    if steer < 0:
                        steering = max(steer, min_steering)
                    else:
                        steering = min(steer, max_steering)

                elif float(buffer[2]) == 1:
                    
                    th = (abs(float(buffer[3]) -1 ) /2) * 0.2
                    if th < 0:
                        throttle = max(th, min_throttle)
                    else:
                        throttle = min(th, max_throttle)


                print(throttle, steering)

                myCar.read_write_std(throttle=throttle, steering=steering)
            except:
                print("||| Invalid Packet Size |||")
            
         
