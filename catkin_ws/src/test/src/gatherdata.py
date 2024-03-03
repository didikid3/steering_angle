#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import socket
import payload
PORT = 38827  # Port to listen on (non-privileged ports are > 1023)

def drive():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        pub = rospy.Publisher('steering_topic', String, queue_size=15)
        rospy.init_node('Steering_Commander', anonymous=False)

        s.bind(('', PORT))
        s.listen()
        conn, addr = s.accept()

        while not rospy.is_shutdown():
            try:
                data = conn.recv(100).decode('utf-8')
                if not data:
                    rospy.loginfo('Connection Closed')
                    break

                packet = payload.payload_handler(data)
                buffer = []
                # try:
                if packet.read(buffer, 4) == -1 or \
                   packet.read(buffer, 4) == -1 or \
                   packet.read(buffer, 8) == -1 or \
                   packet.read(buffer, 8) == -1:
                    rospy.logwarn("Warning: Packet Length Too short")
                    continue
                # Buffer - [Joystick, Event, Axis, Value]
                msg = String()
                msg.data = " ".join(buffer[1:])
                pub.publish(msg)

            except Exception as e:
                rospy.logerr("Invalid Packets")
                rospy.logerr(e)


if __name__ == '__main__':
    drive()