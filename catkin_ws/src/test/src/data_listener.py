#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + '-> [%s]' % data.data)

if __name__ == '__main__':
	rospy.Subscriber('steering_topic', String, callback)
	rospy.init_node('Data_listener', anonymous=True)
	rospy.spin()
