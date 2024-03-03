#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + ' I heard %s' % data.data)

if __name__ == '__main__':
	rospy.init_node('hello_python_listener', anonymous=True)
	sub = rospy.Subscriber('test_topic', String, callback)
	rate = rospy.Rate(10)
	rospy.spin()
