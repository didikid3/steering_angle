#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
	rospy.init_node('hello_python', anonymous=True)     # 初始化 hello_python node
	pub = rospy.Publisher('test_topic', String, queue_size=10)
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		msg = String()
		msg.data = "Hello, ROS!"
		pub.publish(msg)		
		rospy.loginfo(msg) 
		rospy.sleep(0.1)
