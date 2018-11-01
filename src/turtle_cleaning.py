#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

x = 0
y = 0
z = 0
yaw = 0

def poseCallback(pose):
	global x, y, z, yaw
	x = pose.x
	y = pose.y
	yaw = pose.theta

def move(speed, distance, isForward):

	twist = Twist()
	global x, y
	x0 = x
	y0 = y
	if isForward:
		twist.linear.x = abs(speed)
	else:
		twist.linear.x = -abs(speed)

	distance_moved = 0.0
	loop = rospy.Rate(10)
	cmd_vel_topic = '/turtle1/cmd_vel'
	pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)

	while True:
		pub.publish(twist)
		loop.sleep()	
		distance_moved += abs(0.5 * math.sqrt((x - x0) ** 2 + (y - y0) ** 2)) # // todo


if __name__ == '__main__':

    try:
		rospy.init_node('turtle_cleaning', anonymous = True)
		
		cmd_vel_topic = '/turtle1/cmd_vel'
		pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)
		
		pose_topic = '/turtle1/pose'
		sub = rospy.Subscriber(pose_topic, Pose, poseCallback)

		time.sleep(2)
    except:
        print 'exception'
