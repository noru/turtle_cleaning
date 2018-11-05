#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import sys
import time
from std_srvs.srv import Empty


PI = math.pi

x = 0
y = 0
theta = 0
POSE_TOPIC = '/turtle1/pose'
CMD_VEL_TOPIC = '/turtle1/cmd_vel'
CMD_PUB = None

def deg2Rad(deg):
    return PI * deg / 180

def poseCallback(pose):
    global x, y, theta
    x = pose.x
    y = pose.y
    theta = pose.theta

def getDistance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def stop():
    CMD_PUB.publish(Twist())


def move(speed, distance, isForward = True):

    twist = Twist()
    if not isForward:
        speed = -speed

    global x, y

    twist.linear.x = speed
    distance_moved = 0.0
    loop = rospy.Rate(10)

    t0 = time.time()
    while True:
        CMD_PUB.publish(twist)
        t1 = time.time()
        distance_moved = speed * (t1 - t0)
        loop.sleep()

        if (distance_moved >= distance):
            break
    stop()

def spin(speed, angle, clockwise = True):

    twist = Twist()
    if clockwise:
        speed = -speed
    twist.angular.z = speed

    current_angle = 0.0
    t0 = time.time()
    loop = rospy.Rate(10)
    while True:
        CMD_PUB.publish(twist)
        t1 = time.time()
        current_angle = speed * (t1 - t0)
        loop.sleep()
        if current_angle >= angle:
            break
    stop()

def setOrientation(orientation):
    delta = orientation - theta
    clockwise = True if delta > 0 else False
    spin(deg2Rad(10), abs(delta), clockwise)

def moveTo(pose, tolerance = 0.01):

    global x, y, theta
    twist = Twist()
    loop = rospy.Rate(100)
    moved = 0.0
    Kp = 1.0

    while True:
        distance = getDistance(x, y, pose.x, pose.y)
        moved += distance
        twist = Twist()
        twist.linear.x = Kp * e
        twist.angular.z = 4 * (math.atan2(pose.y - y, pose.x - x) - theta)
        CMD_PUB.publish(twist)
        loop.sleep()
        if distance <= tolerance:
            break
    stop()

def goSpiral():
    loop = rospy.Rate(1)
    speed = 4
    twist = Twist()
    vk = 1
    wk = 2
    rk = 0.5

    while True:
        rk += 1
        twist.linear.x = rk
        twist.angular.z = speed
        CMD_PUB.publish(twist)
        loop.sleep()
        if (x > 10.5 or y > 10.5):
            break
    stop()


def goGrid():
    loop = rospy.Rate(0.5)
    pose = Pose()
    pose.x = 1
    pose.y = 1
    pose.theta = 0
    moveTo(pose)
    loop.sleep()
    setOrientation(0)
    loop.sleep()

    move(2, 9)
    loop.sleep()
    spin(deg2Rad(10), deg2Rad(90), False)
    loop.sleep()
    move(2, 9)

    loop.sleep()
    spin(deg2Rad(10), deg2Rad(90), False)
    loop.sleep()
    move(2, 1)
    spin(deg2Rad(10), deg2Rad(90), False)
    loop.sleep()
    move(2, 9)
    loop.sleep()
    spin(deg2Rad(30), deg2Rad(90))
    loop.sleep()
    move(2, 1)
    loop.sleep()
    spin(deg2Rad(30), deg2Rad(90))
    move(2, 9)




if __name__ == '__main__':

    global CMD_PUB
    rospy.init_node('turtle_cleaning', anonymous = True)

    CMD_PUB = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size = 10)

    rospy.Subscriber(POSE_TOPIC, Pose, poseCallback)

    while x != 0:
        time.sleep(0.5)

    if len(sys.argv) == 2:
        cleanType = sys.argv[1]
    else:
        sys.exit('No clean type supplied')

    if cleanType == 'spiral':
        goSpiral()
    elif cleanType == 'grid':
        goGrid()
    else:
        sys.exit('Not support cleaning type')
