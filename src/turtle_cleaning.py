#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty


class Cleaner():
    def __init__()

RIGHT_ANGLE = math.pi / 2
poseReceived = False
moving = False
distanceMoved = 0
x = 0
y = 0
yaw = 0
CMD_VEL_TOPIC = '/turtle1/cmd_vel'
CMD_PUB = NONE
POSE_TOPIC = '/turtle1/pose'

def degree2Rad(deg):
    return math.pi * deg / 180

def calcDistance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def didReach(targetX, targetY, tolerance):
    return calcDistance(targetX, targetY, x, y) <= abs(tolerance)

def poseCallback(pose):
    global x, y, z, yaw, poseReceived, distanceMoved
    if moving:
        distanceMoved += math.sqrt((x - pose.x) ** 2 + (y - pose.y) ** 2)
    else:
        distanceMoved = 0
    x = pose.x
    y = pose.y
    yaw = pose.theta
    poseReceived = True

def move(speed, distance, isForward = True, callback = lamda *args: None):

    twist = Twist()
    global x, y, moving, distanceMoved
    x0 = x
    y0 = y

    if not isForward:
        speed = -speed

    twist.linear.x = speed
    distance_moved = 0.0
    loop = rospy.Rate(10)
    moving = True

    while True:
        CMD_PUB.publish(twist)
        loop.sleep()
        if (distanceMoved > distance):
            break

    moving = False
    pub.publish(Twist())
    callback()

def spin(speed, angle, clockwise = True, callback = lamda *args: None):

    twist = Twist()
    twist.angular.z = speed
    global yaw
    yaw0 = yaw
    angle = degree2Rad(angle)
    if clockwise:
        speed = -speed
        angle = -angle

    target = yaw0 + angle
    def reached():
        delta = target - yaw
        if clockwise:
            return delta >= 0
        else
            return delta <= 0

    while True:
        CMD_PUB.publish(twist)
        loop.sleep()
        if reached():
            break

    pub.publish(Twist())
    callback()

def moveTo(targetX, targetY, tolerance = 0.01):

    maxLinearSpeed = 4
    maxAngularSpeed = 1
    global x, y, yaw

    direction = math.atan(x - targetX, y - targetY)

    loop = rospy.Rate(10)

    while True:
        twist = Twist()
        angleDiff = direction - yaw
        clockwise = True if angleDiff < 0 and  else False

        # calculate speed, set to twist msg
        if abs(angleDiff) > RIGHT_ANGLE:
            # rotate only, when the angle is less than 90, call this method again
            rotateAngle = angleDiff - RIGHT_ANGLE
            spin(maxAngularSpeed * angleDiff / math.pi, rotateAngle, clockwise, moveTo(targetX, targetY, tolerance))
            return
        else:
            twist.linear.x = math.min(maxLinearSpeed, maxLinearSpeed * calcDistance(targetX, targetY, x, y))
            twist.angular.z = maxAngularSpeed * angleDiff / RIGHT_ANGLE

        loop.sleep()
        if didReach(targetX, targetY, tolerance):
            break

    pub.publish(Twist())






def goSpiral():
    # todo
def goGrid():
    # todo

if __name__ == '__main__':

    try:
        global CMD_PUB
        rospy.init_node('turtle_cleaning', anonymous = True)

        CMD_PUB = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size = 10)

        rospy.Subscriber(POSE_TOPIC, Pose, poseCallback)

        while x != 0:
            time.sleep(0.5)

        if len(sys.argv) == 2:
            cleanType = sys.argv[1]
        else
            sys.exit('No clean type supplied')

        if cleanType == 'spiral':
            goSpiral()
        else if cleanType == 'grid':
            goGrid()
        else
            sys.exit('Not support cleaning type')

    except:
        print 'exception'
