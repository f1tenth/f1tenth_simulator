#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

# this stupid algorithm goes forward until it almost hits a wall in front
def callback(data):
    # here data is a LaserScan msg, for example
    front_dis = data.ranges[540] # with default setting
    if front_dis > 1:
        speed = 1
    else:
        speed = 0
    drive_msg = AckermannDrive(steering_angle=0, speed=speed)
    drive_st_msg = AckermannDriveStamped(drive=drive_msg)
    drive_pub.publish(drive_st_msg)


rospy.init_node("auto_driver")

scan_topic = rospy.get_param("/f1tenth_simulator/scan_topic")
scan_sub = rospy.Subscriber(scan_topic, LaserScan, callback)

drive_topic = rospy.get_param("/f1tenth_simulator/auto_drive_topic")
drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

rospy.spin()

