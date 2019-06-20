#!/usr/bin/python
import rospy, tf
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
import os
from env.setup_world import SetupWorld
import math
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)

def callback(data):
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"

    msg.drive.speed = 1.0
    msg.drive.acceleration = 1
    msg.drive.jerk = 1
    msg.drive.steering_angle = 0.0
    msg.drive.steering_angle_velocity = 1
    print("hello")

    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('lec_control', anonymous=True)
    env = SetupWorld()

    y = np.random.uniform(-0.5, 1.0)
    yaw = np.random.uniform(-math.pi/6.0, math.pi/6.0)
    position = [0.0, y, 0.05]
    orientation = [0.0, 0.0, yaw]
    env.reset(position, orientation)

    rospy.Subscriber("scan",LaserScan,callback)
    rospy.spin()