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
from gazebo_msgs.msg import ContactsState
import message_filters
from nav_msgs.msg import Odometry


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

    pub.publish(msg)

def callback1(data):
    if len(data.states):
        for i in range(len(data.states)):
            if data.states[i].contact_normals[0].z < 0.1:
                print("collision occurred")

def data_parser(lidar_msg, collision_msg, odom_msg):
    # parse the collision data
    collision = False
    if len(collision_msg.states):
        for i in range(len(collision_msg.states)):
            if collision_msg.states[i].contact_normals[0].z < 0.1:
                collision = True
    
    velocity = math.sqrt(odom_msg.twist.twist.linear.x ** 2 + odom_msg.twist.twist.linear.y ** 2)
    return lidar_msg.ranges, collision, velocity



def ddpg(lidar_msg, collision_msg, odom_msg):
    lidar, collision, velocity = data_parser(lidar_msg, collision_msg, odom_msg)
    print("Velocity: {}".format(velocity))
    if collision:
        print("the vehicle collides the wall")

    # publish the control topic
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"

    msg.drive.speed = 1.0
    msg.drive.acceleration = 1
    msg.drive.jerk = 1
    msg.drive.steering_angle = 0.0
    msg.drive.steering_angle_velocity = 1
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('lec_control', anonymous=True)
    env = SetupWorld()

    y = np.random.uniform(-0.5, 1.0)
    yaw = np.random.uniform(-math.pi/6.0, math.pi/6.0)
    position = [0.0, y, 0.05]
    orientation = [0.0, 0.0, yaw]
    env.reset(position, orientation)

    lidar_sub = message_filters.Subscriber('/scan', LaserScan)
    collision_sub = message_filters.Subscriber('/vehicle_contact', ContactsState)
    odom_sub = message_filters.Subscriber('/vesc/odom', Odometry)
    ts = message_filters.ApproximateTimeSynchronizer([lidar_sub, collision_sub, odom_sub], 1, 0.1, allow_headerless=True)
    ts.registerCallback(ddpg)

    rospy.spin()