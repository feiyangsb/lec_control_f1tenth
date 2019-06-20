#!/usr/bin/python
import rospy, tf
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
import os

if __name__ == '__main__':
    rospy.init_node('lec_control', anonymous=True)
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    orient = tf.transformations.quaternion_from_euler(0,0,0)
    object_pose = Pose()
    object_pose.position.x = float(0.0)
    object_pose.position.y = float(0.0)
    object_pose.position.z = float(0.5)
    object_pose.orientation.x = orient[0] 
    object_pose.orientation.y = orient[1] 
    object_pose.orientation.z = orient[2] 
    object_pose.orientation.w = orient[3]
    file_location = "/home/feiyang/Desktop/current_work/f1tenth/sims_ws/src/racecar-simulator/racecar_description/urdf/racecar.xacro"
    p = os.popen("rosrun xacro xacro.py " + file_location)
    xml_string = p.read()
    p.close()
    srv_spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

    srv_spawn_model("racecar", xml_string, "", object_pose, "world" )

    print("Successfully initialize the control node")
    rospy.spin()