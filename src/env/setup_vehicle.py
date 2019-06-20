#!/usr/bin/python
import rospy, tf
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
import os

class SetupVehicle():
    def __init__(self, vehicle_xml_file = "/home/feiyang/Desktop/current_work/f1tenth/sims_ws/src/racecar-simulator/racecar_description/urdf/racecar.xacro"):
        self.vehicle_xml_file = vehicle_xml_file
        p = os.popen("rosrun xacro xacro.py " + self.vehicle_xml_file)
        self.xml_string = p.read()
        p.close()

    def spawnVehicle(self, position, orientation):
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        orient = tf.transformations.quaternion_from_euler(orientation[0], orientation[1], orientation[2])
        object_pose = Pose()
        object_pose.position.x = float(position[0])
        object_pose.position.y = float(position[1])
        object_pose.position.z = float(position[2])
        object_pose.orientation.x = orient[0]
        object_pose.orientation.y = orient[1]
        object_pose.orientation.z = orient[2]
        object_pose.orientation.w = orient[3]
        srv_spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        srv_spawn_model("racecar", self.xml_string, "", object_pose, "world")
        print("Successfully spawn a new vehicle")

    def deleteVehicle(self):
        rospy.wait_for_service("/gazebo/delete_model")
        srv_delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        try:
            srv_delete_model("racecar")
        except rospy.ServiceException, e:
            pass

