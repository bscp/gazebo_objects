#!/usr/bin/env python

import sys
import rospy
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
import re


def replace(text, key, value):
    regex_pattern = "\\{\\{\\s*%s\\s*\\}\\}"
    regex_pattern = regex_pattern % key
    return re.sub(regex_pattern, str(value), text)


def spawn_cup(name, radius, length, pos_x, pos_y):
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    model_name = sys.argv[1]
    model_namespace = ""
    cylinder_radius = sys.argv[2]
    cylinder_length = sys.argv[3]
    model_reference_frame = "world"

    model_sdf = '<?xml version=\'1.0\'?> <sdf version="1.4"> <model name="{{name}}"><link name="link"><collision name="collision"> <geometry> <cylinder> <radius>{{radius}}</radius> <length>{{length}}</length> </cylinder> </geometry> </collision> <visual name="visual"> <geometry> <cylinder> <radius>{{radius}}</radius> <length>{{length}}</length> </cylinder> </geometry> </visual> </link><plugin name="cylinder_plugin" filename="libgazebo_objects_cylinder_plugin.so"><detected>{{detected}}</detected></plugin></model> </sdf>'
    model_sdf = replace(model_sdf, "name", model_name)
    model_sdf = replace(model_sdf, "radius", cylinder_radius)
    model_sdf = replace(model_sdf, "length", cylinder_length)

    model_pose = Pose()
    model_pose.position.x = float(sys.argv[4])
    model_pose.position.y = float(sys.argv[5])
    model_pose.position.z = float(length) / 2.0

    model_sdf = replace(model_sdf, "detected", sys.argv[6])

    rospy.sleep(3.0)
    spawn_sdf_model(model_name, model_sdf, model_namespace, model_pose, model_reference_frame)


def main():
    spawn_cup(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])


if __name__ == "__main__":
    main()

