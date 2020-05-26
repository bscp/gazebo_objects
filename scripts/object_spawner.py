#!/usr/bin/env python

import sys
import argparse
import rospy
import time
from gazebo_msgs.srv import *
from geometry_msgs.msg import *

cylinder_model_sdf = '<?xml version=\'1.0\'?><sdf version="1.4"><model name="{object_name}"><link name="link"><collision name="collision"><geometry><cylinder><radius>{dimension_x}</radius><length>{dimension_z}</length></cylinder></geometry></collision><visual name="visual"><geometry><cylinder><radius>{dimension_x}</radius><length>{dimension_z}</length></cylinder> </geometry></visual></link><plugin name="gazebo_plugin" filename="libgazebo_object_gazebo_plugin.so"/></model></sdf>'


def parse_args():
    parser = argparse.ArgumentParser()

    # TODO : check if args contains all and valid values

    parser.add_argument('-n', '--name',
        type=str, required=True,
        help='Name of the object',
        dest='object_name',)

    parser.add_argument('-s', '--shape',
        choices=['cylinder'],
        help='Name of the shape',
        dest='shape_name')
    
    parser.add_argument('-d', '--dimension',
        nargs=3, type=float, required=True,
        help='Dimension of the shape as X Y Z',
        dest='dimension')
    
    parser.add_argument('-p', '--position',
        nargs=3, type=float,
        help='Position of the shape center as X Y Z',
        dest='position', default=[0.0, 0.0, 0.0])

    parser.add_argument('-o', '--orientation',
        nargs=3, type=float,
        help='Orientation in Rotate Pitch Yaw',
        dest='orientation')

    args, _ = parser.parse_known_args()
    return args


def render_cylinder_model(object_name, args):
    return cylinder_model_sdf.format(
        object_name=object_name,
        dimension_x=args.dimension[1],
        dimension_z=args.dimension[2])


def main():
    rospy.init_node("object_spawner")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_sdf_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    args = parse_args()
    object_name = args.object_name
    model_sdf = render_cylinder_model(object_name, args)
    
    model_pose = Pose()
    model_pose.position.x = args.position[0]
    model_pose.position.y = args.position[1]
    model_pose.position.z = args.position[2]  # / 2.0

    # TODO : add orientation

    model_namespace = ""
    model_reference_frame = "world"
    spawn_sdf_model(object_name, model_sdf, model_namespace, model_pose, model_reference_frame)


if __name__ == "__main__":
    main()
