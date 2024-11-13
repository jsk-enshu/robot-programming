#!/usr/bin/env python


import argparse

from kxr_controller.kxr_interface import KXRROSRobotInterface
from kxr_models.download_urdf import download_urdf_mesh_files
import numpy as np  # NOQA
import rospy
from skrobot.model import RobotModel


parser = argparse.ArgumentParser(description="Run KXRROSRobotInterface")
parser.add_argument(
    "--viewer", type=str, help="Specify the viewer: trimesh", default=None
)
parser.add_argument(
    "--namespace", type=str, help="Specify the ROS namespace", default=""
)
args = parser.parse_args()

rospy.init_node("kxr_interface", anonymous=True)

download_urdf_mesh_files(args.namespace)

robot_model = RobotModel()
robot_model.load_urdf_from_robot_description(
    args.namespace + "/robot_description_viz"
)
ri = KXRROSRobotInterface(  # NOQA
    robot_model, namespace=args.namespace, controller_timeout=60.0
)


ri.servo_on()

robot_model.init_pose()
robot_model.rarm_joint0.joint_angle(-np.pi / 2)
robot_model.larm_joint0.joint_angle(np.pi / 2)

ri.angle_vector(robot_model.angle_vector(), 5)
