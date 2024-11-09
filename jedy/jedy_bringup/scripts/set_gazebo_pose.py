#!/usr/bin/env python

import rospy
from gazebo_ros.gazebo_interface import set_model_configuration_client
from gazebo_msgs.srv import SetModelConfiguration, SetModelState

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.msg import ModelState


def set_joint_angles():
    model_name = "jedy"  # モデルの名前
    model_param_name = "robot_description"  # URDFが設定されているパラメータサーバの名前
    gazebo_namespace = "/gazebo"

    # 初期位置を空中に設定
    model_state = ModelState()
    model_state.model_name = model_name
    model_state.pose.position.x = 0.0
    model_state.pose.position.y = 0.0
    model_state.pose.position.z = 0.13  # 空中に配置

    # モデルを空中に移動
    rospy.wait_for_service(gazebo_namespace + '/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy(gazebo_namespace + '/set_model_state', SetModelState)
        resp = set_model_state(model_state)
        rospy.loginfo("Model moved to air position: %s" % resp)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to move model: %s" % e)
        return


    # 関節名と設定する角度のリスト
    joint_names = [
        "jedy::rarm_joint0", "jedy::rarm_joint1", "jedy::rarm_joint2", "jedy::rarm_joint3",
        "jedy::rarm_joint4", "jedy::rarm_joint5", "jedy::rarm_joint6", "jedy::rarm_gripper_joint",
        "jedy::larm_joint0", "jedy::larm_joint1", "jedy::larm_joint2", "jedy::larm_joint3",
        "jedy::larm_joint4", "jedy::larm_joint5", "jedy::larm_joint6", "jedy::larm_gripper_joint",
        "jedy::head_joint0", "jedy::head_joint1"
    ]


    joint_positions = [
        1.5708, -0.069813, -0.523599, -1.74533, -0.05236, -1.53589, -0.017453, 0.0,
        -1.5708, 0.069813, 0.523599, -1.74533, -1.53589, -0.10472, 1.55334, -0.017453,
        0.0, 0.0
    ]

    # `set_model_configuration` サービスが利用可能になるまで待機
    rospy.loginfo('Waiting {}'.format(gazebo_namespace + '/set_model_configuration'))
    rospy.wait_for_service(gazebo_namespace + '/set_model_configuration')
    rospy.loginfo('Found {}'.format(gazebo_namespace + '/set_model_configuration'))
    # rospy.sleep(1)  # レースコンディションを避けるための短い遅延

    for i in range(10):
        try:
            # `set_model_configuration`サービスを呼び出し
            set_model_configuration = rospy.ServiceProxy(gazebo_namespace + '/set_model_configuration', SetModelConfiguration)
            rospy.loginfo("Setting initial joint angles...")
            resp = set_model_configuration(model_name, model_param_name, joint_names, joint_positions)
            rospy.loginfo("Set model configuration status: %s" % resp.status_message)

            if resp.success:
                rospy.loginfo("Initial joint angles set successfully.")
            else:
                rospy.logwarn("Failed to set initial joint angles.")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('set_initial_joint_angles', anonymous=True)
    rospy.sleep(3.0)
    rospy.logerr('==============================')
    while not rospy.is_shutdown():
        rospy.logerr('============================== waiting')
        try:
            set_joint_angles()
            break
        except rospy.ROSInterruptException:
            pass
