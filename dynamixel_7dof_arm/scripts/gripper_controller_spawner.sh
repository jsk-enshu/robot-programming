#!/bin/bash

. ~/.bashrc
FLAG=1;
while [ $FLAG -eq 1 ];
do
    rostopic info /fullbody_controller/follow_joint_trajectory/result && rostopic info /fullbody_controller/follow_joint_trajectory/goal > /dev/null;
    if [ $? -eq 0 ]; then FLAG=0;fi;
    echo "Wait for fullbody_controller";
    sleep 1;
done
echo "launch gripper_controller";
roslaunch dynamixel_7dof_arm gripper_controller_spawner.launch
