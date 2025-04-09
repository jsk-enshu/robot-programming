#!/usr/bin/env python3

import time

import rospkg
from rcb4.armh7interface import ARMH7Interface
import yaml
import os.path as osp
import sys
from colorama import Fore

rospack = rospkg.RosPack()
package = 'jedy_description'
try:
    rospackage_path = rospack.get_path(package)
except rospkg.ResourceNotFound as e:
    print(Fore.RED)
    print(f"The ROS package '{package}' could not be found."
          + " You need to make sure your ROS workspace is sourced (enabled) in the terminal.\n"
          + "Try running the following command:\n\n"
          + "source ~/ros/enshu_ws/devel/setup.bash\n\n"
          + "Alternatively, run\n\n"
          + f"'roscd {package}' to check if the package is installed correctly.")
    print(Fore.RESET)
    sys.exit(1)

config_path = osp.join(
    rospackage_path,
    'config',
    'jedy_servo_config.yaml'
)

with open(config_path) as f:
    servo_config = yaml.load(f, Loader=yaml.SafeLoader)

id_to_joint_name = {}
for joint_name, data in servo_config['joint_name_to_servo_id'].items():
    if isinstance(data, dict):
        servo_id = data['id']
    else:
        servo_id = data
    id_to_joint_name[servo_id] = joint_name

interface = ARMH7Interface()
while True:
    try:
        interface.auto_open()
        servo_ids = interface.search_servo_ids()
        break
    except Exception as e:
        print(str(e))
        print('Retry.')
    time.sleep(1.0)

# Found and not found servos
found_servos = set()
for servo_id in servo_ids:
    if servo_id in id_to_joint_name:
        print(Fore.GREEN + f"Servo ID {servo_id} ({id_to_joint_name[servo_id]}) found" + Fore.RESET)
        found_servos.add(servo_id)
    else:
        print(Fore.RED + f"Servo ID {servo_id} found, but joint name not specified in configuration" + Fore.RESET)

# Check for missing servo IDs
missing_servos = set(id_to_joint_name.keys()) - found_servos
if missing_servos:
    print(Fore.RED)
    print("\nServo IDs and joint names not found:")
    for missing_id in missing_servos:
        print(f"Servo ID {missing_id} ({id_to_joint_name[missing_id]}) not found")
    print(Fore.RESET)
else:
    print(Fore.GREEN + "\nAll servo IDs in the configuration were found." + Fore.RESET)
