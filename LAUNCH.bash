#!/bin/bash
# run this inside the dynamixel_hotplug_ros2_python folder
set -e -o pipefail

source /opt/ros/humble/setup.bash || source /opt/ros/foxy/setup.bash || echo Ros2 Humble or Foxy not found, skipped
# rm -rf ./build
# rm -rf ./install
# rm -rf ./log
# colcon build --symlink-install
colcon build
. install/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export RCUTILS_COLORIZED_OUTPUT=1
#export ROS_DOMAIN_ID=58
ros2 launch src/motor_controller/launch/multi_port_launch.py
