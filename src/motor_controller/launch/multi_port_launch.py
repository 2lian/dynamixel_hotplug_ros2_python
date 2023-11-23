"""
This ros2 launcher will launch one node per usb port in setting_to_use.USB_u2d2_port_to_use
and transmit the corresponding parameters.

@author: Elian NEPPEL
@laboratory: Moonshot, Space Robotic Lab, Tohoku University
"""

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import time
import os
import sys

package_name = 'motor_controller'

# Add the launch directory to the Python path to import the settings without rebuilding
workspace_folder = get_package_share_directory(package_name)[:-len("/install/scare/share/scare")]
directory_to_add = f"{get_package_share_directory(package_name)}/launch_files"
directories_inside_of_build = [
    f"{get_package_share_directory(package_name)}/launch_files"
]
directories_inside_of_src = [
    f'./src/{package_name}/launch',
    '${ROS2_DYNA_WS}' + f'/src/{package_name}/launch',
    f'{workspace_folder}/src/{package_name}/launch',
]
directories_to_try = directories_inside_of_src + directories_inside_of_build

for directory in directories_to_try:
    if os.path.exists(f'{directory}/launch_settings.py'):  # launch setting is in the folder
        directory_to_add = directory  # we use that for our pythonpath below
        break

# if directory_to_add is None:
#     raise FileNotFoundError("launch_setting not found")
if directory_to_add in directories_inside_of_build:
    print("\nWARNING \nlaunch settings loaded from build folder, not source\nWARNING \n")
    time.sleep(1)

sys.path.append(directory_to_add)
import launch_settings as setting_to_use

port_controller = [Node(
    package=package_name,
    namespace='',  # Default namespace
    executable='u2d2_dyna_controller',
    name=f'Dyna_{setting_to_use.PortAliasDic[port]}',
    arguments=['--ros-args', '--log-level', "info"],
    parameters=[{
        'UsbPort': port,
        'PortAlias': setting_to_use.PortAliasDic[port],
        'IdRangeMin': setting_to_use.IdRangeMin, 'IdRangeMax': setting_to_use.IdRangeMax,
        'MotorSeries': setting_to_use.MotorSeries,
        'Baudrate': setting_to_use.Baudrate,
        'FullScanPeriod': float(setting_to_use.FullScanPeriod),
        'AngleReadFreq': float(setting_to_use.AngleReadFreq),
        'AngleWriteFreq': float(setting_to_use.AngleWriteFreq),
    }]
) for port in setting_to_use.USB_u2d2_port_to_use]

nodeList = port_controller + [Node(
    package=package_name,
    namespace='',  # Default namespace
    executable='angle_remapper',
    name=f'angle_remapper',
    arguments=['--ros-args', '--log-level', "info"],
    parameters=[{
        'TimeToReach': float(setting_to_use.TimeToReach),
    }]
)]

def generate_launch_description():
    return LaunchDescription(
        nodeList
    )
