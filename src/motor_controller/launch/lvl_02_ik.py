from launch import LaunchDescription
from launch_ros.actions import Node
import sys

package_name = 'motor_controller'

# Add the launch directory to the Python path to import the settings without rebuilding
directory_to_add = f'./src/{package_name}/launch'
sys.path.append(directory_to_add)
import launch_setting

ik_node_list = [Node(
    package=package_name,
    namespace='',  # Default namespace
    executable='multi_dynamixel',
    name=f'multi_dynamixel_usb_{leg}',
    arguments=['--ros-args', '--log-level', "info"],
    parameters=[{
        'UsbPortNumber': leg,
    }]
) for leg in [0, 1, 2, 3, 4]]

nodeList = ik_node_list

def generate_launch_description():
    return LaunchDescription(
        nodeList
    )
