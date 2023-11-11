from launch import LaunchDescription
from launch_ros.actions import Node
import sys

package_name = 'motor_controller'

# Add the launch directory to the Python path to import the settings without rebuilding
directory_to_add = f'./src/{package_name}/launch'
sys.path.append(directory_to_add)
import moonbot_setting as setting_to_use

port_controller = [Node(
    package=package_name,
    namespace='',  # Default namespace
    executable='multi_dynamixel',
    name=f'multi_dynamixel_usb_{port}',
    arguments=['--ros-args', '--log-level', "info"],
    parameters=[{
        'UsbPortNumber': port,
        'IdRangeMin': setting_to_use.IdRangeMin, 'IdRangeMax': setting_to_use.IdRangeMax,
        'Baudrate': setting_to_use.Baudrate,
    }]
) for port in setting_to_use.USB_u2d2_port_to_use]

nodeList = port_controller


def generate_launch_description():
    return LaunchDescription(
        nodeList
    )
