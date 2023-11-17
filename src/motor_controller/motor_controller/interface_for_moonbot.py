"""
Contains one ros2 node responsible for controlling several dynamixels connected to the same usb-u2d2 controller

@author: Elian NEPPEL
@laboratory: Moonshot, Space Robotic Lab, Tohoku University
"""

import time
import numpy as np
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from dyna_controller_messages.msg import AngleTime

import python_package_include.multi_controller as my_controller
import python_package_include.moonbot_motor_mapping as my_mapping
import serial
import termios

port_motor_map = my_mapping.port_motor_map

def error_catcher(func):
    # This is a wrapper to catch and display exceptions
    # Python exceptions don't work because of ros2's multithreading
    # This func cannot be imported for some reasons
    # No need to use it on the __init__ because this part is not threaded
    def wrap(*args, **kwargs):
        try:
            out = func(*args, **kwargs)
        except Exception as exception:
            if exception is KeyboardInterrupt:
                raise exception
            else:
                traceback_logger_node = Node('node_class_traceback_logger')
                traceback_logger_node.get_logger().error(traceback.format_exc())
                raise exception
        return out

    return wrap

class JointCallbackHolder:
    def __init__(self, leg_num, joint_num, delta_time, parent_node):
        self.leg_num = leg_num
        self.joint_num = joint_num
        self.parent_node = parent_node
        self.cbkgrp = ReentrantCallbackGroup()
        self.delta_time = delta_time

        self.port, self.motor_id = port_motor_map[(leg_num, joint_num)]

        self.highlevel_sub = self.parent_node.create_subscription(Float64, f"set_joint_{self.leg_num}_{self.joint_num}_real", self.from_highlevel_to_dyna, 10, callback_group=self.cbkgrp)
        self.to_dyna_pub = self.parent_node.create_publisher(AngleTime, f"set_port_{self.port}_mot_{self.motor_id}", 10, callback_group=self.cbkgrp)

        self.dyna_sub = self.parent_node.create_subscription(Float64, f"angle_port_{self.port}_mot_{self.motor_id}", self.from_dyna_to_highlevel, 10, callback_group=self.cbkgrp)
        self.to_highlevel_pub = self.parent_node.create_publisher(Float64, f"angle_{self.leg_num}_{self.joint_num}", 10, callback_group=self.cbkgrp)

    def from_highlevel_to_dyna(self, msg):
        angle = msg.data
        new_msg = AngleTime()
        new_msg.angle = msg.data
        new_msg.seconds = self.delta_time + 0.1
        self.to_dyna_pub.publish(new_msg)

    def from_dyna_to_highlevel(self, msg):
        angle = msg.data
        new_msg = Float64()
        new_msg.data = msg.data
        self.to_highlevel_pub.publish(new_msg)

class MoonbotInterface(Node):

    def __init__(self):
        super().__init__(f'MultiDynamixel_node')

        joint_cbk_hldr_arr = np.empty((4,3), dtype=object)

        ############   V ros2 parameters V
        #   \  /   #
        #    \/    #
        self.declare_parameter('AngleUpdateRate', 10.0)
        self.AngleUpdateRate = self.get_parameter('AngleUpdateRate').get_parameter_value().double_value
        #    /\    #
        #   /  \   #
        ############   ^ ros2 parameters ^

        ############   V CallbackHolder V
        #   \  /   #
        #    \/    #
        for leg in range(4):
            for joint in range(3):
                joint_cbk_hldr_arr[leg, joint] = JointCallbackHolder(leg, joint, 1/self.AngleUpdateRate, self)
        #    /\    #
        #   /  \   #
        ############   ^ CallbackHolder ^

        ############   V Service V
        #   \  /   #
        #    \/    #
        self.iAmAlive = self.create_service(Empty, f'dynamixel_interface_alive', lambda: None)
        #    /\    #
        #   /  \   #
        ############   ^ Service ^

        


def main(args=None):
    rclpy.init()

    node = MoonbotInterface()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt as e:
        node.get_logger().info('KeyboardInterrupt, shutting down, bye bye <3')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
