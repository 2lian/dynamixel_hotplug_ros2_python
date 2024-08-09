#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Converts simple Angle (float) topic to AngleTime (float, float) by always sending the same time to reach the target.
Also maps subscribers and publisher name according to python_package_include.topic_remapping.py

@author: Elian NEPPEL
@laboratory: Moonshot, Space Robotic Lab, Tohoku University
"""
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from dyna_controller_messages.msg import AngleTime

# import python_package_include.topic_remapping as my_mapping
from python_package_include.topic_remapping import *

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
                traceback_logger_node = Node("node_class_traceback_logger")
                traceback_logger_node.get_logger().error(traceback.format_exc())
                raise exception
        return out

    return wrap


class JointCallbackHolder:
    def __init__(
        self, higher_level_sub_name, higher_level_pub_name, delta_time, parent_node
    ):
        self.parent_node = parent_node
        self.cbkgrp = ReentrantCallbackGroup()
        self.delta_time = delta_time
        self.transform_function = input_shaping_map[higher_level_sub_name]
        self.untransform_function = output_shaping_map[higher_level_sub_name]
        self.offset = offset_map[higher_level_sub_name]
        self.gain = gain_map[higher_level_sub_name]

        self.highlevel_sub = self.parent_node.create_subscription(
            Float64,
            higher_level_sub_name,
            self.from_highlevel_to_dyna,
            10,
            callback_group=self.cbkgrp,
        )
        self.to_dyna_pub = self.parent_node.create_publisher(
            AngleTime, set_map[higher_level_sub_name], 10, callback_group=self.cbkgrp
        )

        self.dyna_sub = self.parent_node.create_subscription(
            Float64,
            angle_map[higher_level_pub_name],
            self.from_dyna_to_highlevel,
            10,
            callback_group=self.cbkgrp,
        )
        self.to_highlevel_pub = self.parent_node.create_publisher(
            Float64, higher_level_pub_name, 10, callback_group=self.cbkgrp
        )

    @error_catcher
    def from_highlevel_to_dyna(self, msg):
        new_msg = AngleTime()
        new_msg.angle = (self.transform_function(msg.data) + self.offset) * self.gain
        new_msg.seconds = self.delta_time
        self.to_dyna_pub.publish(new_msg)

    @error_catcher
    def from_dyna_to_highlevel(self, msg):
        new_msg = Float64()
        new_msg.data = self.untransform_function(msg.data / self.gain - self.offset)
        self.to_highlevel_pub.publish(new_msg)


class AngleRemapper(Node):

    def __init__(self):
        super().__init__(f"AngleRemapper")

        cbk_hldr_list = []

        ############   V ros2 parameters V
        #   \  /   #
        #    \/    #
        self.declare_parameter("TimeToReach", 1 / 20 + 0.1)
        self.TimeToReach = (
            self.get_parameter("TimeToReach").get_parameter_value().double_value
        )
        #    /\    #
        #   /  \   #
        ############   ^ ros2 parameters ^

        ############   V CallbackHolder V
        #   \  /   #
        #    \/    #
        for k in range(len(angle_map)):
            higher_level_sub_name = list(set_map.keys())[k]
            higher_level_pub_name = list(angle_map.keys())[k]
            cbk_hldr_list.append(
                JointCallbackHolder(
                    higher_level_sub_name, higher_level_pub_name, self.TimeToReach, self
                )
            )
        #    /\    #
        #   /  \   #
        ############   ^ CallbackHolder ^

        ############   V Service V
        #   \  /   #
        #    \/    #
        self.iAmAlive = self.create_service(Empty, f"remapper_alive", lambda: None)
        #    /\    #
        #   /  \   #
        ############   ^ Service ^


def main(args=None):
    rclpy.init()

    node = AngleRemapper()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt as e:
        node.get_logger().info("KeyboardInterrupt, shutting down, bye bye <3")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
