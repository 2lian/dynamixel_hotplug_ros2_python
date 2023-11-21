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
import serial
import termios


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

class CallbackHolder:
    def __init__(self, motor_number:int, curr_angle_dic:dict, MotorHandler, parent_node):
        self.motor_number = motor_number
        self.curr_angle_dic = curr_angle_dic
        self.MotorHandler = MotorHandler
        self.parent_node = parent_node

        # self.sub = self.parent_node.create_subscription(Float64, f"set_port_{self.parent_node.UsbPortNumber}_mot_{self.motor_number}", self.angle_cbk, 10)
        self.sub = self.parent_node.create_subscription(AngleTime, f"set_port_{self.parent_node.UsbPortNumber}_mot_{self.motor_number}", self.angle_time_cbk, 1, callback_group=ReentrantCallbackGroup())
        self.pub = self.parent_node.create_publisher(Float64, f"angle_port_{self.parent_node.UsbPortNumber}_mot_{self.motor_number}", 10, callback_group=ReentrantCallbackGroup())
        self.new_target_available = False
        self.target_angle = None
        self.target_time = None


    def write_target_time(self, angle, deltatime):
        self.target_angle = angle
        self.target_time = deltatime
        self.new_target_available = True

    def angle_cbk(self, msg):
        angle = msg.data
        self.write_target_time(angle, deltatime=1)

    def angle_time_cbk(self, msg):
        angle = msg.angle
        deltatime = msg.seconds
        self.write_target_time(angle, deltatime)

    def publish_current_angle(self):
        angle = self.curr_angle_dic[self.motor_number]
        if np.isnan(angle):
            self.parent_node.get_logger().warning(f"Port {self.parent_node.UsbPortNumber} Motor {self.motor_number}: "
                                                  f"Invalid angle (not published)")
            return
        msg = Float64()
        msg.data = angle
        self.pub.publish(msg)
        


class MultiDynamixel(Node):
    """
    Handles communication between ros2 and ONE u2d2 port with several dynamixel motors that can be plugged/unplugged
    """
    def __init__(self):
        super().__init__(f'MultiDynamixel_node')

        ############   V ros2 parameters V
        #   \  /   #
        #    \/    #
        self.declare_parameter('UsbPortNumber', 1)
        self.UsbPortNumber = self.get_parameter('UsbPortNumber').get_parameter_value().integer_value

        self.declare_parameter('Baudrate', 57_600)
        self.BAUDRATE = self.get_parameter('Baudrate').get_parameter_value().integer_value

        self.declare_parameter('MotorSeries', "X_SERIES")
        self.MotorSeries = self.get_parameter('MotorSeries').get_parameter_value().string_value

        self.declare_parameter('IdRangeMin', 1)
        self.IdRangeMin = self.get_parameter('IdRangeMin').get_parameter_value().integer_value
        self.declare_parameter('IdRangeMax', 3)
        self.IdRangeMax = self.get_parameter('IdRangeMax').get_parameter_value().integer_value
        self.id_range = list(range(self.IdRangeMin, self.IdRangeMax + 1))
        #    /\    #
        #   /  \   #
        ############   ^ ros2 parameters ^

        # Use the actual port assigned to the U2D2.
        # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
        self.DEVICENAME = f'/dev/ttyUSB{self.UsbPortNumber}'

        self.PROTOCOL_VERSION = 2.0

        # Will be defined when connecting
        self.controller = None
        self.groupBulkWrite = None
        self.groupBulkRead = None
        self.packetHandler = None
        self.portHandler = None
        self.bypass_alive_check = False

        self.curr_angle_dic = {}
        self.callback_holder_dic = {}

        ############   V Service V
        #   \  /   #
        #    \/    #
        self.iAmAlive = self.create_service(Empty, f'usb_{self.UsbPortNumber}_alive', lambda: None)
        #    /\    #
        #   /  \   #
        ############   ^ Service ^
        grp1 = MutuallyExclusiveCallbackGroup()
        grp2 = MutuallyExclusiveCallbackGroup()

        time_to_search_all_id = 2
        search_delta_t = time_to_search_all_id/len(self.id_range)

        ############   V Timers V
        #   \  /   #
        #    \/    #
        self.search_timer = self.create_timer(search_delta_t, self.search_for_next_motor, callback_group=grp1)
        self.delete_the_dead_timer = self.create_timer(2, self.delete_the_dead, callback_group=grp1)
        self.refresh_and_publish_angle_timer = self.create_timer(0.1, self.refresh_and_publish_angles, callback_group=grp1)
        self.send_writen_angles_timer = self.create_timer(0.01, self.send_writen_angles, callback_group=grp1)
        #    /\    #
        #   /  \   #
        ############   ^ Timers ^
        # self.move_dtime = 0.1
        # # self.move_timer = self.create_timer(self.move_dtime, self.wave_test, callback_group=grp1)
        # self.sin_amplitude = 0.5  # rad
        # self.sin_amplitude = min(self.sin_amplitude, 2 * np.pi)
        # self.sin_period = 5  # sec
        # self.last_index_checked = 0
#
        # self.refresh_and_publish_angle_timer = self.create_timer(0.1, self.refresh_and_publish_angles, callback_group=grp1)
        # self.send_writen_angles_timer = self.create_timer(0.01, self.send_writen_angles, callback_group=grp1)
        # self.last = 0


    @error_catcher
    def send_writen_angles(self):

        there_is_something_to_publish = False

        for my_motor in self.controller.motor_list:
            corresponding_cbk_holder = self.callback_holder_dic[my_motor.id]
            if corresponding_cbk_holder.new_target_available:

                if not there_is_something_to_publish: # new angle is here and there is work to be done
                    self.refresh_and_publish_angles()
                    self.refresh_and_publish_angle_timer.reset()
                    there_is_something_to_publish = True

                target_angle = corresponding_cbk_holder.target_angle
                delta_time = corresponding_cbk_holder.target_time
                delta_time = np.clip(delta_time, a_min=0.0001, a_max=None) # avoid division by zero and negative values
                current_pos = self.curr_angle_dic[my_motor.id]

                speed = abs((target_angle - current_pos) / delta_time)
                my_motor.write_max_speed(speed)

        if there_is_something_to_publish:
            # now = time.time()
            # self.get_logger().warning(f"{now - self.last:.2f}")
            # self.last = now
            self.controller.publish()

            for my_motor in self.controller.motor_list:
                corresponding_cbk_holder = self.callback_holder_dic[my_motor.id]

                if corresponding_cbk_holder.new_target_available:
                    target_angle = corresponding_cbk_holder.target_angle
                    comm_fail = my_motor.write_position(target_angle)
                    if not comm_fail:
                        corresponding_cbk_holder.new_target_available = False

            self.controller.publish()


    @error_catcher
    def search_for_next_motor(self):
        """
        pings one id that belong to a not yet connected motor
        :return:
        """
        list_of_alive_motors = self.controller.get_motor_id_in_order()
        for i in self.id_range:
            self.last_index_checked = (1 + self.last_index_checked) % len(self.id_range)
            id_to_check = self.id_range[self.last_index_checked]
            if id_to_check in list_of_alive_motors:
                pass  # will try the next id
            else:
                # pings the id and returns
                motor_found = self.search_motors([id_to_check])
                if motor_found:
                    self.get_logger().warning(f"Port {self.UsbPortNumber}: Motor {motor_found} found :)")
                    self.search_for_next_motor()
                return

    @error_catcher
    def search_motors(self, id_range: list):
        motor_found = self.controller.refresh_motors(id_range)
        if motor_found:
            for motor_id in motor_found:
                self.add_new_motor(motor_id)
        return motor_found

    @error_catcher
    def add_new_motor(self, motor_id):
        self.curr_angle_dic[motor_id] = None
        self.callback_holder_dic[motor_id] = CallbackHolder(motor_number = motor_id,
                                                            curr_angle_dic = self.curr_angle_dic, 
                                                            MotorHandler = self.controller, 
                                                            parent_node = self,
        )

    @error_catcher
    def delete_motor(self, motor_id):
        del self.curr_angle_dic[motor_id]
        self.destroy_subscription(self.callback_holder_dic[motor_id].sub)
        self.destroy_publisher(self.callback_holder_dic[motor_id].pub)
        del self.callback_holder_dic[motor_id]

    @error_catcher
    def refresh_all_current_angles(self):
        angle_arr = self.controller.get_angles()
        for arr_index, motor_id in enumerate(self.controller.get_motor_id_in_order()):
            self.curr_angle_dic[motor_id] = angle_arr[arr_index]
        # self.get_logger().warning(f"{self.curr_angle_dic}")

    @error_catcher
    def refresh_and_publish_angles(self):
        self.refresh_all_current_angles()
        for cbk_holder in self.callback_holder_dic.values():
            cbk_holder.publish_current_angle()

    @error_catcher
    def delete_the_dead(self):
        disconnected_motors = self.controller.delete_dead_motors()
        if disconnected_motors:
            self.get_logger().warning(f"Port {self.UsbPortNumber}: Motor {disconnected_motors} lost")
            for motor_id in disconnected_motors:
                self.delete_motor(motor_id)

    @error_catcher
    def wave_test(self):
        period = self.sin_period
        x = (time.time() % period) / period * 2 * np.pi
        angle = my_controller.wave(x) / (2 * np.pi) * self.sin_amplitude
        angle += np.pi - self.sin_amplitude / 2
        # print(angle)
        comm_success = self.controller.broadcast_target_on_time(angle, self.move_dtime + 0.1)
        if not comm_success:
            self.delete_the_dead()

    @error_catcher
    def close_port(self):
        self.portHandler.closePort()

    @error_catcher
    def connect_and_setup(self, delete_controller=True):

        rate = self.create_rate(1)

        if delete_controller:
            del self.controller
        self.portHandler = my_controller.PortHandler(self.DEVICENAME)
        self.packetHandler = my_controller.PacketHandler(self.PROTOCOL_VERSION)
        self.groupBulkRead = my_controller.GroupBulkRead(self.portHandler, self.packetHandler)
        self.groupBulkWrite = my_controller.GroupBulkWrite(self.portHandler, self.packetHandler)

        while 1:  # Open port
            opened = False
            try:
                opened = self.portHandler.openPort()
            except serial.serialutil.SerialException as e:
                self.get_logger().warning(f'Port {self.UsbPortNumber}: Serial error on opening, port may not exist')
            if opened:
                self.get_logger().info(f"Port {self.UsbPortNumber}: opened :)")
                break
            else:
                self.get_logger().warning(f"Port {self.UsbPortNumber}: failed to open", once=True)
            rate.sleep()

        while 1:  # Set port baudrate
            if self.portHandler.setBaudRate(self.BAUDRATE):
                self.get_logger().info(f"Baudrate [{self.BAUDRATE}] set :)")
                break
            else:
                self.get_logger().warning("Failed to change the baudrate", once=True)
            rate.sleep()

        self.controller = my_controller.MotorHandler(packetHandler=self.packetHandler,
                                                     portHandler=self.portHandler,
                                                     groupBulkRead=self.groupBulkRead,
                                                     groupBulkWrite=self.groupBulkWrite,
                                                     deviceName=self.DEVICENAME,
                                                     motor_series=self.MotorSeries)

        motor_found = self.search_motors(self.id_range)
        if motor_found:
            self.get_logger().warning(f"Port {self.UsbPortNumber}: Motor {motor_found} found :)")
        else:
            self.get_logger().warning(f"Port {self.UsbPortNumber}: NO MOTOR, but setup successful :)")


def main(args=None):
    rclpy.init()

    node = MultiDynamixel()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    already_setup = False

    while 1:
        try:
            node.connect_and_setup(delete_controller=already_setup)
            already_setup = True
            executor.spin()
        except KeyboardInterrupt as e:
            node.get_logger().info('KeyboardInterrupt, shutting down, bye bye <3')
            break
        except serial.serialutil.SerialException as e:
            node.get_logger().error('Serial error occurred')
            node.portHandler.closePort()
            node.get_logger().info('Port closed and restarting in 5s')
            time.sleep(5)
            node.get_logger().info('restarting')
        except termios.error as e:
            node.get_logger().error('Termios error occurred')
            node.portHandler.closePort()
            node.get_logger().info('Port closed and restarting in 5s')
            time.sleep(5)
            node.get_logger().info('restarting')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
