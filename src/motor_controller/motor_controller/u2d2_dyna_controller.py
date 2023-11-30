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
    # wrapper to catch and display exceptions in ros2 log
    # This func cannot be imported for some reasons
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


class MotorCallbackHolder:
    """
    Represents a single motor and its corresponding callbacks
    """
    def __init__(self, motor_number: int,  MotorHandler, parent_node):
        self.motor_number = motor_number
        self.MotorHandler = MotorHandler
        self.parent_node = parent_node

        self.subscriber = self.parent_node.create_subscription(AngleTime,
                                                        f"set_{self.parent_node.PortAlias}_mot_{self.motor_number}",
                                                        self.angle_time_cbk, 1, callback_group=ReentrantCallbackGroup())
        self.publisher = self.parent_node.create_publisher(Float64,
                                                     f"angle_{self.parent_node.PortAlias}_mot_{self.motor_number}",
                                                     10, callback_group=ReentrantCallbackGroup())
        self.new_target_available = False
        self.target_angle = None
        self.target_time = None
        self.current_angle = None
        self.last_valid_angle = None

    def write_target_time(self, angle, deltatime):
        """
        Write the given target and time in the shared target dictionary
        Those angle+time values are then applied by a timer in the main node
        :param angle: angle to reach
        :param deltatime: time to reach
        :return:
        """
        self.target_angle = angle
        self.target_time = deltatime
        self.new_target_available = True

    def angle_time_cbk(self, msg):
        """
        Callback executed when a target angle+time arrives
        :param msg: custom message AngleTime
        :return:
        """
        angle = msg.angle
        deltatime = msg.seconds
        self.write_target_time(angle, deltatime)

    def publish_current_angle(self):
        """
        Publishes the corresponding angle stored in the shared curr_angle_dic
        :return:
        """
        angle = self.current_angle
        if np.isnan(angle):
            self.parent_node.get_logger().warning(f"Port {self.parent_node.PortAlias} Motor {self.motor_number}: "
                                                  f"Invalid angle (not published)")
            return
        msg = Float64()
        msg.data = angle
        self.publisher.publish(msg)


class U2D2DynaController(Node):
    """
    Handles communication between ros2 and ONE u2d2 port with several dynamixel motors that can be plugged/unplugged
    """

    def __init__(self):
        super().__init__(f'U2D2DynaController')

        ############   V ros2 parameters V
        #   \  /   #
        #    \/    #
        self.declare_parameter('UsbPort', '/dev/ttyUSB2')
        self.UsbPort = self.get_parameter('UsbPort').get_parameter_value().string_value

        self.declare_parameter('PortAlias', f'port_{self.UsbPort[-1]}')
        self.PortAlias = self.get_parameter('PortAlias').get_parameter_value().string_value

        self.declare_parameter('Baudrate', 4_000_000)
        self.BAUDRATE = self.get_parameter('Baudrate').get_parameter_value().integer_value

        self.declare_parameter('MotorSeries', "X_SERIES")
        self.MotorSeries = self.get_parameter('MotorSeries').get_parameter_value().string_value

        self.declare_parameter('IdRangeMin', 1)
        self.IdRangeMin = self.get_parameter('IdRangeMin').get_parameter_value().integer_value
        self.declare_parameter('IdRangeMax', 10)
        self.IdRangeMax = self.get_parameter('IdRangeMax').get_parameter_value().integer_value
        self.id_range = list(range(self.IdRangeMin, self.IdRangeMax + 1))

        self.declare_parameter('FullScanPeriod', 2.0)
        self.FullScanPeriod = self.get_parameter('FullScanPeriod').get_parameter_value().double_value
        self.declare_parameter('CleanupPeriod', 2.0)
        self.CleanupPeriod = self.get_parameter('CleanupPeriod').get_parameter_value().double_value
        self.declare_parameter('AngleReadFreq', 10.0)
        self.AngleReadFreq = self.get_parameter('AngleReadFreq').get_parameter_value().double_value
        self.declare_parameter('AngleWriteFreq', 100.0)
        self.AngleWriteFreq = self.get_parameter('AngleWriteFreq').get_parameter_value().double_value

        #    /\    #
        #   /  \   #
        ############   ^ ros2 parameters ^

        # Use the actual port assigned to the U2D2.
        # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
        self.DEVICENAME = self.UsbPort

        self.PROTOCOL_VERSION = 2.0

        # Will be defined when connecting
        self.controller = None
        self.groupBulkWrite = None
        self.groupBulkRead = None
        self.packetHandler = None
        self.portHandler = None
        self.bypass_alive_check = False
        self.last_index_checked = 0

        self.motor_cbk_holder_dict = {}

        ############   V Service V
        #   \  /   #
        #    \/    #
        self.iAmAlive = self.create_service(Empty, f'usb_{self.PortAlias}_alive', lambda: None)
        #    /\    #
        #   /  \   #
        ############   ^ Service ^
        grp1 = MutuallyExclusiveCallbackGroup()

        time_to_search_all_id = self.FullScanPeriod  # will loop over all searched motor id in this specified time in seconds
        search_delta_t = time_to_search_all_id / len(self.id_range)

        ############   V Timers V
        #   \  /   #
        #    \/    #
        self.search_timer = self.create_timer(search_delta_t, self.search_for_next_motor, callback_group=grp1)
        self.delete_the_dead_timer = self.create_timer(self.CleanupPeriod, self.clean_dead_motors, callback_group=grp1)
        self.refresh_and_publish_angle_timer = self.create_timer(1/self.AngleReadFreq, self.refresh_and_publish_angles,
                                                                 callback_group=grp1)
        self.send_writen_angles_timer = self.create_timer(1/self.AngleWriteFreq, self.send_writen_angles, callback_group=grp1)
        #    /\    #
        #   /  \   #
        ############   ^ Timers ^

    @error_catcher
    def send_writen_angles(self):
        """
        Sends target angle held by the motor_cbk_holder(s) to the motors using bulkwrite.
        Target are stored in the motor_cbk_holder as a buffer then all sent together by this function.
        This makes the subscriber asynchronous while the serial port stays synchronous and able to send data in bulk.

        Calling this function (in the event of a new target) will also refresh and publish the current motor position.
        :return:
        """
        something_to_publish = False

        for my_motor in self.controller.motor_list:  # Prepare speed message
            corresponding_cbk_holder = self.motor_cbk_holder_dict[my_motor.id]
            if corresponding_cbk_holder.new_target_available:

                if not something_to_publish:  # bulk angle update should only be done once
                    self.refresh_and_publish_angles()  # making this async would be good
                    self.refresh_and_publish_angle_timer.reset()
                    something_to_publish = True

                target_angle = corresponding_cbk_holder.target_angle
                delta_time = corresponding_cbk_holder.target_time
                delta_time = np.clip(delta_time, a_min=0.0001, a_max=None)  # avoid division by zero and negative values
                current_pos = corresponding_cbk_holder.last_valid_angle

                speed = abs((target_angle - current_pos) / delta_time)
                my_motor.write_max_speed(speed)

        if something_to_publish:
            self.controller.publish()  # sends speed to motors

            for my_motor in self.controller.motor_list:  # Prepare angle message
                corresponding_cbk_holder = self.motor_cbk_holder_dict[my_motor.id]

                if corresponding_cbk_holder.new_target_available:
                    target_angle = corresponding_cbk_holder.target_angle
                    comm_fail = my_motor.write_position(target_angle)
                    if not comm_fail:
                        corresponding_cbk_holder.new_target_available = False

            self.controller.publish()  # sends angle to motors

    @error_catcher
    def search_for_next_motor(self):
        """
        Pings all ids with no associated motors until a ping is unsuccessful
        Tries next id on the next call of the function
        :return:
        """
        list_of_alive_motors = self.controller.get_motor_id_in_order()
        id_found_list = []
        for i in self.id_range:
            self.last_index_checked = (1 + self.last_index_checked) % len(self.id_range)
            id_to_check = self.id_range[self.last_index_checked]
            if id_to_check in list_of_alive_motors:  # If known motor
                pass  # loops and tries the next id
            else:  # If unknown motor
                motor_found_list = self.search_motors([id_to_check])
                if motor_found_list:  # if list not empty
                    id_found_list.append(motor_found_list[0])
                    # then loops and tries next id
                else:  # List empty: so it terminates once a ping is unsuccessful
                    break
        if id_found_list:  # if one motor in the list, we log
            self.get_logger().warning(f"Port {self.PortAlias}: Motor {id_found_list} found :)")
        return

    @error_catcher
    def search_motors(self, id_range: list):
        """
        Searches for motors with ids in the provided id_range
        Then begins to handle them. The controller should automatically handle new motors found its own search
        :param id_range:
        :return:
        """
        motor_found = self.controller.refresh_motors(id_range)
        if motor_found:
            for motor_id in motor_found:
                self.add_new_motor(motor_id)
        return motor_found

    @error_catcher
    def add_new_motor(self, motor_id):
        """
        Motor with the id will have its callback holder created
        :param motor_id:
        :return:
        """
        self.motor_cbk_holder_dict[motor_id] = MotorCallbackHolder(motor_number=motor_id,
                                                                   MotorHandler=self.controller,
                                                                   parent_node=self,
                                                                   )

    @error_catcher
    def delete_motor(self, motor_id):
        """
        Destroys pub sub and associated cbk holder to the motor id
        :param motor_id:
        :return:
        """
        self.destroy_subscription(self.motor_cbk_holder_dict[motor_id].sub)
        self.destroy_publisher(self.motor_cbk_holder_dict[motor_id].pub)
        del self.motor_cbk_holder_dict[motor_id]

    @error_catcher
    def refresh_all_current_angles(self):
        """
        Bulk reads angles of all motors connected, then stores tehm in the motor_cbk_holder
        :return:
        """
        angle_arr = self.controller.get_angles()
        for arr_index, motor_id in enumerate(self.controller.get_motor_id_in_order()):
            new_angle = angle_arr[arr_index]
            self.motor_cbk_holder_dict[motor_id].current_angle = new_angle
            if not np.isnan(new_angle):
                self.motor_cbk_holder_dict[motor_id].last_valid_angle = new_angle

    @error_catcher
    def refresh_and_publish_angles(self):
        """
        Bulk reads angles of all connected motors; stores and publishes (on ros2) all of the angles
        :return:
        """
        self.refresh_all_current_angles()
        for cbk_holder in self.motor_cbk_holder_dict.values():
            cbk_holder.publish_current_angle()

    @error_catcher
    def clean_dead_motors(self):
        """
        Stops the handling of motor that are considered dead by the controller
        :return:
        """
        self.controller.delete_dead_motors()
        alive_id_list = self.controller.get_motor_id_in_order()
        disconnected_motors = [ID for ID in self.motor_cbk_holder_dict.keys() if ID not in alive_id_list]
        if disconnected_motors:
            self.get_logger().warning(f"Port {self.PortAlias}: Motor {disconnected_motors} lost")
            for motor_id in disconnected_motors:
                self.delete_motor(motor_id)

    @error_catcher
    def close_port(self):
        self.portHandler.closePort()

    @error_catcher
    def connect_and_setup(self, delete_controller=True):
        """
        Open and connects to the serial port. Creates the corresponding MotorHandler. Scans for all motors.
        :param delete_controller: force deletes the previous controller to not rely on the garbage collector
        :return:
        """

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
                self.get_logger().warning(f'Port {self.PortAlias}: Serial error on opening, port may not exist')
            if opened:
                self.get_logger().info(f"Port {self.PortAlias}: opened :)")
                break
            else:
                self.get_logger().warning(f"Port {self.PortAlias}: failed to open", once=True)
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
            self.get_logger().warning(f"Port {self.PortAlias}: Motor {motor_found} found :)")
        else:
            self.get_logger().warning(f"Port {self.PortAlias}: NO MOTOR, but setup successful :)")


def main(args=None):
    rclpy.init()

    node = U2D2DynaController()
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
