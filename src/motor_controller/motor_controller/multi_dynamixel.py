import time
import numpy as np
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

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


class MultiDynamixel(Node):

    @error_catcher
    def __init__(self):
        super().__init__(f'MultiDynamixel_node')

        ############   V ros2 parameters V
        #   \  /   #
        #    \/    #
        self.declare_parameter('UsbPortNumber', 1)
        self.UsbPortNumber = self.get_parameter('UsbPortNumber').get_parameter_value().integer_value

        self.declare_parameter('Baudrate', 57_600)
        self.BAUDRATE = self.get_parameter('Baudrate').get_parameter_value().integer_value

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

        ############   V Publishers V
        #   \  /   #
        #    \/    #
        # self.joint_pub_list = [None] * 3
        # for joint in range(3):
        #     pub = self.create_publisher(Float64, f'set_joint_{self.leg_num}_{joint}_real', 10)
        #     self.joint_pub_list[joint] = pub
        #    /\    #
        #   /  \   #
        ############   ^ Publishers ^

        ############   V Subscribers V
        #   \  /   #
        #    \/    #
        # self.sub_rel_target = self.create_subscription(Float64, f'Motor_01',
        #                                                self.set_motor_1,
        #                                                10
        #                                                )
        #    /\    #
        #   /  \   #
        ############   ^ Subscribers ^

        ############   V Service V
        #   \  /   #
        #    \/    #
        self.iAmAlive = self.create_service(Empty, f'usb_{self.UsbPortNumber}_alive', lambda: None)
        #    /\    #
        #   /  \   #
        ############   ^ Service ^
        grp1 = MutuallyExclusiveCallbackGroup()
        grp2 = MutuallyExclusiveCallbackGroup()

        self.search_timer = self.create_timer(0.5, self.search_for_motors, callback_group=grp1)
        self.delete_the_dead_timer = self.create_timer(0.5, self.delete_the_dead, callback_group=grp1)
        self.move_dtime = 0.25
        self.move_timer = self.create_timer(self.move_dtime, self.wave_test, callback_group=grp1)
        self.sin_amplitude = 0.5  # rad
        self.sin_amplitude = min(self.sin_amplitude, 2 * np.pi)
        self.sin_period = 10  # sec
        self.last_index_checked = 0

    @error_catcher
    def search_for_motors(self):
        """
        pings one id that belong to a not yet connected motor
        :return:
        """
        list_of_alive_motors = [motor.id for motor in self.controller.motor_list]
        for i in self.id_range:
            self.last_index_checked = (1 + self.last_index_checked) % len(self.id_range)
            id_to_check = self.id_range[self.last_index_checked]
            if id_to_check in list_of_alive_motors:
                pass  # will try the next id
            else:
                # pings the id and returns
                motor_found = self.controller.refresh_motors([id_to_check])
                if motor_found:
                    self.get_logger().warning(f"Port {self.UsbPortNumber}: Motor {motor_found} found :)")
                return

    @error_catcher
    def delete_the_dead(self):
        disconnected_motors = self.controller.delete_dead_motors()
        if disconnected_motors:
            self.get_logger().warning(f"Port {self.UsbPortNumber}: Motor {disconnected_motors} lost")

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
    def set_motor_1(self, msg):
        angle = msg.data
        return

    @error_catcher
    def close_port(self):
        self.portHandler.closePort()
        return

    @error_catcher
    def connect_and_setup(self, delete_controller=True):
        connected = False
        if delete_controller:
            del self.controller
        self.portHandler = my_controller.PortHandler(self.DEVICENAME)
        self.packetHandler = my_controller.PacketHandler(self.PROTOCOL_VERSION)
        self.groupBulkRead = my_controller.GroupBulkRead(self.portHandler, self.packetHandler)
        self.groupBulkWrite = my_controller.GroupBulkWrite(self.portHandler, self.packetHandler)

        while not connected:
            # Open port
            opened = False
            try:
                opened = self.portHandler.openPort()
            except:
                pass
            if opened:
                connected = True
                self.get_logger().info(f"Port {self.UsbPortNumber}: opened :)")
                break
            else:
                self.get_logger().warning(f"Port {self.UsbPortNumber}: failed to open", once=True)
            time.sleep(1)

        connected = False
        while not connected:
            # self.BAUDRATE = my_controller.motor_table["X_SERIES"]["BAUDRATE"]
            # Set port baudrate
            if self.portHandler.setBaudRate(self.BAUDRATE):
                connected = True
                self.get_logger().info(f"Baudrate [{self.BAUDRATE}] set :)")
                break
            else:
                self.get_logger().warning("Failed to change the baudrate")
            time.sleep(1)

        self.controller = my_controller.MotorHandler(packetHandler=self.packetHandler,
                                                     portHandler=self.portHandler,
                                                     groupBulkRead=self.groupBulkRead,
                                                     groupBulkWrite=self.groupBulkWrite,
                                                     deviceName=self.DEVICENAME,
                                                     motor_series="X_SERIES")

        for k in self.id_range:
            self.search_for_motors()


def main(args=None, dotheinit=True):
    if dotheinit:
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
            node.get_logger().debug('KeyboardInterrupt caught, node shutting down cleanly\nbye bye <3')
            node.destroy_node()
            rclpy.shutdown()
            break
        except serial.serialutil.SerialException as e:
            node.get_logger().error('Serial error occurred')
            node.portHandler.closePort()
            node.get_logger().info('Port closed and restarting in 5s')
            time.sleep(5)  # if you don wait it crashes the jetson
            node.get_logger().info('restarting')
        except termios.error as e:
            node.get_logger().error('Termios error occurred')
            node.portHandler.closePort()
            node.get_logger().info('Port closed and restarting in 5s')
            time.sleep(5)  # if you don wait it crashes the jetson
            node.get_logger().info('restarting')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
