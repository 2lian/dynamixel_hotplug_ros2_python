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

        self.bypass_alive_check = False

        self.declare_parameter('UsbPortNumber', 1)
        self.UsbPortNumber = self.get_parameter('UsbPortNumber').get_parameter_value().integer_value

        # Use the actual port assigned to the U2D2.
        # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
        self.DEVICENAME = f'/dev/ttyUSB{self.UsbPortNumber}'

        # DYNAMIXEL Protocol Version (1.0 / 2.0)
        # https://emanual.robotis.com/docs/en/dxl/protocol2/
        self.PROTOCOL_VERSION = 2.0

        # self.connect_and_setup(delete_controller=False)

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
        self.move_dtime = 0.5
        self.move_timer = self.create_timer(self.move_dtime, self.move, callback_group=grp1)
        self.sin_amplitude = 0.5 # rad 
        self.sin_amplitude = min(self.sin_amplitude, 2 * np.pi)
        self.sin_period = 10 # sec
        self.last_id_checked = 0
        self.id_range = [1, 2, 3]

    @error_catcher
    def search_for_motors(self):
        self.last_id_checked = (1 + self.last_id_checked) % len(self.id_range)
        id_now = self.id_range[self.last_id_checked]
        if id_now in [motor.id for motor in self.controller.motor_list]:
            return
        result = self.controller.refresh_motors([id_now])
        if result:
            self.get_logger().warning(f"Port {self.UsbPortNumber}: Motor {result} found :)")

    @error_catcher
    def delete_the_dead(self):
        result = self.controller.delete_dead_motors()
        if result:
            self.get_logger().warning(f"Port {self.UsbPortNumber}: Motor {result} lost")

    @error_catcher
    def move(self):
        period = self.sin_period
        x = (time.time() % period) / period * 2 * np.pi
        angle = my_controller.wave(x) / (2 * np.pi) * self.sin_amplitude
        angle += np.pi - self.sin_amplitude/2
        # print(angle)
        comm_success = self.controller.broadcast_target_on_time(angle, self.move_dtime+0.1)
        if not comm_success:
            self.delete_the_dead()

    @error_catcher
    def set_motor_1(self, msg):
        angle = msg.data
        return

    def close_port(self):
        self.portHandler.closePort()
        return

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
            else:
                self.get_logger().warning(f"Port {self.UsbPortNumber}: failed to open", once=True)
            time.sleep(1)

        connected = False
        while not connected:
            self.BAUDRATE = my_controller.motor_table["X_SERIES"]["BAUDRATE"]
            # Set port baudrate
            if self.portHandler.setBaudRate(self.BAUDRATE):
                connected = True
                self.get_logger().info(f"Baudrate [{self.BAUDRATE}] set :)")
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

        # self.controller.refresh_motors()
        # while not self.controller.motor_list:
        #     self.get_logger().warning(
        #         f'''Port {self.UsbPortNumber}: no motors connected''', once=False)
        #     if self.bypass_alive_check:
        #         self.get_logger().warning(
        #             f'''Motor check bypassed :)''')
        #         break
        #     self.controller.refresh_motors()
        #     time.sleep(1)
        # if not self.bypass_alive_check:
        #     self.get_logger().warning(
        #         f'''Port {self.UsbPortNumber}: total of {len(self.controller.motor_list)} motors connected :)''')

        # self.controller.broadcast_max_speed(1)


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
            quit()
        except serial.serialutil.SerialException as e:
            node.get_logger().error('Serial error occurred')
            # node.portHandler.clearPort()
            # time.sleep(1)
            node.portHandler.closePort()
            node.get_logger().info('Port closed, restarting in 5s')
            # break
            time.sleep(5)  # if you don wait it crashes the jetson
            node.get_logger().info('restarting')
            # main(dotheinit=False)
        except termios.error as e:
            node.get_logger().error('Termios error occurred')
            # node.portHandler.clearPort()
            # time.sleep(1)
            node.portHandler.closePort()
            node.get_logger().info('Port closed, restarting in 5s')
            # break
            time.sleep(5)  # if you don wait it crashes the jetson
            node.get_logger().info('restarting')
            # main(dotheinit=False)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
