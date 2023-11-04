import time
import numpy as np
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import python_package_include.multi_controller as my_controller

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

    def __init__(self):
        super().__init__(f'MultiDynamixel_node')

        bypass_alive_check = False

        connected = False

        # Use the actual port assigned to the U2D2.
        # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
        DEVICENAME = '/dev/ttyUSB0'

        # DYNAMIXEL Protocol Version (1.0 / 2.0)
        # https://emanual.robotis.com/docs/en/dxl/protocol2/
        PROTOCOL_VERSION = 2.0

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        portHandler = my_controller.PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        packetHandler = my_controller.PacketHandler(PROTOCOL_VERSION)

        # Initialize GroupBulkRead instance for Present Position
        groupBulkRead = my_controller.GroupBulkRead(portHandler, packetHandler)
        # Initialize GroupBulkWrite instance
        groupBulkWrite = my_controller.GroupBulkWrite(portHandler, packetHandler)

        while not connected:
            # Open port
            if portHandler.openPort():
                connected = True
                self.get_logger().info("Port opened :)")
            else:
                self.get_logger().warning("Failed to open the port")
                time.sleep(1)

        connected = False
        while not connected:
            BAUDRATE = my_controller.motor_table["X_SERIES"]["BAUDRATE"]
            # Set port baudrate
            if portHandler.setBaudRate(BAUDRATE):
                connected = True
                self.get_logger().info(f"Baudrate [{BAUDRATE}] set :)")
            else:
                self.get_logger().warning("Failed to change the baudrate")
                time.sleep(1)

        self.controller = my_controller.MotorHandler(packetHandler=packetHandler,
                                                     portHandler=portHandler,
                                                     groupBulkRead=groupBulkRead,
                                                     groupBulkWrite=groupBulkWrite,
                                                     deviceName='/dev/ttyUSB0',
                                                     motor_series="X_SERIES")
        self.controller.refresh_motors()
        while not self.controller.motor_list:
            self.get_logger().warning(
                f'''no motors connected''')
            if bypass_alive_check:
                self.get_logger().warning(
                    f'''Motor check bypassed :)''')
                break
            self.controller.refresh_motors()
        if not bypass_alive_check:
            self.get_logger().warning(
                f'''{len(self.controller.motor_list)} Motor connected :)''')

        self.controller.broadcast_max_speed(999)

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
        self.sub_rel_target = self.create_subscription(Float64, f'Motor_01',
                                                       self.set_motor_1,
                                                       10
                                                       )
        #    /\    #
        #   /  \   #
        ############   ^ Subscribers ^

        ############   V Service V
        #   \  /   #
        #    \/    #
        self.iAmAlive = self.create_service(Empty, f'motor_alive', lambda: None)
        #    /\    #
        #   /  \   #
        ############   ^ Service ^
        grp1 = MutuallyExclusiveCallbackGroup()
        grp2 = MutuallyExclusiveCallbackGroup()

        self.search_timer = self.create_timer(0.5, self.search_for_motors, callback_group=grp1)
        self.delete_the_dead_timer = self.create_timer(2, self.delete_the_dead, callback_group=grp1)
        self.move_dtime = 1
        self.move_timer = self.create_timer(self.move_dtime, self.move, callback_group=grp2)

        self.last_id_checked = 0
        self.id_range = list(range(1, 5))
        self.target = 2

    @error_catcher
    def search_for_motors(self):
        self.last_id_checked = (1 + self.last_id_checked) % len(self.id_range)
        id_now = self.id_range[self.last_id_checked]
        if id_now in [motor.id for motor in self.controller.motor_list]:
            return
        result = self.controller.refresh_motors([id_now])
        if result:
            self.get_logger().warning(f"Motor {result} found :)")

    @error_catcher
    def delete_the_dead(self):
        result = self.controller.delete_dead_motors()
        if result:
            self.get_logger().warning(f"Motor {result} lost")

    @error_catcher
    def move(self):
        period = 10
        x = (time.time() % period) / period * 2 * np.pi
        self.controller.broadcast_target_on_time(my_controller.wave(x), self.move_dtime+0.1)

    @error_catcher
    def set_motor_1(self, msg):
        angle = msg.data
        return


def main(args=None):
    rclpy.init()
    node = MultiDynamixel()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt as e:
        node.get_logger().debug('KeyboardInterrupt caught, node shutting down cleanly\nbye bye <3')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
