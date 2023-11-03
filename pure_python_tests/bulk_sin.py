#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

import numpy as np

if os.name == 'nt':
    import msvcrt


    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)


    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *  # Uses Dynamixel SDK library

# ********* DYNAMIXEL Model definition *********
# ***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'  # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V

motor_table = {
    'X_SERIES': {
        "ADDR_TORQUE_ENABLE": 64,
        "ADDR_LED_RED": 65,
        "LEN_LED_RED": 1,
        "ADDR_GOAL_POSITION": 116,
        "LEN_GOAL_POSITION": 4,
        "ADDR_PRESENT_POSITION": 132,
        "LEN_PRESENT_POSITION": 4,
        "DXL_MINIMUM_POSITION_VALUE": 0,
        "DXL_MAXIMUM_POSITION_VALUE": 4095,
        "BAUDRATE": 57600,
    }
    ,
    'MX_SERIES': {
        "ADDR_TORQUE_ENABLE": 64,
        "ADDR_GOAL_POSITION": 116,
        "ADDR_PRESENT_POSITION": 132,
        "DXL_MINIMUM_POSITION_VALUE": 0,
        "DXL_MAXIMUM_POSITION_VALUE": 4095,
        "BAUDRATE": 57600
    }
    ,
    'PRO_SERIES': {
        "ADDR_TORQUE_ENABLE": 562,
        "ADDR_LED_RED": 563,
        "LEN_LED_RED": 1,
        "ADDR_GOAL_POSITION": 596,
        "LEN_GOAL_POSITION": 4,
        "ADDR_PRESENT_POSITION": 611,
        "LEN_PRESENT_POSITION": 4,
        "DXL_MINIMUM_POSITION_VALUE": -150000,
        "DXL_MAXIMUM_POSITION_VALUE": 150000,
        "BAUDRATE": 57600,
    }
    ,
    'P_SERIES': {
        "ADDR_TORQUE_ENABLE": 512,
        "ADDR_LED_RED": 513,
        "LEN_LED_RED": 1,
        "ADDR_GOAL_POSITION": 564,
        "LEN_GOAL_POSITION": 4,
        "ADDR_PRESENT_POSITION": 580,
        "LEN_PRESENT_POSITION": 4,
        "DXL_MINIMUM_POSITION_VALUE": -150000,
        "DXL_MAXIMUM_POSITION_VALUE": 150000,
        "BAUDRATE": 57600,
    }
    ,
    'PRO_A_SERIES': {
        "ADDR_TORQUE_ENABLE": 512,
        "ADDR_LED_RED": 513,
        "LEN_LED_RED": 1,
        "ADDR_GOAL_POSITION": 564,
        "LEN_GOAL_POSITION": 4,
        "ADDR_PRESENT_POSITION": 580,
        "LEN_PRESENT_POSITION": 4,
        "DXL_MINIMUM_POSITION_VALUE": -150000,
        "DXL_MAXIMUM_POSITION_VALUE": 150000,
        "BAUDRATE": 57600,
    }
    ,
    'XL320': {
        "ADDR_TORQUE_ENABLE": 24,
        "ADDR_GOAL_POSITION": 30,
        "ADDR_PRESENT_POSITION": 37,
        "DXL_MINIMUM_POSITION_VALUE": 0,
        "DXL_MAXIMUM_POSITION_VALUE": 1023,
        "BAUDRATE": 1000000
    }
    ,
}

BAUDRATE = motor_table["X_SERIES"]["BAUDRATE"]

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME = '/dev/ttyUSB0'

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION = 2.0

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupBulkRead instance for Present Position
groupBulkRead = GroupBulkRead(portHandler, packetHandler)
# Initialize GroupBulkWrite instance
groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)

# Open port
if portHandler.openPort():
    print("Port opened :)")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print(f"Baudrate [{BAUDRATE}] set :)")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()


def search_for_motor_broadcast():
    dxl_data_dic, dxl_comm_result = packetHandler.broadcastPing(portHandler)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    id_list = list(dxl_data_dic.keys())
    motor_data_array = np.empty((len(id_list), 3), int)
    motor_data_array[:, 0] = id_list
    for dxl_id in id_list:
        motor_data_array[:, [1, 2]] = dxl_data_dic.get(dxl_id)

    return motor_data_array


def search_for_motor(id_range):
    id_list = []

    for id in id_range:
        dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, id)
        if dxl_comm_result == COMM_SUCCESS:
            print(f"Motor [{id}] found :)")
            id_list.append([id, dxl_model_number])

    motor_data_array = np.empty((len(id_list), 3), int)
    if id_list:
        motor_data_array[:, [0, 1]] = id_list
    return motor_data_array


class Motor:
    def __init__(self, motor_data, packetHandler, portHandler, groupBulkRead, groupBulkWrite,
                 deviceName='/dev/ttyUSB0',
                 motor_series="X_SERIES"):
        self.id = motor_data[0]
        self.model = motor_data[1]
        self.firmware = motor_data[2]

        self.packetHandler = packetHandler
        self.portHandler = portHandler
        self.deviceName = deviceName
        self.groupBulkRead = groupBulkRead
        self.groupBulkWrite = groupBulkWrite

        self.addr_table = motor_table[motor_series]

        self.minout = self.addr_table["DXL_MINIMUM_POSITION_VALUE"]
        self.maxout = self.addr_table["DXL_MAXIMUM_POSITION_VALUE"]

        dxl_addparam_result = self.groupBulkRead.addParam(self.id, self.addr_table["ADDR_PRESENT_POSITION"],
                                                          self.addr_table["LEN_PRESENT_POSITION"])

        self.alive = True

    def check_motor_alive(self, trial=0):
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, self.id)
        if dxl_comm_result == COMM_SUCCESS:
            print(f"after {trial} attempts ping successful, Motor {self.id} Alive :)")
            return True
        elif trial > 3:
            print(f"Motor {self.id} is dead\n{packetHandler.getRxPacketError(dxl_error)}")
            self.alive = False
            return False
        else:
            print(f"secondary ping #{trial}")
            return self.check_motor_alive(trial + 1)

    def enable(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler,
                                                                       self.id,
                                                                       self.addr_table["ADDR_TORQUE_ENABLE"],
                                                                       1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            self.check_motor_alive()
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            self.check_motor_alive()
        else:
            print(f"Dynamixel {self.id} enabled")

    def disable(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler,
                                                                       self.id,
                                                                       self.addr_table["ADDR_TORQUE_ENABLE"],
                                                                       0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            self.check_motor_alive()
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            self.check_motor_alive()
        else:
            print(f"Dynamixel {self.id} disabled")

    def add_param_for_read_pos(self):
        # Add parameter storage for Dynamixel#1 present position
        dxl_addparam_result = self.groupBulkRead.addParam(self.id, self.addr_table["ADDR_PRESENT_POSITION"],
                                                          self.addr_table["LEN_PRESENT_POSITION"])
        if dxl_addparam_result != True:
            print(f"[ID:{self.id:03d}] add_param_for_read_pos failed\n{dxl_addparam_result}")
            self.check_motor_alive()

    def subscribe_position(self):
        dxl_addparam_result = self.groupBulkRead.addParam(self.id,
                                                          self.addr_table["ADDR_PRESENT_POSITION"],
                                                          self.addr_table["LEN_PRESENT_POSITION"])

        if dxl_addparam_result != True:
            print(f"[ID:{self.id:03d}] subscribe_position failed\n{dxl_addparam_result}")
            self.check_motor_alive()

    def position_available(self):
        dxl_getdata_result = self.groupBulkRead.isAvailable(self.id,
                                                            self.addr_table["ADDR_PRESENT_POSITION"],
                                                            self.addr_table["LEN_PRESENT_POSITION"])
        if dxl_getdata_result != True:
            print(f"[ID:{self.id:03d}] position_available failed\n{dxl_getdata_result}")
            self.check_motor_alive()
        return dxl_getdata_result

    def get_position(self):
        dxl1_present_position = self.groupBulkRead.getData(self.id,
                                                           self.addr_table["ADDR_PRESENT_POSITION"],
                                                           self.addr_table["LEN_PRESENT_POSITION"])
        return (dxl1_present_position - self.minout) / (self.maxout - self.minout) * (2 * np.pi)

    def write_position(self, angle: float):
        raw = int(self.minout + (self.maxout - self.minout) * (angle / (2 * np.pi)))
        raw = np.clip(raw, self.minout, self.maxout)
        # print(raw)
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(raw)),
                               DXL_HIBYTE(DXL_LOWORD(raw)),
                               DXL_LOBYTE(DXL_HIWORD(raw)),
                               DXL_HIBYTE(DXL_HIWORD(raw))]

        dxl_addparam_result = self.groupBulkWrite.addParam(self.id,
                                                           self.addr_table["ADDR_GOAL_POSITION"],
                                                           self.addr_table["LEN_GOAL_POSITION"],
                                                           param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] write_position failed" % self.id)
            self.check_motor_alive()


def publish():
    dxl_comm_result = groupBulkWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    groupBulkWrite.clearParam()
    return


def request_update():
    dxl_comm_result = groupBulkRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    return


# motor_data_array = search_for_motor()
#
# while motor_data_array.shape[0] < 1:
#     print("no motors")
#     motor_data_array = search_for_motor()
# print(f"motors found:\n[[ID, model, firmware], ...]\n{motor_data_array}")
#
# my_motor_list = [Motor(motor_data=motor_data_array[row, :],
#                        packetHandler=packetHandler,
#                        portHandler=portHandler,
#                        groupBulkRead=groupBulkRead,
#                        groupBulkWrite=groupBulkWrite,
#                        deviceName='/dev/ttyUSB0',
#                        motor_series="X_SERIES") for row in range(motor_data_array.shape[0])]
#
# [my_motor.enable() for my_motor in my_motor_list]
# [my_motor.subscribe_position() for my_motor in my_motor_list]

# while len(my_motor_list)>0:
while 1:
    motor_data_array = search_for_motor(range(1, 16))

    while motor_data_array.shape[0] < 1:
        print("no motors")
        motor_data_array = search_for_motor(range(1, 16))
    print(f"motors found:\n[[ID, model, firmware], ...]\n{motor_data_array}")

    my_motor_list = [Motor(motor_data=motor_data_array[row, :],
                           packetHandler=packetHandler,
                           portHandler=portHandler,
                           groupBulkRead=groupBulkRead,
                           groupBulkWrite=groupBulkWrite,
                           deviceName='/dev/ttyUSB0',
                           motor_series="X_SERIES") for row in range(motor_data_array.shape[0])]

    [my_motor.enable() for my_motor in my_motor_list]
    [my_motor.subscribe_position() for my_motor in my_motor_list]

    print(f"\nretart\n\n{len(my_motor_list)}\n")

    period = 2  # sec

    start_time = time.time()
    now_time = start_time
    rel_time = 0
    angle = 0

    while rel_time < period:
        for index in range(len(my_motor_list) - 1, -1, -1):
            motor = my_motor_list[index]
            if not motor.alive:
                del my_motor_list[index]

        now_time = time.time()
        rel_time = now_time - start_time
        x = rel_time / period
        prev_angle = angle
        angle = (np.cos(2 * np.pi * x) + 1) / 2 * 2 * np.pi
        [my_motor.write_position(angle) for my_motor in my_motor_list]

        if my_motor_list:
            publish()
            request_update()
            while not all([(my_motor.position_available() or not my_motor.alive) for my_motor in my_motor_list]):
                # print("\nthat was clsoe\n")
                request_update()
            pos_list = [my_motor.get_position() for my_motor in my_motor_list]
            print(
                f"""{[f"{np.rad2deg(val):.1f}" for val in pos_list]}, lag = {[f"{np.rad2deg(prev_angle - val):.2f}" for val in pos_list]}""")
        # print(rel_time)
# Disable Dynamixel Torque
[my_motor.disable() for my_motor in my_motor_list]

# Close port
portHandler.closePort()
