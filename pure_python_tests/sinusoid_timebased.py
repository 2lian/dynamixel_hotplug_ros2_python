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


# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132
    DXL_MINIMUM_POSITION_VALUE = 0  # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE = 4095  # Refer to the Maximum Position Limit of product eManual
    BAUDRATE = 57600
elif MY_DXL == 'PRO_SERIES':
    ADDR_TORQUE_ENABLE = 562  # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION = 596
    ADDR_PRESENT_POSITION = 611
    DXL_MINIMUM_POSITION_VALUE = -150000  # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE = 150000  # Refer to the Maximum Position Limit of product eManual
    BAUDRATE = 57600
elif MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
    ADDR_TORQUE_ENABLE = 512  # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION = 564
    ADDR_PRESENT_POSITION = 580
    DXL_MINIMUM_POSITION_VALUE = -150000  # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE = 150000  # Refer to the Maximum Position Limit of product eManual
    BAUDRATE = 57600
elif MY_DXL == 'XL320':
    ADDR_TORQUE_ENABLE = 24
    ADDR_GOAL_POSITION = 30
    ADDR_PRESENT_POSITION = 37
    DXL_MINIMUM_POSITION_VALUE = 0  # Refer to the CW Angle Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE = 1023  # Refer to the CCW Angle Limit of product eManual
    BAUDRATE = 1000000  # Default Baudrate of XL-320 is 1Mbps

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID = 1

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME = '/dev/ttyUSB0'

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]  # Goal position

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")


def send_target(target: int, wait_for_convergence=False):
    # Write goal position
    if (MY_DXL == 'XL320'):  # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION,
                                                                  target)
    else:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION,
                                                                  target)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))
    #
    # while 1:
    #     # Read present position
    #     if (
    #             MY_DXL == 'XL320'):  # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
    #         dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID,
    #                                                                                        ADDR_PRESENT_POSITION)
    #     else:
    #         dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID,
    #                                                                                        ADDR_PRESENT_POSITION)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))
    #
    #     print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, target, dxl_present_position))
    #
    #     if not wait_for_convergence or not abs(target - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
    #         break
    return


minval = DXL_MINIMUM_POSITION_VALUE
maxval = DXL_MAXIMUM_POSITION_VALUE

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    # if getch() == chr(0x1b):
    #     break

    period = 10  # sec

    start_time = time.time()
    now_time = start_time
    rel_time = 0

    while rel_time < period:
        now_time = time.time()
        rel_time = now_time - start_time
        x = rel_time/period
        angle = int(minval + 0.5 * (maxval - minval) * (1 + np.cos(2 * np.pi * x)))
        send_target(angle)

    break

# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
