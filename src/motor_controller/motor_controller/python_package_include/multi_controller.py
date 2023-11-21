#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Provides functions to control dynamixel motors using bulkread and bulkwrite
(sending and receiving data from several motors on the same serial port)

The motor class
- prepare the messages to be sent on the serial port.
- directly send/receive to one motor
- stores the motor state
The controller object
- contains several motors
- pings for motors
- handles when motors are disconnected
- ask those motors to prepare their messages
- sends and receives those messages

@author: Elian NEPPEL
@laboratory: Moonshot, Space Robotic Lab, Tohoku University

"""

import os
import time

import numpy as np

from dynamixel_sdk import *  # Uses Dynamixel SDK library

# ********* DYNAMIXEL Model definition *********
# ***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'  # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V
# see https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#control-table-of-ram-area or equivalent for you motor

motor_address_table = {
    'X_SERIES': {
        "ADDR_TORQUE_ENABLE": 64,

        "ADDR_LED_RED": 65,
        "LEN_LED_RED": 1,

        "ADDR_GOAL_POSITION": 116,
        "LEN_GOAL_POSITION": 4,

        "ADDR_PRESENT_POSITION": 132,
        "LEN_PRESENT_POSITION": 4,

        "ADDR_MAX_VELOCITY": 112,
        "LEN_MAX_VELOCITY": 4,

        "DXL_MINIMUM_POSITION_VALUE": 0,
        "DXL_MAXIMUM_POSITION_VALUE": 4095,

        # "BAUDRATE": 57600,
        # "BAUDRATE": 1000000,
        "BAUDRATE": 4000000,
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


class Motor:
    """
    represents and handles a single motor
    """

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

        self.addr_table = motor_address_table[motor_series]

        self.minraw = self.addr_table["DXL_MINIMUM_POSITION_VALUE"]
        self.maxraw = self.addr_table["DXL_MAXIMUM_POSITION_VALUE"]

        self.alive = True

    def rad2raw(self, radiant: float) -> int:
        """
        Converts radiant angles to the corresponding raw value for the motor
        :param radiant: angle in radiant
        :return: corresponding raw value for the motor
        """
        radiant += np.pi  # 0 is everything pointing sraight, positive and negative angles should be used
        raw = self.minraw + (self.maxraw - self.minraw) * (radiant / (2 * np.pi))
        return int(np.clip(raw, self.minraw, self.maxraw))

    def raw2rad(self, raw:int) -> float:
        return (raw - self.minraw) / (self.maxraw - self.minraw) * (2 * np.pi) - np.pi

    def check_motor_alive(self, trial: int = 0) -> bool:
        """
        Pings the motor 3 times (recursively) to check if it's alive
        if motor is dead, self.alive is switched to False
        :param trial: number of time the motors has been pinged
        :return: True if motor is alive, else False
        """
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, self.id)
        if dxl_comm_result == COMM_SUCCESS:
            print(f"after {trial} attempts ping successful, Motor {self.id:03d} Alive :)")
            self.alive = True
            return True
        elif trial > 3:
            print(f"Motor {self.id} is dead\n{self.packetHandler.getRxPacketError(dxl_error)}")
            self.alive = False
            return False
        else:
            return self.check_motor_alive(trial + 1)

    def enable(self) -> None:
        """
        Enables torque on the motor (the motor will have power)
        :return:
        """
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
            print(f"Dynamixel {self.id:03d} enabled")

    def disable(self) -> None:
        """
        Disable torque on the motor (the motor will power down)
        :return:
        """
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
            print(f"Dynamixel {self.id:03d} disabled")

    def subscribe_position(self) -> None:
        """
        The position of the motor will be requested on the bulk read
        :return:
        """
        dxl_addparam_result = self.groupBulkRead.addParam(self.id,
                                                          self.addr_table["ADDR_PRESENT_POSITION"],
                                                          self.addr_table["LEN_PRESENT_POSITION"])

        if dxl_addparam_result != True:
            print(f"[ID:{self.id:03d}] subscribe_position failed\n{dxl_addparam_result}")
            self.check_motor_alive()

    def position_available(self) -> bool:
        """
        The position has been returned and the data is available
        :return:
        """
        dxl_getdata_result = self.groupBulkRead.isAvailable(self.id,
                                                            self.addr_table["ADDR_PRESENT_POSITION"],
                                                            self.addr_table["LEN_PRESENT_POSITION"])
        if dxl_getdata_result != True:
            print(f"[ID:{self.id:03d}] position_available failed\n{dxl_getdata_result}")
            self.check_motor_alive()
        return dxl_getdata_result

    def get_position(self) -> float:
        """
        Reads the position data received from the bulk read
        :return:
        """
        dxl1_present_position = self.groupBulkRead.getData(self.id,
                                                           self.addr_table["ADDR_PRESENT_POSITION"],
                                                           self.addr_table["LEN_PRESENT_POSITION"])
        return self.raw2rad(dxl1_present_position)

    def write_position(self, angle: float) -> None:
        """
        Write a motor target position onto the bulk write
        (warning you cannot write two things for the same motor on the bulkwrite)
        :param angle:
        :return:
        """
        raw = self.rad2raw(angle)
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

    def write_max_speed(self, speed_rads) -> None:
        """
        Write a maximum allowed speed for the motor on the bulk write
        (warning you cannot write two things for the same motor on the bulkwrite)
        :param speed_rads:
        :return:
        """
        rpm = speed_rads / (2 * np.pi) * 60
        scaled = rpm / 0.229 * 1.030  # XSERTIES unit scaling
        raw = np.clip(int(np.floor(scaled)), self.minraw, self.maxraw)
        # print(raw)

        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(raw)),
                               DXL_HIBYTE(DXL_LOWORD(raw)),
                               DXL_LOBYTE(DXL_HIWORD(raw)),
                               DXL_HIBYTE(DXL_HIWORD(raw))]

        dxl_addparam_result = self.groupBulkWrite.addParam(self.id,
                                                           self.addr_table["ADDR_MAX_VELOCITY"],
                                                           self.addr_table["LEN_MAX_VELOCITY"],
                                                           param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] write_max_speed failed" % self.id)
            self.check_motor_alive()


class MotorHandler:
    """
    represent and handle several motors connected to the same port
    """

    def __init__(self, packetHandler, portHandler, groupBulkRead, groupBulkWrite,
                 deviceName='/dev/ttyUSB0',
                 motor_series="X_SERIES"):
        self.motor_list = []
        self.idrange = range(1, 17)

        self.packetHandler = packetHandler
        self.portHandler = portHandler
        self.deviceName = deviceName
        self.groupBulkRead = groupBulkRead
        self.groupBulkWrite = groupBulkWrite
        self.motor_series = motor_series

    def delete_dead_motors(self) -> list:
        """
        delete motors that died from the list of motors being managed by the handler
        :return:
        """
        motor_died = []
        for index in range(len(self.motor_list) - 1, -1, -1):
            if not self.motor_list[index].alive:
                this_id = int(self.motor_list[index].id)
                motor_died.append(this_id)
                print(f"Motor ID{this_id:03d} Deleted")
                del self.motor_list[index]

        if motor_died:
            self.reset_bulkread()
            for motor in self.motor_list:
                motor.subscribe_position()
        return motor_died

    def refresh_motors(self, idrange: list = range(1, 17)) -> list:
        """
        check if the motors with the provided ids are available
        if yes those motor are added to the list of motors managed by the handler
        :param idrange: range of motor id to check
        :return: list of new motors
        """
        # self.delete_dead_motors()
        id_list = list(idrange)
        already_here_list = [motor.id for motor in self.motor_list]
        for id in id_list:
            if id in already_here_list:
                id_list.remove(id)

        motor_data_array = search_for_motor(id_list, packetHandler=self.packetHandler, portHandler=self.portHandler)

        new_motors = [Motor(motor_data=motor_data_array[row, :],
                            packetHandler=self.packetHandler,
                            portHandler=self.portHandler,
                            groupBulkRead=self.groupBulkRead,
                            groupBulkWrite=self.groupBulkWrite,
                            deviceName=self.deviceName,
                            motor_series=self.motor_series) for row in range(motor_data_array.shape[0])]
        if new_motors:
            self.motor_list += new_motors

            [my_motor.enable() for my_motor in new_motors]
            [my_motor.subscribe_position() for my_motor in new_motors]
            return list(motor_data_array[:, 0])
        else:
            return []

    def get_motor_id_in_order(self) -> list:
        return [motor.id for motor in self.motor_list]

    def disable(self):
        """
        Disables all motor
        :return:
        """
        [my_motor.disable() for my_motor in self.motor_list]
        return

    def enable(self):
        """
        Enables all motor
        :return:
        """
        [my_motor.enable() for my_motor in self.motor_list]
        return

    def broadcast_target(self, angle: float):
        """
        Sends the same target to all connected motors
        :param angle: target angle
        :return: True if success
        """
        return self.distibute_targets(np.full(len(self.motor_list), angle, dtype=float))

    def broadcast_max_speed(self, ang_speed: float):
        """
        Sends the same max speed to all connected motors
        :param ang_speed: max speed allowed
        :return: True if success
        """
        return self.distibute_max_speeds(np.full(len(self.motor_list), ang_speed, dtype=float))

    def distibute_targets(self, angle_arr):
        """
        Sends a list of target to corresponding motors
        :param angle_arr: target angle array
        :return: True if success
        """
        for index, my_motor in enumerate(self.motor_list):
            my_motor.write_position(angle_arr[index])
        return self.publish()

    def distibute_max_speeds(self, ang_speed_arr: np.ndarray):
        """
        Sends a list of max speed to corresponding motors
        :param ang_speed_arr: max speed array
        :return: True if success
        """
        for index, my_motor in enumerate(self.motor_list):
            my_motor.write_max_speed(ang_speed_arr[index])
        return self.publish()

    def all_positions_available(self) -> bool:
        """
        Ture, if each alive motor has resonded to their data resquest
        :return:
        """
        return all([my_motor.position_available() or not my_motor.alive for my_motor in self.motor_list])

    def get_angles(self) -> np.ndarray:
        """
        blocks and return the current angles of every motor
        :return:
        """
        comm_success = self.request_update()
        while not self.all_positions_available() or not comm_success:
            comm_success = self.request_update()
            if not comm_success:  # try twice
                break
        if comm_success:
            return np.array([my_motor.get_position() for my_motor in self.motor_list], dtype=float)
        else:
            return np.full(len(self.motor_list), np.nan, dtype=float)


    def broadcast_target_on_time(self, angle: float, delta_time: float) -> bool:
        """
        all motors will reach the target angle in delta_time
        :param angle: target angle
        :param delta_time: time in sec to reach the target
        :return: True if success
        """
        return self.to_target_same_time(
            np.full(len(self.motor_list), angle, dtype=float),
            delta_time)

    def to_target_same_time(self, angle_arr: np.ndarray, delta_time: float) -> bool:
        """
        all motors will reach their target angle from the array in delta_time
        :param angle_arr: target angle array
        :param delta_time: time in sec to reach the target
        :return: True if success
        """
        delta_time_arr = np.full(shape=angle_arr.shape, fill_value=delta_time, dtype=float)
        return self.to_target_using_angle_time(angle_arr, delta_time_arr)

    def to_target_using_angle_time(self, angle_arr: np.ndarray, delta_time_arr: np.ndarray) -> bool:
        """
        all motors will reach their target angle in  the corresponding time in the delta_time_arr array
        :param angle_arr: target angle array
        :param delta_time_arr: array of time in sec to reach the target
        :return: True if success
        """
        delta_time_arr_safe = np.clip(delta_time_arr, a_min=0.0001, a_max=None) # avoid division by zero and negative values
        angle_now = self.get_angles()
        speed = abs((angle_arr - angle_now) / delta_time_arr_safe)
        comm_result = self.distibute_max_speeds(speed)
        comm_result = comm_result and self.distibute_targets(angle_arr)
        return comm_result

    def publish(self) -> bool:
        """
        Publishes all the data writen on the bulkwrite
        :return:
        """
        if not self.motor_list:
            return True
        dxl_comm_result = self.groupBulkWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("Publish failed")
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            for index, my_motor in enumerate(self.motor_list):
                my_motor.check_motor_alive()
            self.groupBulkWrite.clearParam()
            return False
        else:
            self.groupBulkWrite.clearParam()
            return True

    def request_update(self) -> bool:
        """
        request the data writen on the bulkread (does not wait for the response data)
        :return:
        """
        if not self.motor_list:
            return True
        dxl_comm_result = self.groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("Request_update failed")
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            for index, my_motor in enumerate(self.motor_list):
                my_motor.check_motor_alive()
            return False
        else:
            return True

    def reset_bulkread(self) -> None:
        """
        clears everyting writen on the bulkread
        :return:
        """
        self.groupBulkRead.clearParam()
        return


def search_for_motor_broadcast(packetHandler) -> np.ndarray:
    """
    broadcasts a ping on the port
    :param packetHandler:
    :return: data of motors that responded, column 0 is the motor id
    """
    dxl_data_dic, dxl_comm_result = packetHandler.broadcastPing(portHandler)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    id_list = list(dxl_data_dic.keys())
    motor_data_array = np.empty((len(id_list), 3), int)
    motor_data_array[:, 0] = id_list
    for dxl_id in id_list:
        motor_data_array[:, [1, 2]] = dxl_data_dic.get(dxl_id)

    return motor_data_array


def search_for_motor(id_range: list, packetHandler, portHandler) -> np.ndarray:
    """
    pings each motor id in the id_range one by one
    :param id_range: list of id to ping
    :param packetHandler:
    :param portHandler:
    :return: data of motors that responded, column 0 is the motor id
    """
    id_list = []

    for id in id_range:
        dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, id)
        if dxl_comm_result == COMM_SUCCESS:
            print(f"Motor {id:03d} found :)")
            id_list.append([id, dxl_model_number])

    motor_data_array = np.empty((len(id_list), 3), int)
    if id_list:
        motor_data_array[:, [0, 1]] = id_list
    return motor_data_array


def wave(x: float) -> float:
    return (-np.cos(x) + 1) / 2 * 2 * np.pi


if __name__ == "__main__":
    try:
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
            input()
            quit()

        BAUDRATE = motor_address_table["X_SERIES"]["BAUDRATE"]
        # Set port baudrate
        if portHandler.setBaudRate(BAUDRATE):
            print(f"Baudrate [{BAUDRATE}] set :)")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            input()
            quit()

        if 1:  # continously do a sinusoid

            controller = MotorHandler(packetHandler=packetHandler,
                                      portHandler=portHandler,
                                      groupBulkRead=groupBulkRead,
                                      groupBulkWrite=groupBulkWrite,
                                      deviceName='/dev/ttyUSB0',
                                      motor_series="X_SERIES")

            controller.refresh_motors()
            print(f"number of motors: {len(controller.motor_list)}\n")

            Hz = 10
            totaltime = 10

            dtime = 1 / Hz
            samples = int(totaltime * Hz)

            controller.broadcast_max_speed(999)
            controller.broadcast_target(wave(0))
            time.sleep(1.5)
            while 1:
                controller.refresh_motors()
                for x in wave(np.linspace(0, 2 * np.pi, samples)):
                    before = time.time()
                    controller.delete_dead_motors()
                    controller.broadcast_target_on_time(x, dtime + 0.1)
                    after = time.time()
                    if dtime - (after - before) > 0:
                        time.sleep(dtime - (after - before))

        if 0:  # compare set max speed and real speed
            controller = MotorHandler(packetHandler=packetHandler,
                                      portHandler=portHandler,
                                      groupBulkRead=groupBulkRead,
                                      groupBulkWrite=groupBulkWrite,
                                      deviceName='/dev/ttyUSB0',
                                      motor_series="X_SERIES")

            controller.refresh_motors()
            print(f"number of motors: {len(controller.motor_list)}\n")

            period = 1  # sec

            max_angle = np.pi * 1

            start_time = time.time()
            now_time = start_time
            rel_time = 0
            prev_angle, angle = 0, 0

            for k in range(10):
                controller.broadcast_target_on_time(0, 0.1)
                time.sleep(1)

                target = np.random.uniform(0, max_angle - 0.3)

                speed = target / period

                controller.broadcast_max_speed(speed)
                controller.broadcast_target(max_angle)

                time.sleep(period)
                angle_arr = controller.get_angles()

                real_speed = angle_arr / period

                print(f"target speed = {speed}\nreal speed = {real_speed}\ncorrection factor = {speed / real_speed}")
    except KeyboardInterrupt:
        pass
    # Disable Dynamixel Torque
    controller.disable()

    # Close port
    portHandler.closePort()
