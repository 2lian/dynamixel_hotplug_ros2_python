"""
Motors settings to be use by the launch file
Made for the moonbot
"""

# Baud-rate and motorID are motor specific, if you want to change it, use the dynamixel wizard

USB_u2d2_port_to_use = [f"/dev/ttyUSB{n}" for n in [0, 1, 2, 3, 4]]
# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
# use `ls /dev/ttyUSB*` to see which ports are active on linux
# if you have only one u2d2 plugged in, you should see it assigned to `/dev/ttyUSB0`

MotorSeries = "X_SERIES"
# The motor series changes the adresses used on the serial port
# Only the Xseries addresses are usable, update `motor_address_table` in multi_controller.py for another motor
# 'X_SERIES'  # X330 (5.0 V recommended), X430, X540, 2X430
# 'MX_SERIES'    # MX series with 2.0 firmware update.
# 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# 'PRO_A_SERIES' # PRO series with (A) firmware update.
# 'P_SERIES'     # PH54, PH42, PM54
# 'XL320'        # [WARNING] Operating Voltage : 7.4V

Baudrate = 4_000_000
# Default: 57_600, Pro series default: 1_000_000, Max: 4_000_000
# if the baudrate is wrong motors won't be detected

IdRangeMin = 1
IdRangeMax = 3
# defines the id range of the motors to detect
# two motors CANNOT share the same id, it WILL bug

FullScanPeriod = 2  # seconds
# will scan all ids in the idrange in 2 s

AngleReadFreq = 10  # Hz
# Freq at which the angles of all connected dynamixel is read and published on ros2

AngleWriteFreq = 100  # Hz
# Freq at which the bulkwrite will send all new targets in the buffer to the motors
