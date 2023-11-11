USB_u2d2_port_to_use = [0, 1, 2, 3, 4]
# use `ls /dev/*USB*` to see which port is active on the PC
# if you have only one u2d2 plugged in, you should see it assigned to `/dev/ttyUSB0` , so this is port 0

Baudrate = 4_000_000
# Default: 57_600, Pro series default: 1_000_000, Max: 4_000_000
# if the baudrate is wrong motors won't be detected

IdRangeMin = 1
IdRangeMax = 3
# defines the id range of the motors to detect
# two motors CANNOT sahre the same id, it WILL bug