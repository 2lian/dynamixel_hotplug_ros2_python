# Dynamixel Motor Ros2 package using Pyhton, Hotplug capable

This repo provides python libraries and ros2 nodes to control several dynamixels on several u2d2 interfaces 
with hotplug capabilities (motor can be (dis)connected at runtime).

- `multi_controller.py`: Custom python library to control multiple dynamixels connected to the same serial port 
using velocity profile and bulk raed/write serial commands.
- `u2d2_dyna_controller.py`: Ros2 node (responsible for one single serial port) 
setting up the controller and providing ros2 subscribers/publisher.
- `multi_port_launch.py`: Launches several nodes, one node per serial port, with the corresponding parameters.

Youtube video:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=wYH8rg-nyjc" target="_blank">
 <img src="http://img.youtube.com/vi/wYH8rg-nyjc/mqdefault.jpg" alt="Watch the video" width="240" height="180" border="10" />
</a>

# Installation

## Software

Ros2 needs to be installed and working, this was made using Ros2 Foxy but should work with newer versions [(installation of Foxy)](https://docs.ros.org/en/foxy/Installation.html).

Python dependencies:
```bash
python3 -m pip install numpy
python3 -m pip install serial
```

Clone this repo and open it with:
```bash
git clone https://github.com/hubble14567/dynamixel_hotplug_ros2_python
cd dynamixel_hotplug_ros2_python
```

Source ros2, build, and source the workspace with:
```bash
source opt/foxy/setup.bash
colcon build --symlink-install
. install/setup.bash
```

Launch the default configuration with:
```bash
ros2 launch src/motor_controller/launch/multi_port_launch.py
```

This may detect motors right away

## Hardware / motors

Ensure that your setup is properly working by trying out the
[Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
and the [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/).
This repo requires a minimum of knowledge to set up and should not be used as your first step.

Using the dynamixel wizard, assign a unique motor ID to each motor connected to an usb controller. 
Motor on different controllers can share the same ID. Also set the Baudrate of your motors

# Setup

All the setup can be done by changing [src/launch/launch_settings.py](https://github.com/hubble14567/dynamixel_with_ros2/blob/60a4ab21f1bc3ffd34d84ef4dbea916901f28f65/src/motor_controller/launch/launch_settings.py)
Those parameters will be passed along to ros2 and the nodes handling the dynamixels (no need to rebuild when changing this file)

The following parameters needs to be properly modified in the file for this repo to work:
- `USB_u2d2_port_to_use`: List of strings representing path to the USB ports to use. 
One node per USB port will be spun up.
Use `ls /dev/ttyUSB*` to see which ports are active on linux
- `MotorSeries`: Series of dynamixel you are using. 
Only the Xseries addresses are usable, update `motor_address_table` in multi_controller.py 
if you want to add support to a new series
- `Baudrate`: Needs to correspond to the baudrate of every motor on the USB controller. 
Use the Dynamixel Wizard to change it on the motor.
- `IdRangeMin; IdRangeMax`: The node will detect motors with IDs between those two values (included).  
Two motors CANNOT share the same id when using on the same controller.
Use the Dynamixel Wizard to change the ID on the motor.

Other settings in [src/launch/launch_settings.py](https://github.com/hubble14567/dynamixel_with_ros2/blob/60a4ab21f1bc3ffd34d84ef4dbea916901f28f65/src/motor_controller/launch/launch_settings.py)
should be changed according to your need.

# Launch and use

Open this repo's workspace, source ros2, build, source the workspace and launch with:
```bash
cd dynamixel_hotplug_ros2_python
source opt/foxy/setup.bash
colcon build --symlink-install
. install/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export RCUTILS_COLORIZED_OUTPUT=1
ros2 launch src/motor_controller/launch/multi_port_launch.py
```

Without any changes, to the default settings [src/launch/launch_settings.py](https://github.com/hubble14567/dynamixel_with_ros2/blob/60a4ab21f1bc3ffd34d84ef4dbea916901f28f65/src/motor_controller/launch/launch_settings.py); 
this will: 
- Launch 5 nodes for the ports [/dev/ttyUSB0, /dev/ttyUSB1, /dev/ttyUSB2, /dev/ttyUSB3, /dev/ttyUSB4]. 
  - Each node looks for X_SERIES motors with the IDs [1,2,3] and baud-rate 4Mbps.
  - All motor IDs are scanned every 2s.
  - Current motor angle are read then published on `angle_port_X_mot_Y` at 10Hz
  - A subscribers listens to the topic `set_port_X_mot_Y` with messages containing 'angle' `float64` and 'seconds' `float64`.
  At 100Hz it sends the command to the motor to reach the 'angle' in the time 'seconds' by moving at a constant speed.
- The node angle_remapper 
  - Converts topics names according to the table inside topic_remapping.py
  - Subscribes to topics containing only 'angle' `set_joint_{0}_{0}_real` then repeats onto `set_port_X_mot_Y` 
using 'angle' and always the same value for 'seconds'.

Messages should indicate which USB port is detecting which motor. 
Unplugging and plugging motor should also display a message.

```bash

```

# About
## Dynamixel Motors settings and connection

Several dynamixel can be plugged onto the same serial controller. 
The Dynamixel Wizard should be used beforehand to set a unique ID to each motor, and set the baud-rate of the motor.

For the ros2 node, the motor id to look for, the baud-rate and usb port is set in the launcher `multi_port_launch.py`
and loaded from `launch_settings.py`.

The node will continuously scan for new motors connected or disconnected on the port and react accordingly.
You can plug motors while the node is running, start the node with no motors, or start the node with all motors, 
if the motor has an id that the node is looking for and the corresponding baud-rate
it will be found and a subscriber/publisher will be created.

## Velocity profile

Using velocity profile means the motors will reach the given target while moving at a specified speed, 
this ensures smooth movement by giving a target and a time to reach the target. 

Using this, angle update rates as low as 2Hz can produce smooth motions. Hence, high baud-rate is not required.

Ideally, if you send targets at `x`Hz and want to ensure a smooth transition between each target
(= motor not stopping while waiting for the next target),
then you want to set the time to reach the target to `1/x + 0.1`s. 
The motor will continuously be `0.1`s behind, but this is negligible and won't build up over time.

## Bulk read/write

Bulk read/write sends messages to several motors at once, thus lowering the load on the serial bus.
A bulk message contains messages for every motor (no more than one per motor id if sending), 
those small motor messages are prepared by the `Motor` class in `multi_controller.py`. 
The `MotorHandler` then coordinates several `Motor` objects and publishes the resulting big bulk message on the bus.

## Serial Port and Single/Multi Threading

A serial port is a single threaded object; thereby, two messages cannot be sent at the same time on the same port, 
and two programs cannot use the same port simultaneously.
Hence, one single threaded Ros2 node is responsible for one serial port. This ensures single threaded access to the port.

If several ports are used, each one has it own thread; thereby, several messages can be sent on DIFFERENT ports simultaneously.
The launcher creates several nodes, each responsible for one port. 
Each node runs in its own thread and can be called at any time in any order.



[//]: # (connecting usbv through WSL https://devblogs.microsoft.com/commandline/connecting-usb-devices-to-wsl/)

[//]: # ()
[//]: # (```bash)

[//]: # (usbipd wsl attach --busid 3-2)

[//]: # (```)
