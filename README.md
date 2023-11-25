# Simple Dynamixel Motor Ros2 package using Pyhton, Hotplug capable
## Send an angle target and a deltatime to your Dynamixel!

This repo provides python libraries and ros2 nodes to control several dynamixels on several u2d2 interfaces 
with hotplug capabilities (motor can be (dis)connected at runtime). Commands supported are anglular commands with a 
deltatime specifying the time the motor will take to reach the targeted angle.

- `multi_controller.py`: Custom python library to control multiple dynamixels connected to the same serial port 
using velocity profile and bulk read/write serial commands. Provides new motor detection functions, 
bulk read/write for angle and speed, automatically handles unresponsive motors.
- `u2d2_dyna_controller.py`: Ros2 node (responsible for one single serial port) 
setting up the above-mentioned controller and ros2 timer for periodic motor scanning; 
providing ros2 subscribers/publisher as a simple interface.
  - **[Publisher]** Current angle of the motor
    - topic: `angle_port_X_mot_Y`
    - message: `std_msgs/msg/Float64`
  - **[Subscriber]** Target angle and time to reach the target (using constant speed velocity profile of the dynamixel  
to generate smooth motion)
    - topic: `set_port_X_mot_Y`
    - message: `dyna_controller_messages/msg/AngleTime` (target in rad and time in seconds)
- `angle_remapper.py`: Maps topics of `u2d2_dyna_controller.py` onto other topics (such as leg 3 joint 0, instead of port 2 motor 5) 
using only `std_msgs/msg/Float64` for the target angle and a fix time to reach the target.
- `multi_port_launch.py`: Launches several nodes, one node per serial port and the remapper, 
with the corresponding parameters in `launch_settings.py`.

Youtube demo:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=wYH8rg-nyjc" target="_blank">
 <img src="http://img.youtube.com/vi/wYH8rg-nyjc/mqdefault.jpg" alt="Watch the video" width="576" height="324" border="10" />
</a>

# Why ?

This is mainly aimed at student in my/your lab that do not have the time, nor need, to learn the intricacies of Ros2, serial, and C++:
- Python is easier to understand than the [currently available Ros2 in C++](https://github.com/dynamixel-community/dynamixel_hardware)
Therefor modifications, additions and debugging are easy; and close to the serial interface.
- Only standard subscribers/publishers are needed, no need to learn Ros2's Action to only send an angle.
- Hotplug capabilities

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
Two motors CANNOT share the same id on the same controller.
Use the Dynamixel Wizard to change the ID of the motor.

Other settings in [src/launch/launch_settings.py](https://github.com/hubble14567/dynamixel_with_ros2/blob/60a4ab21f1bc3ffd34d84ef4dbea916901f28f65/src/motor_controller/launch/launch_settings.py)
should be changed according to your need.

# Launch and use

Open this repo's workspace, 
```bash
cd ~/dynamixel_hotplug_ros2_python
```
source ros2, build, source the workspace
```bash
source opt/foxy/setup.bash
colcon build --symlink-install
. install/setup.bash
```
and launch with:
```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export RCUTILS_COLORIZED_OUTPUT=1
ros2 launch src/motor_controller/launch/multi_port_launch.py
```

Without any changes, to the default settings [src/launch/launch_settings.py](https://github.com/hubble14567/dynamixel_with_ros2/blob/60a4ab21f1bc3ffd34d84ef4dbea916901f28f65/src/motor_controller/launch/launch_settings.py); 
this will: 
- Launch 5 nodes for the ports [`/dev/ttyUSB0`, `/dev/ttyUSB1`, `/dev/ttyUSB2`, `/dev/ttyUSB3`, `/dev/ttyUSB4`]:
  - Each node looks for X_SERIES motors with the IDs [`1`, `2`, `3`] and baud-rate `4Mbps`.
  - All motor IDs are scanned every 2s.
  - Current motor angles are read then published on `angle_port_X_mot_Y` at 10Hz.
  - A subscribers listens to the topic `set_port_X_mot_Y` with messages containing 'angle' `Float64` and 'seconds' `Float64`.
  At 100Hz, it sends the command to the motor to reach the 'angle' in the time 'seconds' by moving at a constant speed.
  - Those subscriber and publishers are created/deleted when a motor is connected/disconnected
- The node angle_remapper:
  - Converts topics names according to the table inside `topic_remapping.py`. Modify this file according to your needs.
  - Only uses standard `Float64` messages.
  - Subscribes to topics `set_joint_A_B_real` transmitting only angle (as `data: Float64`), then repeats onto `set_port_X_mot_Y` 
using 'angle' and always the same fix value for 'seconds'.

Terminal log should indicate which USB port is working and detecting which motor. 
Unplugging and plugging motor should also display a message.

# Commands example
## Controller node

Listen to port 1 motor 1's angle
````bash
ros2 topic echo /angle_port_1_mot_1
````

Commands port 1 motor 1 to go to angle 0 in 3 seconds: 
````bash
ros2 topic pub /set_port_1_mot_1 dyna_controller_messages/msg/AngleTime "{angle: 0.0, seconds: 3.0}" -1
````

Commands port 1 motor 1 to go to angle 1 rad in 3 seconds:
````bash
ros2 topic pub /set_port_1_mot_1 dyna_controller_messages/msg/AngleTime "{angle: 1.0, seconds: 3.0}" -1
````

## Remapper node

Commands joint 1_0 to go to angle 0 in the time specified in launch setting (default is 0.15s).
According to the default remapping (topic_remapping.py) joint 1_0 corresponds to port 1 motor 1.
````bash
ros2 topic pub /set_joint_1_0_real std_msgs/msg/Float64 "{data: 0.0}" -1
````

Current angles of port 1, motor 1 can also be listend to after the remapping
````bash
ros2 topic echo /angle_1_0
````

# About
## Dynamixel Motors settings and connection

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
