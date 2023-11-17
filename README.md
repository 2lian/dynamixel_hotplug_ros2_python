# Dynamixel Motor Ros2 package using Pyhton, Hotplug capable

This repo provides python libraries and ros2 nodes to control several dynamixels on several u2d2 interfaces 
with hotplug capabilities (motor can be (dis)connected at runtime).

- `multi_controller.py`: Custom python library to control multiple dynamixels connected to the same serial port 
using velocity profile and bulk raed/write serial commands.
- `multi_dynamixel.py`: Ros2 node (responsible for one single serial port) 
setting up the controller and providing ros2 subscribers/publisher.
- `multi_port_launch.py`: Launches several nodes, one node per serial port, with the corresponding motor parameters.

# About
## Dynamixel Motors settings and connection

Several dynamixel can be plugged onto the same serial controller. 
The Dynamixel Wizard should be used beforehand to set a unique ID to each motor, and set the baud-rate of the motor.

For the ros2 node, the motor id to look for, the baud-rate and usb port is set in the launcher `multi_port_launch.py`
and loaded from `moonbot_setting.py`.

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
Hence, the launcher creates several nodes, each responsible for one port. 
Each node runs in its own thread and can be called at any time in any order.



[//]: # (connecting usbv through WSL https://devblogs.microsoft.com/commandline/connecting-usb-devices-to-wsl/)

[//]: # ()
[//]: # (```bash)

[//]: # (usbipd wsl attach --busid 3-2)

[//]: # (```)
