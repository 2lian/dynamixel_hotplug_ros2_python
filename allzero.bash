export ANGLE=0.0
export TIME=5.0
. install/setup.bash

ros2 topic pub /set_port_1_mot_1 dyna_controller_messages/msg/AngleTime "{angle: ${ANGLE}, seconds: ${TIME}}" -1 &
ros2 topic pub /set_port_1_mot_2 dyna_controller_messages/msg/AngleTime "{angle: ${ANGLE}, seconds: ${TIME}}" -1 &
ros2 topic pub /set_port_1_mot_3 dyna_controller_messages/msg/AngleTime "{angle: ${ANGLE}, seconds: ${TIME}}" -1 

ros2 topic pub /set_port_2_mot_1 dyna_controller_messages/msg/AngleTime "{angle: ${ANGLE}, seconds: ${TIME}}" -1 &
ros2 topic pub /set_port_2_mot_2 dyna_controller_messages/msg/AngleTime "{angle: ${ANGLE}, seconds: ${TIME}}" -1 &
ros2 topic pub /set_port_2_mot_3 dyna_controller_messages/msg/AngleTime "{angle: ${ANGLE}, seconds: ${TIME}}" -1 

ros2 topic pub /set_port_3_mot_1 dyna_controller_messages/msg/AngleTime "{angle: ${ANGLE}, seconds: ${TIME}}" -1 &
ros2 topic pub /set_port_3_mot_2 dyna_controller_messages/msg/AngleTime "{angle: ${ANGLE}, seconds: ${TIME}}" -1 &
ros2 topic pub /set_port_3_mot_3 dyna_controller_messages/msg/AngleTime "{angle: ${ANGLE}, seconds: ${TIME}}" -1 

ros2 topic pub /set_port_0_mot_1 dyna_controller_messages/msg/AngleTime "{angle: ${ANGLE}, seconds: ${TIME}}" -1 &
ros2 topic pub /set_port_0_mot_2 dyna_controller_messages/msg/AngleTime "{angle: ${ANGLE}, seconds: ${TIME}}" -1 &
ros2 topic pub /set_port_0_mot_3 dyna_controller_messages/msg/AngleTime "{angle: ${ANGLE}, seconds: ${TIME}}" -1 
