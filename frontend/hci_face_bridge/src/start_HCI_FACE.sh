# !/bin/bash

source /home/qtrobot/robot/autostart/qt_robot.inc

SCRIPT_NAME="start_HCI_FACE.sh"
LOG_FILE=$(prepare_logfile "$SCRIPT_NAME")

{
prepare_ros_environment
wait_for_ros_node "/rosout" 60

roslaunch HCI_FACE_Bridge qt_robot_pi.launch
} &>> ${LOG_FILE}