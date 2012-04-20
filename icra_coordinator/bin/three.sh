#!/bin/bash

ELECTRIC_BASH="source /opt/ros/electric/setup.bash"
FUERTE_BASH="source /opt/ros/fuerte/setup.bash"
MASTER_HOST="export ROS_MASTER_URI=http://localhost"
PACKAGE_PATH="export ROS_PACKAGE_PATH=~/wg/icra:\$ROS_PACKAGE_PATH"

TMP_HISTORY=/tmp/.bash_history_coordinator
echo "roslaunch icra_coordinator coordinator.launch --screen" > $TMP_HISTORY
gnome-terminal --title=Coordinator -x bash -i -c "export HISTFILE=$TMP_HISTORY; $FUERTE_BASH; $PACKAGE_PATH; $MASTER_HOST:11401; bash -i"

TMP_HISTORY=/tmp/.bash_history_arm
echo "roslaunch arm_service_interface arm.launch --screen" > $TMP_HISTORY
gnome-terminal --title=Arm -x bash -i -c "export HISTFILE=$TMP_HISTORY; $FUERTE_BASH; $PACKAGE_PATH; $MASTER_HOST:11411; bash -i"

TMP_HISTORY=/tmp/.bash_history_pr2
echo "roslaunch pr2_service_interface pr2.launch --screen" > $TMP_HISTORY
gnome-terminal --title=PR2 -x bash -i -c "export HISTFILE=$TMP_HISTORY; $FUERTE_BASH; $PACKAGE_PATH; $MASTER_HOST:11421; bash -i"

TMP_HISTORY=/tmp/.bash_history_vehicle
echo "roslaunch vehicle_service_interface vehicle.launch --screen" > $TMP_HISTORY
gnome-terminal --title=Vehicle -x bash -i -c "export HISTFILE=$TMP_HISTORY; $FUERTE_BASH; $PACKAGE_PATH; $MASTER_HOST:11431; bash -i"
