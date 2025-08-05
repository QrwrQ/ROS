#!/bin/bash
source ./devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/rossi/catkin_ws/src/ars-2021-g15/src/minitask4/maps/mymap.yaml
