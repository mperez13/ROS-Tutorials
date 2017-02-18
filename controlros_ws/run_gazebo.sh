#!/bin/bash
source ~/ROS-Tutorials/controlros_ws/devel/setup.bash
source gz_setup.sh
sudo killall rosmaster
sudo killall gzserver
sudo killall gzclient

roslaunch mybot_gazebo samplebot.launch