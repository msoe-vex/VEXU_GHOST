#!/bin/bash
source /opt/ros/foxy/setup.bash
#source ~/rplidar_ros/install/setup.bash
source ~/VEXU_GHOST/install/setup.bash

#export LD_LIBRARY_PATH=/usr/local/lib:/usr/local/${LD_LIBRARY_PATH:+:$LD_LIBRAR>
#export DISPLAY=:0.0
pros v5 stop
pros v5 run 1


#ros2 launch ghost_ros hardware.launch.py
ros2 launch ghost_ros launch_robot.launch.py
pros v5 stop
