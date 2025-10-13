#!/bin/bash
source install/setup.bash
ros2 launch rb2301_gz ca2_gazebo.launch.py x:=0.0 y:=0.0
./kill.sh # kills any lingering Gazebo