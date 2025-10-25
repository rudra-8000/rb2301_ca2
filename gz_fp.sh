#!/bin/bash
source install/setup.bash
ros2 launch rb2301_gz fp_gazebo.launch.py x:=3.6 y:=-2.6
./kill.sh # kills any lingering Gazebo