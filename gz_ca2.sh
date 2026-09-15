#!/bin/bash
source install/setup.bash
# There is only one Gazebo world for CA2, so maze_id does not apply here (see ca2.sh).
ros2 launch rb2301_gz ca2_gazebo.launch.py x:=0.0 y:=0.0
./kill.sh # kills any lingering Gazebo