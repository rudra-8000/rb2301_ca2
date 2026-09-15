#!/bin/bash
source install/setup.bash
# Optional first arg selects which real maze to use (default 0). Only affects the real
# robot; ignored when running against the Gazebo sim.
maze_id=${1:-0}
ros2 run rb2301_ca2 path_planning --ros-args -p maze_id:=$maze_id
