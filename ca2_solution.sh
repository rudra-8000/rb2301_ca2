#!/bin/bash
source install/setup.bash
# Optional first arg selects which real maze to use (default 0). Only affects the real
# robot; ignored when running against the Gazebo sim. Runs the reference solution
# (rb2301_ca2.path_planning_solution), not the student stub -- use ca2.sh for that.
maze_id=${1:-0}
ros2 run rb2301_ca2 ca2_solution --ros-args -p maze_id:=$maze_id
