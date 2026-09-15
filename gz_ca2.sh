#!/bin/bash
source install/setup.bash
# There is only one Gazebo world for CA2, so maze_id does not apply here (see ca2.sh).
# Pass 'headless' as the first arg to run without a GUI window (for remote/SSH sessions
# with no reachable display) -- physics and /odom still work, only the 3D viewer is skipped.
headless=false
if [ "$1" = "headless" ]; then
    headless=true
fi
ros2 launch rb2301_gz ca2_gazebo.launch.py x:=0.0 y:=0.0 headless:=$headless
./kill.sh # kills any lingering Gazebo