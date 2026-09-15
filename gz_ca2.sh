#!/bin/bash
source install/setup.bash
# There is only one Gazebo world for CA2, so maze_id does not apply here (see ca2.sh).
# Pass 'headless' as the first arg to run without a GUI window (for remote/SSH sessions
# with no reachable display) -- physics and /odom still work, only the 3D viewer is skipped.
# Note: the robot's camera/LiDAR sensors need *some* render context even with no GUI, so
# headless mode also needs a virtual display (Xvfb) -- install with:
#   sudo apt-get install -y xvfb
headless=false
if [ "$1" = "headless" ]; then
    headless=true
fi

if [ "$headless" = "true" ] && command -v xvfb-run >/dev/null 2>&1; then
    xvfb-run -a ros2 launch rb2301_gz ca2_gazebo.launch.py x:=0.0 y:=0.0 headless:=$headless
elif [ "$headless" = "true" ]; then
    echo "WARNING: xvfb-run not found -- sensor rendering will likely crash Gazebo." >&2
    echo "         Install it with: sudo apt-get install -y xvfb" >&2
    ros2 launch rb2301_gz ca2_gazebo.launch.py x:=0.0 y:=0.0 headless:=$headless
else
    ros2 launch rb2301_gz ca2_gazebo.launch.py x:=0.0 y:=0.0 headless:=$headless
fi
./kill.sh # kills any lingering Gazebo