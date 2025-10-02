1. Removed Test scripts and folders from package to declutter.

2. Removed "include=" keyword argument from `setup.py`. It was causing problems in the builds from the first part of this course.

3. Added .gitignore to ignore any automatically generated `__pycache__` directory, in case the students want to upload to GitHub.

4. `obstacle_avoidance.py`:
    1. Renamed `ca1_obstacle_avoidance.py` to `obstacle_avoidance.py` to match the entry point name with the node name.
    2. Renamed the node name to `ObstacleAvoidanceNode` for the same reason.
    3. Replaced `/scan` topic to `scan`. I'm not sure if the node is run in a different namespace, and if `/scan` is necessary. For consistency, replaced with `scan`, which will be in the same namespace as the node.
    4. Added timer callback for the control loop, where students will implement their control from there. It is not illegal to use the subscriber callback for control, but in most control systems, the rate at which the control is running is conventionally fixed (theory etc.).
    5. Added guard to run the loop only after the first scan message is received, so that the node does not prematurely throw a run time error.
    6. Modified and added some comments.
    7. Formatted the code partially with Black Formatter VSCode Extension from Microsoft.


5. Suggest to change the workspace to `rb2301`, which is used in the first part. They have each a private GitHub repo for this already.

6. `ca1.sh` and avoid putting the source into the `.bashrc`. Sourcing `setup.bash` in `.bashrc` can cause unnecessary warnings when the directory is deleted after the course. It is possible to:
    1. Provide them with the ROS2 commands first, and then,
    2. **Advise them** to pack it into `.sh` scripts so it is easier to run (I have covered this in the first part). 
    3. For example, let the script `ca1.sh` in the **workspace directory** contain:
        
        ```bash
        #!/bin/bash
        source install/setup.bash
        ros2 run rb2301_ca1 ca1_obstacle_avoidance
        ```
    
        Assign permission only once in its lifetime:
    
        ```bash
        # cd to workspace first.
        chmod +x *.sh
        ```

        and then the two commands can now be run repeatedly with:

        ```bash
        ./ca1.sh
        ```

7. `gz.sh`. Created the gazebo running script `gz.sh` for the same reason as `ca1.sh`. You may want to rename this if the CA2, CA1, and final project use different worlds.

8. `kill.sh`. Added a kill Gazebo script `kill.sh` in case `Ctrl+C` does not terminate Gazebo properly, which can occur sometimes. This script kills all ruby scripts (from which Gazebo is run), so it will kill other ruby running scripts (for a fresh installation containing ROS2 and other course software, this would not be a problem). I have added this guy into `gz.sh` so they do not have to manually kill Gz.