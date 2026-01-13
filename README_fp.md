1. Extract compressed rb2301_fp folder to desired directory (e.g. ~/Documents).
2. Change directory to extracted folder then colcon build.\
`cd ~/Documents/rb2301_fp`	\
`colcon build --symlink-install`
3. Start the gazebo sim in a terminal while in the workspace folder.\
`./gz_fp.sh` \
If unable to run, you may need to assign permisions to run .sh files, if you have never done so. Do this with: \
`chmod +x *.sh`
4. In another terminal, run the path planning script. \
`./fp.sh`
5. Add your code in the relevant portion within the rb2301_fp package's `obstacle_course.py`. 
To enable/disable randomisation of Gazebo map, set the various randomise variables to either `True` or `False` respectively in `obstacle_modifier.py`.

