1. Extract compressed rb2301_ca2 folder to desired directory (e.g. ~/Documents)
2. Change directory to extracted folder then colcon build \
`cd ~/Documents/rb2301_ca2`\
`colcon build --symlink-install` \
3. Start the gazebo sim in a terminal while in the workspace folder \
`./gz_ca2.sh`\
If unable to run, you may need to assign permisions to run .sh files, if you have never done so. Do this with: \
`chmod +x *.sh`   
4. In another terminal, run the path planning script. \
`./ca2.sh` \
Add your code in the relevant portion within the rb2301_ca2 package's `path_planning.py`.
