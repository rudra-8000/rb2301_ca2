1. Extract compressed rb2301_ca2 folder to desired directory (e.g. ~/Documents)
2. Change directory to extracted folder then colcon build
   '''bash
      cd ~/Documents/rb2301_ca2	
      colcon build --symlink-install
   '''
3. Start the gazebo sim in a terminal while in the workspace folder
   '''bash
      ./gz_ca2.sh
   '''
   
4. In another terminal, run the obstacle avoidance script.
   '''bash
      ./ca2.sh
   '''
   Add your code in the relevant portion within the rb2301_ca2 package's path_planning.py 

Notes for Yan Kai:
1. No longer uses rviz; maps are printed with PIL library
2. No longer uses the map_loader; occupancy grid given as a np array in the package file itself
2. Not sure how to implement shutdown procedure once all goal coordinates have been reached; line 149 of path_planning.py
3. 
