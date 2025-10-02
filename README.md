1. Extract compressed rb2301 folder to desired directory (e.g. ~/Documents)
2. Change directory to extracted folder then colcon build
   '''bash
      cd ~/Documents/rb2301	
      colcon build --symlink-install
   '''
3. Start the gazebo sim in a terminal 
   '''bash
      ./gz_ca1.sh
   '''
   If unable to run, you may need to assign permisions to run .sh files, if you have never done so. Do this with:
   '''bash
      chmod +x *.sh
   '''
   After that, run the gz_ca1.sh script as usual. It might take awhile to load, but you should see a randomly generated world with coke cans as obstacles and the Sparklife robot. Modify can spawning properties under rb2301/src_ca1/rb2301_ca1/rb2301_ca1/obstacle_generator.py
   
4. In another terminal, run the obstacle avoidance script.
   '''bash
      ./ca1.sh
   '''
   Add your code in the relevant portion within rb2301/src_ca1/rb2301_ca1/rb2301_ca1/obstacle_avoidance.py 
