# MComp Lawn Mower Obstacle Avoidance

 Obstacle avoidance using LiDAR testing and simulation. 
 Development and testing of methods for eventual use in the automatic lawn mower. 
 To use input a perimeter, a list of nogo zones, a start point, and an end point. These points should be in GPS format as they will then be converted to UTM.
 The nogo zones and perimeter can typed literally, but the current method (and would be easier) to use a .out file created using np.savetxt.
 
 UTM has been chosen as the distance the lawn mower will travel is small enough such that the curvature of the Earth is thought to have negligible affect on accuracy. UTM also provides an easy way to traverse the space in metres, using the compass on the robot.
 
 A perimeter is needed but to test random nogos within the perimeter use the `-t` arg, followed by the number of runs. This will output the final image,
 total distance, coverage, and the number of skipped checkpoints when finished. To output printing messages the user can pass `-v`, and to output images 
 for each movement (to create GIFs similar to those shown in this README) the user can use `-p`.
 
 **Note**: Images can cause a long wait time due to the number of detected points being plotted.
 
 If the user does not wish to use GPS points for the perimeter or nogo zones then generic x, y points should be fine but the conversion to UTM needs
 to be omitted.
 
 To generate a path for the robot to follow within the given perimeter one can use my `mapping` repo. 
 
	This repo is intended to be used to learn about methods, 
	and test said methods to determine efficiency 
	and find issues before implementing on the 
	robot designed for this MCOMP project.
	
	
# Usage

  1. Place a `perimeter.out` file in the root directory. This file contains the perimeter points in a format of *x,y\n*.
  2. Repeat step 1 for any nogo zones, naming them `obstacle_0.out`, `obstacle_1.out`, ... etc.
  3. Run `lidar_sim.py` with the desired cmd args {-p, -t [some number], -v, -r, -s}.
  4. The program will run, outputting to stdout if using -v, and saving images after each movement if using -p.
  5. Images will be saved in the 'Imgs' folder.
  6. Tests run with -t will output the obstacles to the 'Tests' folder.
  
  ![Testing_Obstacle_Avoidance_Animated](./Images/dir_example.png)
 
  
# Known Issues

  * Max tries methods have different logic and so the name and return need to be changed.
  * Some methods have grown too large and less succinct, some refactoring is needed.
  * Currently, the ROS node uses the detect object method for 'detecting' the virtual boundaries, and the LiDAR system
  to detect unknown, real objects. To do this the nogo-zones and perimeter are sometimes used the same, and sometimes not.
  Additionally, in some cases a Polygon is used, in others a list of points. This is not clear and needs cleaning up.
  * Checks to determine if moving crosses a virtual boundary are in place, but none are in place for detected objects. Again,
  this is an artifact of combining the 'simulation' methods with the real-world sensor. Once the current methods have been tested, this
  will be addressed. 
  
# State Diagrams

*Work in progress* - states are split to make them more readable and easier to understand (links only work when viewing raw image in browser)

  * Main State - four main states, moving to point, avoiding obstacles, A* path, and back on track.
  
  ![Testing_Obstacle_Avoidance_Animated](./Images/main.svg)
  
  * A* Path State - Similar to main state but only initialised when the robot cannot reach a point within N amount of moves, 
  in this state the robot does not become off course as this allows path (if not perfect) to pull the robot towards 
  the desired point. If the robot makes no progress in this state then it skips a point, otherwise a recurisve loop of A* could occur.
  
   ![Testing_Obstacle_Avoidance_Animated](./Images/atrack.svg)
   
  * Back on Track State - Similar to main state but only initialised when the robot is off course **and not avoiding obstacles**, and has only one point. 
  A* can be used to find a path to this point, but as this point moves as the robot moves this should not occur often. Ultimately, the back-on-track point 
  is inaccessable and A* cannot find this point, then it is likely the robot cannot reached the original point and should therefore skip it.
  
  ![Testing_Obstacle_Avoidance_Animated](./Images/btrack.svg)
  

# Current Pipeline

  * Generate a 'perfect' route with some amount of overlap, in the below case it is 25% the width of the robot
  
  ![Testing_Obstacle_Avoidance_Animated](./Images/Coverage_Route.png)
  
  * Reduce the number of points by removing those along the same line
      * This reduces the memory requirements but also reduces the chances a point is within an unknown object
	  
  ![Testing_Obstacle_Avoidance_Animated](./Images/Direction_Change_Route.png)
	  
  * Follow this route with unknown obstacles within
  
  ![Testing_Obstacle_Avoidance_Animated](./Images/Slightly_Improved_Final_Route.gif)
  
  * Without map matching to the route this gives the following result
  
  ![Testing_Obstacle_Avoidance_Animated](./Images/Slightly_Improved_Final_Coverage.png)
  
  * Applying map matching is the next step and should improve the total coverage
  
  
  * Using the robot's location and points detected from the LiDAR unknown objects can be detected
      * These points can be sent back to the server for a more optimal coverage map
      * This method could also be applied to improve the digital map's accuracy
	  
  ![Testing_Obstacle_Avoidance_Animated](./Images/Detected_Boundaries.png)
  
  * Applying the map matching to improve the overall coverage
  
  ![Testing_Obstacle_Avoidance_Animated](./Images/Integrated_MapMatching.gif)
  
	  When the robot is considered off course a new target point is determined.
	  This point lies on the line between the origin and current destination, 
	  initially it is the closest point on the line to the robot. However,
	  this creates problems when an obstacle is between it and this new point - 
	  the robot is most often off course when avoiding obstacles. In an attempt to 
	  avoid obstacles and move in the correct direction the new point is offset
	  by some value. It appears, the greater the value the less chance of getting stuck, 
	  but the lower total coverage.
  
  ![Testing_Obstacle_Avoidance_Animated](./Images/Integrated_MapMatching_Coverage.png)
  
	  Overall coverage can be limited to RTK accuracy, route, and unknown obstacles. 
	  These inaccuracies can be hard, or in some cases not possible due to the limitations
	  of the hardware in use. To overcome this one method is to provide more than one 
	  route. This provides more work for the robot and is in-efficient however, for 
	  an automated robot the time to finish requirement is not necessarily the primary
	  factor - though battery usage may be. In this project total coverage is the primary
	  focus and as such this is a convinient way to produce desired results.
	  
  * Using two overlapping routes to increase overall coverage 
  
  ![Testing_Obstacle_Avoidance_Animated](./Images/Overlapping_Route.png)
  ![Testing_Obstacle_Avoidance_Animated](./Images/Overlapping_Route.gif)
  ![Testing_Obstacle_Avoidance_Animated](./Images/Overlapping_Coverage.png)
  
# Tasks

  * What level of route overlap do we need? 
  * What balance of accuracy to memory efficiency should we have?
      * More points means a higher coverage percentage, the above routes are 907 and 118 points respectively. 
      * Can we find a middle ground or should we perform multiple different routes to account for the innacuracy.
  * What else, other than RTK inaccuracy, will affect our route?
  * What is an acceptable time to compute, total route distance, and time to complete route.
  * Testing different off course offset values.
  * A* is used to find a new path dependant on the known boundaries, when to ignore this route and skip to 
	the next checkpoint - deciding between coverage and time to find next point.


# Progress
	
![Testing_Obstacle_Avoidance_Animated](./Images/Target_Focused_SM.gif)
![Testing_Obstacle_Avoidance_Animated](./Images/Endpoint_Focused_SM.gif)
![Testing_Obstacle_Avoidance_Animated](./Images/No_Xray_Vision_SM.gif)
![Testing_Obstacle_Avoidance_Animated](./Images/Seperate_Objects_SM.gif)
![Testing_Obstacle_Avoidance_Animated](./Images/Smaller_LiDAR_Distance_SM.gif)
![Testing_Obstacle_Avoidance_Animated](./Images/Coverage_No_Mapping_Unknown_Obstacles_SM.gif)
![Testing_Obstacle_Avoidance_Animated](./Images/Integrated_MapMatching_SM.gif.gif)


# Changelog 

	Some updates do not come with a changelog update or 
	descriptive commit, this is due to the repo being a 
	place to test, develop, and keep backups of this work. 
	Only major changes which are considered to be long 
	term or substantial will be described in this section.
	This repo is primarily for learning, testing, and 
	theoretical development, it is not necessarily 
	intended to be used in actual projects, 
	but it is hoped that this repo can at least help 
	those understand and visualise methods required for 
	path traversal and obstacle detection.
	
	
  * 17/02/2023: Added methods to stop the robot seeing through obstacles. The method is more relatively exhaustive but wouldn't be needed in real-life.
      * Testing with a smaller LiDAR range. Fixed getting stuck on a wall. Allowing for more direct travel.
  * 18/02/2023: Tested on coverage map generated without known obstacles.
  * 08/03/2023: Integrated the route traversal methods from the `Mapping` repo
      * Fixed incorrect back-on-track point
      * Removed section instructing robot to move within the line detection method
  * 09/03/2023: Fixed doubling back if off course. 
      * If the robot becomes 'off-course' but an obstacle is between it and the ideal path then it continues as normal until past the obstacle
  * 10/03/2023: Fixed issues with robot seeing through walls
      * Added method to prevent movement that results in moving through walls
      * Changed logic for determining which direction to move when an object is found
  * 11/03/2023: Added methods to produce randomly generated nogos within scene, output the shapes, and output coverage, total distance, and number of points skipped
      * Now the robot has N number of tries (relating to distance from origin) to reach a point, if this fails then A* is used along with the robot's
	  knowledge of the scene from detecting points to determine if the point is accessible or not. **Needs more testing**
          * If the point is determined inaccessible then the robot will skip this checkpoint.
  * 12/03/2023: Instead of N amount of tries, A* is used when the robot makes minimal progress within N amount of moves.
  * 15/03/2023: Fixed issue with A* not providing a correct path
      * Testing more *realistic* values based on RTK inaccuracy and LiDAR ranges
  * 04/04/2023: Added method to reduce number of detected points to minimal set required for A*. Method needs more testing, but appears stable.
  * 16/04/2023: Added interfacing with a LiDAR sensor
      * Began working on a ROS node for the navigation system. 

 
# TODO 

  * [x] Handle multiple objects
      * Both close together and far apart (allowing movement between)
  * [x] Stop the robot seeing through walls
  * [x] Handle case if point is inaccessible due to an obstacle covering, surrounding or blocking
  * [x] Combine with route mapping - if off course and no obstacles, move back to route.
  * [ ] ~~Move robot to end of detected end point~~
  * [x] Test scenes with randomly generated nogos
  * [x] Reduce number of detected points to minimal set
  * [ ] Consider dimensions of robot - not just centroid
  * [ ] More user friendly input, options, and output
  * [ ] User guide
  * [x] Full documentation of methods
  

# References

The algorithm in its current state is based primarily on the work found in:

Peng, Y., Qu, D., Zhong, Y., Xie, S., Luo, J., & Gu, J. (2015, August). The obstacle detection and obstacle avoidance algorithm based on 2-d lidar. In 2015 IEEE international conference on information and automation (pp. 1648-1653). IEEE.

Further reading has been and will continue to be conducted therefore, this section will be updated when ever the implementation draws from those sources.
