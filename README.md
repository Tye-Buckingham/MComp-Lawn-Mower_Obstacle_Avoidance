# MComp Lawn Mower Obstacle Avoidance

 Obstacle avoidance using LiDAR testing and simulation. 
 Development and testing of methods for eventual use in the automatic lawn mower. 
 To use input a perimeter, a list of nogo zones, a start point, and an end point. These points should be in GPS format as they will then be converted to UTM.
 
 UTM has been chosen as the distance the lawn mower will travel is small enough such that the curvature of the Earth is thought to have negligible affect on accuracy. UTM also provides an easy way to traverse the space in metres, using the compass on the robot.
 
# Changelog 

  * 17/02/2023: Added methods to stop the robot seeing through obstacles. The method is more relatively exhaustive but wouldn't be needed in real-life.
      * Testing with a smaller LiDAR range. Fixed getting stuck on a wall. Allowing for more direct travel.
  * 18/02/2023: Tested on coverage map generated without known obstacles.
  * 08/03/2023: Integrated the route traversal methods from the `Mapping` repo
      * Fixed incorrect back-on-track point
      * Removed section instructing robot to move within the line detection method
 
# TODO 

  * [x] Handle multiple objects
      * Both close together and far apart (allowing movement between)
  * [x] Stop the robot seeing through walls
  * [ ] Handle case if point is inaccessible due to an obstacle covering, surrounding or blocking
  * [x] Combine with route mapping - if off course and no obstacles, move back to route.
  * [ ] Move robot to end of detected end point
  
# Known Issues

  * Integrating the route traversal has produced some undesirable code and logic issues - **priority**
  * Robot is seen doubly back on itself when off course to find a way round.
	Will test different movement amounts, number of off course points and off course point offset
	to determine if these can reduce this issue or whether more realistic movement may make it worse.

# Current Pipeline

  * Generate a perfect route with some amount of overlap, in the below case it is 25% the width of the robot
  
  ![Testing_Obstacle_Avoidance_Animated](./Images/Coverage_Route.png)
  
  * Reduce the number of points by remove those along the same line
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
  
# Tasks

  * What level of route overlap do we need? 
  * What balance of accuracy to memory efficiency should we have?
      * More points means a higher coverage percentage, the above routes are 907 and 118 points respectively. 
      * Can we find a middle ground or should we perform multiple different routes to account for the innacuracy.
  * What else, other than RTK inaccuracy, will affect our route?
  * What is an acceptable time to compute, total route distance, and time to complete route.
  * Testing different off course offset values.
  * Test functions to generate random arenas to produce coverage routes, traverse, and report if successfuly traversed.

# Progress

	
![Testing_Obstacle_Avoidance_Animated](./Images/Target_Focused_SM.gif)
![Testing_Obstacle_Avoidance_Animated](./Images/Endpoint_Focused_SM.gif)
![Testing_Obstacle_Avoidance_Animated](./Images/No_Xray_Vision_SM.gif)
![Testing_Obstacle_Avoidance_Animated](./Images/Seperate_Objects_SM.gif)
![Testing_Obstacle_Avoidance_Animated](./Images/Smaller_LiDAR_Distance_SM.gif)
![Testing_Obstacle_Avoidance_Animated](./Images/Coverage_No_Mapping_Unknown_Obstacles_SM.gif)
![Testing_Obstacle_Avoidance_Animated](./Images/Integrated_MapMatching_SM.gif.gif)


# References

The algorithm in its current state is based primarily on the work found in:

Peng, Y., Qu, D., Zhong, Y., Xie, S., Luo, J., & Gu, J. (2015, August). The obstacle detection and obstacle avoidance algorithm based on 2-d lidar. In 2015 IEEE international conference on information and automation (pp. 1648-1653). IEEE.

Further reading has been and will continue to be conducted therefore, this section will be updated when ever the implementation draws from those sources.
