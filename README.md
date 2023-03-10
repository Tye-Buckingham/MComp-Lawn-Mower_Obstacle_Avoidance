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
  * 09/03/2023: Fixed doubling back if off course. 
      * If the robot becomes 'off-course' but an obstacle is between it and the ideal path then it continues as normal until past the obstacle
  * 10/03/2023: Fixed issues with robot seeing through walls
      * Added method to prevent movement that results in moving through walls
      * Changed logic for determining which direction to move when an object is found
 
# TODO 

  * [x] Handle multiple objects
      * Both close together and far apart (allowing movement between)
  * [x] Stop the robot seeing through walls
  * [ ] Handle case if point is inaccessible due to an obstacle covering, surrounding or blocking
  * [x] Combine with route mapping - if off course and no obstacles, move back to route.
  * [ ] Move robot to end of detected end point
  
# Known Issues

  * Integrating the route traversal has produced some undesirable code and logic issues 
  * If an object is flat between the target and the robot it has the chance to go back and forth without making progress
  * If the robot gets too close to an obstacle there are some issues with detecting points, this is thought to be because:
      * The 'LiDAR' comes from a single points but a real sensor would have some width
      * A real LiDAR would not need functions to find points and remove points behind walls
	  This may be fixed by changing the movement distance and bearing to keep some distance between the obstacle and the robot.
      * Currently a check has been added to determine if an object is too close or if movement results in moving within an object to adjust movement accordingly
	  It is still possible for robot to get stuck or pass through walls, but it is less common

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
