# MComp Lawn Mower Obstacle Avoidance

 Obstacle avoidance using LiDAR testing and simulation. 
 Development and testing of methods for eventual use in the automatic lawn mower. 
 To use input a perimeter, a list of nogo zones, a start point, and an end point. These points should be in GPS format as they will then be converted to UTM.
 
 UTM has been chosen as the distance the lawn mower will travel is small enough such that the curvature of the Earth is thought to have negligible affect on accuracy. UTM also provides an easy way to traverse the space in metres, using the compass on the robot.
 
# Changelog 

  * 17/02/2023: Added methods to stop the robot seeing through obstacles. The method is more relatively exhaustive but wouldn't be needed in real-life.
  * 17/02/2023: Testing with a smaller LiDAR range. Fixed getting stuck on a wall. Allowing for more direct travel.
 
# TODO 

  
  * [x] Handle multiple objects
      * Both close together and far apart (allowing movement between)
  * [x] Stop the robot seeing through walls
  * [ ] Handle case if point is inaccessible due to an obstacle covering, surrounding or blocking

# Examples

	These examples can be found in larger verisions within './Images'
	
![Testing_Obstacle_Avoidance_Animated](./Images/Target_Focused_SM.gif)
![Testing_Obstacle_Avoidance_Animated](./Images/Endpoint_Focused_SM.gif)
![Testing_Obstacle_Avoidance_Animated](./Images/No_Xray_Vision_SM.gif)
![Testing_Obstacle_Avoidance_Animated](./Images/Seperate_Objects_SM.gif)
![Testing_Obstacle_Avoidance_Animated](./Images/Smaller_LiDAR_Distance_SM.gif)
