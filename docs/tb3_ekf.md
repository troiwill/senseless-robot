# Running TurtleBot3 with EKF
These instructions are for localizing a TurtleBot3 with the EKF instead of AMCL on ROS Noetic.

## Setting up TurtleBot3
Clone the following repos into your catkin_ws and build.
1) https://github.com/ROBOTIS-GIT/turtlebot3 

2) https://github.com/ROBOTIS-GIT/turtlebot3_simulations 

3) https://github.com/ROBOTIS-GIT/turtlebot3_msgs 

## Adding the Launch File
Add `assets/turtlebot3_navigation_ekf.launch` from this repo to `catkin_ws/src/turtlebot3/turtlebot3_navigation/launch`.

## Running the EKF
After building and sourcing your workspace, do the following.

1) Run `export TURTLEBOT3_MODEL=burger`

2) Run `roslaunch turtlebot3_gazebo turtlebot3_world.launch`

3) Run `roslaunch turtlebot3_navigation turtlebot3_navigation_ekf.launch`

4) You should see an Rviz window open. Using the purple marker, you can create a goal for the TurtleBot3. Once you create the goal, the robot should automatically find a path and proceed to the goal.