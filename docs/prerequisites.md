# Prerequisites
These instructions are for installing required dependencies on Ubuntu 18.04.

## Setting up TurtleBot2
Follow the instructions in [this](https://www.youtube.com/watch?v=rniyH8dY5t4) video. Note: Instead of making a new turtlebot_ws, you may just follow the instructions in your own catkin_ws.

## Adding the Aruco Marker
1) Locate the kobuki_description project inside of your catkin workspace

2) Replace `kobuki_description/urdf/kobuki.urdf.xacro` with `senseless-robot/docs/assets/kobuki.urdf.xacro`

3) Replace `kobuki_description/urdf/kobuki_gazebo.urdf.xacro` with `senseless-robot/docs/assets/kobuki_gazebo.urdf.xacro`

4) Replace `kobuki_description/package.xml` with `senseless-robot/docs/assets/package.xml`

5) Copy `senseless-robot/docs/assets/media` to `kobuki_description`

6) Go to root of catkin workspace, build + source

7) To test, run `roslaunch turtlebot_gazebo turtlebot_world.launch`

## Running the EKF
1) Clone [this](https://github.com/troiwill/dyfos-ros-target-localization/tree/main) repository

2) Switch to the tb-ekf branch and change directory into the tb folder

3) Copy all the files there to their respective folders in the catkin workspace

4) Run
```
sudo apt-get install ros-melodic-octomap ros-melodic-octomap-mapping
sudo apt-get install ros-melodic-robot-localization
```

5) Find the line in `turtlebot_gazebo/launch/turtlebot_world_ekf.launch` that creates the octomap node and replace it with
```
<node pkg="octomap_server" type="octomap_server_node" name="octomap_server" args="$(find dyfos_target_localize)/worlds/sim/gazebo/random_cylinders.bt">
```

6) Run the following. Look for errors during launch, there might be some Python packages to install (like pyquaternion)
```
roslaunch turtlebot_gazebo turtlebot_world_ekf.launch
```

7) Change directory to `catkin_ws/src/src/turtlebot_apps/turtlebot_navigation/launch/`. Run
```
roslaunch move_base_laser.launch
```

8) Run
```
rosrun turtlebot_gazebo send_goal.py -x 7 -y 3 -yaw 1.5
```
