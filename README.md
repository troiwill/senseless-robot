# Senseless Robot ROS

## Overview

This package provides a state machine for a robot that moves along a path and stop incrementally to wait for measurments from an external source (such as another robot, a motion capture system, or a simulation).

**Note:** Currently, this package only works for the Turtlebot3 Waffle *with no exteroceptive sensors*. However, this package is expandable to other Turtlebots and, more generally, other ground robots that use move base to navigate.

## Installation

### Prerequisites

1. Install the ROS packages.
```
sudo apt-get install ros-noetic-robot-localization \
    ros-noetic-octomap-server \
    ros-noetic-smach
```

2. Clone the custom Turtlebot3 packages in your catkin workspace.
```
cd <catkin_ws>/src
git clone https://github.com/troiwill/turtlebot3.git -b senseless-tb3
git clone https://github.com/troiwill/turtlebot3_simulations.git -b senseless-tb3
```

3. Clone this repository in your catkin workspace.
```
cd <catkin_ws>/src
git clone https://github.com/troiwill/senseless-robot-ros.git
```

4. Install these Python packages.
```
python3 -m pip install numpy \
    spatialmath-python
```

5. Build your catkin workspace.
```
cd <catkin_ws>
catkin build
source devel/setup.bash
```

## Quick Start

1. Open two terminal windows.

2. In the first window, run the command below. This command will launch Rviz, Gazebo with the `turtlebot3_senseless_waffle` robot in the turtlebot3 world, and a few ROS nodes.
```
TURTLEBOT3_MODEL="senseless_waffle" roslaunch senseless_robot_ros turtlebot3_world_tour.launch
```
After you run this command, wait for ~8 seconds after everything loads.

3. In the second window, run the command below. This command runs the rendezvous state machine.
```
roslaunch senseless_robot_ros rendezvous_state_machine.launch
```
