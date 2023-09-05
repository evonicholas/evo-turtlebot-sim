## Autonomous Robot Navigation using move_base in ROS and Gazebo

Program which control Turtlebot 3 to keep moving and avoiding obstacles in the Gazebo world simulation.

## Environment
Ubuntu 20.04

ROS: Noetic

## Dependencies

ROS Packages:

turtlebot3
```bash
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
```


turtlebot3_msgs
```bash
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
```


turtlebot3_simulation
```bash
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

## Installation
This package has been tested with ROS Noetic

1. Clone this repository into your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/evonicholas/evo-turtlebot-sim.git
```
Assuming your workspace is "catkin_ws". In case its different, please change the catkin_ws with the name of your workspace.

2. Make sure to install external dependencies mentioned above

3. Build the package:  

```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

### Usage

```bash
export TURTLEBOT3_MODEL=burger # or waffle, waffle_pi
roslaunch evo-turtlebot-sim navigation.launch
```

The launch file will automatically launch gazebo and rviz for Turtlebot3 navigation.

Run the script:

Make sure the script is executable. You can make it executable by running the command: chmod +x script_name.py

Example:

```bash
cd catkin_ws/src/evo-turtlebot-sim/scripts/
chmod +x test_move.py
```bash

The script in this package are:

Make the robot move to the predetermine location:
```bash
rosrun evo-turtlebot-sim test_move.py
```

Make the robot move randomly
```bash
rosrun evo-turtlebot-sim random_move.py
```

In case the launch file did not work, you can launch it manually by follow this step:

Turtlebot3 gazebo simulation:

```bash
export TURTLEBOT3_MODEL=burger # or waffle, waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Turtlebot3 navigation for move_base:

```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```

Run the script again using same command as above



