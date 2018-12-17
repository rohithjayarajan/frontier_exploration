# Frontier Exploration
[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Build Status](https://travis-ci.org/rohithjayarajan/frontier_exploration.svg?branch=master)](https://travis-ci.org/rohithjayarajan/frontier_exploration)
[![Coverage Status](https://coveralls.io/repos/github/rohithjayarajan/frontier_exploration/badge.svg?branch=master)](https://coveralls.io/github/rohithjayarajan/frontier_exploration?branch=master)

---

## Overview
frontier_exploration, is a ROS package to map an unknown indoor environment using an autonomous mobile ground robot. It combines an exploratory behavior with  simultaneous localization and mapping (SLAM) to autonomously map any unknown indoor environment.

## Design and Development Process

The Solo Iterative Process (SIP) model, which is an iterative process that repeatedly modifies software and is suitable for the stages of software evolution, testing and servicing, will
be followed throughout the course of the project as it is being built by a single developer (rohithjayarajan).

[SIP Log Details for the Project][reference-id-for-log-details]

[Sprint Planning Notes for the Project][reference-id-for-SIP-planning-notes]

[Presentation Slides for the Project][reference-id-for-ppt]

[Presentation Video for the Project][reference-id-for-vid]

[reference-id-for-log-details]: https://docs.google.com/spreadsheets/d/1UNbX6C1j1Hg_Kum1Vs86YjXetcIUCho0L21Nnyfcikk/edit?usp=sharing
[reference-id-for-SIP-planning-notes]: https://docs.google.com/document/d/1JhXZGQyf6TzoRhE24SGvOkqI1fCqavrlQTYm3uAaZm8/edit?usp=sharing
[reference-id-for-ppt]: https://docs.google.com/presentation/d/1-oxa_dVyRKhQ15pwvrmVlU6Bpb2wde8cZan42DYR5Ts/edit?usp=sharing
[reference-id-for-vid]: https://youtu.be/S-Ho4W4Ud3M
## Dependencies

### ROS

[ROS][reference-id-for-ROS] The Robot Operating System (ROS) is a flexible 
framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify 
the task of creating complex and robust robot behavior across a wide variety of robotic platforms.
This package is developed and tested in ROS Kinetic on Ubuntu 16.04 LTS and needs ROS Kinetic Kame installed for use. 
The entire installation instructions for ROS Kinetic Kame and its dependencies can be found [here][reference-id-for-ROS Kinetic].

- [Gazebo][reference-id-for-Gazebo]: Gazebo is included by default when ROS is installed. It is a set of ROS packages that provide the necessary interfaces to simulate a robot in the Gazebo 3D rigid body simulator for robots. Installation instructions can be found [here][reference-id-for-GazeboInstall]. Gazebo version 7.0 or higher is recommended.

- [RViz][reference-id-for-rviz]: The ROS 3-D robot visualizer. This should be installed by default when the full version of ROS has been installed. If not, the clickable [link][reference-id-for-rviz] has installation instructions.

- [gmapping][reference-id-for-gmapping]: This package consists of a ROS wrapper for OpenSlam's Gmapping. The gmapping package provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called slam_gmapping. This should be installed by default when the full version of ROS has been installed. If not, the clickable [link][reference-id-for-gmapping] has installation instructions.

- [actionlib][reference-id-for-actionlib]: The actionlib stack provides a standardized interface for interfacing with preemptable tasks. Examples of this include moving the base to a target location, performing a laser scan and returning the resulting point cloud, detecting the handle of a door, etc. This should be installed by default when the full version of ROS has been installed. If not, the clickable [link][reference-id-for-actionlib] has installation instructions.

- [move_base][reference-id-for-movebase]: The move_base package provides an implementation of an action (see the actionlib package) that, given a goal in the world, will attempt to reach it with a mobile base. This should be installed by default when the full version of ROS has been installed. If not, the clickable [link][reference-id-for-movebase] has installation instructions.

[reference-id-for-ROS Kinetic]: http://wiki.ros.org/kinetic
[reference-id-for-ROS]: http://www.ros.org/install/
[reference-id-for-Gazebo]: http://gazebosim.org/
[reference-id-for-GazeboInstall]: http://gazebosim.org/tutorials?tut=ros_installing
[reference-id-for-gmapping]: http://wiki.ros.org/gmapping
[reference-id-for-actionlib]: http://wiki.ros.org/actionlib
[reference-id-for-movebase]: http://wiki.ros.org/move_base
[reference-id-for-rviz]: http://wiki.ros.org/rviz

### catkin

Catkin is included by default when ROS is installed. Catkin can also be installed from source or prebuilt packages. 
Most users will want to use the prebuilt packages, but installing it from source is also quite simple. Installation 
instructions can be found [here][reference-id-for-catkin]

[reference-id-for-catkin]: http://wiki.ros.org/catkin

### Package Dependencies
- roscpp
- sensor_msgs
- std_msgs
- nav_msgs
- actionlib
- actionlib_msgs
- move_base_msgs
- tf
- turtlebot packages: These can be installed in ROS Kinetic running on Ubuntu 16.04 by using the below command in the terminal

```
sudo apt-get install ros-kinetic-turtlebot-*
```

## Build Instructions

### Creating catkin workspace:
Follow the below comamnds to create a catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
The above commands will create a workspace for your packages with CMakeLists.txt link in the src folder of catkin_ws. 
Source the setup.*sh file: 
```
source devel/setup.bash
```
### Building package inside catkin workspace: 
Follow the below comamnds and clone this package in the src folder of the catkin workspace 
```
cd ~/catkin_ws/src/
git clone https://github.com/rohithjayarajan/frontier_exploration.git
```
Follow the below comamnds to build the package
```
cd ~/catkin_ws/
catkin_make
```

## Run Instructions Using Launch File

Follow the below instructions in the terminal to run TurtleBot simulation which exhibits a forntier exploration algorithm and displays the simulation in Gazebo as well as Rviz

First of all, bring up the turtlebot in a world and visualize the same in both Gazebo and Rviz. This has to be done first in a new terminal. Follow the below lines of instructions in a new terminal which will bring up the turtlebot in a world and visualize the messages and sensor information in Rviz.

```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch frontier_exploration visualizer.launch
```

After the above instruction has been successfully performed, we start the gmapping node as well as the node for the navigation stack of the turtlebot. Follow the below lines of instructions in a new terminal which will start the map building process, start the navigation stack activity and in turn start frontier exploration.

```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch frontier_exploration frontier_exploration.launch 
```


## Run Instructions by Running Nodes Individually

Another way to get the project to run is by follwing the below mentioned steps in order to get the same results of that of the run instructions using a launch file. It is important to maintain the order of running these instructions. It is assumed that the bash file of ROS has been source in the bashrc. This is a safe assumption to make as it is a step in the installation process.

Bring up the turtlebot in the Gazebo simulation in a preferred world. The step to bringup the turtlebot in the default world is given here. Follow these commands in a new terminal:

```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch turtlebot_gazebo turtlebot_world.launch 
```

Next, turn on RViz to visualze the robot scans and messages in 3D. Follow these commands in a new terminal:

```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch turtlebot_rviz_launchers view_navigation.launch 
```

Then, start the mappping and navigation stack for the turtlebot. Follow these commands in a new terminal:

```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch turtlebot_navigation gmapping_demo.launch 
```

Finally, after these three instructions have been succesully completed, start the frontier exploration on the turtlebot. Follow these commands in a new terminal:

```
cd ~/catkin_ws/
source devel/setup.bash
rosrun frontier_exploration frontier_exploration_node
```

This should give same results as that of the run instructions using the launch files.

## Logging

Once the node is running in the background using the roslaunch method, to visualize the logger messages in a GUI, in a new terminal follow the below commands

```
source devel/setup.bash
rosrun rqt_console rqt_console
```

Once the node is running in the background using the roslaunch method, to visualize logger_level GUI, in a new terminal follow the below commands

```
source devel/setup.bash
rosrun rqt_logger_level rqt_logger_level
```

## Running Rostest

## Running Rostest

Rostest and gtest have been used to write the unit tests for the talker node service. Run the following commands in a new terminal to build the tests

```
cd ~/catkin_ws/
source devel/setup.bash
catkin_make run_tests_frontier_exploration
```

An output similar to the below is produced on following the above command:

```
[ROSTEST]-----------------------------------------------------------------------

[frontier_exploration.rosunit-frontier_exploration_tests/testPubIsOkay][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/testMapIsOkay][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/testUpdateRobotPose][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/testRotate][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/getSetIsOnFrontier][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/getSetfrontierClass][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/getSetMapXYCoordinate][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/getSetMapData][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/getSetMapReslution][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/getSetMapHeight][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/getSetMapWidth][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/getSetMapOigin][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/testIsFrontier][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/testClassifyFrontiers][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/testFrontierCentroid][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/testGrid2world][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/testDetectFrontiersCenterWorld][passed]
[frontier_exploration.rosunit-frontier_exploration_tests/testReadOccupancyGrid][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 18
 * ERRORS: 0
 * FAILURES: 0
```

## Building for Code Coverage
```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.

Running locally, the follwing result is obtained:
![alt text](https://github.com/rohithjayarajan/frontier_exploration/blob/master/results/revision2/coverage.png)

## Doxygen Documentation

To generate Doxygen Documentation,
```
cd <path to repository>
mkdir <documentation_folder_name>
cd <documentation_folder_name>
doxygen -g <config_file_name>

```
Update PROJECT_NAME and INPUT fields in the configuration file.

Then run the following command to generate the documentations,
```
doxygen <config_file_name>
```

## Killing Processes

Kill the above three processes by pressing CTRL+C in the aforementioned terminals where roscore and rosrun have been run.
Another way to kill the nodes is by running the below command in a new terminal

```
rosnode kill [node_name]
```

## Known Issues/Bugs

Issue for future work: Try using RGBD SLAM instead of SLAM from the laserscanner as the pipelin used by the gmapping SLAM package is not feature rich.

## About the Developer

Rohith Jayarajan- Robotics graduate student in the third semester of the Masters of Engineering program at the University of Maryland, College Park. Primary interests lie in computer vision, machine learning and the intersection the two fields.
