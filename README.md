# Frontier Exploration
[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

---

## Overview
frontier_exploration, is a ROS package to map an unknown indoor environment using an autonomous mobile ground robot. It combines an exploratory behavior with  simultaneous localization and mapping (SLAM) to autonomously map any unknown indoor environment.

## Design and Development Process

The Solo Iterative Process (SIP) model, which is an iterative process that repeatedly modifies software and is suitable for the stages of software evolution, testing and servicing, will
be followed throughout the course of the project as it is being built by a single developer (rohithjayarajan).

[Log Details for the Project][reference-id-for-log-details]

[reference-id-for-log-details]: https://docs.google.com/spreadsheets/d/1UNbX6C1j1Hg_Kum1Vs86YjXetcIUCho0L21Nnyfcikk/edit?usp=sharing

## Dependencies

### ROS

[ROS][reference-id-for-ROS] The Robot Operating System (ROS) is a flexible 
framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify 
the task of creating complex and robust robot behavior across a wide variety of robotic platforms.
This package is developed and tested in ROS Kinetic on Ubuntu 16.04 LTS and needs ROS Kinetic Kame installed for use. 
The entire installation instructions for ROS Kinetic Kame and its dependencies can be found [here][reference-id-for-ROS Kinetic].

[reference-id-for-ROS Kinetic]: http://wiki.ros.org/kinetic
[reference-id-for-ROS]: http://www.ros.org/install/

### catkin

Catkin is included by default when ROS is installed. Catkin can also be installed from source or prebuilt packages. 
Most users will want to use the prebuilt packages, but installing it from source is also quite simple. Installation 
instructions can be found [here][reference-id-for-catkin]

[reference-id-for-catkin]: http://wiki.ros.org/catkin

### Package Dependencies
- roscpp
- std_msgs
- sensor_msgs

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
## Known Issues/Bugs

//TODO(rohithjayarajan) update with any known issues/bugs after baseline build

## Run Instructions

//TODO(rohithjayarajan) add run instructions after baseline build

## Logging

//TODO(rohithjayarajan) add logging information instructions after baseline build

## Running Rostest

//TODO(rohithjayarajan) add ROS test instructions after baseline build

## Killing Processes

//TODO(rohithjayarajan) add information to kill processes after baseline build
