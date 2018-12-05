/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    main.cpp
 *  @author  rohithjayarajan
 *  @date 12/2/2018
 *  @version 0.1
 *
 *  @brief main file for the node
 *
 *  @section DESCRIPTION
 *
 *  main file to run the frontier_exploration_node
 *
 */
#include "frontier_exploration_node.hpp"

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command
   * line. For programmatic remappings you can use a different version of init()
   * which takes remappings directly, but for most command-line programs,
   * passing argc and argv is the easiest way to do it.  The third argument to
   * init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "frontier_exploration_node");
  // create an object of RoombaWalker class
  FrontierExplore turtlebot;
  // start the walker behavior for the robot
  turtlebot.startExploration();
  return 0;
}
