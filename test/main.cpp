/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    main.cpp
 *  @author  rohithjayarajan
 *  @date 12/3/2018
 *  @version 1.3
 *
 *  @brief Tests for the ROS package
 *
 *  @section DESCRIPTION
 *
 *  Code to run ROS tests for classes and ROS features
 *
 */

#include <gtest/gtest.h>
// ROS header
#include <ros/ros.h>

/**
 *   @brief main function for ROS tests
 *
 *   @param argc  The argc
 *   @param argv  The argv
 *   @return int value of 0 on successful execution of function, -1 otherwise
 */
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
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
  return RUN_ALL_TESTS();
}
