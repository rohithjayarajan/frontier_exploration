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
  return RUN_ALL_TESTS();
}
