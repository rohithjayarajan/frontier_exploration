/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    helper_functions_test.cpp
 *  @author  rohithjayarajan
 *  @date 12/2/2018
 *  @version 0.1
 *
 *  @brief source file for testing HelperFunctions class
 *
 *  @section DESCRIPTION
 *
 *  source file which contains tests for HelperFunctions class
 */
#include "helper_functions.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
/**
 *   @brief test function to test computeDistance method of HelperFunctions
 * class
 *
 *   @param HelperFunctionsTests, the gtest framework
 *   @param computeDistanceTest, the name of test
 */
TEST(HelperFunctionsTests, computeDistanceTest) {
  HelperFunctions testObject1;
  geometry_msgs::Point p1;
  geometry_msgs::Point p2;
  p1.x = 10;
  p1.y - 10;
  P1.z = 0;
  p2.x = 4;
  p2.y - 2;
  p2.z = 0;
  EXPECT_EQ(testObject1.computeDistance(p1, p2), 10.0);
}
/**
 *   @brief test function to test computeGradient method of HelperFunctions
 * class
 *
 *   @param HelperFunctionsTests, the gtest framework
 *   @param computeGradientTest, the name of test
 */
TEST(HelperFunctionsTests, computeGradientTest) {
  HelperFunctions testObject2;
  geometry_msgs::Point p1;
  geometry_msgs::Point p2;
  p1.x = 10;
  p1.y - 10;
  P1.z = 0;
  p2.x = 6;
  p2.y - 6;
  p2.z = 0;
  EXPECT_EQ(testObject2.computeGradient(p1, p2), M_PI_4);
}
/**
 *   @brief test function to test rad2deg method of HelperFunctions
 * class
 *
 *   @param HelperFunctionsTests, the gtest framework
 *   @param computeDistance, the name of test
 */
TEST(HelperFunctionsTests, rad2degTest) {
  HelperFunctions testObject3;
  double angleInRad = M_PI;
  double angleInDegree = 180.0;
  EXPECT_EQ(testObject3.rad2deg(angleInRad), angleInDegree);
}
/**
 *   @brief test function to test computeDistance method of HelperFunctions
 * class
 *
 *   @param HelperFunctionsTests, the gtest framework
 *   @param deg2radTest, the name of test
 */
TEST(HelperFunctionsTests, deg2radTest) {
  HelperFunctions testObject4;
  double angleInRad = M_PI;
  double angleInDegree = 180.0;
  EXPECT_EQ(testObject4.deg2rad(angleInDegree), angleInRad);
}
