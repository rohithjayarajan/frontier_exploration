/******************************************************************************
 *                          GNU GENERAL PUBLIC LICENSE
 *                           Version 3, 29 June 2007
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *****************************************************************************/
/**
 *  @file    map_structure_tests.cpp
 *  @author  rohithjayarajan
 *  @date 12/2/2018
 *  @version 0.1
 *
 *  @brief source file for testing MapStructure class
 *
 *  @section DESCRIPTION
 *
 *  source file which contains tests for MapStructure class
 */
#include <geometry_msgs/PoseStamped.h>
#include "map_structure.hpp"
/**
 *   @brief test function to test getter & setter method for resolution of
 * MapStructure class
 *
 *
 *   @param MapStructureTests, the gtest framework
 *   @param getSetResolution, the name of test
 */
TEST(MapStructureTests, getSetResolution) {
  HelperFunctions testObject1;
  double resolution_ = 0.05;
  testObject1.setResolution(resolution_);
  EXPECT_EQ(getResolution(), resolution_);
}
/**
 *   @brief test function to test getter & setter method for width of
 * MapStructure class
 *
 *
 *   @param MapStructureTests, the gtest framework
 *   @param getSetWidth, the name of test
 */
TEST(MapStructureTests, getSetWidth) {
  HelperFunctions testObject2;
  unsigned int width_ = 56;
  testObject2.setWidth(width_);
  EXPECT_EQ(getWidth(), width_);
}
/**
 *   @brief test function to test getter & setter method for height of
 * MapStructure class
 *
 *
 *   @param MapStructureTests, the gtest framework
 *   @param getSetHeight, the name of test
 */
TEST(MapStructureTests, getSetHeight) {
  HelperFunctions testObject3;
  unsigned int height_ = 96;
  testObject3.setHeight(height_);
  EXPECT_EQ(getHeight(), height_);
}
/**
 *   @brief test function to test getter & setter method for origin of
 * MapStructure class
 *
 *
 *   @param MapStructureTests, the gtest framework
 *   @param getSetOrigin, the name of test
 */
TEST(MapStructureTests, getSetOrigin) {
  HelperFunctions testObject4;
  geometry_msgs::Pose origin_;
  origin_.position.x = 1.2;
  origin_.position.y = 2.4;
  origin_.position.z = 3.6;
  origin_.orientation.x = 0;
  origin_.orientation.y = 0;
  origin_.orientation.z = 0;
  origin_.orientation.w = 1;
  testObject4.setOrigin(origin_);
  EXPECT_EQ(getOrigin(), origin_);
}
/**
 *   @brief test function to test getter & setter method for isOnFrontier of
 * MapStructure class
 *
 *
 *   @param MapStructureTests, the gtest framework
 *   @param getSetIsOnFrontier, the name of test
 */
TEST(MapStructureTests, getSetIsOnFrontier) {
  HelperFunctions testObject5;
  bool isOnFrontier_ = false;
  testObject5.setIsOnFrontier(isOnFrontier_);
  EXPECT_EQ(getIsOnFrontier(), isOnFrontier_);
}
/**
 *   @brief test function to test getter & setter method for frontierClass_ of
 * MapStructure class
 *
 *
 *   @param MapStructureTests, the gtest framework
 *   @param getSetFrontierClass, the name of test
 */
TEST(MapStructureTests, getSetFrontierClass) {
  HelperFunctions testObject6;
  unsigned int frontierClass_ = 23;
  testObject6.setFrontierClass(frontierClass_);
  EXPECT_EQ(getFrontierClass(), frontierClass_);
}
/**
 *   @brief test function to test getter & setter method for x_ of
 * MapStructure class
 *
 *
 *   @param MapStructureTests, the gtest framework
 *   @param getSetX, the name of test
 */
TEST(MapStructureTests, getSetX) {
  HelperFunctions testObject7;
  int x_ = 12;
  testObject7.setMapXCoordinate(x_);
  EXPECT_EQ(getMapXCoordinate(), x_);
}
/**
 *   @brief test function to test getter & setter method for y_ of
 * MapStructure class
 *
 *
 *   @param MapStructureTests, the gtest framework
 *   @param getSetY, the name of test
 */
TEST(MapStructureTests, getSetY) {
  HelperFunctions testObject8;
  int y_ = 24;
  testObject8.setMapYCoordinate(y_);
  EXPECT_EQ(getMapYCoordinate(), y_);
}
/**
 *   @brief test function to test getter & setter method for data_ of
 * MapStructure class
 *
 *
 *   @param MapStructureTests, the gtest framework
 *   @param getSetData, the name of test
 */
TEST(MapStructureTests, getSetData) {
  HelperFunctions testObject9;
  double data_ = 0.95;
  testObject9.setMapData(data_);
  EXPECT_EQ(getMapData(), data_);
}
